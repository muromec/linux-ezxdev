/*
 * Originally by Linus Torvalds.
 * Smart CONFIG_* processing by Werner Almesberger, Michael Chastain.
 *
 * Usage: mkdep cflags -- file ...
 * 
 * Read source files and output makefile dependency lines for them.
 * I make simple dependency lines for #include <*.h> and #include "*.h".
 * I also find instances of CONFIG_FOO and generate dependencies
 *    like include/config/foo.h.
 *
 * 1 August 1999, Michael Elizabeth Chastain, <mec@shout.net>
 * - Keith Owens reported a bug in smart config processing.  There used
 *   to be an optimization for "#define CONFIG_FOO ... #ifdef CONFIG_FOO",
 *   so that the file would not depend on CONFIG_FOO because the file defines
 *   this symbol itself.  But this optimization is bogus!  Consider this code:
 *   "#if 0 \n #define CONFIG_FOO \n #endif ... #ifdef CONFIG_FOO".  Here
 *   the definition is inactivated, but I still used it.  It turns out this
 *   actually happens a few times in the kernel source.  The simple way to
 *   fix this problem is to remove this particular optimization.
 *
 * 2.3.99-pre1, Andrew Morton <andrewm@uow.edu.au>
 * - Changed so that 'filename.o' depends upon 'filename.[cS]'.  This is so that
 *   missing source files are noticed, rather than silently ignored.
 *
 * 2.4.2-pre3, Keith Owens <kaos@ocs.com.au>
 * - Accept cflags followed by '--' followed by filenames.  mkdep extracts -I
 *   options from cflags and looks in the specified directories as well as the
 *   defaults.   Only -I is supported, no attempt is made to handle -idirafter,
 *   -isystem, -I- etc.
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * modified by w15879, for EZX platform
 */


#include <ctype.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#define MAX_PATH_LENGTH	1024
#define MAX_INCLUDE	1000
#define MAX_SUB_INCLUDE 100
#define MAX_HASH	512

/*
 * include tree + hash table access 
 */

typedef struct include_dep {
    struct include_dep *next; /* hash table link */
    char *name;               /* complete absolute filename */
    int nb_includes;          /* number of sub_includes or -1 if not filled */
    char *includes[MAX_SUB_INCLUDE]; /* full name of dependancies */
} *inc_dep;

inc_dep inc_hash[MAX_HASH];
int nb_include_in_hash = 0;

void init_dep(void)
{
    int i;

    for (i = 0;i < MAX_HASH;i++) inc_hash[i] = NULL;
    nb_include_in_hash = 0;
}

int get_hash(char *filename)
{
    char idx;
    char *cour = &filename[strlen(filename)];

    do {
        idx = *cour;
        cour--;
        if (*cour == '/') return((int) idx);
    } while (cour > filename);
    return((int) idx);
}

inc_dep add_inc(char *filename)
{
    inc_dep prev = NULL, cour;
    int res;
    int hash = get_hash(filename);

    cour = inc_hash[hash];

    /*
     * go through the list looking for the include.
     */
    while (cour != NULL) {
        res = strcmp(cour->name, filename);
        if (res == 0) return(cour);
        if (res < 0) break;
        prev = cour;
        cour = cour->next;
    }

    /*
     * not found, allocate and fill a new one.
     */
    nb_include_in_hash++;
    cour = (inc_dep) malloc(sizeof(struct include_dep));
    cour->name = strdup(filename);
    cour->nb_includes = -1;

    /*
     * not found, allocate and fill a new one.
     */
    if (prev == NULL) {
        /*
         * add at the head of the hash list.
         */
        cour->next = inc_hash[hash];
        inc_hash[hash] = cour;
    } else {
        /*
         * add if after prev.
         */
        cour->next = prev->next;
        prev->next = cour;
    }
    return(cour);
}

void add_inc_dep(inc_dep cour, char *filename)
{
    if (cour->nb_includes < 0)
        cour->nb_includes = 0;
    if (cour->nb_includes == MAX_SUB_INCLUDE) {
	fprintf(stderr,"increase MAX_SUB_INCLUDE : %d not sufficient\n",
                MAX_SUB_INCLUDE);
        return;
    }
    cour->includes[cour->nb_includes++] = strdup(filename);
}

char depname[MAX_PATH_LENGTH] = "";
int hasdep;

struct path_struct {
	int len;
	char *buffer;
};
struct path_struct *path_array;
int paths;

int nb_path = 0;

typedef char include_name[MAX_PATH_LENGTH];

include_name include_list[MAX_INCLUDE];

int nb_include = 0;

int handling_includes = 0;
inc_dep current_include = NULL;

static void add_local_include(char *name)
{
	int i = 0;

        add_inc(name);
        if ((handling_includes != 0) &&
            (current_include != NULL))
            add_inc_dep(current_include, name);

        for (i = 0;i < nb_include;i++) {
	    if (!strcmp(include_list[i], name)) return;
	}
	strcpy(include_list[nb_include], name);
	nb_include++;
}

/* Current input file */
static const char *g_filename;

/*
 * This records all the configuration options seen.
 * In perl this would be a hash, but here it's a long string
 * of values separated by newlines.  This is simple and
 * extremely fast.
 */
char * str_config  = NULL;
int    size_config = 0;
int    len_config  = 0;

/*
 * Grow the configuration string to a desired length.
 * Usually the first growth is plenty.
 */
void grow_config(int len)
{
	while (len_config + len > size_config) {
		if (size_config == 0)
			size_config = 2048;
		str_config = realloc(str_config, size_config *= 2);
		if (str_config == NULL)
			{ perror("malloc config"); exit(1); }
	}
}



/*
 * Lookup a value in the configuration string.
 */
int is_defined_config(const char * name, int len)
{
	const char * pconfig;
	const char * plast = str_config + len_config - len;
	for ( pconfig = str_config + 1; pconfig < plast; pconfig++ ) {
		if (pconfig[ -1] == '\n'
		&&  pconfig[len] == '\n'
		&&  !memcmp(pconfig, name, len))
			return 1;
	}
	return 0;
}



/*
 * Add a new value to the configuration string.
 */
void define_config(const char * name, int len)
{
	grow_config(len + 1);

	memcpy(str_config+len_config, name, len);
	len_config += len;
	str_config[len_config++] = '\n';
}



/*
 * Clear the set of configuration strings.
 */
void clear_config(void)
{
	len_config = 0;
	define_config("", 0);
}



/*
 * Handle an #include line.
 */
void handle_include(int start, const char * name, int len)
{
	struct path_struct *path;
	int i;

	if (len == 14 && !memcmp(name, "linux/config.h", len))
		return;

	if (len >= 7 && !memcmp(name, "config/", 7))
		define_config(name+7, len-7-2);

	for (i = start, path = path_array+start; i < paths; ++i, ++path) {
		memcpy(path->buffer+path->len, name, len);
		path->buffer[path->len+len] = '\0';
		if (access(path->buffer, F_OK) == 0) {
			add_local_include(path->buffer);
			return;
		}
	}

}



/*
 * Add a path to the list of include paths.
 */
void add_path(const char * name)
{
	struct path_struct *path;
	char resolved_path[PATH_MAX+1];
	const char *name2;

	if (strcmp(name, ".")) {
		name2 = realpath(name, resolved_path);
		if (!name2) {
			fprintf(stderr, "realpath(%s) failed, %m\n", name);
			exit(1);
		}
	}
	else {
		name2 = "";
	}

	path_array = realloc(path_array, (++paths)*sizeof(*path_array));
	if (!path_array) {
		fprintf(stderr, "cannot expand path_arry\n");
		exit(1);
	}

	path = path_array+paths-1;
	path->len = strlen(name2);
	path->buffer = malloc(path->len+1+256+1);
	if (!path->buffer) {
		fprintf(stderr, "cannot allocate path buffer\n");
		exit(1);
	}
	strcpy(path->buffer, name2);
	if (path->len && *(path->buffer+path->len-1) != '/') {
		*(path->buffer+path->len) = '/';
		*(path->buffer+(++(path->len))) = '\0';
	}
}


/*
 * Record the use of a CONFIG_* word.
 */
void use_config(const char * name, int len)
{
	char *pc;
	int i;
	char config_path[MAX_PATH_LENGTH]="$(wildcard ";

	pc = path_array[paths-1].buffer + path_array[paths-1].len;
	memcpy(pc, "config/", 7);
	pc += 7;

	for (i = 0; i < len; i++) {
	    char c = name[i];
	    if (isupper((int)c)) c = tolower((int)c);
	    if (c == '_')   c = '/';
	    pc[i] = c;
	}
	pc[len] = '\0';

	if (is_defined_config(pc, len))
	    return;

	define_config(pc, len);

	memcpy(config_path+11, path_array[paths-1].buffer, path_array[paths-1].len+len+7);
	memcpy(config_path+path_array[paths-1].len+len+18, ".h)\0", 4);
	
	add_local_include(config_path);
}



/*
 * Macros for stunningly fast map-based character access.
 * __buf is a register which holds the current word of the input.
 * Thus, there is one memory access per sizeof(unsigned long) characters.
 */

#if defined(__alpha__) || defined(__i386__) || defined(__ia64__)  || defined(__x86_64__) || defined(__MIPSEL__)	\
    || defined(__arm__)
#define LE_MACHINE
#endif

#ifdef LE_MACHINE
#define next_byte(x) (x >>= 8)
#define current ((unsigned char) __buf)
#else
#define next_byte(x) (x <<= 8)
#define current (__buf >> 8*(sizeof(unsigned long)-1))
#endif

#define GETNEXT { \
	next_byte(__buf); \
	if ((unsigned long) next % sizeof(unsigned long) == 0) { \
		if (next >= end) \
			break; \
		__buf = * (unsigned long *) next; \
	} \
	next++; \
}

/*
 * State machine macros.
 */
#define CASE(c,label) if (current == c) goto label
#define NOTCASE(c,label) if (current != c) goto label

/*
 * Yet another state machine speedup.
 */
#define MAX2(a,b) ((a)>(b)?(a):(b))
#define MIN2(a,b) ((a)<(b)?(a):(b))
#define MAX5(a,b,c,d,e) (MAX2(a,MAX2(b,MAX2(c,MAX2(d,e)))))
#define MIN5(a,b,c,d,e) (MIN2(a,MIN2(b,MIN2(c,MIN2(d,e)))))



/*
 * The state machine looks for (approximately) these Perl regular expressions:
 *
 *    m|\/\*.*?\*\/|
 *    m|\/\/.*|
 *    m|'.*?'|
 *    m|".*?"|
 *    m|#\s*include\s*"(.*?)"|
 *    m|#\s*include\s*<(.*?>"|
 *    m|#\s*(?define|undef)\s*CONFIG_(\w*)|
 *    m|(?!\w)CONFIG_|
 *
 * About 98% of the CPU time is spent here, and most of that is in
 * the 'start' paragraph.  Because the current characters are
 * in a register, the start loop usually eats 4 or 8 characters
 * per memory read.  The MAX5 and MIN5 tests dispose of most
 * input characters with 1 or 2 comparisons.
 */
void state_machine(const char * map, const char * end)
{
	const char * next = map;
	const char * map_dot;
	unsigned long __buf = 0;

	for (;;) {
start:
	GETNEXT
__start:
	if (current > MAX5('/','\'','"','#','C')) goto start;
	if (current < MIN5('/','\'','"','#','C')) goto start;
	CASE('/',  slash);
	CASE('\'', squote);
	CASE('"',  dquote);
	CASE('#',  pound);
	CASE('C',  cee);
	goto start;

/* // */
slash_slash:
	GETNEXT
	CASE('\n', start);
	NOTCASE('\\', slash_slash);
	GETNEXT
	goto slash_slash;

/* / */
slash:
	GETNEXT
	CASE('/',  slash_slash);
	NOTCASE('*', __start);
slash_star_dot_star:
	GETNEXT
__slash_star_dot_star:
	NOTCASE('*', slash_star_dot_star);
	GETNEXT
	NOTCASE('/', __slash_star_dot_star);
	goto start;

/* '.*?' */
squote:
	GETNEXT
	CASE('\'', start);
	NOTCASE('\\', squote);
	GETNEXT
	goto squote;

/* ".*?" */
dquote:
	GETNEXT
	CASE('"', start);
	NOTCASE('\\', dquote);
	GETNEXT
	goto dquote;

/* #\s* */
pound:
	GETNEXT
	CASE(' ',  pound);
	CASE('\t', pound);
	CASE('i',  pound_i);
	CASE('d',  pound_d);
	CASE('u',  pound_u);
	goto __start;

/* #\s*i */
pound_i:
	GETNEXT NOTCASE('n', __start);
	GETNEXT NOTCASE('c', __start);
	GETNEXT NOTCASE('l', __start);
	GETNEXT NOTCASE('u', __start);
	GETNEXT NOTCASE('d', __start);
	GETNEXT NOTCASE('e', __start);
	goto pound_include;

/* #\s*include\s* */
pound_include:
	GETNEXT
	CASE(' ',  pound_include);
	CASE('\t', pound_include);
	map_dot = next;
	CASE('"',  pound_include_dquote);
	CASE('<',  pound_include_langle);
	goto __start;

/* #\s*include\s*"(.*)" */
pound_include_dquote:
	GETNEXT
	CASE('\n', start);
	NOTCASE('"', pound_include_dquote);
	handle_include(0, map_dot, next - map_dot - 1);
	goto start;

/* #\s*include\s*<(.*)> */
pound_include_langle:
	GETNEXT
	CASE('\n', start);
	NOTCASE('>', pound_include_langle);
	handle_include(1, map_dot, next - map_dot - 1);
	goto start;

/* #\s*d */
pound_d:
	GETNEXT NOTCASE('e', __start);
	GETNEXT NOTCASE('f', __start);
	GETNEXT NOTCASE('i', __start);
	GETNEXT NOTCASE('n', __start);
	GETNEXT NOTCASE('e', __start);
	goto pound_define_undef;

/* #\s*u */
pound_u:
	GETNEXT NOTCASE('n', __start);
	GETNEXT NOTCASE('d', __start);
	GETNEXT NOTCASE('e', __start);
	GETNEXT NOTCASE('f', __start);
	goto pound_define_undef;

/*
 * #\s*(define|undef)\s*CONFIG_(\w*)
 *
 * this does not define the word, because it could be inside another
 * conditional (#if 0).  But I do parse the word so that this instance
 * does not count as a use.  -- mec
 */
pound_define_undef:
	GETNEXT
	CASE(' ',  pound_define_undef);
	CASE('\t', pound_define_undef);

	        NOTCASE('C', __start);
	GETNEXT NOTCASE('O', __start);
	GETNEXT NOTCASE('N', __start);
	GETNEXT NOTCASE('F', __start);
	GETNEXT NOTCASE('I', __start);
	GETNEXT NOTCASE('G', __start);
	GETNEXT NOTCASE('_', __start);

	map_dot = next;
pound_define_undef_CONFIG_word:
	GETNEXT
	if (isalnum(current) || current == '_')
		goto pound_define_undef_CONFIG_word;
	goto __start;

/* \<CONFIG_(\w*) */
cee:
	if (next >= map+2 && (isalnum((int)next[-2]) || next[-2] == '_'))
		goto start;
	GETNEXT NOTCASE('O', __start);
	GETNEXT NOTCASE('N', __start);
	GETNEXT NOTCASE('F', __start);
	GETNEXT NOTCASE('I', __start);
	GETNEXT NOTCASE('G', __start);
	GETNEXT NOTCASE('_', __start);

	map_dot = next;
cee_CONFIG_word:
	GETNEXT
	if (isalnum(current) || current == '_')
		goto cee_CONFIG_word;
	use_config(map_dot, next - map_dot - 1);
	goto __start;
    }
}



/*
 * Generate dependencies for one file.
 */
void do_depend(const char * filename)
{
	int mapsize;
	int pagesizem1 = getpagesize()-1;
	int fd;
	struct stat st;
	char * map;

	if (!memcmp(filename, "$(wildcard", 10)) return;

	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		perror(filename);
		return;
	}

	fstat(fd, &st);
	if (st.st_size == 0) {
		fprintf(stderr,"%s is empty\n",filename);
		close(fd);
		return;
	}

	mapsize = st.st_size;
	mapsize = (mapsize+pagesizem1) & ~pagesizem1;
	map = mmap(NULL, mapsize, PROT_READ, MAP_PRIVATE, fd, 0);
	if ((long) map == -1) {
		perror("mkdep: mmap");
		close(fd);
		return;
	}
	if ((unsigned long) map % sizeof(unsigned long) != 0)
	{
		fprintf(stderr, "do_depend: map not aligned\n");
		exit(1);
	}

	hasdep = 0;
	clear_config();
	state_machine(map, map+st.st_size);

	munmap(map, mapsize);
	close(fd);
}



/*
 * Generate dependencies for all files.
 */
int main(int argc, char **argv)
{
	int len;
	const char *hpath;

	/* Initialize include hash tabel */
        init_dep();

	hpath = getenv("HPATH");
	if (!hpath) {
		fputs("mkdep: HPATH not set in environment.  "
		      "Don't bypass the top level Makefile.\n", stderr);
		return 1;
	}

	add_path(".");		/* for #include "..." */

	while (++argv, --argc > 0) {
		if (strncmp(*argv, "-I", 2) == 0) {
			if (*((*argv)+2)) {
				add_path((*argv)+2);
			}
			else {
				++argv;
				--argc;
				add_path(*argv);
			}
		}
		else if (strcmp(*argv, "--") == 0) {
			break;
		}
	}

	add_path(hpath);	/* must be last entry, for config files */

	while (--argc > 0) {
		const char * filename = *++argv;
		int i;
		g_filename = 0;
		len = strlen(filename);
		memcpy(depname, filename, len+1);
		if (len > 2 && filename[len-2] == '.') {
			if (filename[len-1] == 'c' || filename[len-1] == 'S') {
			    depname[len-1] = 'o';
			    g_filename = filename;
			}
		}
		nb_include = 0;

                /*
		 * print the base dependancy between the .o and .c file.
		 */
		if (depname[len-1] == 'h') {
			continue;
		}
		printf("%s: %s", depname,filename);

		/*
		 * do basic dependancies on the C source file.
		 */
                handling_includes = 0;
		do_depend(filename);

		/*
		 * recurse dependancies on the included files.
		 * Warning, nb_include will grow over the loop.
		 */
                handling_includes = 1;
		for (i = 0;i < nb_include;i++) {
		    filename = include_list[i];
		    /*
		     * check the hash for existing dependencies.
		     * !!!
		     */
		    if (handling_includes) {
		        current_include = add_inc((char *)filename);
		        if (current_include->nb_includes >= 0) {
			    int i;

			    /*
			     * do not load and parse the file,
                             * dump the cache instead.
			     */
		            for (i = 0;i < current_include->nb_includes;i++)
                                add_local_include(current_include->includes[i]);

                            continue;
		        }
		    }
		    current_include = NULL;
		    do_depend(filename);
		}

                /*
		 * dump the dependancies found.
		 */
		for (i = 0;i < nb_include;i++)
			printf(" \\\n   %s", include_list[i]);
		printf("\n");
	}

	return 0;
}
