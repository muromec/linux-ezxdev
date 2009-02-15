#!/bin/sh -
#

# FLASK

set -e

awk=$1
shift 1

# output file
output_file="flask.h"
debug_file="class_to_string.h"
debug_file2="initial_sid_to_string.h"

cat $* | $awk "
BEGIN	{
		outfile = \"$output_file\"
		debugfile = \"$debug_file\"
		debugfile2 = \"$debug_file2\"
		"'
		nextstate = "CLASS";

		printf("/* This file is automatically generated.  Do not edit. */\n") > outfile;

		printf("#ifndef _LINUX_FLASK_H_\n") > outfile;
		printf("#define _LINUX_FLASK_H_\n") > outfile;
		printf("\n#include <linux/flask/flask_types.h>\n") > outfile;

		printf("\n/*\n * Security object class definitions\n */\n") > outfile;
		printf("/* This file is automatically generated.  Do not edit. */\n") > debugfile;
		printf("/*\n * Security object class definitions\n */\n") > debugfile;
		printf("static char *class_to_string[] =\n{\n") > debugfile;
		printf("    \"null\",\n") > debugfile;
		printf("/* This file is automatically generated.  Do not edit. */\n") > debugfile2;
		printf("static char *initial_sid_to_string[] =\n{\n") > debugfile2;
		printf("    \"null\",\n") > debugfile2;
	}
/^[ \t]*#/	{ 
			next;
		}
$1 == "class"	{ 
			if (nextstate != "CLASS")
			{
				printf("Parse error:  Unexpected class definition on line %d\n", NR);
				next;	
			}

			if ($2 in class_found)
			{
				printf("Duplicate class definition for %s on line %d.\n", $2, NR);
				next;
			}	
			class_found[$2] = 1;

			class_value++;

			printf("#define SECCLASS_%s", toupper($2)) > outfile;
			for (i = 0; i < 40 - length($2); i++) 
				printf(" ") > outfile; 
			printf("%d\n", class_value) > outfile; 

			printf("    \"%s\",\n", $2) > debugfile;
		}
$1 == "sid"	{ 
			if (nextstate == "CLASS")
			{
			    nextstate = "SID";
			    printf("};\n\n") > debugfile;
			    printf("\n/*\n * Security identifier indices for initial entities\n */\n") > outfile;			    
			}

			if ($2 in sid_found)
			{
				printf("Duplicate SID definition for %s on line %d.\n", $2, NR);
				next;
			}	
			sid_found[$2] = 1;
			sid_value++;

			printf("#define SECINITSID_%s", toupper($2)) > outfile;
			for (i = 0; i < 37 - length($2); i++) 
				printf(" ") > outfile; 
			printf("%d\n", sid_value) > outfile; 
			printf("    \"%s\",\n", $2) > debugfile2;
		}
END	{
		if (nextstate != "SID")
			printf("Parse error:  Unexpected end of file\n");

		printf("\n#define SECINITSID_NUM") > outfile;
		for (i = 0; i < 34; i++) 
			printf(" ") > outfile; 
		printf("%d\n", sid_value) > outfile; 
		printf("\n#endif\n") > outfile;
		printf("};\n\n") > debugfile2;
	}'

# FLASK