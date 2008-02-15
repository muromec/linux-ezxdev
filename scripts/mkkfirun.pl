#!/usr/bin/perl
#
# BRIEF MODULE DESCRIPTION
#    Parses a Kernel Function Instrumentation config file. The output
#    is C code representing the KFI logging run parameters listed in
#    in the config file.
#
# Copyright 2002 MontaVista Software Inc.
# Author: MontaVista Software, Inc.
#		stevel@mvista.com or source@mvista.com
#
#  This program is free software; you can redistribute	 it and/or modify it
#  under  the terms of	 the GNU General  Public License as published by the
#  Free Software Foundation;  either version 2 of the	License, or (at your
#  option) any later version.
#
#  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
#  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
#  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
#  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
#  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  You should have received a copy of the  GNU General Public License along
#  with this program; if not, write  to the Free Software Foundation, Inc.,
#  675 Mass Ave, Cambridge, MA 02139, USA.
#

sub parse_args {
    local($argstr) = $_[0];
    local(@arglist);
    local($i) = 0;

    for (;;) {
	while ($argstr =~ /^\s*$/ || $argstr =~ /^\s*\#/) {
	    $argstr = <RUNFILE>;
	}
    
	while ($argstr =~ s/^\s*(\w+)\s*(.*)/\2/) {
	    $arglist[$i++] = $1;
	    if (!($argstr =~ s/^\,(.*)/\1/)) {
		return @arglist;
	    }
	}
    }
}

sub parse_run {
    local($thisrun, $nextrun) = @_;
    local($start_type) = "TRIGGER_NONE";
    local($stop_type) = "TRIGGER_NONE";

    local($filter_noint) = 0;
    local($filter_onlyint) = 0;
    local(@filter_func_list) = (0);
    local($filter_func_list_size) = 0;
    local(@filter_time_args) = (0,0);
    local($logsize) = "MAX_RUN_LOG_ENTRIES";

    while (<RUNFILE>) {

	last if /^\s*end\b/;
	
	if ( /^\s*trigger\s+(\w+)\s+(\w+)\b\s*([\w\,\s]*)/ ) {

	    $trigwhich = $1;
	    $trigtype = $2;
	    @trigargs = &parse_args($3);
	    
	    if ($trigwhich eq "start") {
		if ($trigtype eq "entry") {
		    $start_type = "TRIGGER_FUNC_ENTRY";
		} elsif ($trigtype eq "exit") {
		    $start_type = "TRIGGER_FUNC_EXIT";
		} elsif ($trigtype eq "time") {
		    $start_type = "TRIGGER_TIME";
		} else {
		    die "#### PARSE ERROR: invalid trigger type ####\n";
		    }
		@start_args = @trigargs;
	    } elsif ($trigwhich eq "stop") {
		if ($trigtype eq "entry") {
		    $stop_type = "TRIGGER_FUNC_ENTRY";
		} elsif ($trigtype eq "exit") {
		    $stop_type = "TRIGGER_FUNC_EXIT";
		} elsif ($trigtype eq "time") {
		    $stop_type = "TRIGGER_TIME";
		} else {
		    die "#### PARSE ERROR: invalid trigger type ####\n";
		    }
		@stop_args = @trigargs;
	    } else {
		die "#### PARSE ERROR: invalid trigger ####\n";
		}
	    
	} elsif ( /^\s*filter\s+(\w+)\b\s*([\w\,\s]*)/ ) {
	    
	    $filtertype = $1;
	    
	    if ($filtertype eq "time") {
		@filter_time_args = &parse_args($2);
	    } elsif ($filtertype eq "noint") {
		$filter_noint = 1;
	    } elsif ($filtertype eq "onlyint") {
		$filter_onlyint = 1;
	    } elsif ($filtertype eq "funclist") {
		@filter_func_list = &parse_args($2);
		$filter_func_list_size = $#filter_func_list + 1;
	    } else {
		die "#### PARSE ERROR: invalid filter ####\n";
		}
	    
	} elsif ( /^\s*logsize\s+(\d+)/ ) {
	    @logargs = &parse_args($1);
	    $logsize = $logargs[0];
	}
    }

    # done parsing this run, now spit out the C code

    # print forward reference to next run
    if ($nextrun != 0) {
	printf("kfi_run_t kfi_run%d;\n", $nextrun);
    }

    if ($start_type eq "TRIGGER_FUNC_ENTRY" ||
	$start_type eq "TRIGGER_FUNC_EXIT") {
	printf("extern void %s(void);\n\n", $start_args[0]);
    }
    
    if ($stop_type eq "TRIGGER_FUNC_ENTRY" ||
	$stop_type eq "TRIGGER_FUNC_EXIT") {
	printf("extern void %s(void);\n\n", $stop_args[0]);
    }
    
    if ($filter_func_list_size) {
	$funclist_name = sprintf("run%d_func_list", $thisrun);

	for ($i = 0; $i < $filter_func_list_size; $i++) {
	    print "extern void $filter_func_list[$i](void);\n"
		if (!($filter_func_list[$i] =~ /^[0-9]/));
	}
	
	printf("\nstatic void* %s[] = {\n", $funclist_name);
	
	for ($i = 0; $i < $filter_func_list_size; $i++) {
	    printf("\t(void*)%s,\n", $filter_func_list[$i]);
	}
	printf("};\n\n");
    } else {
	$funclist_name = "NULL";
    }
    
    printf("static kfi_entry_t run%d_log[%s];\n\n", $thisrun, $logsize);
    
    printf("kfi_run_t kfi_run%d = {\n", $thisrun);
    
    printf("\t0, 0,\n"); # triggered and complete flags
    
    # start trigger struct
    if ($start_type eq "TRIGGER_FUNC_ENTRY" ||
	$start_type eq "TRIGGER_FUNC_EXIT") {
	printf("\t{ %s, { func_addr: (void*)%s } },\n",
	       $start_type, $start_args[0]);
    } elsif ($start_type eq "TRIGGER_TIME") {
	printf("\t{ %s, { time: %d } },\n", $start_type, $start_args[0]);
    } else {
	printf("\t{ %s, {0} },\n", $start_type);
    }
    
    # stop trigger struct
    if ($stop_type eq "TRIGGER_FUNC_ENTRY" ||
	$stop_type eq "TRIGGER_FUNC_EXIT") {
	printf("\t{ %s, { func_addr: (void*)%s } },\n",
	       $stop_type, $stop_args[0]);
    } elsif ($stop_type eq "TRIGGER_TIME") {
	printf("\t{ %s, { time: %d } },\n", $stop_type, $stop_args[0]);
    } else {
	printf("\t{ %s, {0} },\n", $stop_type);
    }

    # filters struct
    printf("\t{ %d, %d, %d, %d, %s, %d, {0} },\n",
	   $filter_time_args[0], $filter_time_args[1],
	   $filter_noint, $filter_onlyint,
	   $funclist_name, $filter_func_list_size);

    if ($nextrun != 0) {
	printf("\trun%d_log, %s, 0, %d, &kfi_run%d,\n",
	       $thisrun, $logsize, $thisrun, $nextrun);
    } else {
	printf("\trun%d_log, %s, 0, %d, NULL,\n",
	       $thisrun, $logsize, $thisrun);
    }
    
    printf("};\n\n");
}


$numrun = 0;

open(RUNFILE, $ARGV[0]) || die "Can't open KFI run config file";

# first pass get number of run configs listed
while (<RUNFILE>) {
    if ( /^\s*begin\b/ ) {
	$numrun++;
    }
}

$numrun != 0 || die "No run listed???\n";
    
close(RUNFILE);
open(RUNFILE, "$ARGV[0]");

# print warning
print "/* DO NOT EDIT! It was automatically generated by mkkfirun.pl */\n\n";

# print needed headers
print "#include <linux/types.h>\n";
print "#include <linux/kfi.h>\n\n";

$runindex = 0;
while (<RUNFILE>) {
    if ( /^\s*begin\b/ ) {
	if ($runindex == $numrun-1) {
	    &parse_run($runindex, 0);
	} else {
	    &parse_run($runindex, $runindex+1);
	}
	$runindex++;
    }
}

printf("const int kfi_num_runs = %d;\n", $numrun);
printf("kfi_run_t* kfi_first_run = &kfi_run0;\n");
printf("kfi_run_t* kfi_last_run = &kfi_run%d;\n", $numrun-1);
