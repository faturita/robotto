#include <stdlib.h>
#include <string.h>

#include "cmd.h"

/**
 * Read the command line and get proper parameter.
 * 
 * This is a very common routine to get the parameters from the command line.
 * 
 **/
char *cmd_value(int argc, char *argv[], char str[])
{
	int i=0;
	
	for(i=0;i<argc;i++) {
		if (strcmp(argv[i],str)==0) {
			if (i+1<argc) return argv[i+1];
				else return argv[i];
			
		}
	}
	
	return NULL;
}
