/*******************************************************************************
 * File:		FILE
 * Module:		MODULE
 * Author:		AUTHOR
 * Date:		DATE
 * 
 * Notes:
 *
 * $Id$
 *
 * History:
 * $Log$
 *
 ******************************************************************************/
 
/*
 *
 *
 * 3.2. Non-Canonical Input Processing
 * 
 * In non-canonical input processing mode, input is not assembled into lines and
 * input processing (erase, kill, delete, etc.) does not occur. Two parameters
 * control the behavior of this mode: c_cc[VTIME] sets the character timer, and
 * c_cc[VMIN] sets the minimum number of characters to receive before satisfying
 * the read.
 * 
 * If MIN > 0 and TIME = 0, MIN sets the number of characters to receive before
 * the read is satisfied. As TIME is zero, the timer is not used.
 * 
 * If MIN = 0 and TIME > 0, TIME serves as a timeout value. The read will be
 * satisfied if a single character is read, or TIME is exceeded (t = TIME *0.1
 * s). If TIME is exceeded, no character will be returned.
 * 
 * If MIN > 0 and TIME > 0, TIME serves as an inter-character timer. The read
 * will be satisfied if MIN characters are received, or the time between two
 * characters exceeds TIME. The timer is restarted every time a character is
 * received and only becomes active after the first character has been received.
 * 
 * If MIN = 0 and TIME = 0, read will be satisfied immediately. The number of
 * characters currently available, or the number of characters requested will be
 * returned. According to Antonino (see contributions), you could issue a fcntl
 * (fd, F_SETFL, FNDELAY); before reading to get the same result.
 * 
 * By modifying newtio.c_cc[VTIME] and newtio.c_cc[VMIN] all modes described
 * above can be tested.
 * 
 */

#include <sys/types.h>                                                    
#include <sys/stat.h>                                                     
#include <fcntl.h>                                                        
#include <termios.h>                                                      
#include <stdio.h>                                                        

#define BAUDRATE B1200                                                   
#define MODEMDEVICE "/dev/ttyS0"                                          
#define _POSIX_SOURCE 1 /* POSIX compliant source */                      
#define FALSE 0                                                           
#define TRUE 1                                                            

volatile int STOP=FALSE;                                                  

main()                                                                    
{                                                                         
        int fd,c, res;                                                          
        struct termios oldtio,newtio;                                           
        unsigned char buf[255];                                                          

        int bytes;
        unsigned char mouse[3];

        fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );                             
        if (fd <0) {perror(MODEMDEVICE); exit(-1); }                            

        tcgetattr(fd,&oldtio); /* save current port settings */                 

        bzero(&newtio, sizeof(newtio));                                         
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;             
        newtio.c_iflag = IGNPAR;                                                
        newtio.c_oflag = 0;                                                     

        /* set input mode (non-canonical, no echo,...) */                       
        newtio.c_lflag = 0;                                                     

        newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */         
        newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */ 

        tcflush(fd, TCIFLUSH);                                                  
        tcsetattr(fd,TCSANOW,&newtio);                                          


        bytes = 0;
        while (STOP==FALSE) {       /* loop for input */                        
                unsigned char c;

                res = read(fd,buf,1);   /* returns after 5 chars have been input */ 
                buf[res]=0;               /* so we can printf... */                   
                //fprintf(stderr, ":%02x:%d\n", buf[0], res);                                         
                //if (buf[0]=='z') STOP=TRUE;                                           

                c = buf[0];

                if ( c & 0x40 ) {
                        bytes = 1;
                        mouse[0] = c;
                }
                else if (bytes == 1) {
                        bytes = 2;
                        mouse[1] = c;
                }
                else if (bytes == 2) {
                        bytes = 0;
                        mouse[2] = c;
                        fprintf(stderr, "%02x %02x %02x\n", mouse[0], mouse[1], mouse[2]);
                        //printf("%c%c%c", mouse[0], mouse[1], mouse[2]);
                }
        }                                                                       
        tcsetattr(fd,TCSANOW,&oldtio);                                          
}                                                                         


/* End of FILE */
