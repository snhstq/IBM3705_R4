/* Copyright (c) 2024, Edwin Freekenhorst

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   HENK STEGEMAN AND EDWIN FREEKENHORST BE LIABLE FOR ANY CLAIM, DAMAGES OR
   OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
   ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ---------------------------------------------------------------------------

   NModem_mm.C   (c) Copyright  Edwin Freekenhost

   This module emulates a Null Modem.
   It is intended to connect two 3705s via SDLC lines,
   but can also be used to connect to BSC lines.
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <string.h>
#include <ifaddrs.h>
#include <errno.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define RDY 1
#define NRDY 0
#define ON 1
#define OFF 0

#define LINEBASE   37500

uint16_t Tdbg_flag = OFF;          /* 1 when Ttrace.log open */
FILE *T_trace;

uint8_t bfr[256];


/* Variablers used */
uint8_t        LINE_rbuf[65536];    /* Line Read Buffer                      */
uint16_t       LINErlen;            /* Buffer size of received data          */

int SocketReadAct (int fd);


//*********************************************************************
// Function to check if socket is (still) connected                   *
//*********************************************************************
static bool IsSocketConnected(int sockfd) {
   int rc;
   struct sockaddr_in peer_addr;
   int addrlen = sizeof(peer_addr);

   if (sockfd < 1) {
      return false;
   }
   rc = getpeername(sockfd, (struct sockaddr*)&peer_addr, &addrlen);
   if (rc != 0) {
      return false;
   }
   return true;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Main section - establish and manage TCP connections                        */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void main(int argc, char *argv[]) {
   int      sockopt;                     /* Used for setsocketoption          */
   int      pendingrcv;                  /* pending data on the socket        */
   struct   addrinfo *line1host;
   struct   addrinfo *line2host;
   int      line1num = 20;               /* Line number (default 20)          */
   int      line2num = 20;               /* Line number (default 20)          */
   struct   sockaddr_in host1addr;       /* host1 address details             */
   struct   sockaddr_in host2addr;       /* host2 address details             */
   struct   sockaddr_in line1addr;       /* Line-1 connection details         */
   struct   sockaddr_in line2addr;       /* Line-1 connection details         */
   int      line1_fd;                    /* Line-1 socket                     */
   int      rs232f_fd;                   /* RS232 line 1 signal socket        */
   int      line2_fd;                    /* Line-1 socket                     */
   int      rs232s_fd;                   /* RS232 line 1 signal socket        */
   int      line1state;                  /* state of line 1                   */
   int      line2state;                  /* state of line 2                   */
   uint8_t  signal;                      /* RS232 signal                      */
   int      raiseRTS;                    /* flag when RTS should be raised    */
   int      i, rc, rc1, rc2;
   char     *host1name;
   char     *host2name;
   char     host1[NI_MAXHOST];
   char     host2[NI_MAXHOST];
   char     portl1[6];
   char     portl2[6];

   char ipv4addr[sizeof(struct in_addr)];

   /* Read command line arguments */
   if (argc == 1) {
      printf("\rDLSw: Error - Arguments missing\n");
      printf("\r   Valid arguments are:\n");
      printf("\r   -cchn1 {hostname}  : hostname of host running the first 3705\n");
      printf("\r   -ccip1 {ipaddress} : ipaddress of host running the first 3705 \n");
      printf("\r   -cchn2 {hostname}  : hostname of host running the second 3705\n");
      printf("\r   -ccip2 {ipaddress} : ipaddress of host running the second 3705 \n");
      printf("\r   -line1 {line number} : Line number on first 3705 to connect to\n");
      printf("\r   -line2 {line number} : Line number on second 3705 to connect to\n");
      printf("\r   -d : switch debug on  \n");
      return;
   }
   Tdbg_flag = OFF;
   i = 1;

   while (i < argc) {
      if (strcmp(argv[i], "-d") == 0) {
         Tdbg_flag = ON;
         printf("\rNModem: Debug on. Trace file is trace_NModem.log\n");
         i++;
         continue;
      } else if (strcmp(argv[i], "-cchn1") == 0) {
         if ( (rc = getaddrinfo(argv[i+1],NULL,NULL,&line1host )) != 0 ) {
            printf("\rNULL: Cannot resolve 3705 hostname 1 %s\n", argv[i+1]);
            return;                /* error */
         }  // End if line1ent
         printf("\rNModem: Connection to be established with line-1 at 3705 on host %s\n", argv[i+1]);
         host1name = argv[i+1];
         i = i+2;
         continue;
      } else if (strcmp(argv[i], "-cchn2") == 0) {
         if ( (rc = getaddrinfo(argv[i+1],NULL,NULL,&line2host )) != 0 ) {
            printf("\rNULL: Cannot resolve 3705 hostname 2 %s\n", argv[i+1]);
            return;                /* error */
         }  // End if line2ent
         printf("\rNModem: Connection to be established with line-2 at 3705 on host %s\n", argv[i+1]);
         host2name = argv[i+1];
         i = i+2;
         continue;
      } else if (strcmp(argv[i], "-ccip1") == 0) {
         if ( inet_pton(AF_INET, argv[i+1], &host1addr.sin_addr) != 1) {
            printf("\rNModem: Cannot convert 3705 1 ip address %s, error: %s\n", argv[i+1],strerror(errno));
            return; /* error */
          }
          host1addr.sin_family=AF_INET;
          if (rc =(getnameinfo((struct sockaddr*)&host1addr,sizeof(host1addr), host1,sizeof(host1),NULL,0, NI_NOFQDN | NI_NAMEREQD )) != 0 ) {
             printf("\rNModem: Cannot resolve 3705 1 ip address %s, error: %s, rc: %s\n", argv[i+1],strerror(errno), gai_strerror(rc));
             return; /* error */
          } // End if lineent
          printf("\rNModem: Connection to be established with line-1 at 3705 on host %s\n",host1);
          host1name = host1;
          i = i + 2;
          continue;
      } else if (strcmp(argv[i], "-ccip2") == 0) {
         if ( inet_pton(AF_INET, argv[i+1], &host2addr.sin_addr) != 1) {
            printf("\rNModem: Cannot convert 3705 2 ip address %s, error: %s\n", argv[i+1],strerror(errno));
            return; /* error */
         }
         host2addr.sin_family=AF_INET;
         if (rc =(getnameinfo((struct sockaddr*)&host2addr,sizeof(host2addr), host2,sizeof(host2),NULL,0, NI_NOFQDN | NI_NAMEREQD )) != 0 ) {
            printf("\rNModem: Cannot resolve 3705 2 ip address %s, error: %s, rc: %s\n", argv[i+1],strerror(errno), gai_strerror(rc));
            return; /* error */
         }  // End if lineent
         printf("\rNModem: Connection to be established with line-2 at 3705 on host %s\n",host2);
         host2name = host2;
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-line1") == 0) {
         sscanf(argv[i+1], "%d", &line1num);
         printf("\rNModem: Connection to be established with line-1 %d\n", line1num);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-line2") == 0) {
         sscanf(argv[i+1], "%d", &line2num);
         printf("\rNModem: Connection to be established with line-2 %d\n", line2num);
         i = i + 2;
         continue;
      } else {
         printf("\rNULL: invalid argument %s\n", argv[i]);
         printf("\r     Valid arguments are:\n");
         printf("\r     -cchn1 {hostname}    : hostname of host running the first 3705\n");
         printf("\r     -ccip1 {ipaddress}   : ipaddress of host running the first 3705 \n");
         printf("\r     -cchn2 {hostname}    : hostname of host running the second 3705\n");
         printf("\r     -ccip2 {ipaddress}   : ipaddress of host running the second 3705 \n");
         printf("\r     -line1 {line number} : Line number on the first 3705 to connect to\n");
         printf("\r     -line2 {line number} : Line number on the second 3705 to connect to\n");
         printf("\r     -d : switch debug on  \n");
         return;
      }  // End else
   }  // End while

   //********************************************************************
   // Null modem debug trace facility
   //********************************************************************
   if (Tdbg_flag == ON) {
      T_trace = fopen("trace_NModem.log", "w");
      fprintf(T_trace, "     ****** Null modem log file ****** \n\n"
                       "     NModem_mm -d : trace all Null Modem activities\n"
                       );
   }

   //*******************************************************************************
   //* Prepare the line connections
   //* A parallel connection will be established to send RS232 signals to the LIB
   //* these signals are used to steer the action of the 3705 scanner
   //*******************************************************************************
   // Assign IP addr and PORT numbers.

   snprintf(portl1, 6, "%d", LINEBASE + line1num);
   if ( (rc = getaddrinfo(host1name,portl1,NULL,&line1host )) != 0 ) {
      printf("\rNModem: Cannot resolve first 3705 address for %s\n", host1name);
      return; /* error */
    }  // End if ( (rc = getaddrinfo

   snprintf(portl2, 6, "%d", LINEBASE + line2num);
   if ( (rc = getaddrinfo(host2name,portl2,NULL,&line2host )) != 0 ) {
      printf("\rNModem: Cannot resolve second 3705 address for %s\n", host2name);
      return; /* error */
    }  // End if ( (rc = getaddrinfo

   // Line-1 socket creation
   line1_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (line1_fd <= 0) {
      printf("\rNModem: Cannot create socket socket for line 1\n");
      return;
   }

   // RS232 line-1 socket creation
   rs232f_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (rs232f_fd <= 0) {
      printf("\rNModem: Cannot create rs232 socket for line 1\n");
      return;
   }
   // Line-2 socket creation
   line2_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (line2_fd <= 0) {
      printf("\rNModem: Cannot create socket socket for line 2\n");
      return;
   }

   // RS232 line-21 socket creation
   rs232s_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (rs232s_fd <= 0) {
      printf("\rNModem: Cannot create rs232 socket for line 2\n");
      return;
   }

   // Initialize state values
   line1state = NRDY;   // Line initially not ready
   line2state = NRDY;   // Line initially not ready
   raiseRTS = ON;       // RTS should be raised after lines are connected

   while (1) {
      if (line1state == NRDY) {
         // Line and signal sockets have been created. The connection to the LIBs will be done next
         if (!(IsSocketConnected(line1_fd))) {
            rc1 = connect(line1_fd, line1host->ai_addr,line1host->ai_addrlen);
         }  // End if (!(IsSocketConnected(line1_fd)))
         if ((rc1 == 0) && (!(IsSocketConnected(rs232f_fd)))) {
            rc2 = connect(rs232f_fd, line1host->ai_addr,line1host->ai_addrlen);
         }  // End if ((rc1 = 0) && (!(IsSocketConnected(rs232f_fd))))
         if ((rc1 == 0) && (rc2 == 0)) {
            printf("\rNModem: Line 1 connection has been established\n");
            line1state = RDY;    // Line is now ready
         }  // End if ((rc1 == 0) && (rc2 == 0))
      }  // End  if (line1state == NRDY)

      if (line2state == NRDY) {
         if (!(IsSocketConnected(line2_fd))) {
            rc1 = connect(line2_fd, line2host->ai_addr,line2host->ai_addrlen);
         }  // End if (!(IsSocketConnected(line2_fd)))
         if ((rc1 == 0) && (!(IsSocketConnected(rs232s_fd)))) {
            rc2 = connect(rs232s_fd, line2host->ai_addr,line2host->ai_addrlen);
         }  // End if ((rc1 = 0) && (!(IsSocketConnected(rs232s_fd))))
         if ((rc1 == 0) && (rc2 == 0)) {
            printf("\rNModem: Line 2 connection has been established\n");
            line2state = RDY; // Line is now ready
         }  // End if ((rc1 == 0) && (rc2 == 0))
      }  // End  if (line12state == NRDY)

      if ((line1state == RDY) && (line2state == RDY)) {
         pendingrcv = 0;
         rc = ioctl(rs232f_fd, FIONREAD, &pendingrcv);
         if (pendingrcv > 0) {
            for (int i=0; i < pendingrcv; i++) {
            rc = read(rs232f_fd, &signal, 1);
         }
         rc = send(rs232s_fd, &signal,1, 0);     // Pass signal on to other link
         }  // End if (pendingrcv > 0)
         pendingrcv = 0;
         rc = ioctl(rs232s_fd, FIONREAD, &pendingrcv);
         if (pendingrcv > 0) {
            for (int i=0; i < pendingrcv; i++) {
            rc = read(rs232s_fd, &signal, 1);
         }
         rc = send(rs232f_fd, &signal,1, 0);     // Pass signal on to other link
         }  // End if (pendingrcv > 0)
         //*****************************************************************************************
         // Check if there is data to be received from line 1
         // If the connection was lost, try to re-establish
         //*****************************************************************************************
         pendingrcv = 0;

         if (IsSocketConnected(line1_fd)) {
            rc = ioctl(line1_fd, FIONREAD, &pendingrcv);
            // *** Keep the below for now; under investgation ***
         } else {
            printf("\rNModem: Line 1 connection dropped, trying to re-establish\n");
            // Lower RTS so remote side stops sending
            // Line and signal socket re-creation. First close the sockets
            close(line1_fd);                      // Close data connection
            close(rs232f_fd);                     // Close RS232 connection
            line1state = NRDY;                    // Set line to Not Ready
            line1_fd = 0;                         // Reset file descriptor for data connection
            rs232f_fd = 0;                        // Reset file descriptor for RS232 connection
            line1_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (line1_fd <= 0) {
               printf("\rNModem: Cannot create line-1 socket\n");
               return;
            }  // End if (line_fd <= 0)
            rs232f_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (rs232f_fd <= 0) {
               printf("\rNModem: Cannot create RS232 line 1 signal socket\n");
               return;
            }  // End if (rs232f_fd <= 0)
         }  // End if (IsSocketConnected(line1_fd))

         if (pendingrcv > 0) {
            LINErlen = read(line1_fd, LINE_rbuf, sizeof(LINE_rbuf));
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rLine 1 Read Buffer: ");
               for (int i = 0; i < LINErlen; i ++) {
                  fprintf(T_trace, "%02X ", LINE_rbuf[i]);
               }  // End for (int i = 0;
               fprintf(T_trace, "\n");
               fflush(T_trace);
            }  // End if debug
            // Forward the received data to the other line
            rc = send(line2_fd, LINE_rbuf, LINErlen, 0);
            if (LINErlen != rc) {
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\rNModem: Line 2 Transmit buffer size %d bytes, actual transmitted %d bytes\n",LINErlen,rc);
            }  // End  if (LINErlen != rc
         }  // End if (pendingrcv > 0)

         //*****************************************************************************************
         // Check if there is data to be received from line 2
         // If the connection was lost, try to re-establish
         //*****************************************************************************************
         pendingrcv = 0;
         if (IsSocketConnected(line2_fd)) {
            rc = ioctl(line2_fd, FIONREAD, &pendingrcv);
         } else {
            printf("\rNModem: Line 2 connection dropped, trying to re-establish\n");
            // Lower RTS so remote side stops sending
            // Line and signal socket re-creation. First close the sockets
            close(line2_fd);                      // Close data connection
            close(rs232s_fd);                     // Close RS232 connection
            line2state = NRDY;                    // Set line to Not Ready
            line2_fd = 0;                         // Reset file descriptor for data connection
            rs232s_fd = 0;                        // Reset file descriptor for RS232 connection
            line2_fd = socket(AF_INET, SOCK_STREAM, 0);

            if (line2_fd <= 0) {
               printf("\rNModem: Cannot create line-2 socket\n");
               return;
            }  // End if (line2_fd <= 0)
            rs232s_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (rs232s_fd <= 0) {
               printf("\rNModem: Cannot create RS232 line 2 signal socket\n");
               return;
            }  // End if (rs232s_fd <= 0)
         }  // End if (IsSocketConnected(line2_fd))

         if (pendingrcv > 0) {
            LINErlen = read(line2_fd, LINE_rbuf, sizeof(LINE_rbuf));
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rLine 2 Read Buffer: ");
               for (int i = 0; i < LINErlen; i ++) {
                  fprintf(T_trace, "%02X ", LINE_rbuf[i]);
               } // End for (int i = 0;
               fprintf(T_trace, "\n");
               fflush(T_trace);
            }  // End if debug
            // Forward the received data to the other line
            rc = send(line1_fd, LINE_rbuf, LINErlen, 0);
            if (LINErlen != rc) {
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\rNModem: Line 1 Transmit buffer size %d bytes, actual transmitted %d bytes\n",LINErlen,rc);
            }  // End  if (LINErlen != rc
         }  // End if (pendingrcv > 0)
      }  // End if ((line1state == RDY) && (line2state == RDY))
   }  // End while (1)
   return;
}

/*-------------------------------------------------------------------*/
/* Check if there is read activiy on the socket                      */
/* This is used by the caller to detect a connection break           */
/*-------------------------------------------------------------------*/
int  SocketReadAct(int fd) {
   int rc;
   fd_set fdset;
   struct timeval timeout;
   timeout.tv_sec = 0;
   timeout.tv_usec = 0;
   FD_ZERO(&fdset);
   FD_SET(fd, &fdset);
   return select(fd + 1, &fdset, NULL, NULL,  &timeout);
}
