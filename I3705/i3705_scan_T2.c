/* Copyright (c) 2020, Henk Stegeman and Edwin Freekenhorst

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

   3705_scan_T2.c: IBM 3705 Communication Scanner Type 2 simulator

   Some notes:
   1) The scanner has:
      - a ICW work register and is implemented in Ereg_Inp[0x44->0x47]
        Function PutICW will transfer ICW work reg to ICW storage[ABAR].
      - a ICW local storage register (See below)
        Function GetICW will transfer ICW storage[ABAR] to ICW input reg.
      - a ICW input register and is implemented in Ereg_Out[0x44/45/47]
   2) The 2 bits in SDF for Business Clock Osc selection bits are
      not implemented.  Reason: for programming simplicity.
   3) This code is pre-positined for multiple lines, but correctly only
      suitable for only ONE line.

   *** Input to CS2 (CCU output) Eregs ***
   Label      Ereg         Function
   --------------------------------------------------------------
   CMBAROUT   0x40         // ABAR Interface address
   CMADRSUB   0x41         // Scanner addr substitution.
   CMSCANLT   0x42         // Upper scan limit modification.
   CMCTL      0x43         // CA Address and ESC status.
   CMICWB0F   0x44         // ICW  0 THRU 15
   CMICWLP    0x45         // ICW 16 THRU 23
   CMICWS     0x46         // ICW 24 THRU 33
   CMICWB34   0x47         // ICW 34 THRU 43

   *** Output from CS2 (CCU input) Eregs ***
   Label      Ereg         Function
   --------------------------------------------------------------
   CMBARIN    0x40         // ABAR Interface address
              0x41         // Unused
              0x42         // Unused
   CMERREG    0x43         // Scan Error register
   CMICWB0F   0x44         // ICW  0 THRU 15
   CMICWLPS   0x45         // ICW 16 THRU 31
   CMICWDSP   0x46         // Display register
   CMICWB32   0x47         // ICW 32 THRU 45

                        PCF state
   +------------> +---->  [0] NO-OP
   |              |
   |              |
   |              L2 <--  [1] Set Mode - DTR on
   |              ^
   |              |
   |         +----<-----  [2] Monitor DSR
   |         |    |
   |         |    |
   |         L2   +-----  [3] Monitor DSR or RI on
   |         |
   |         |
   |         +--------->  [4] Monitor flag - Block DSR error
   |         +-----flag--/
   |         |
   |    +----|--------->  [5] Monitor flag - Allow DSR error
   |    |    +-----flag--/
   |    |    v
   |    |    L2 ------->  [6] Receive Info - Block Data Interrupts
   |    |    ^    +------/
   |    |    |    L2
   |    |    |    +---->  [7] Receive Info - Allow Data Interrupts
   L2   |    +-----flag--/
   |    |
   |    L2   +-----CTS--  [8] Transmit Initial - RTS on
   |    |    |
   |    |    |
   |    |    +--------->  [9] Transmit Normal
   |    |
   |    |
   |    +-SDF is empty--  [C] Tx -> Rx turnaround RTS off
   |    |
   |    |
   |    +-SDF is empty--  [D] Tx -> Rx turnaround RTS on.
   |
   |
   +-- no DSR | no DCD--  [F] Disable

   L2 = Level 2 interrupt
   See IBM 3705 hardware documentation for more details.
*/

#include "sim_defs.h"
#include "i3705_defs.h"
#include "i3705_sdlc.h"
#include "i3705_scanner.h"
#include "i3705_Eregs.h"               /* External regs defs */
#include <signal.h>
#include <ctype.h>
#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>

#define BUFFER_SIZE   16384            /* Line Send/Receive buffer  */
                                       /* Make sure this matches the Buffer of the attached device */
extern int32 debug_reg;
extern int32 Eregs_Inp[];
extern int32 Eregs_Out[];
extern int8  svc_req_L2;               /* SVC L2 request flag */
extern FILE *trace;
extern int32 lvl;
extern int32 cc;

extern int Ireg_bit(int reg, int bit_mask);
extern void wait();

int8 Eflg_rcvd[MAX_LINES];              /* Eflag received                            */
int8 FCS_rcvd[MAX_LINES][2];            /* Frame Check Sequence bytes Received       */

int abar;                              /* Attach Buffer Addr Reg (020-1FF) to CS2   */
int abar_int;                          /* ABAR of line interrupt (020-1FF) from CS2 */

/* ICW Local Store Registers */
uint8_t icw_scf[MAX_LINES];            /* ICW[ 0- 7] SCF - Secondary Control Field  */
uint8_t icw_pdf[MAX_LINES];            /* ICW[ 8-15] PDF - Parallel Data Field      */
uint8_t icw_lcd[MAX_LINES];            /* ICW[16-19] LCD - Line Code Definer        */
uint8_t icw_pcf[MAX_LINES];            /* ICW[20-23] PCF - Primary Control Field    */
uint8_t icw_sdf[MAX_LINES];            /* ICW[24-31] SDF - Serial Data Field        */
                                       /* ICW[32-33] Not implemented (OSC sel bits) */
uint16_t icw_Rflags[MAX_LINES];        /* ICW[34-47] flags                          */

/* Additional icw fields for emulator  */
uint8_t icw_pcf_prev[MAX_LINES];       /* Previous icw_pcf                          */
uint8_t icw_lne_stat[MAX_LINES];       /* Line state: RESET, TX or RX               */
uint8_t icw_pcf_nxt[MAX_LINES];        /* What will be the next pcf value           */
uint8_t icw_pdf_reg[MAX_LINES];        /* Status ICW PDF reg: FILLED or EMPTY       */
extern uint8_t RS232[MAX_LINES];       /* RS232 signals                             */

int8 CS2_req_L2_int = OFF;
pthread_mutex_t icw_lock;              // ICW lock (0 - 45)
extern pthread_mutex_t rs232_lock;            // RS232 signal lock

// Trace variables
uint16_t Sdbg_reg = 0x00;              // Bit flags for debug/trace
uint16_t Sdbg_flag = OFF;              // 1 when Strace.log open
FILE  *S_trace;                        // Scanner trace file fd

// This table contains the SMD area addresses of each scanner line.
int8 line_smd_addr[48];

int line;                              // ICW table index pointer

void Put_ICW(int i);
void Get_ICW(int i);
void Init_ICW(int max);
void proc_LIBdisbuf(int line);
void proc_LIBtdata(unsigned char transmitChar, uint8_t state, int line);
int  proc_LIBrdata(unsigned char *receivedChar, uint8_t state, int line);


void *CS2_thread(void *arg) {
   int Bptr = 0;                       // Tx/Rx buffer index pointer
   int i, c;
   register char *s;
   unsigned char receivedChar[MAX_LINES], transmitChar;
   int ret;

   fprintf (stderr, "\rCS-T2: Thread %ld started succesfully...\n", syscall(SYS_gettid));
   // core_id = 1 (CPU), 2 (SCAN), 3 (SDLC)

   /****************
   int core_id = 2;
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);

   if ((core_id > 0) && (core_id <= num_cores)) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(core_id, &cpuset);
      pthread_t current_thread = pthread_self();
      pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
      fprintf (stderr, "\rCS-T2: Thread assigned to core #%1d.\n", core_id);
   }
   ***************/

   Init_ICW(MAX_LINES);                // Initialize scanner & buffers
   fprintf (stderr, "\rCS-T2: Scanner initialized with %d lines...\n", MAX_LINES);

   // ********************************************************************
   //  Scanner debug trace facility
   // ********************************************************************
   if (Sdbg_flag == OFF) {
      S_trace = fopen("trace_S.log", "w");
      fprintf(S_trace, "\n\r     ****** 3705 SCANNER log file ****** "
                       "\n\r     sim> d debugS 01 - spare \n"
                       "\n\r                   02 - trace scanner activities [scan_T2.c]"
                       "\n\r                   04 - trace line I/O buffers [lib.c] "
                       "\n\r"
                       "\n\r     All trace lines are prefixed with: #xxLny \n"
                       "\n\r        xx = 02 scanner activity"
                       "\n\r             04 Line I/O buffer content"
                       "\n\r        Ln = Line number n = 0 -> 9 "
                       "        y  = '>' Tx/request; '<' Rx/response \n"
                       );
      Sdbg_flag = ON;
   }
   Sdbg_reg = 0x00;

   // ********************************************************************
   // Scanner loop starts here...
   // ********************************************************************
   while(1) {
      for (line = 0; line < MAX_LINES; line++) {     // Scan all lines

         pthread_mutex_lock(&icw_lock);
         if (icw_pcf[line] != icw_pcf_nxt[line]) {   // pcf changed by NCP ?
            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
               fprintf(S_trace, "\n\n\r#02L%1d> CS2[%1X]: NCP changed PCF to %1X ",
                                 line, icw_pcf[line], icw_pcf_nxt[line]);
            if (icw_pcf_nxt[line] == 0x0)            // NCP changed PCF to 0 ?
               icw_lne_stat[line] = RESET;           // Line state = RESET
            icw_pcf_prev[line] = icw_pcf[line];      // Save current pcf and
            icw_pcf[line] = icw_pcf_nxt[line];       // Set new current pcf
         }

         switch (icw_pcf[line]) {
            case 0x0:                                // NO-OP
               if (icw_pcf_prev[line] != icw_pcf[line]) {
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: PCF = 0 entered, scf = %02X, sdf = %02X. Next PCF will be set by NCP ",
                                       line, icw_pcf[line], icw_scf[line], icw_sdf[line]);
               }
               break;

            case 0x1:                                // Set mode
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: PCF = 1 entered, scf = %02X, sdf = %02X. Next PCF will be 0 ",
                                       line, icw_pcf[line], icw_scf[line], icw_sdf[line]);
               }
                 icw_scf[line] &= 0x4A;           // Reset all check cond. bits.
                  // if service request not set -> NCP will go to PCF F, else PCF 2 will be set
                  //if (RS232[line] & DSR) {           // If DSR high .....
                //     icw_scf[line] |= 0x40;          // Set service request flag
                  icw_sdf[line] |= 0x08;             // Set DTR in ICW
                  pthread_mutex_lock(&rs232_lock);
                  RS232[line] |= DTR;                // Set DTR on line
                  pthread_mutex_unlock(&rs232_lock);
                  //}
                  icw_scf[line] |= 0x40;             // Set service request flag
                  icw_pcf_nxt[line] = 0x0;           // Goto PCF = 0...
                  CS2_req_L2_int = ON;               // ...and issue a L2 int
               break;

            case 0x2:                                // Mon DSR on
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: PCF = 2 entered. scf = %02X, sdf = %02X,  RS232=%02X. Next PCF will be set by NCP ",
                                       line, icw_pcf[line],icw_scf[line], icw_sdf[line], RS232[line]);
               }
               if (RS232[line] & DCD) {              // If DCD on .....
                  icw_scf[line] |= 0x08;             // ...flag DCD  on
                  if (RS232[line] & DSR) {           // If DSR high .....
                     icw_scf[line] |= 0x40;          // Set service request flag
                     icw_pcf_nxt[line] = 0x4;        // Goto PCF = 4
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  } else {
                     icw_scf[line] |= 0x40;          // Set service request flag
                     icw_pcf_nxt[line] = 0x2;        // Stay in PCF = 2...
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  }  // End  if (RS232[line] & DSR)
               } else {
                  icw_pcf_nxt[line] = 0x2;           // Stay in PCF = 2
                  icw_scf[line] &= 0xF7;             // ...flag DCD  off
               }  // End  if (RS232[line] & DCD)
               break;

            case 0x3:                                // Mon RI or DSR on
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: PCF = 3 entered, next PCF will be 0 ",
                                       line, icw_pcf[line]);
                  icw_scf[line] |= 0x40;             // Set norm char serv flag
                  icw_pcf_nxt[line] = 0x0;           // Goto PCF = 0...
                  CS2_req_L2_int = ON;               // ...and issue a L2 int
               }
               break;

            case 0x4:                                // Monitor - block DSR error
            case 0x5:                                // Monitor - allow DSR error
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                     fprintf(S_trace, "\n#02L%1d> CS2[%1X]: PCF = %d entered, RS232=%02X. next PCF will be 6 (SDLC) or 7 (BSC)",
                                       line, icw_pcf[line], icw_pcf[line], RS232[line]);
                  }
               }
               if (!(RS232[line] & DSR)) {           // If DSR low .....
                  icw_scf[line] |= 0x40;             // Set norm char serv flag
                  icw_pcf_nxt[line] = 0x2;           // Go to PCF = 2...
                  CS2_req_L2_int = ON;               // ...and issue a L2 int
                  break;
               }

               //if (icw_lne_stat[line] == RESET)      // Line is silent. Wait for NCP time out.
               //   break;
               if (icw_lne_stat[line] == TX)         // Line is silent. Wait for NCP action.
                  break;

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  //*******************************************************
                  ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                  //*******************************************************
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))
                     fprintf (trace, "\n#02L%1d> CS2[%1X]: Read ret=%d, ch=%02X", line, icw_pcf[line], ret, receivedChar[line]);
                  if (ret != 1) break;
                  if (receivedChar[line] == 0x32)  {       // Found a SYN flag
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))
                        fprintf(S_trace, "\n#02L%1d> CS2[%1X]: Received SYN! - goto state 7", line, icw_pcf[line]);
                     icw_pdf[line] = receivedChar[line];
                     icw_pcf_nxt[line]  = 0x7;       // Goto PCF = 7
                     icw_sdf[line] |= 0x04;          // Set SYNC flag
                  }  // End if receivedChar[line]
               }  // end BSC

               if ((icw_lcd[line] == 0x8) || (icw_lcd[line] == 0x9)) {  // SDLC ?
                  Eflg_rcvd[line] = OFF;                      // Reset Eflag
                  //*******************************************************
                  ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                  //*******************************************************
                  if (ret == 0) break;                    // Break if nothing received.
                  if (receivedChar[line] == 0x7E) {
                     //icw_sdf[line] &= 0x00;             // Clear SDF
                     icw_scf[line] |= 0x04;             // Set 7E detected flag
                     icw_lcd[line]  = 0x9;              // LCD = 9 (SDLC 8-bit)
                     icw_pcf_nxt[line] = 0x6;           // Goto PCF = 6...
                     CS2_req_L2_int = ON;               // ...and issue a L2 int
                 } else {
                     proc_LIBdisbuf(line);              // Discard line buffer content
                     icw_pcf_nxt[line] = 0x5;           // Stay in PCF = 5...
                 } // End  if (receivedChar[line] == 0x7E)
               }  // End if (icw_lcd[line] == 0x8...
               break;

            case 0x6:                                // Receive info-inhibit data interrupt
               if ((svc_req_L2 == ON) || (lvl == 2)) {  // Is L2 interrupt active ?
                  break;                             // Loop till inactive...
               }
                     //*******************************************************
                     ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                     //*******************************************************
                  if (ret == 0) break;                    // Break if nothing received.
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {     // Trace scanner activities ?
                        fprintf(S_trace, "\n#02L%1d< CS2[%1X]: proc_LBrdata rc=%d ",
                                          line, icw_pcf[line],ret);
                     }
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                     fprintf(S_trace, "\n#02L%1d< CS2[%1X]: PCF = 6 (re-)entered ",
                                       line, icw_pcf[line]);
                     fprintf(S_trace, "\n#02L%1d< CS2[%1X]: Received byte = *** %02X ***, Eflag=%d",
                                       line, icw_pcf[line], receivedChar[line], Eflg_rcvd[line]);
                  }

                  if (receivedChar[line] == 0x7E) {        //  flag?
                     icw_pcf_nxt[line] = 0x6;        // Stay in PCF = 6...
                     icw_scf[line] |= 0x04;          // Set flag det bit
                  } else {
                     icw_scf[line] &= ~0x04;         // Reset flag det bit
                     icw_pdf[line] = receivedChar[line];
                     icw_pdf_reg[line] = FILLED;     // Signal NCP to read pdf.
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x7;        // Stay in PCF = 7...
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               break;

            case 0x7:                                // Receive info-allow data interrupt
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;                             // Loop till inactive...

               if (icw_lcd[line] == 0xC) {           // BSC ?
                  if ((icw_scf[line] & 0x40) == 0) { // NCP has read pdf ?
                     //*******************************************************
                     ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                     //*******************************************************
                     if (ret != 1) receivedChar[line] = 0xFF;
                     icw_pdf[line] = receivedChar[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                        fprintf(S_trace, "\n#02L%1d< CS2[%1X]: State 7 ch = %02X\n", line, icw_pdf[line], receivedChar[line]);
                     }
                     //icw_pdf_reg[line] = FILLED;   // Signal NCP to read pdf.
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x7;        // Stay in PCF = 7...
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }  // End if icw_scf[line]
               }  // End BSC

               if ((icw_lcd[line] == 0x8) || (icw_lcd[line] == 0x9)) {  // SDLC ?
                  //pthread_mutex_lock(&icw_lock);
                  //*******************************************************
                  ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                  //*******************************************************
                  if (ret == 0) {                       // No characters received?
                     //pthread_mutex_unlock(&icw_lock);
                     break;
                  }
                  // Check end of frame when a flag byte is detected. If the FCS matches, this is really the end of the frame.,
                  // else we bumped into a regular x7E character,
                  if ((receivedChar[line] == 0x7E) && (FCS_rcvd[line][0] == 0x47) && (FCS_rcvd[line][1] == 0x0F)) {
                     Eflg_rcvd[line] = ON;
                  } else {
                     FCS_rcvd[line][0] = FCS_rcvd[line][1];
                     FCS_rcvd[line][1] = receivedChar[line];
                     Eflg_rcvd[line] = OFF;                 // No Eflag
                  }

                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {     // Trace scanner activities ?
                     fprintf(S_trace, "\n#02L%1d< CS2[%1X]: PCF = 7 (re-)entered ",
                                       line, icw_pcf[line]);
                     fprintf(S_trace, "\n#02L%1d< CS2[%1X]: Received byte = *** %02X ***, Eflag=%d",
                                       line, icw_pcf[line], receivedChar[line], Eflg_rcvd[line]);
                  } // End if ((receivedChar[line] == 0x7E)
                  if (Eflg_rcvd[line] == ON) {
                     icw_lne_stat[line] = TX;           // Line turnaround to transmitting...
                     icw_scf[line] |= 0x44;             // Set 7E detected flag
                     icw_lcd[line]  = 0x9;              // LCD = 9 (SDLC 8-bit)
                     icw_pcf_nxt[line] = 0x6;           // Goto PCF = 6...
                     CS2_req_L2_int = ON;               // ...and issue a L2 int
                     Eflg_rcvd[line] = OFF;             // Reset Eflag
                     FCS_rcvd[line][0] = 0x00;          // Clear FCS buffer
                     FCS_rcvd[line][1] = 0x00;          // Clear FCS buffer
                  } else {
                     icw_pdf[line] = receivedChar[line];
                     icw_pdf_reg[line] = FILLED;        // Signal NCP to read pdf.
                     icw_scf[line] |= 0x40;             // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x7;           // Stay in PCF = 7...
                     CS2_req_L2_int = ON;               // Issue a L2 interrupt
                  } // End if (Eflg_rcvd[line] == ON)
                  //pthread_mutex_unlock(&icw_lock);
               }  // End if (icw_lcd[line] == 0x8...

               break;

            case 0x8:                                // Transmit initial-turn RTS on
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;

               if (!(RS232[line] & CTS)) {           // if not Clear To Send, skip and try again later
                  RS232[line] |= RTS;                // Raise Request To Send
                  break;
               }

               if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))   // Trace scanner activities ?
                  fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: PCF = 8 entered, next PCF will be 9 ",
                                    line, icw_pcf[line]);

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: icw_pdf=%02X icw_scf=%02X icw_scf&0x40=%02X\n",
                                       line, icw_pcf[line], 0xff & icw_pdf[line], 0xff & icw_scf[line], icw_scf[line] & 0x40);
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: 1. condition=%01X icw_pdf=%02X icw_scf=%02X\n",
                                       line, icw_pcf[line], (icw_scf[line] & 0x40) == 0, 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                  }
                  if ((icw_scf[line] & 0x40) == 0) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                        fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: 2. condition=%01X icw_pdf=%02X icw_scf=%02X\n",
                                          line, icw_pcf[line], (icw_scf[line] & 0x40) == 0, 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                        fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: State 8 ch=%02X \n",line, icw_pcf[line], transmitChar);
                     }
                  }
                  //*******************************************************
                  proc_LIBtdata(transmitChar,icw_pcf[line], line);
                  //*******************************************************
                  // Next byte please...
                  icw_pdf_reg[line] = EMPTY;         // Ask NCP for next byte
                  icw_scf[line] |= 0x40;             // Set norm char serv flag
                  icw_pcf_nxt[line] = 0x9;           // Go to PCF = 9...
                  CS2_req_L2_int = ON;               // Issue a L2 interrupt
               }  // End BSC

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  // Call LIBtdata to indicate start of a new frame. transmitChar is ignored.
                  //*******************************************************
                  proc_LIBtdata(transmitChar, icw_pcf[line], line);
                  //*******************************************************
                  icw_scf[line] &= 0xFB;             // Reset flag detected flag
                  // CTS is now on.
                  icw_pcf_nxt[line] = 0x9;           // Goto PCF = 9
                  // NO CS2_req_L2_int !
               }  // End SDLC
               break;

            case 0x9:                                // Transmit normal
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: icw_pdf=%02X icw_scf=%02X lvl=%d\n",
                                       line, icw_pcf[line], 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                  if ((icw_scf[line] & 0x40) == 0) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {   // Trace scanner activities ?
                        fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: State 9 ch=%02X \n", line, icw_pcf[line], transmitChar);
                     }
                     //*******************************************************
                     proc_LIBtdata(transmitChar,icw_pcf[line], line);
                     //*******************************************************

                     // Next byte please...
                     icw_pdf_reg[line] = EMPTY;      // Ask NCP for next byte
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x9;        // Stay in PCF = 9...
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               }

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  if (icw_pdf_reg[line] == FILLED) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                        fprintf(S_trace, "\n#02L%1d> CS2[%1X]: PCF = 9 (re-)entered ",
                                          line, icw_pcf[line]);
                        fprintf(S_trace, "\n#02L%1d> CS2[%1X]: Transmitting PDF = *** %02X ***",
                                          line, icw_pcf[line], icw_pdf[line]);
                     }
                     //*******************************************************
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);
                     //*******************************************************

                     // Next char please...
                     icw_pdf_reg[line] = EMPTY;      // Ask NCP for next byte
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x9;        // Stay in PCF = 9...
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               }  // End SDLC
               break;

            case 0xA:                                // Transmit normal with new sync
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: icw_pdf=%02X icw_scf=%02X lvl=%d\n",
                                       line, icw_pcf[line], 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                  if ((icw_scf[line] & 0x40) == 0) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {   // Trace scanner activities ?
                        fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: State 9 ch=%02X \n",
                                          line, icw_pcf[line], transmitChar);
                     }
                     //*******************************************************
                     proc_LIBtdata(transmitChar,icw_pcf[line], line);
                     //*******************************************************

                     // Next byte please...
                     icw_pdf_reg[line] = EMPTY;      // Ask NCP for next byte
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0xA;        // Stay in PCF = A...
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               }  // End BSC
               break;

            case 0xB:                                // Not used
               break;

            case 0xC:                                // Transmit turnaround-turn RTS off
               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))
                        fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: Now into state C.\n", line, icw_pcf[line]);
                     transmitChar = icw_pdf[line];   // Will not be used.
                     //*******************************************************
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);  //Signal line we are done
                     //*******************************************************

                     icw_lne_stat[line] = RX;        // Line turnaround to receiving...
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x5;        // Goto PCF = 5...
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  }
               }  //end BSC

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?

                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                        fprintf(S_trace, "\n#02L%1d> CS2[%1X]: PCF = C entered, next PCF will be set by NCP ",
                                          line, icw_pcf[line]);

                     // ******************************************************************
                     // Signal SDLC that final character has been received.
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);
                     // ******************************************************************
                     icw_lne_stat[line] = RX;        // Line turnaround to receiving...
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x5;        // Goto PCF = 5...
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  }
               }  // End SDLC
               RS232[line] &= ~RTS;                 // Drop Request To Send and
               RS232[line] &= ~CTS;                 // ...drop Clear To Send and
               break;

            case 0xD:                                // Transmit turnaround-keep RTS on
               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))    // Trace scanner activities ?
                        fprintf(S_trace, "\n#02L%1d> CS2[%1X]: PCF = D entered, next PCF will be set by NCP \n\r", line, icw_pcf[line]);
                  }
                     icw_pcf_nxt[line] = 0x5;        // Goto PCF = 5...
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                                                     // ==> ??? NO CS2_req_L2_int !
               }  // End BSC

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                        fprintf(S_trace, "\n#02L%1d> CS2[%1X]: PCF = D entered, next PCF will be set by NCP ",
                                          line, icw_pcf[line]);
                  }
                  // NO CS2_req_L2_int !
               }  // End SDLC

               break;

            case 0xE:                                // Not used
               break;

            case 0xF:                                // Disable line
               if (icw_pcf_prev[line] != icw_pcf[line]) {
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
                     fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: PCF = F entered, next PCF will be set by NCP ",
                                       line, icw_pcf[line]);
               }
               icw_scf[line] |= 0x40;                // Set norm char serv flag   <======!!!!!
               icw_sdf[line] &= ~0x08;               // reset DTR
               RS232[line] &= ~DTR;                  // reset DTR on line
               icw_pcf_nxt[line] = 0x0;              // Goto PCF = 0...
               CS2_req_L2_int = ON;                  // ...and issue a L2 int
               break;

         }  // End of switch (icw_pcf[line])
         pthread_mutex_unlock(&icw_lock);


         // ========  POST-PROCESSING SCAN A LINE CYCLE  ========

         if (CS2_req_L2_int) {                       // CS2 L2 interrupt requested ?
            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
               fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: SVCL2 interrupt issued for PCF = %1X ",
                                 line, icw_pcf[line], icw_pcf[line]);

            while (svc_req_L2 == ON) {               // Wait till CCU has finished L2 processing
               usleep(1000);                         // some time to finish L2.
            }

            abar_int = line + 0x020;                 // Set ABAR with line # that caused the L2 int.

            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
               fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: abar_int = %04X ",
                                 line, icw_pcf[line], abar_int );

            svc_req_L2 = ON;                         // Issue a level 2 interrrupt
            CS2_req_L2_int = OFF;                    // Reset int req flag
         }


         pthread_mutex_lock(&icw_lock);
         icw_pcf_prev[line] = icw_pcf[line];         // Save current pcf
         if (icw_pcf[line] != icw_pcf_nxt[line]) {   // pcf state changed ?
            icw_pcf[line]   = icw_pcf_nxt[line];     // Set new current pcf
         }
         pthread_mutex_unlock(&icw_lock);

         if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02))  // Trace scanner activities ?
               fprintf(S_trace, "\n\r#02L%1d> CS2[%1X]: Next PCF = %1X ",
                                 line, icw_pcf_prev[line], icw_pcf[line] );
         }
      }  // End for line = 0 ---> MAX_LINES           // End of scanning one line, next please...
      usleep(500);

   }  // End of while(1)...
   return (0);
}


// *******************************************************************
// Function to copy ICW[line] to input regs used in CCU coding.
// *******************************************************************
void Get_ICW(int line) {                  // See 3705 CE manauls for details.

   Eregs_Inp[0x44] = (icw_scf[line] << 8)  | icw_pdf[line];
   Eregs_Inp[0x45] = (icw_lcd[line] << 12) | (icw_pcf[line] << 8) | icw_sdf[line];
   Eregs_Inp[0x46] =  ((RS232[line] & 0xF0) | 0x02) << 8;   // Display reg
   Eregs_Inp[0x47] =  icw_Rflags[line];   // ICW 32 - 47

   return;
}

// *******************************************************************
// Function to initialize the ICW and buffer of the scanner
// *******************************************************************
void Init_ICW(int max) {

   for (int j = 0; j < max; j++) {        // Initialize all lines
   // ICW Local Store Registers
      icw_scf[j] = 0;                     // ICW[ 0- 7] SCF - Secondary Control Field
      icw_pdf[j] = 0;                     // ICW[ 8-15] PDF - Parallel Data Field
      icw_lcd[j] = 0;                     // ICW[16-19] LCD - Line Code Definer
      icw_pcf[j] = 0xE;                   // ICW[20-23] PCF - Primary Control Field
      icw_sdf[j] = 0;                     // ICW[24-31] SDF - Serial Data Field
                                          // ICW[32-33] Not implemented (OSC sel bits)
      icw_Rflags[j] = 0;                  // ICW[34-47] flags
   // Additional icw fields for emulator
      icw_pcf_prev[j] = 0x0;              // Previous icw_pcf
      icw_lne_stat[j] = RESET;            // Line state: RESET, TX, RX
      icw_pcf_nxt[j] = 0x0;               // What will be the next pcf value
      icw_pdf_reg[j] = EMPTY;             // Status ICW PDF reg: NCP FILLED pdf for Tx
                                          //                     NCP EMPTY pdf during Rx
   }
   return;
}

