/* Copyright (c) 2024, Edwin Freekenhorst

   DLSw defintions are taken from Matt Burke
   Matt Burke's DLSw testserver has been used as the starting point.
   See Matt's website: www.track9.net/hercules/DLSw

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

   DLSw__rt.C   (c) Copyright  Edwin Freekenhost

   This module emulates a Data Llink Switch router.
   It connects one end to an SDLC link, the other connects to another DSLw device
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

#define DLSW_PORT  2065
#define SDLCBASE   37500
#define OFF        0
#define ON         1
#define NO         0
#define YES        1

/*-------------------------------------------------------------------*/
/* DLSw definitions                                                  */
/*-------------------------------------------------------------------*/

/* Message types */

#define CANUREACH       0x03                            /* Can U Reach Station */
#define ICANREACH       0x04                            /* I Can Reach Station */
#define REACH_ACK       0x05                            /* Reach Acknowledgment */
#define DGRMFRAME       0x06                            /* Datagram Frame */
#define XIDFRAME        0x07                            /* XID Frame */
#define CONTACT         0x08                            /* Contact Remote Station */
#define CONTACTED       0x09                            /* Remote Station Contacted */
#define RESTART_DL      0x10                            /* Restart Data Link */
#define DL_RESTARTED    0x11                            /* Data Link Restarted */
#define ENTER_BUSY      0x0C                            /* Enter Busy */
#define EXIT_BUSY       0x0D                            /* Exit Busy */
#define INFOFRAME       0x0A                            /* Information (I) Frame */
#define HALT_DL         0x0E                            /* Halt Data Link */
#define DL_HALTED       0x0F                            /* Data Link Halted */
#define NETBIOS_NQ      0x12                            /* NETBIOS Name Query */
#define NETBIOS_NR      0x13                            /* NETBIOS Name Recog */
#define DATAFRAME       0x14                            /* Data Frame */
#define HALT_DL_NOACK   0x19                            /* Halt Data Link with no Ack */
#define NETBIOS_ANQ     0x1A                            /* NETBIOS Add Name Query */
#define NETBIOS_ANR     0x1B                            /* NETBIOS Add Name Response */
#define KEEPALIVE       0x1D                            /* Transport Keepalive Message */
#define CAP_EXCHANGE    0x20                            /* Capabilities Exchange */
#define IFCM            0x21                            /* Independent Flow Control Message */
#define TEST_CIRC_REQ   0x7A                            /* Test Circuit Request */
#define TEST_CIRC_RSP   0x7B                            /* Test Circuit Response */

/* SSP flags */

#define SSPex           0x80                            /* explorer message */

/* Frame direction */

#define DIR_TGT         0x01                            /* origin to target */
#define DIR_ORG         0x02                            /* target to origin */

/* Header constants */

#define DLSW_VER        0x31                            /* DLSw version 1 */
#define LEN_CTRL        72                              /* control header length */
#define LEN_INFO        16                              /* info header length */

#define DLSW_PORT       2065

/* Common header fields */

#define HDR_VER         0x00                            /* Version Number */
#define HDR_HLEN        0x01                            /* Header Length */
#define HDR_MLEN        0x02                            /* Message Length */
#define HDR_RDLC        0x04                            /* Remote Data Link Correlator */
#define HDR_RDPID       0x08                            /* Remote DLC Port ID */
#define HDR_MTYP        0x0E                            /* Message Type */
#define HDR_FCB         0x0F                            /* Flow Control Byte */

/* Control header fields */

#define HDR_PID         0x10                            /* Protocol ID */
#define HDR_NUM         0x11                            /* Header Number */
#define HDR_LFS         0x14                            /* Largest Frame Size */
#define HDR_SFLG        0x15                            /* SSP Flags */
#define HDR_CP          0x16                            /* Circuit Priority */
#define HDR_TMAC        0x18                            /* Target MAC Address */
#define HDR_OMAC        0x1E                            /* Origin MAC Address */
#define HDR_OSAP        0x24                            /* Origin Link SAP */
#define HDR_TSAP        0x25                            /* Target Link SAP */
#define HDR_DIR         0x26                            /* Frame Direction */
#define HDR_DLEN        0x2A                            /* DLC Header Length */
#define HDR_ODPID       0x2C                            /* Origin DLC Port ID */
#define HDR_ODLC        0x30                            /* Origin Data Link Correlator */
#define HDR_OTID        0x34                            /* Origin Transport ID */
#define HDR_TDPID       0x38                            /* Target DLC Port ID */
#define HDR_TDLC        0x3C                            /* Target Data Link Correlator */
#define HDR_TTID        0x40                            /* Target Transport ID */

/* Flow control fields */

#define FCB_FCI         0x80                            /* Flow control indicator */
#define FCB_FCA         0x40                            /* Flow control acknowledge */
#define FCB_FCO         0x07                            /* Flow control operator */

#define FCO_RPT         0x00                            /* Repeat window operator */
#define FCO_INC         0x01                            /* Increment window operator */
#define FCO_DEC         0x02                            /* Decrement window operator */
#define FCO_RST         0x03                            /* Reset window operator */
#define FCO_HLV         0x04                            /* Halve window operator */

/* Capabilities Exchange Subfields */

#define CAP_VID         0x81                            /* Vendor ID */
#define CAP_VER         0x82                            /* DLSw Version */
#define CAP_IPW         0x83                            /* Initial Pacing Window */
#define CAP_VERS        0x84                            /* Version String */
#define CAP_MACX        0x85                            /* MAC Address Exclusivity */
#define CAP_SSL         0x86                            /* Supported SAP List */
#define CAP_TCP         0x87                            /* TCP Connections */
#define CAP_NBX         0x88                            /* NetBIOS Name Exclusivity */
#define CAP_MACL        0x89                            /* MAC Address List */
#define CAP_NBL         0x8A                            /* NetBIOS Name List */
#define CAP_VC          0x8B                            /* Vendor Context */

/* Capabilities Exchange Subfield offsets */

#define CAP_VID_OFF     0x05                            /* Offfset to Vendor ID */
#define CAP_VER_OFF     0x10                            /* Offset to DLSw Version */
#define CAP_IPW_OFF     0x0D                            /* Offset to Initial Pacing Window */
#define CAP_VERS_OFF    0x18                            /* Offset to Version String */

uint8_t CONTROL_MSG_Hdr[] = {
      DLSW_VER, 0x48, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00,      /* 0x00 - 0x07 */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x08 - 0x0F */
      0x42, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x10 - 0x17 */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x18 - 0x1F */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,          /* 0x20 - 0x27 */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x28 - 0x2F */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x30 - 0x37 */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x38 - 0x3F */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};         /* 0x40 - 0x47 */
uint8_t CAP_EXCHANGE_Msg[] = {
      0x00, 0x26, 0x15, 0x20, 0x05, CAP_VID, 0x00, 0x00,       /* 0x48 - 0x5F */
      0x00, 0x04, CAP_VER, 0x02, 0x00, 0x04, CAP_IPW,  0x00,   /* 0x50 - 0x57 */
      0x14, 0x12, CAP_SSL, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,       /* 0x58 - 0x5F */
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,          /* 0x60 - 0x67 */
      0xFF, 0xFF, 0xFF, 0x03, CAP_TCP, 0x02 };                 /* 0x68 - 0x6D */
uint8_t CAP_EXCHANGE_Rsp[] = {
      0x00, 0x4, 0x15, 0x21 };                                 /* 0x48 - 0x4B */
uint8_t XIDFRAME_Rsp[] = {
      0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x48 - 0x4F */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          /* 0x50 - 0x58 */
      0x00, 0x00, 0x00, 0x00 };                                /* 0x58 - 0x5B */
uint8_t INFOFRAME_Hdr[] = {
      DLSW_VER, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      /* 0x00 - 0x07 */
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };        /* 0x08 - 0x0F */

/*-------------------------------------------------------------------*/
/* End of DLSw definitions                                           */
/*-------------------------------------------------------------------*/

uint16_t Tdbg_flag = OFF;          /* 1 when Ttrace.log open */
FILE *T_trace;

uint8_t bfr[256];

uint8_t TMAC_addr[] = { 0x40, 0x00, 0x09, 0x99, 0x10, 0xC1 };  // PU
uint8_t OMAC_addr[] = { 0x40, 0x00, 0x10, 0x20, 0x10, 0x00 };  // NCP

/* XID information */
uint16_t IDBLK;
uint16_t IDNUM;
uint8_t  PUtype;

uint8_t SDLC_UA[]  = { 0x7E, 0xC1, 0x73, 0x47, 0x0F, 0x7E };
uint8_t SDLC_RR[]  = { 0x7E, 0xC1, 0x11, 0x47, 0x0F, 0x7E };
uint8_t SDLC_RNR[] = { 0x7E, 0xC1, 0x05, 0x47, 0x0F, 0x7E };
uint8_t SDLC_FCSLT[] = { 0x47, 0x0F, 0x7E };

/* SDLC frame defintion */
#define BFlag          0
#define FAddr          1
#define FCntl          2
#define CPoll          0x10
#define CFinal         0x10

/* Used for Unnumbered cmds/resp */
#define UNNUM          0x03
#define SNRM           0x83
#define DISC           0x43
#define XID            0xAF
#define UA             0x63
#define DM             0x0F
#define FRMR           0x87
#define TEST           0xE3

/* Used for Supervisory cmds/resp */
#define SUPRV          0x01
#define RR             0x01
#define RNR            0x05
#define REJ            0x09

/* Used for Information frame cmds/resp */
#define IFRAME         0x00

/* RS232 signals. The 4 high order bit positions are alinged with the scanner Display Register   */
#define CTS 0x80       /* Clear To Send                 */
#define RI  0x40       /* Ring Indicator                */
#define DSR 0x20       /* Data Set Ready                */
#define DCD 0x10       /* Data Carrier Detect           */
#define RTS 0x08       /* Request To Send               */
#define DTR 0x04       /* Data Terminal Ready           */

/* Variablers used */
uint8_t        dlc[4];              /* data link correlator                  */
uint8_t        dlc_pid[4];          /* DLC port id                           */
uint8_t        SDLC_rbuf[65536];    /* SDLC Read Buffer                      */
uint8_t        SDLC_wbuf[65536];    /* SDLC Write Buffer                     */
uint8_t        DLSw_rbuf[65536];    /* DLSw Read Buffer                      */
uint8_t        DLSw_wbuf[65536];    /* DLSw Write Buffer                     */
uint16_t       DLSwrlen;            /* Buffer size of received DLSw data     */
uint16_t       DLSwwlen;            /* Buffer size of transmitted DLSw data  */
uint16_t       SDLCrlen;            /* Buffer size of received SDLC data     */
uint16_t       SDLCwlen;            /* Buffer size of transmitted SDLC data  */
uint16_t       IFRAMlen;            /* Zize of I-Frame to be transmitted     */
uint8_t        fc_byte;             /* Flow Control Byte                     */
int            fca_owed;            /* Flow control acknowledge owed         */
int            fca_due;             /* Flow control acknowledge due          */
int            fc_init_window_size; /* initial value for current window      */
int            fc_current_window;   /* Basis for granting addional units     */
int            fc_granted_units;    /* Number of units sender may sent       */
int            rp_granted_units;    /* Number of units remote peer may sent  */
int            lp_granted_units;    /* Number of units local peerm ay sent   */
int            flow_control = 0;    /* Flow control on/off switch            */
int            dlsw_wfd;            /* DLSw outbound socket (write)          */
int            dlsw_rfd;            /* DLSw inbound socket (read)            */
int            dlsw_sfd;            /* Our DLSw server socket                */
int            conrfd = OFF;        /* Status of DLSw read connection        */
int            conwfd = OFF;        /* Status of DLSw write connection       */
int            conlfd = OFF;        /* Status of SDLC line connection        */

uint8_t        seq_Nr = 0;          /* SDLC frame sequence receive number    */
uint8_t        seq_Ns = 0;          /* SDLC frame sequence send number       */

int SocketReadAct (int fd);
void ReadSig (int rs232_fd, int state);

struct         sockaddr_in lineaddr; /* SDLC line connection                 */
int            line_fd;             /* SDLC line socket                      */
int            rs232_fd;            /* SDLC RS232 signal socket              */
uint8_t        rs232_stat;          /* RS232 signal status                   */
int            rc;                  /* Various return codes                  */
int            state;               /* DLSw state                            */

// DLSw states
#define        DISCONNECTED        0
#define        CIRCUIT_PENDING     1
#define        CIRCUIT_START       2
#define        CIRCUIT_RESTART     3
#define        CIRCUIT_ESTABLISHED 4
#define        CONNECT_PENDING     5
#define        CONNECTED           6

/*********************************************************************/
/* Function to pint state change                                     */
/*********************************************************************/
void print_state() {
   switch(state) {
      case (DISCONNECTED):
         printf("\rDLSw: state DISCONNECTED\n");
      break;
      case (CIRCUIT_START):
         printf("\rDLSw: state CIRCUIT_START\n");
      break;
      case (CIRCUIT_RESTART):
         printf("\rDLSw: state CIRCUIT_RESTART\n");
      break;
      case (CIRCUIT_ESTABLISHED):
         printf("\rDLSw: state CIRCUIT_ESTABISHED\n");
      break;
      case (CIRCUIT_PENDING):
         printf("\rDLSw: state CIRCUIT_PENDING\n");
      break;
      case (CONNECT_PENDING):
         printf("\rDLSw: state CONNECT_PENDING\n");
      break;
      case (CONNECTED):
         printf("\rDLSw: state CONNECTED\n");
      break;
   }
}

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

/*-------------------------------------------------------------------*/
/* Process DLSw message                                              */
/*-------------------------------------------------------------------*/
/*    DLSw frames layout (picture from RFC1795):

      CONTROL MESSAGES (72 Bytes)
       (zero based offsets below shown in hex )
      +-----------------------------+-----------------------------+
      | (00) Version Number         | (01) Header Length (= 48)   |
      +-----------------------------+-----------------------------+
      | (02) Message Length                                       |
      +-----------------------------+-----------------------------+
      | (04) Remote Data Link Correlator                          |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (08) Remote DLC Port ID                                   |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (0C) Reserved Field                                       |
      +-----------------------------+-----------------------------+
      | (0E) Message Type           | (0F) Flow Control Byte      |
      +-----------------------------+-----------------------------+
      | (10) Protocol ID            | (11) Header Number          |
      +-----------------------------+-----------------------------+
      | (12) Reserved                                             |
      +-----------------------------+-----------------------------+
      | (14) Largest Frame Size     | (15) SSP Flags              |
      +-----------------------------+-----------------------------+
      | (16) Circuit Priority       | (17) Message Type (see note)|
      +-----------------------------+-----------------------------+
      | (18) Target MAC Address (non-canonical format)            |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -|
      |                                                           |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (1E) Origin MAC Address (non-canonical format)            |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -|
      |                                                           |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |         .                              .                  |
      +-----------------------------+-----------------------------+
      | (24) Origin Link SAP        | (25) Target Link SAP        |
      +-----------------------------+-----------------------------+
      | (26) Frame Direction        | (27) Reserved               |
      +-----------------------------+-----------------------------+
      | (28) Reserved                                             |
      +-----------------------------+-----------------------------+
      | (2A) DLC Header Length                                    |
      +-----------------------------+-----------------------------+
      | (2C) Origin DLC Port ID                                   |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (30) Origin Data Link Correlator                          |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (34) Origin Transport ID                                  |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (36) Target DLC Port ID                                   |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (3C) Target Data Link Correlator                          |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (40) Target Transport ID                                  |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (44) Reserved Field                                       |
      +-----------------------------+-----------------------------+
      | (46) Reserved Field                                       |
      +-----------------------------+-----------------------------+
               (Even Byte)                  (Odd Byte)


       INFORMATION MESSAGE (16 Bytes)
      +-----------------------------+-----------------------------+
      | (00) Version Number         | (01) Header Length (= 10)   |
      +-----------------------------+-----------------------------+
      | (02) Message Length                                       |
      +-----------------------------+-----------------------------+
      | (04) Remote Data Link Correlator                          |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (08) Remote DLC Port ID                                   |
      +- - - - - - - - - - - - - - -+- - - - - - - - - - - - - - -+
      |                                                           |
      +-----------------------------+-----------------------------+
      | (0C) Reserved Field                                       |
      +-----------------------------+-----------------------------+
      | (0E) Message Type           | (0F) Flow Control Byte      |
      +-----------------------------+-----------------------------+
               (Even Byte)                 (Odd Byte)
*/

int proc_DLSw(unsigned char DLSw_rbuf[], int DLSwrlen, unsigned char *DLSw_wbuf) {
   uint8_t MSG_type = DLSw_rbuf[HDR_MTYP];
   uint16_t MSG_len = (DLSw_rbuf[HDR_MLEN] << 8) + DLSw_rbuf[HDR_MLEN+1];
   uint8_t HDR_len = DLSw_rbuf[HDR_HLEN];
   uint16_t GDS_id;
   int DLSwwlen = 0;
   uint8_t signal;

   fc_byte = 0x00;
   // Handle flow control for the sending side (remote peer) (RFC 1795, section 8.7)
   if (DLSw_rbuf[HDR_FCB] & FCB_FCI) {
      fc_byte |= FCB_FCA;
      fca_due = 1;
   }
   // Handle flow control at the receiving side (local peer) (RFC 1795, section 8.7)
   if (flow_control) {
      rp_granted_units--;   // Frame received, so decrease senders granted units count
      if (DLSw_rbuf[HDR_FCB] & FCB_FCA) {
         if (fca_owed)
            fca_owed = 0;
         else {
            printf("\rDLSw: Flow Control Protocol Error\n");
         }  // End if (fca_owed)
      }  // End  if (rbuf[HDR_FCB] & FCB_FCA)

      // Create and send an Independent Flow Control Message
      // If pending flow control acknowledge, do not send a new flow control byte.
      if (!fca_owed) {
         // Handle flow control (RFC 1795, section 8.7)
         if (rp_granted_units <= fc_current_window) {                      // If granted units below current window size...
            memcpy(DLSw_wbuf, INFOFRAME_Hdr, sizeof(INFOFRAME_Hdr));       // ...create a flow control message...
            DLSw_wbuf[HDR_MTYP] = IFCM;                                    // Set message type
            DLSw_wbuf[HDR_FCB] = FCB_FCI | FCO_RPT;                        // Set operation (repeat)
            fca_owed = 1;                                                  // Indicate an acknowledge is required
            rp_granted_units += fc_current_window;                         // ...increase granted units by current window siz
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "DLSw: Peer Granted Units increased to %d\n", rp_granted_units);
            }  // End if  (Tdbg_flag == ON)
            memcpy(DLSw_wbuf + HDR_RDLC, &dlc, 4);                         // Set remote correlator
            memcpy(DLSw_wbuf + HDR_RDPID, &dlc_pid, 4);                    // Set remote port ID
            DLSw_wbuf[HDR_MLEN] = 0x00;                                    // No message (header only)
            DLSw_wbuf[HDR_MLEN+1] = 0x00;                                  // No message (header only)
            DLSwwlen = sizeof(INFOFRAME_Hdr);                              // Total frame length equals to header length
            rc = send(dlsw_wfd, DLSw_wbuf, DLSwwlen, 0);                   // Send to peer DLSw
         }  // End if (rp_granted_units
      }  // End  if (!fca_owed)
   }  // End if (flow_control)

   // Process remote peer's command message and build a response
   // The remote peer's command may set/change the circuit state
   switch (MSG_type) {
      case CANUREACH:                                                      // Can U reach command received
         if (Tdbg_flag == ON) {
            if (DLSw_rbuf[HDR_SFLG] & SSPex) {
               fprintf(T_trace, "\rCANUREACH_EX\n");
               printf("\rDLSW: Received CANUREACH_EX\n");
            }
            else {
               fprintf(T_trace, "\rCANUREACH_CS\n");
               printf("\rDLSW: Received CANUREACH_CS\n");
            }
         }
         if (conlfd == ON) {
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rSending ICANREACH\n");
               printf("\rDLSW: Sending ICANREACH\n");
            }
            memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);                         // Copy header from received command to buffer
            DLSw_wbuf[HDR_MTYP] = ICANREACH;                               // Change type to I can reach
            DLSw_wbuf[HDR_MLEN] &= 0x00;                                   // No message (header only)
            DLSw_wbuf[HDR_MLEN+1] &= 0x00;                                 // No message (header only)
            DLSw_wbuf[HDR_DIR] = DIR_ORG;                                  // Set Direction of this reply
            DLSw_wbuf[HDR_SFLG] = DLSw_rbuf[HDR_SFLG];                     // Set SSP flag
            memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);         // Copy origin DLC to remote DLC
            memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);       // Copy Origin PID to remote PID
            DLSwwlen = HDR_len;                                            // Total response length equals header length
            state = CIRCUIT_START;                                         // Update DLSw state
            print_state();                                                 // Show it
         }
      break;
      case REACH_ACK:                                                      // Message ICANREACH is acknowledged
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rREACH_ACK\n");
            printf("\rDLSW: Received REACH_ACK\n");
         }
         state = CIRCUIT_ESTABLISHED;                                      // Update DLSw state
         flow_control = 1;                                                 // Handle flow control from here on
         memcpy(dlc, &DLSw_rbuf[HDR_ODLC], 4);                             // Copy origin DLC to remote DLC
         memcpy(dlc_pid, &DLSw_rbuf[HDR_ODPID], 4);                        // Copy Origin PID to remote PID
         print_state();                                                    // Show state
      break;
      case XIDFRAME:
         if (Tdbg_flag == ON) {                                            // Received XID from remote peer
            fprintf(T_trace, "\rXIDFRAME\n");
            printf("\rDLSW: Received XIDFRAME\n");
         }
         if (MSG_len > 0) {
            PUtype=DLSw_rbuf[HDR_len];                                     // Copy PU type
            IDBLK=(DLSw_rbuf[HDR_len+2] << 8) + (DLSw_rbuf[HDR_len+3]);    // Copy IDBLK and IDNUM (First 4 bits)
            IDNUM=(DLSw_rbuf[HDR_len+4] << 8) + (DLSw_rbuf[HDR_len+5]);    // Copy IDNUM (Last 16 bits)
            memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);
            DLSw_wbuf[HDR_MTYP] = CONTACT;                                 // Set message type
            DLSw_wbuf[HDR_MLEN] &= 0x00;                                   // No message (header only)
            DLSw_wbuf[HDR_MLEN+1] &= 0x00;                                 // No message (header only)
            DLSw_wbuf[HDR_DIR] = DIR_ORG;                                  // Set message direction
            DLSw_wbuf[HDR_FCB] = fc_byte;                                  // Set flow control byte
            memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);         // Copy origin DLC to remote DLC
            memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);       // Copy Origin PID to remote PID
            DLSwwlen = HDR_len;
         } else {                                                          // empty XID message received
            memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);                         // copy received header
            DLSw_wbuf[HDR_MTYP] = XIDFRAME;                                // Set message type
            DLSw_wbuf[HDR_DIR] = DIR_ORG;                                  // Set message direction
            DLSw_wbuf[HDR_FCB] = fc_byte;                                  // Set flow control byte
            memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);         // Copy origin DLC to remote DLC
            memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);       // Copy Origin PID to remote PID
            memcpy(DLSw_wbuf + sizeof(CONTROL_MSG_Hdr), XIDFRAME_Rsp, sizeof(XIDFRAME_Rsp));   // Copy XID response message after header
            DLSw_wbuf[HDR_MLEN] = (sizeof(XIDFRAME_Rsp) >> 8) & 0xFF;      // Set message length
            DLSw_wbuf[HDR_MLEN + 1] = sizeof(XIDFRAME_Rsp) & 0x00FF;       // 2nd byt of message length
            DLSwwlen = sizeof(CONTROL_MSG_Hdr) + sizeof(XIDFRAME_Rsp);     // total lenght = header + XID response
         }
      break;
      case CONTACT:                                                        // Received Contact message
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rCONTACT\n");
            printf("\rDLSW: Received CONTACT\n");
         }
         memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);                            // copy received header
         DLSw_wbuf[HDR_MLEN] &= 0x00;                                      // No message (header only)
         DLSw_wbuf[HDR_MLEN+1] &= 0x00;                                    // No message (header only)
         DLSw_wbuf[HDR_MTYP] = CONTACT;                                    // Set message type
         DLSw_wbuf[HDR_DIR] = DIR_ORG;                                     // Set message direction
         DLSw_wbuf[HDR_FCB] = fc_byte;                                     // Set flow control byte
         memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);            // Copy origin DLC to remote DLC
         memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);          // Copy Origin PID to remote PID
         DLSwwlen = HDR_len;                                               // Length is header length only
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rSending CONTACT\n");
            printf("\rDLSW: Sending CONTACT\n");
         }
         state = CONNECT_PENDING;                                          // Update DLSw state
         print_state();                                                    // Show it
      break;
      case CONTACTED:                                                      // Received Contacted message
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rCONTACTED\n");
            printf("\rDLSW: Received CONTACTED\n");
         }
         memcpy(dlc, &DLSw_rbuf[HDR_ODLC], 4);                             // Copy origin DLC to remote DLC
         memcpy(dlc_pid, &DLSw_rbuf[HDR_ODPID], 4);                        // Copy Origin PID to remote PID
         state = CONNECTED;                                                // Update DLSw state
         print_state();                                                    // Show it
         signal = RTS;                                                     // set sinal to RTS
         rc = send(rs232_fd, &signal, 1, 0);                               // Set RTS signal high (ready to send/receive)
      break;
      case ICANREACH:
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rICANREACH\n");
            printf("\rDLSW: Received ICANREACH\n");
         }
         memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);
         DLSw_wbuf[HDR_MLEN] &= 0x00;                                      // No message (header only)
         DLSw_wbuf[HDR_MLEN + 1] &= 0x00;                                  // No message (header only)
         DLSw_wbuf[HDR_MTYP] = REACH_ACK;                                  // Set message type
         DLSw_wbuf[HDR_DIR] = DIR_ORG;                                     // Set message direction
         DLSw_wbuf[HDR_FCB] = fc_byte;                                     // Set flow control byte
         memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);            // Copy origin DLC to remote DLC
         memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);          // Copy Origin PID to remote PID
         DLSwwlen = HDR_len;
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rSending REACH_ACK\n");
            printf("\rDLSW: Sending REACH_ACK\n");
         }
      break;
      case INFOFRAME:
         /* Information Frames received from peer DLSw.                                         */
         /* The message part following the header is extracted and converted into an SDLC frame.*/
         /* This SDLC frame will be send to the 3705 following RR from the 3705.                */
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rDLSw: Received DLSw INFOFRAME\n");
         }
         // Store I-Frame length in buffer as a prefix to the actual I-Frame
         IFRAMlen = MSG_len + 6;                                            // I-Frame length = DLSw MSG length + LH and LT
         SDLC_wbuf[SDLCwlen++] = IFRAMlen >> 8;                             // Store high order length byte in buffer
         SDLC_wbuf[SDLCwlen++] = IFRAMlen & 0x00FF;                         // Store low order length byte in buffer
         memcpy(SDLC_wbuf + SDLCwlen + 3, DLSw_rbuf + HDR_len, MSG_len);
         // Add LH, FCntl, Faddr. FCS and LT
         SDLC_wbuf[SDLCwlen + BFlag] = 0x7E;
         SDLC_wbuf[SDLCwlen + FCntl] = 0x00 + CFinal;
         SDLC_wbuf[SDLCwlen + FAddr] = 0xC1;                                // Station address hardcode (to be reviewed)
         SDLC_wbuf[SDLCwlen + FCntl] = (SDLC_wbuf[SDLCwlen + FCntl] & 0x1F) | (seq_Nr << 5);      // Insert SDLC frame receive sequence
         SDLC_wbuf[SDLCwlen + FCntl] = (SDLC_wbuf[SDLCwlen + FCntl] & 0xF1) | (seq_Ns << 1);      // Insert SDLC frame send sequence
         memcpy(SDLC_wbuf + SDLCwlen + 3 + MSG_len, SDLC_FCSLT, 3);
         seq_Ns++;                                                          // Update SDLC frame send sequence number
         if (seq_Ns == 8) seq_Ns = 0;                                       // If SDLC frame send sequence number > 7 reset to 0
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rDLSw: DLSw INFOFRAME Payload (size: %d, sent return code: %d): ", MSG_len + 7, rc);
            for (int i = 0; i < MSG_len + 6; i ++) {
               fprintf(T_trace, "%02X ", SDLC_wbuf[SDLCwlen+i]);
            }
            fprintf(T_trace, "\n");
            fflush(T_trace);
         }  // End if debug
         SDLCwlen =  SDLCwlen + MSG_len + 6;                                // New size = existing buffer content + iframe + lh + lt
      break;
      case HALT_DL:
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rHALT_DL\n");
            printf("\rDLSW: Received HALT_DL\n");
         }
         memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);
         DLSw_wbuf[HDR_MLEN] &= 0x00;                                       // No message (header only)
         DLSw_wbuf[HDR_MLEN+1] &= 0x00;                                     // No message (header only)
         DLSw_wbuf[HDR_MTYP] = DL_HALTED;                                   // Set message type
         DLSw_wbuf[HDR_DIR] = DIR_ORG;                                      // Set message directio
         memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);             // Copy origin DLC to remote DLC
         memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);           // Copy Origin PID to remote PID
         DLSwwlen = HDR_len;
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rSending DL_HALTED\n");
            printf("\rDLSW: Sending DL_HALTED\n");
         }
         signal = ~RTS;
         rc = send(rs232_fd, &signal, 1, 0);                                // Set RTS signal low (can no longer send/receive)
      break;                                                                // Send signal to 3705
      case RESTART_DL:
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rRESTART_DL\n");
            printf("\rDLSW: Received RESTART_DL\n");
         }
         memcpy(DLSw_wbuf, DLSw_rbuf, HDR_len);
         DLSw_wbuf[HDR_MLEN] &= 0x00;                                       // No message (header only)
         DLSw_wbuf[HDR_MLEN+1] &= 0x00;                                     // No message (header only)
         DLSw_wbuf[HDR_MTYP] = DL_RESTARTED;                                // Set message type
         DLSw_wbuf[HDR_DIR] = DIR_ORG;                                      // Set message direction
         memcpy(DLSw_wbuf + HDR_RDLC, &DLSw_rbuf[HDR_ODLC], 4);             // Copy origin DLC to remote DLC
         memcpy(DLSw_wbuf + HDR_RDPID, &DLSw_rbuf[HDR_ODPID], 4);
         DLSwwlen = HDR_len;
         DLSwwlen = HDR_len;
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rSending DL_RESTARTED\n");
            printf("\rDLSW: Sending DL_RESTARTED\n");
         }
      break;
      case CAP_EXCHANGE:                                                    // Received Capability Exchange Message
         GDS_id = (DLSw_rbuf[HDR_len + 2] << 8) | DLSw_rbuf[HDR_len+3] ;    // Get type of cap exchange message
         if (GDS_id == 0x1520) {                                            // Received capabiltiy parameters, requires a response
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rCAP_EXCHANGE Received\n");
               printf("\rDLSw: Received CAP_EXCHANGE\n");
            }
            // *  Set initial pacing values.
            fc_init_window_size = (DLSw_rbuf[HDR_len + CAP_IPW_OFF + 2] << 8) + (DLSw_rbuf[HDR_len + CAP_IPW_OFF+3]);
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rCAP_EXCHANGE: Initial Window size: %d\n", fc_init_window_size );
               printf("\rDLSw: Received CAP_EXCHANGE: Initial Window size: %d\n", fc_init_window_size);
            }
            // Init flow control variables
            fc_current_window = fc_init_window_size;                        // Set current flow control size to inital windoopow size
            rp_granted_units = fc_current_window;                           // Remote peer granted units (i.e. # of messaged remote peer may send)
            lp_granted_units = fc_current_window;                           // Local peer granted units (i.e. # of messaged local peer may send)
            fca_owed = 0;                                                   // no flow control response required

            memcpy(DLSw_wbuf, CONTROL_MSG_Hdr, sizeof(CONTROL_MSG_Hdr));    // Copy header to write buffer
            DLSw_wbuf[HDR_MTYP] = CAP_EXCHANGE;                             // Set message type
            DLSw_wbuf[HDR_DIR] = DIR_TGT;                                   // Set message direction
            memcpy(DLSw_wbuf + sizeof(CONTROL_MSG_Hdr), CAP_EXCHANGE_Rsp, sizeof(CAP_EXCHANGE_Rsp));
            memcpy(DLSw_wbuf + HDR_MLEN, CAP_EXCHANGE_Rsp, 2);
            DLSwwlen = sizeof(CONTROL_MSG_Hdr) + sizeof(CAP_EXCHANGE_Rsp);  // Length = header size + message size
            break;
         }
         if (GDS_id == 0x1521) {                                            // Received acceptance of Capability parameters
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rCAP_EXCHANGE RESPONSE\n");
               printf("\rDLSw: Received CAP_EXCHANGE RESPONSE\n");
            }
            break;
         }
   } // End Switch
   if (Tdbg_flag == ON)
      fflush(T_trace);                                                      // write trace buffer
   return DLSwwlen;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Main section - establish and manage TCP connections                        */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void main(int argc, char *argv[]) {
   struct         sockaddr_in dlswaddr;  /* Our DLSw connection               */
   struct         sockaddr_in peeraddr;  /* Peer DLSw connection              */
   struct         epoll_event event, events[1];
   in_addr_t      lineip;                /* DLSw line listening address       */
   int            epoll_fd;              /* Event polling socket              */
   int            sockopt;               /* Used for setsocketoption          */
   int            pendingrcv;            /* pending data on the socket        */
   int            event_count;           /* # events received                 */
   char           *ipaddr;
   struct         hostent *dlswent;
   struct         hostent *lineent;
   int            peerlen;               /* size of peer ip address           */
   int            linenum = 20;          /* SDLC line number (default 20)     */
   char           *peeraddrp;
   int            capex = NO;
   int            Fptr, FptrL, frame_len; /* SDLC frame pointers and length   */
   uint8_t        Cfield;                /* SDLC Control Field work byte      */
   int            i, rc, rc1, rc2;
   char ipv4addr[sizeof(struct in_addr)];

   /* Read command line arguments */
   if (argc == 1) {
      printf("\rDLSw: Error - Arguments missing\n");
      printf("\r   Valid arguments are:\n");
      printf("\r   -peerhn {hostname}  : hostname of peer DLSw\n");
      printf("\r   -peerip {ipaddress} : ipaddress of peer DLSw \n");
      printf("\r   -cchn {hostname}  : hostname of host running the 3705\n");
      printf("\r   -ccip {ipaddress} : ipaddress of host running the 3705 \n");
      printf("\r   -line {line number} : SDLC line number to connect to\n");
      printf("\r   -d : switch debug on  \n");
      return;
   }
   Tdbg_flag = OFF;
   i = 1;

   while (i < argc) {
      if (strcmp(argv[i], "-d") == 0) {
         Tdbg_flag = ON;
         printf("\rDLSw: Debug on. Trace file is trace_dslw.log\n");
         i++;
         continue;
      } else if (strcmp(argv[i], "-cchn") == 0) {
         if ( (lineent = gethostbyname(argv[i+1]) ) == NULL ) {
            printf("\rDLSw: Cannot resolve 3705 hostname %s\n", argv[i+1]);
            return;                /* error */
         }  // End if linewent
         printf("\rDLSw: Connection to be established with SLDC line at 3705 on host %s\n", argv[i+1]);
         i = i+2;
         continue;
      } else if (strcmp(argv[i], "-ccip") == 0) {
         inet_pton(AF_INET, argv[i+1], ipv4addr);
         if ( (lineent = gethostbyaddr(&ipv4addr, sizeof(ipv4addr), AF_INET) ) == NULL ) {
            printf("\rDLSw: Cannot resolve ip address %s\n", argv[i+1]);
            return; /* error */
         }  // End if lineent
         printf("\rDLSw: Connection to be established with SDLC line at 3705 on ip address %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-line") == 0) {
         sscanf(argv[i+1], "%d", &linenum);
         printf("\rDLSw: Connection to be established with SDLC line %d\n", linenum);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-peerhn") == 0) {
         if ( (dlswent = gethostbyname(argv[i+1]) ) == NULL ) {
            printf("\rDLSw: Cannot resolve hostname %s\n", argv[i+1]);
            return;                /* error */
         }  // End if dlswent
         printf("\rDLSw: Connection to be established with peer DLSw %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-peerip") == 0) {
         inet_pton(AF_INET, argv[i+1], ipv4addr);
         if ( (dlswent = gethostbyaddr(&ipv4addr, sizeof(ipv4addr), AF_INET) ) == NULL ) {
            printf("\rDLSw: Cannot resolve ip address %s\n", argv[i+1]);
            return; /* error */
         }  // End if dlswent
         printf("\rDLSw: Connection to be established with peer DLSw at ip address %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else {
         printf("\rDLS: invalid argument %s\n", argv[i]);
         printf("\r     Valid arguments are:\n");
         printf("\r     -cchn {hostname}    : hostname of host running the 3705\n");
         printf("\r     -ccip {ipaddress}   : ipaddress of host running the 3705 \n");
         printf("\r     -peerhn {hostname}  : hostname of peer DLSw\n");
         printf("\r     -peerip {ipaddress} : ipaddress of peer DLSw \n");
         printf("\r     -line {line number} : SDLC line number to connect to\n");
         printf("\r     -d : switch debug on  \n");
         return;
      }  // End else
   }  // End while

   //********************************************************************
   // DLSw debug trace facility
   //********************************************************************
   if (Tdbg_flag == ON) {
      T_trace = fopen("trace_DLSw.log", "w");
      fprintf(T_trace, "     ****** DLSw log file ****** \n\n"
                       "     DLSw_rt -d : trace all DLSw activities\n"
                       );
   }
   state = DISCONNECTED;
   print_state();

   //*******************************************************************************
   //* Prepare the SDLC line connection
   //* A parallel connection will be established to send RS232 signals to the LIB
   //* these signals are used to steer the action of the 3705 scanner
   //*******************************************************************************
   // SDLC line socket creation
   line_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (line_fd <= 0) {
      printf("\rDLSw: Cannot create line socket\n");
      return;
   }

   // SDLC line socket creation
   rs232_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (rs232_fd <= 0) {
      printf("\rDLSw: Cannot create rs232 socket\n");
      return;
   }

   // Assign IP addr and PORT number
   lineaddr.sin_family = AF_INET;
   memcpy(&lineaddr.sin_addr, lineent->h_addr_list[0], lineent->h_length);
   lineaddr.sin_port = htons(SDLCBASE + linenum);

   // Line and signal sockets have been created. The connection to the LIB will be done
   // after the DLSw connections have been prepared.
   printf("\rDLSw: Waiting for SDLC line connection to be established\n");

   //*******************************************************************************
   // Prepare the inbound DLSw connection (Read from peer DLSw)
   //*******************************************************************************
   if ((dlsw_sfd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1) {
      printf("\rDLSw: Inbound socket creation failed with error %s\n", strerror(errno));
      exit(-1);
   }
   /* Reuse the address regardless of any */
   /* spurious connection on that port    */
   sockopt = 1;
   setsockopt(dlsw_sfd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));
   memset((void *) &dlswaddr, 0, sizeof(dlswaddr));

   /* Bind the socket */
   dlswaddr.sin_family = AF_INET;
   dlswaddr.sin_addr.s_addr = htonl(INADDR_ANY);
   dlswaddr.sin_port = htons(DLSW_PORT);           // Default DLSw read port (2067)
   if (bind(dlsw_sfd, (struct sockaddr *)&dlswaddr, sizeof(dlswaddr)) < 0) {
       printf("\rDLSw: Inbound socket bind failed with %s\n", strerror(errno));
       exit(EXIT_FAILURE);
   }

   /* Listen and verify */
   if ((listen(dlsw_sfd, 10)) != 0) {
      printf("\rDLSw: Inbound socket listen failed %s\n", strerror(errno));
      exit(-1);
   }
   // Add polling events for the port
   epoll_fd = epoll_create(1);
   if (epoll_fd == -1) {
      printf("\nDLSw: failed to created the epoll file descriptor\n\r");
      exit(-2);
   }
   event.events = EPOLLIN;
   event.data.fd = dlsw_sfd;
   if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, dlsw_sfd, &event) == -1) {
      printf("\nDLSw: Add polling event failed with error %s \n\r", strerror(errno));
      close(epoll_fd);
      exit(-3);
   }
   printf("\rDLSw: DLSw ready, waiting for connection on TCP port %d\n\r", DLSW_PORT );

   //*************************************************************************************
   // Establish the outbound and inbound DLSw connections (write/read to/from peer DLSw)
   //*************************************************************************************
   // DLSw peer write socket creation
   dlsw_wfd = socket(AF_INET, SOCK_STREAM, 0);
   if (dlsw_wfd <= 0) {
      printf("\rDLSw: Create outbound socket to peer failed with %s\n", strerror(errno));
      exit(-1);
   }

   // DLSw peer read socket creation
   dlsw_rfd = socket(AF_INET, SOCK_STREAM, 0);
   if (dlsw_rfd <= 0) {
      printf("\rDLSw: Create inbound socket from peer failed with %s\n", strerror(errno));
      exit(-1);
   }

   // Assign IP addr and PORT number
   peeraddr.sin_family = AF_INET;
   memcpy(&peeraddr.sin_addr, dlswent->h_addr_list[0], dlswent->h_length);
   peeraddr.sin_port = htons(DLSW_PORT);
   printf("\rDLSw: Waiting for DLSw peer outbound connection to be established\n");

   dlsw_rfd = 0;  /* Clear DLSw read file descriptor                         */
   seq_Nr = 0;    /* Initialize SDLC frame sequence receive number           */
   seq_Ns = 0;    /* Initialize SDLC frame sequence send number              */
   SDLCwlen = 0;  /* Initialize SDLC write buffer length                     */

   while (1) {
      //*****************************************************************************************
      // Check if there is data to be received from the peer DLSw
      // If the connection is not active yet or if the connection was lost, try to re-establish
      //*****************************************************************************************
      if (conrfd == OFF) {
         event_count = epoll_wait(epoll_fd, events, 1, 50);
         if ((event_count > 0) && (dlsw_rfd < 1)) {
            /* Accept */
            peerlen = sizeof(peeraddr);
            if ((dlsw_rfd = accept(dlsw_sfd, (struct sockaddr *) &peeraddr, &peerlen)) == -1) {
               printf("\rDLSw: Inbound peer DLSw connection accept failed with %s\n", strerror(errno));
               exit(-1);
            }  // End if dlsw_rfd
            peeraddrp = inet_ntoa(peeraddr.sin_addr);
            printf("\rDLSw: Inbound connection from peer DLSw at %s\n", peeraddrp);
            conrfd = ON;
         }  // End if (event_count > 0)
      }  // End if (conrfd == OFF)

      if (conwfd == OFF) {
         rc = connect(dlsw_wfd, (struct sockaddr*)&peeraddr, sizeof(peeraddr));
         if (rc == 0) {
            printf("\rDLSw: Outbound connection to peer has been established\n");
            conwfd = ON;
         }  // End if rc == 0
      }  // End if conwfd == OFF

      if ((conrfd == ON) && (conwfd == ON) && (capex == NO)) {
         memcpy(&DLSw_wbuf, CONTROL_MSG_Hdr, sizeof(CONTROL_MSG_Hdr));
         memcpy(&DLSw_wbuf[sizeof(CONTROL_MSG_Hdr)], CAP_EXCHANGE_Msg, sizeof(CAP_EXCHANGE_Msg));
         DLSw_wbuf[HDR_MTYP] = CAP_EXCHANGE;
         memcpy(DLSw_wbuf + HDR_OMAC, OMAC_addr, 6);
         memcpy(&DLSw_wbuf[HDR_MLEN], CAP_EXCHANGE_Msg, 2);
         rc = send(dlsw_wfd, DLSw_wbuf, sizeof(CAP_EXCHANGE_Msg) + sizeof(CONTROL_MSG_Hdr), 0);
         printf("\rDLSw: CAP_EXCHANGE sent\n");
         if (Tdbg_flag == ON) {
            fprintf(T_trace, "\rDLSw CAP_EXCHANGE sent: ");
            for (int i = 0; i < rc; i ++) {
               fprintf(T_trace, "%02X ", DLSw_wbuf[i]);
            }
            fprintf(T_trace, "\n");
            fflush(T_trace);
         }  // End if Tdbg_lag
         capex = YES;
      }  // End if ((conrfd == ON) && (conwfd == ON) && (capex == NO))

      if (conrfd == ON) {
         pendingrcv = 0;
         rc = ioctl(dlsw_rfd, FIONREAD, &pendingrcv);
         if ((pendingrcv < 1) && (SocketReadAct(dlsw_rfd))) rc = 0;
         if (rc < 0) {                      // Retry once to account for timing delays in TCP.
            if ((pendingrcv < 1) && (SocketReadAct(dlsw_wfd))) rc = -1;
         }
         if (rc < 0) {
            printf("\rDLS: DLSw inbound connection dropped, trying to re-establish\n");
            // DLSw read socket recreation
            close(dlsw_rfd);
            dlsw_rfd = socket(AF_INET, SOCK_STREAM, 0);
            if (dlsw_rfd <= 0) {
               printf("\rDLSw: Cannot create inbound socket\n");
               return;
            }
            conrfd = OFF;
         } else {
            if (pendingrcv > 0) {
               DLSwrlen = read(dlsw_rfd, DLSw_rbuf, sizeof(DLSw_rbuf));
               if (Tdbg_flag == ON) {
                  fprintf(T_trace, "\rDLSw Read Buffer: ");
                  for (int i = 0; i < DLSwrlen; i ++) {
                     fprintf(T_trace, "%02X ", DLSw_rbuf[i]);
                  }
                  fprintf(T_trace, "\n");
                  fflush(T_trace);
               }  // End if debug

               DLSwwlen = proc_DLSw(&DLSw_rbuf[0], DLSwrlen, DLSw_wbuf);
               if (DLSwwlen != 0) {
                  rc = send(dlsw_wfd, DLSw_wbuf, DLSwwlen, 0);
                  if (Tdbg_flag == ON) {
                     fprintf(T_trace, "\rDLSw Write Buffer (sent=%d): ", rc);
                     for (int i = 0; i < DLSwwlen; i ++) {
                        fprintf(T_trace, "%02X ", DLSw_wbuf[i]);
                     }
                     fprintf(T_trace, "\n\r");
                     fflush(T_trace);
                  }  // End if debug
               }  // End if (DLSwrlen != 0)
            }  // End if (pendingrcv > 0)
         }  // End if (rc < 0)
      }  // End if (conrfd == ON)

      //*****************************************************************************************
      //  Check if there is data to be received from the SDLC line
      //  If the connection is not active yet or if the connection was lost, try to re-establish
      //*****************************************************************************************
      if (conlfd == OFF) {
         if (!(IsSocketConnected(line_fd))) {
            rc1 = connect(line_fd, (struct sockaddr*)&lineaddr, sizeof(lineaddr));
         }  // End if (!(IsSocketConnected(line_fd)))
         if ((rc1 == 0) && (!(IsSocketConnected(rs232_fd)))) {
            rc2 = connect(rs232_fd, (struct sockaddr*)&lineaddr, sizeof(lineaddr));
         }  // End if ((rc1 = 0) && (!(IsSocketConnected(rs232_fd))))
         if ((rc1 == 0) && (rc2 == 0)) {
            printf("\rDLSw: SDLC line connection has been established\n");
            conlfd = ON;
         }  // End if ((rc1 == 0) && (rc2 ==0))
      }  // End if conlfd

      if (conlfd == ON) {
         pendingrcv = 0;
         if (IsSocketConnected(line_fd)) {
            ReadSig(rs232_fd,state);
            rc = ioctl(line_fd, FIONREAD, &pendingrcv);
            // *** Keep the below for now; under investgation ***
            //if ((pendingrcv < 1) && (SocketReadAct(line_fd))) rc = -1;
            //if (rc < 0) {                      // Retry once to account for timing delays in TCP.
            //if ((pendingrcv < 1) && (SocketReadAct(line_fd))) rc = -1;
         } else {
            printf("\rDLSw: SDLC  connection dropped, trying to re-establish\n");
            // SDLC line and signal socket re-creation. First close the sockets
            close(line_fd);
            close(rs232_fd);
            conlfd = OFF;
            line_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (line_fd <= 0) {
               printf("\rDLSw: Cannot create line socket\n");
               return;
            }  // End if (line_fd <= 0)
            rs232_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (rs232_fd <= 0) {
               printf("\rDLSw: Cannot create RS232 signal socket\n");
               return;
            }  // End if (rs232_fd <= 0)
         }  // End if (IsSocketConnected(line_fd))

         if (pendingrcv > 0) {
            SDLCrlen = read(line_fd, SDLC_rbuf, sizeof(SDLC_rbuf));
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\rSDLC Read Buffer: ");
               for (int i = 0; i < SDLCrlen; i ++) {
                  fprintf(T_trace, "%02X ", SDLC_rbuf[i]);
               }  // End for (int i = 0;
               fprintf(T_trace, "\n");
               fflush(T_trace);
            }  // End if debug

            //***********************************************************************************************
            // Skip modem clocking and consecutive start flags
            //***********************************************************************************************
            Fptr = 0;
            if ((SDLC_rbuf[Fptr] == 0x00) || (SDLC_rbuf[Fptr] == 0xAA)) Fptr = 1;   // If modem clocking is used skip first char
            while ((SDLC_rbuf[Fptr] == 0x7E) && (SDLC_rbuf[Fptr+1] == 0x7E) && (Fptr < SDLCrlen-1)) {
               Fptr++;
            }
            //***********************************************************************************************
            // Search for SDLC frames.
            // The Received data may contain multiple frames. These will be handled one at a time.
            //***********************************************************************************************
            if (Fptr <= SDLCrlen-6) {   // If there is at least one frame
               do {                                                              // Do till Poll bit found...
                  frame_len = 0;
                  // Find end of SDLC frame...
                  while (!((SDLC_rbuf[Fptr + frame_len + 0] == 0x47) &&
                           (SDLC_rbuf[Fptr + frame_len + 1] == 0x0F) &&
                           (SDLC_rbuf[Fptr + frame_len + 2] == 0x7E))) {
                     frame_len++;
                  }  // End while

                  frame_len = frame_len + 3;                                     // Correction length LT
                  if (Tdbg_flag == ON) {
                     fprintf(T_trace, "\rDLSW: SDLC Frame found (%d): ", frame_len);
                     for (int i = 0; i < frame_len; i ++) {
                        fprintf(T_trace, "%02X ", SDLC_rbuf[Fptr+i]);
                     }
                     fprintf(T_trace, "\n");
                     fflush(T_trace);
                  }  // End if debug

                  //**************************************************************************************************************
                  // Process SDLC frame.
                  //**************************************************************************************************************
                  Cfield = SDLC_rbuf[Fptr+FCntl] & 0x03;
                  FptrL = 0;
                  switch (Cfield) {
                     //******************************************************
                     //* Check received frame for SNRM or RR
                     //******************************************************
                     case UNNUM:        //  Unnumbered ?
                        if ((SDLC_rbuf[Fptr + FCntl] & 0xEF) == XID) {           //  Exchange ID ?
                           if (Tdbg_flag == ON)            // Debug tracing on ?
                              fprintf(T_trace, "\rDLSw: XID received.\n");
                           if (state == CONNECTED) {                             // If state CONNECTED sent a UA response else ignore
                              if (SDLC_rbuf[Fptr + FCntl] & CPoll) {             // Poll command ?
                                 SDLC_wbuf[FptrL++] = 0x7E;                      // Add link header
                                 SDLC_wbuf[FptrL++] = SDLC_rbuf[Fptr+FAddr];     // Copy station ID
                                 SDLC_wbuf[FptrL++] = XID + CFinal;              // Insert XID response and set final bit
                                 SDLC_wbuf[FptrL++] = PUtype;                    // Copy Format + PU type
                                 SDLC_wbuf[FptrL++] = 0x00;                      // Variable format field lengthh
                                 SDLC_wbuf[FptrL++] = IDBLK >> 8;                // IDBLK (First 8 bits)
                                 SDLC_wbuf[FptrL++] = IDBLK & 0x00FF;            // IDBLK (last 4 bits)+ IDNUM (First 4 bits)
                                 SDLC_wbuf[FptrL++] = IDNUM >> 8;                // IDNUM (Middle 8 bits)
                                 SDLC_wbuf[FptrL++] = IDNUM & 0x00FF;            // IDNUM (Last 8 bits)
                                 memcpy(&SDLC_wbuf[FptrL], SDLC_FCSLT, sizeof(SDLC_FCSLT)); // Append FCS and Link trailer
                                 FptrL = FptrL + 3;
                                 rc = send(line_fd, SDLC_wbuf, FptrL, 0);
                                 if (Tdbg_flag == ON) {
                                    fprintf(T_trace, "\rDLSW: Send XID to SDLC Downstream\n");
                                    for (int i = 0; i < FptrL; i ++) {
                                       fprintf(T_trace, "%02X ", SDLC_wbuf[i]);
                                    }
                                    fprintf(T_trace, "\n");
                                    fflush(T_trace);
                                 }  // End if debug
                              }  // End if (SDLC_rbuf[FCntl] & CPoll)
                           }  // End (state == CONNECTED)
                        }  // End if ((Fcntl & 0xEF) == SNRM)

                        if ((SDLC_rbuf[Fptr + FCntl] & 0xEF) == SNRM) {          // Normal Response Mode ?
                           if (Tdbg_flag == ON)                                  // Debug tracing on ?
                              fprintf(T_trace, "\rDLSw: SNRM received.\n");
                           if (SDLC_rbuf[Fptr + FCntl] & CPoll) {                // Poll command ?
                              SDLC_wbuf[FptrL++] = 0x7E;                         // Add link header
                              SDLC_wbuf[FptrL++] = SDLC_rbuf[Fptr + FAddr];      // Copy station ID
                              SDLC_wbuf[FptrL++] = UA + CFinal;                  // Insert UA response and set final bit
                              memcpy(&SDLC_wbuf[FptrL], SDLC_FCSLT, sizeof(SDLC_FCSLT)); // Append FCS and Link trailer
                              FptrL = FptrL + 3;
                              rc = send(line_fd, SDLC_wbuf, FptrL, 0);
                              if (Tdbg_flag == ON) {
                                 fprintf(T_trace, "\rDLSW: Send UA to SDLC Downstream\n");
                                 for (int i = 0; i < FptrL; i ++) {
                                    fprintf(T_trace, "%02X ", SDLC_wbuf[i]);
                                 }
                                 fprintf(T_trace, "\n");
                                 fflush(T_trace);
                              }  // End if debug
                           }  // End if (SDLC_rbuf[FCntl] & CPoll)
                           seq_Nr = 0;                                           // Init SDLC frame seq receive number
                           seq_Ns = 0;                                           // Init SDLC frame seq send number
                           SDLCwlen = 0;                                         // Reset buffer content length

                        } // End if ((Fcntl & 0xEF) == SNRM)
                        break;
                     case SUPRV:        //  Supervisor ?
                        if (((SDLC_rbuf[Fptr+FCntl] & 0x0F) == RR) ||            //  Normal Response Mode or Not Ready Response ?
                            ((SDLC_rbuf[Fptr+FCntl] & 0x0F) == RNR)) {
                           if (Tdbg_flag == ON)                                  // Trace Terminal Controller ?
                              fprintf(T_trace, "DLSw: RR/RNR received.\n");
                           if (SDLC_rbuf[Fptr+FCntl] & CPoll) {                  // Poll command ?
                              if ((SDLCwlen > 0) && ((SDLC_rbuf[Fptr + FCntl] & 0x0F) == RR)) { // If RR and write buffer filled
                                 IFRAMlen = (SDLC_wbuf[0] << 8) + SDLC_wbuf[1];  // Get I-Frame length
                                 rc = send(line_fd, SDLC_wbuf + 2, IFRAMlen, 0); // send write buffer
                                 if (Tdbg_flag == ON) {
                                    fprintf(T_trace, "DLSW: Send IFRAME to SDLC Downstream\n");
                                    for (int i = 0; i < IFRAMlen; i ++) {
                                       fprintf(T_trace, "%02X ", SDLC_wbuf[i+2]);
                                    }
                                    fprintf(T_trace, "\n");
                                    fflush(T_trace);
                                 }  // End if debug
                                 SDLCwlen = (SDLCwlen - IFRAMlen) - 2;
                                 if (SDLCwlen > 0)
                                    memmove(SDLC_wbuf, &SDLC_wbuf[IFRAMlen+2], SDLCwlen); // Move remaining I-Frames to front of buffer
                              } else {
                                 SDLC_wbuf[FptrL++] = 0x7E;                      // Add link header
                                 SDLC_wbuf[FptrL++] = SDLC_rbuf[Fptr + FAddr];   // Copy station ID
                                 if (lp_granted_units > 0) {                     // If remote allows more messages
                                    SDLC_wbuf[FptrL++] = RR + CFinal;            // ...insert RR response and set final bit
                                 } else {                                        // If granted messages are exhausted...
                                    SDLC_wbuf[FptrL++] = RNR + CFinal;           // ...insert RNR response and set final bit
                                 }  // End if (lp_granted_units > 0)
                                 memcpy(&SDLC_wbuf[FptrL], SDLC_FCSLT, sizeof(SDLC_FCSLT)); // Append FCS and Link trailer
                                 FptrL = FptrL + 3;
                                 SDLC_wbuf[FCntl] = (SDLC_wbuf[FCntl] & 0x1F) | (seq_Nr << 5); // Insert receive sequence
                                 rc = send(line_fd, SDLC_wbuf, FptrL, 0);
                                 if (Tdbg_flag == ON) {
                                    fprintf(T_trace, "DLSW: Send response to RR/RNR to SDLC Downstream\n");
                                    for (int i = 0; i < FptrL; i ++) {
                                       fprintf(T_trace, "%02X ", SDLC_wbuf[i]);
                                    }
                                    fprintf(T_trace, "\n");
                                    fflush(T_trace);
                                 }  // End if debug
                              }  // End if ((SDLCwlen > 0)
                           }  // End (SDLC_rbuf[Fptr+FCntl] & CPoll)
                        }  // End if ((Fcntl & 0x0F) == RR)
                        break;
                     default:                                                    // Info frames will be forwarded to the DLSw peer
                        if (Tdbg_flag == ON)                                     // Trace Terminal Controller ?
                           fprintf(T_trace, "DLSw: SDLC IFRAME received.\n");
                        seq_Nr++;                                                // Update receive sequence number
                        if (seq_Nr == 8) seq_Nr = 0;                             // If sequence number > 7 reset to 0

                        // Create and send INFOFRAME with SDLC data
                        memcpy(DLSw_wbuf, INFOFRAME_Hdr, sizeof(INFOFRAME_Hdr));
                        memcpy(DLSw_wbuf + sizeof(INFOFRAME_Hdr), &SDLC_rbuf[Fptr+3], frame_len - 6 );
                        DLSw_wbuf[HDR_MTYP] = INFOFRAME;                         // Set message type
                        DLSw_wbuf[HDR_FCB] = fc_byte;                            // Set flow control byte
                        memcpy(DLSw_wbuf + HDR_RDLC, &dlc, 4);                   // Set Remote DLC
                        memcpy(DLSw_wbuf + HDR_RDPID, &dlc_pid, 4);              // Set Remote PID
                        DLSw_wbuf[HDR_MLEN] = ((frame_len - 6) >> 8) & 0xFF;     // Set message size (SDLC frame size - LH and LT)
                        DLSw_wbuf[HDR_MLEN+1] = (frame_len - 6) & 0x00FF;        // 2nd byet of message size
                        DLSwwlen = sizeof(INFOFRAME_Hdr) + (frame_len - 6);      // Total size is header + message size

                        // NOTE: filedescriptor set to rfd !!!
                        if (state == CONNECTED) {                                // If state CONNECTED sent a UA response else ignore
                           rc = send(dlsw_wfd, DLSw_wbuf, DLSwwlen, 0);          // Send Info frame to DLSw peer
                           if (Tdbg_flag == ON) {
                              fprintf(T_trace, "DLSw: Upstream Write Buffer (send=%d): ", rc);
                              for (int i = 0; i < DLSwwlen; i ++) {
                                 fprintf(T_trace, "%02X ", DLSw_wbuf[i]);
                              }
                              fprintf(T_trace, "\n\r");
                              fflush(T_trace);
                           }  // End if debug
                        } else {
                           if (Tdbg_flag == ON) {
                              fprintf(T_trace, "DLSw: Not Connected - Upstream Write Buffer NOT send ");
                              for (int i = 0; i < DLSwwlen; i ++) {
                                 fprintf(T_trace, "%02X ", DLSw_wbuf[i]);
                              }
                              fprintf(T_trace, "\n\r");
                              fflush(T_trace);
                           }  // End if debug
                        }  // End if (state == CONNECTED)
                        break;
                  }  // End switch

                  /*********************************************************/
                  /* Search for next frame                                 */
                  /*********************************************************/
                  Fptr = Fptr + frame_len;
               }  // End Do
               while (Fptr < SDLCrlen);
            }  // End if (Fptr <= SDLCrlen-6)
         }  // End if (pendingrcv > 0)
      }  // End if (conlfd == ON)
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

//*******************************************************************************************
// Check if there is a signal update from the RS232 connection                              *
// If so, receive signal data from the RS232 connection and respond if needed               *
//*******************************************************************************************
void ReadSig(int rs232_fd, int state) {
   int rc, pendingrcv;
   uint8_t sig;
   if (rs232_fd > 0) {                                 // If there is a connection...
      pendingrcv = 0;
      rc = ioctl(rs232_fd, FIONREAD, &pendingrcv);     // ...check for (signal) data in the TCP buffer
      if (pendingrcv > 0) {                                      // If there is data...
         //******************************************************
         for (int i = 0; i < pendingrcv; i++) {
            rc = read(rs232_fd, &sig, 1);              // ...read it
         }
         //******************************************************
         if (rc == 1) {                                // If signal data was received (must be 1 byte only)...
            if ((sig & RTS) && (state == CONNECTED)) { // If remote DCE has set RTS and DLSW is CONNECTED...
               rs232_stat |= CTS;                      // ...raise CTS
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r3271 received RS232=%02X, return signal=%02X\n", sig, rs232_stat);
               // Send the current RS232 signal back.
               //******************************************************
               rc = send(rs232_fd, &rs232_stat, 1, 0); // send current RS232 signal.
               //******************************************************
            }
         }  // End if (rc == 1)
      }  // if (pendingrcv > 0)
   }  // End  if (rs232_fd > 0)
   return;
}
