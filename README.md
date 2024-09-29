#IBM3705_R4

The main updates with Release 4 of the IBM 3705 SIMH emulator:

1 - Line Interface Base emulation added (with RS232 signal emulation)
2 - DLSw support added
3 - Null modem added (3705 to 3705 connection)

Below is a detailed overview of updates of this release:

Central Control Unit (CCU/CPU)

None 

Channel Adapter

None 

Scanner

Scanner code rationalized: With the addition of the LIB, the differences in handling BSc and SDLC could be removed. There is now a single approach yielding performance improvements for SDLC.

LIB

Line Interface Base (LIB) with RS232 emulation: A LIB in a 3705 is the place where the communication lines are physically connected. The emulator now has a LIB module, to which all lines are connected. The LIB manages the line connections, which are TCP/IP connections, it receives SDLC/BSC frames from a line and passes this byte for byte to the 3705 scanner and receives bytes from the scanner, which are collated to one or more SDLC/BSC frames and then sends these frames across the relevant line. 
The LIB includes RS232 signal emulation. It takes on the role of
a Data Communication Equipment (DCE). The RS232 signals are determined by the state of the TCP/IP connection or are set/reset by the scanner. The RS232 signals are important to the behavior of the scanner, which determines a course of action based on the presence or absence of certain signals. In case the scanner wants to transmit data it will raise a Request To Send (RTS, which the LIB sends across the relevant line.  The remote side of the line (the remote DCE) will with a Clear To Send (CTS) if the remote PU or Cluster is ready to receive data. 
The LIB comes with a panel, which shows the state of the signals. (Like in the good old day the lights flashing on a modem)
    
SDLC / BSC

None

IBM 3274

DCE emulation in  i3274: The SNA PU T.2 emulation (i3274) includes a limited Data Communication Equipment (DCE) function, which handles responses to a Request To Send (RTS) from the 3705 scanner.

IBM 3271

DCE emulation in i3271. The BSC cluster emulation (i3271)  includes a limited Data Communication Equipment (DCE) function, which handles responses to a Request To Send (RTS) from the 3705 scanner.

IBM 3705 Front-Panel

None

DLSw 
A stand-alone module emulating a Data Link Switch (DLSw) is included. This allows real IBM equipment (e.g. 3174 or a PC with SDLC adapter) to be connected to the 3705 emulator. The DLSw module connects at one end to a line of the LIB, the other side connects to a real DLSw router. The emulated DLSw is build according to RFC1795 standard. This includes local handling of Supervisory and Unnumbered SDLC frames as well as handling frame sequence numbers locally. All this reduces the amount of traffic that has to flow between the 2 DLSw’s.

Null Modem

Null Modem (NModem). This is a stand-alone module allowing to interconnect two emulated 3705’s. For MVS 3.8 systems this is not yet relevant as VTAM L2 does not support cross-domain connections. Once support for a (emulated) remote 3705 is available, this can be used to connect a channel attached 3705 to a remote 3705. For Higher version of VTAM and NCP this can be used for cross-domain connections, e.g. connect a channel attached 3705 from one host to a channel attached 3705 at another host. 

To do

    V24 (DB25) to USB interface (in progress)
    Full duplex support for SDLC lines

EF & HJS (C)2024
