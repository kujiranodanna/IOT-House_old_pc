.\" usage this file.
.\" groff -Tascii -man epicon.nr >epocon.1
.\"
.TH epicon 1 "epicon Manuals" "24 Feb 2021" "epicon Manuals"
.PP
.PP
.SH NAME
epicon is Easy Personal Interface Console terminal software.
First I am sorry. Because my English linguistic power is very shabby, this sentence is being translated by the machine.
Because of that, read it in the interpretation which it is tolerant of though it thinks that it is a little funny translation.

.SH SYNOPSIS
usage:
.PD 0
.PP
.PD 1
epicon [-options [argument] [-options [argument]]
       [-b ] <--escape cannot be used
       [-c external_command]
       [-d send_charcacter_delay(ms)]
       [-D send_CR_delay(ms)]
       [-e escape_char]
       [-f send_file]
       [-F send_file_effective_delay]
       [-m ] <--input echo mode
       [-M ] <--line mode
       [-l com_port]
       [-L output_log_file]
       [-n ip_address[:port]]
       [-p [server_port]]
       [-q ] <--quiet mode
       [-s speed]
       [-v ] <--show version
       [-x bit_length (5|6|7) parity(o|e|n) stop_bit (1|2)]
       [-z ] <--auto rz prohibition

 defaults:
     speed:  9600b/s (Higest of 460800)
     com_port :  /dev/ttyS0
     escape: ~ (escape char is three octal digits)

 running epicon escapes:
     escape,. or escape,CTRL-D program end
     escape,!                  into to shell
     escape,escape             send escape char
     escape,b                  send break
     escape,f                  send files
     escape,F                  send files delay effect
     escape,c                  chege speed
     escape,C                  external command
     escape,sz                 send file zmodem
     escape,rz                 recive file zmodem
     escape,sk                 send file gkermit
     escape,rk                 recive file gkermit
     escape,sx                 send file xmodem
     escape,rx                 recive file xmodem
.PP
.SH DESCRIPTION
The epicon is a Linux program for console Terminal.it need libc.so.5
or other libc,lrz,lsz,gkermit,pepolinux,compiling gcc-2.95.3 later.
Symbolic link are being done as flowing.
.PD 0
.PP
lrwxrwxrwx   1 root   root      3 Sep 22 22:39 rb -> lrz*
.PP
lrwxrwxrwx   1 root   root      3 Sep 22 22:39 rx -> lrz*
.PP
lrwxrwxrwx   1 root   root      3 Jul 16 17:34 rz -> lrz*
.PP
lrwxrwxrwx   1 root   root      3 Sep 22 13:46 sz -> lsz*
.PP
lrwxrwxrwx   1 root   root      3 Sep 22 22:38 sb -> lsz*
.PP
lrwxrwxrwx   1 root   root      3 Sep 22 22:38 sx -> lsz*
.PP
.PD 1

It is free software under the GNU Public License. See the COPYING file for details.
It was written for the people who did a setup of an environment of router or switch on pepolinux.
I am lucky if the people who do work related to the network by this software epicon can help it.
.PP
The epicon binary is called "epicon". It should be stored someplace in your UNIX PATH; normally it is available as /usr/local/bin/epicon.
To run epicon, just type "epicon" followed by command-line options that tell it what to do. If wrong options are given, it prints a usage message listing the available options.
.SH OPTIONS
If an option takes an argument, the argument is required; if an option does not take an argument, no argument may be given (exception: -p -z -b -m ).
The action options are -l, -s, etc. Only one action option may be given. If noaction options are given, epicon does nothing. Here are some examples ("$ " is the your shell prompt):
.PP
.TP
default
$ epicon  <-- /dev/ttyS0,9600bps,non-parity,8bits,1stop-bit
.TP
.B \-\^p
Ip server mode default wait port 9999.
.PD 0
.RS
$ epicon -p
.PP
$ epicon -p xxxx
.PP
xxxx is wait port number.
.RE
.PD 1
.TP
.B \-\^n
Ip client mode default connect port 9999.
.PD 0
.RS
.PP
$ epicon -n xxx.xxx.xxx.xxx
.PP
xxx.xxx.xxx.xxx is server's ip address.
.PP
$ epicon -n xxx.xxx.xxx.xxx:pppp
.PP
pppp is connect port number.
.RE
.PD 1
.TP
.B \-\^c
external command. This option can work in the back-ground process.
.PD 0
.PP
.PD 1
.RS
$ epicon -c external_command
.PD 0
.PP
$ epicon -c ./sample.scr & <--back-ground process.
.PD 1
.RE
.TP
.B \-\^l
Connect line.
.PD 0
.PP
.PD 1
.RS
$ epicon -l /dev/ttyS1
.RE
.TP
.B \-\^L
output Log.
.PD 0
.PP
.PD 1
.RS
$ epicon -L log_file_name.
.RE
.TP
.B \-\^d
send charcacter delay.
.PD 0
.PP
.PD 1
.RS
$ epicon -d 20 <--20ms send charcacter delay.
.RE
.TP
.B \-\^D
send CR delay
.PD 0
.PP
.PD 1
.RS
$ epicon -D 20 <--20ms send CR delay.
.RE
.TP
.B \-\^e
Change to escape charcacter.
.PD 0
.PP
.PD 1
.RS
$ epicon -e 007 <--octal digits.
.RE
.TP
.B \-\^f
send Files. a background also operates.
.PD 0
.PP
.PD 1
.RS
$ epicon -f send_file_name.
.PD 0
.PP
$ epicon -f send_file_name & <--a background also operates.
.RE
.PD 1
.TP
.B \-\^F
send Files. Transmitting delay is effective.
.PD 0
.PP
.PD 1
.RS
$ epicon -d 10 -D 50 -F send_file_name.
.RE
.PD 1
.TP
.B \-\^s
Change speed.
.PD 0
.PP
.RS
$ epicon -s 19200 <-- 19200 B/S Send & Recive speed.
.RE
.PD 1
.TP
There are thing that argument is necessary for the option of the command line and thing which is not so.(the examples: -b -m -p -q -x -z which isn't necessary)
.TP
.B \-\^b
Cannot escape mode,To exit kill own process or remote hung-up.
.TP
.B \-\^m
Input echo mode,Use escape.
.TP
.B \-\^M
line mode,send line's characters if CR.
.TP
.B \-\^q
quiet mode.
.TP
.B \-\^x
bit_length[5|6|7] parity[o|e|n] stop_bit[1|2].
.TP
.B \-\^z
No auto call zmodem recive,Default auto call mode.
.TP
.PP
.SH
In the running epicon.. how to do..
CR + escape + type flowing character then do action.
Escape character is '~' default. CR is carriage return.
.TP
.B ~rz
RECEIVE. Wait for incoming zmodem protocols.
Default zmodem auto recive mode.
.TP
.B ~rx
RECEIVE. Wait for incoming xmodem protocols.
.TP
.B ~rk
RECEIVE. Wait for incoming kermit protocols.
.TP
.B ~sz
SEND zmodem protocols. Request to file name so answer it.
.TP
.B ~sx
SEND xmodem protocols. Request to file name so answer it.
.TP
.B ~sk
SEND kermit protocols. Request to file name so answer it.
.TP
.B ~f
Send image file. Of course ascii-text,binary-code file transfer
no protocols.
.TP
.B ~F
Send file. Transmitting delay is effective no protocols.
.TP
.B ~c
CHANGE to speed 300 to 480000.
.TP
.B ~C
External command mode argument max 10. Input,Output descriptor copy to given /dev/ttyXX. How to use your good idea or choice.
.TP
.B ~b
Send break. Suspend transmission, resume when you type something from the keyboard
.TP
.B ~!
Into shell. 'exit' command then back to epicon.
.TP
.B ~.
Exit the program return to your shell.
.PP
.SH PATHNAMES
A file pathnames was not supported.
.SH AUTOCALL ZMODEM
When epicon receives a Zmodem protocol header then auto call Recive Zmodem.
.SH IMPLEMENTATION NOTES
epicon is designed for router or switch configuration tool, be small, portable ,simple.
epicon is a serial line or a TCP/IP connection which works on the pepolinux.
epicon call protocol of Zmodem, Xmodem, Kermit from the inside.
.SH DIAGNOSTICS
This program isn't being created so kindly as to take out a debugging message, so if you find it doing so there's probably something bugs or wrong.
.SH REFERENCES
http://pepolinux.sourceforge.jp/  pepolinux@users.sourceforge.jp
.SH RESOURCES
epicon probably uses resources only a little.
.SH SEE ALSO
gkermit,lrz,lsz.
.PD 0
.PP
.PD 1
The only other program I know with this little functionality is the
ubiquitous console terminal application.
.SH COPYRIGHT
epicon is Copyright Isamu.Yamauchi September 2002-2011.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

.SH BUGS
This software may still keep containing many bugs by the un-maturity.
There probably are some, but I don't know what they are yet.
It is lucky if I can have a report and advice by mail if a bug is found.
.SH AUTHOR
Isamu.Yamauchi
The PepoLinux Project, Only Me.
https://ja.osdn.net/projects/pepolinux/
pepolinux@users.osdn.me


.SH THANKS
Thanks to all the people who did software contribution.
As for appreciating it, I want you to know that a wonderful thing like this could be created.
