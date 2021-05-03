/* epicon.h epicon incude heder file
epicon is Copyright Isamu.Yamauchi 2002-2021.
o 2021.2.24 "/var/tmp/" -> "/var/run/lock/", add <arpa/inet.h>
o 2017.4.20 curses.h,term.h remove
o 2011.10.16 Enhanced file locking, changed the function msleep.
o 2011.9.27 On quiet mode, delete 'Disconnected message'.
o 2009.6.22 SEL_TIME_OUT 20 milliseconds -> 20micro seconds
o 2009.6.7 Epicon_Socket_init  "/var/tmp/epicon_socket." -> "/tmp/epicon_socket."
  <wait.h> - > <sys/wait.h>, delete <malloc.h>
o 2008.3.23 add #include <stdlib.h>, bug fix gcc-4 -> incompatible implicit declaration of built-in function exit
o 2007.1.4 add #include telnet.h, remove ncurses.h
o 2006.8.17 add #define Epicon_Socket
o 2002.11.6 oops typo.  SPEED -> "SPEED"

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/
#define BUFSIZ 2048                 /* io read/write buffer size */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <setjmp.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/telnet.h>
#include <arpa/inet.h>
#include <termios.h>
// #include <curses.h> 2017.4.20 remove
// #include <term.h> 2017.4.20 remove
#define R_WSIZE 1024                /* send file block size */
#define MSIZE 1024*1024             /* defaut memory allocate size */
#define SPEED "9600"                /* default com_port speed */
#define VER "5.2"                   /* Version 2017.4.20 */
#define COMPORT "/dev/ttyS0"        /* default com_port  */
#define ESC     '~'                 /* default esc char  */
#define EOT     '\004'              /* ^D */
#define SEL_TIME_OUT 20             /* select time out micro second for 115200bps */
#ifndef R_OK
#define R_OK 4
#define W_OK 2
#endif
#ifdef __STDC__
#define sigtype void
#else
#define sigtype int
#endif
#define DAY "compiled:"__DATE__
#define Epicon_Socket_init "/var/run/lock/epicon_socket." /* AF_UNIX socket */
#define VAR_LOCK "/var/run/lock/"
#define TTY_LOCK "/var/run/lock/LCK.."
#define MY_TTYPE  "vt100"  /* terminal type */
