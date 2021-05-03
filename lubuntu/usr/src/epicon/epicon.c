/* epicon.c epicon command interface program
epicon is Copyright Isamu.Yamauchi 2002-2021.
o 2021.2.24 gcc version 7.5.0 Warning clean up.
  Mr. Sasano's suggestion specified character length, parity, stop bit, and transmission break.
o 2011.9.27 On quiet mode, delete 'Disconnected message'.
o 2010.11.17 warning: ignoring return value of ‘write’, declared with attribute warn_unused_result. clean up.
o 2009.10.31 rename main.c -> epicon.c, add -q option quiet mode, clean up.
o 2008.5.10 bugs fix alarm(5) -> alarm(10),initialize_com_port(); -> if (ck_pid) initialize_com_port();
o 2008.1.4 easy telnet client is supported, -v option is changed to the line mode.
o 2007.9.29 bugs fix "/var/lock/epicon_socket"+.pid->Epicon_Socket,Auto rz
o 2006.8.17 bugs fix "/var/lock/epicon_socket"->Epicon_Socket
o 2005.9.10 bug fix (usage)
o 2004.10.23 default client port 9999->23
o 2004.10.1 add "-v" vt100 console mode
o 2004.10.1 add "-F" send character file with delay
o 2004.9.20 add "-d" send charcacter delay,add "-D" send CR delay.
o 2003.10.1 clean up
o 2002.12.24 clean up.
o 2002.11.6 add "-L" log option.
o 2002.11.2 add "-c" execute external command option.
o 2002.11.1 add "-f" send file option.

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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "epicon.h"

extern long int *mem_buff;          /* external memory buffer */
char esc[2] = { ESC, '\0' };        /* escape charctor */
extern pid_t ck_pid;                /* check to process id */
char *LOG_file = '\0';              /* console log file */
char *SF_file = '\0';               /* send character file name with delay */
char *CM_file = '\0';               /* external command file */
char *SB_file = '\0';               /* send binary file name */
char *argv_redirect;                /* redirect file */
char Epicon_Socket[128];            /* external AF_Socket */
char *ip_addr;                      /* ip net connect addres */
char *ip_port;                      /* ip net connect port */
char *server_ip_port = "9999";      /* default server port */
char *client_ip_port = "23";        /* default client port */
char com_port[64];                  /* tty_dev name */
int com_port_fd;                    /* com_port file descriptor */
int console_fd;                     /* console file descriptor */
int net_flag = 0;                   /* ip net connect flag */
int server_ip_flag = 0;             /* ip server flag */
int Twostop = 0;                    /* 2 stop bit */
int Datalen = CS8;                  /* data bit length */
int Parity = 0;                     /* parity setting */
int Bin_flag = 0;                   /* binary mode flag cannot escape */
int Echo_flag = 0;                  /* input echo flag mode */
int AZ_flag = 1;                    /* auto call rz flag default auto */
int SB_flag = 0;                    /* send binary file flag */
int SF_flag = 0;                    /* send character file flag with delay*/
int CM_flag = 0;                    /* external command option flag */
int LOG_flag = 0;                   /* console log flag */
unsigned int Char_delay = 0;        /* external send charcacter delay value */
unsigned int CR_delay = 0;          /* external send CR delay value */
int CON_flag = 0;                   /* console flag */
int Quiet_flag = 0;                 /* quiet flag */
FILE *LOG_fp = 0;                   /* log file descriptor */

void show_version();
int usage();
void set_console_mode();
void end_process();
void set_com_port_mode();    /* set & save com_port characteristics   */
void initialize_com_port();  /* reset saved  com_port characteristics */
void com_port_unuse();
int com_port_use_check();
void set_com_port_mode();
void epicon_main();

sigtype end_process();
sigtype dev_timeout();
sigtype end_process()
{
  if (net_flag) {
    set_console_mode(0);
  }
  else {
    if (ck_pid) initialize_com_port();
      com_port_unuse(com_port);
  }
  if (LOG_flag) fclose(LOG_fp);
  unlink(Epicon_Socket);
  free(mem_buff);
  if (ck_pid && !Quiet_flag) fprintf(stderr, "\r\nDisconnected\r\n");
  exit(0);
}

void main(argc, argv)
int argc;
char *argv[];
{
  char *wc,*wc1;
  int done,i,speed,t_result;
  long int epicon_pid;
  done = i = speed = t_result = epicon_pid = 0;
  extern char *optarg;
  extern int optind;
  extern int getopt(), convert_speed(), escape_code(), access(), convert_mode();
  epicon_pid = getpid();
  sprintf( Epicon_Socket ,"%s%ld",Epicon_Socket_init , epicon_pid);
  unlink( Epicon_Socket );
  strcpy(com_port, COMPORT);
  argv_redirect = argv[argc];
  speed = convert_speed(SPEED);
  while((i = getopt(argc, argv,"bmMs:d:D:l:e:n:L:f:F:c:pqvzx:")) != EOF) {
    switch(i) {
      case 'b':
        Bin_flag = 1; /* cannot escape flag set */
        break;
      case 'd': /* set send character delay value */
        Char_delay = atoi(optarg);
        break;
      case 'D': /* set CR character delay value */
        CR_delay = atoi(optarg);
        break;
      case 'c':
        CM_flag = 1; /* external command option flag set */
        CM_file = optarg; /* external command file */
        break;
      case 'm':
        Echo_flag = 1; /* input echo flag set */
        break;
      case 's':
        if((speed = convert_speed(optarg)) == EOF) {
          fprintf(stderr,"%s: invalid speed\n", optarg);
          exit(1);
        }
        break;
      case 'l': /* input & output serial device */
        strncpy(com_port, optarg, sizeof(com_port));
        if(access(com_port, R_OK|W_OK) == -1) {
          perror(optarg);
          exit(2);
        }
        break;
      case 'L':
        LOG_file = optarg; /* output log file */
        LOG_flag = 1; /* output log flag set */
        if ((LOG_fp = fopen(LOG_file,"a+")) < 0) {
          if ((LOG_fp = fopen(LOG_file,"w")) < 0) {
            fprintf(stderr,"%s: %s\n\n",LOG_file,"file open append error !!  ");
            exit(2);
          }
        }
        break;
      case 'n':
        ip_addr = ip_port = optarg;
        wc = strtok(ip_addr,":");
        wc1 = strtok(NULL,":");
        if ( wc1 != NULL ) {  /* exp; epicon -n 127.0.0.1:80 */
          ip_port = wc1 ; /* 80 */
          ip_addr = wc;   /* 127.0.0.1 */
        }
        else {
          ip_port = client_ip_port;
        }
          strncpy(com_port, ip_addr, sizeof(com_port));
          server_ip_flag = 0;
          net_flag = 1;
          epicon_main();
          end_process();
        break;
      case 'p':
        if (argv[optind] != NULL) server_ip_port = argv[optind];
        server_ip_flag = 1;
        net_flag = 1;
        epicon_main();
        end_process();
      case 'q':
        Quiet_flag = 1;
        break;
      case 'f':
        SB_flag = 1;
        SB_file = optarg;
        break;
      case 'F':
        SF_flag = 1;
        SF_file = optarg;
        break;
      case 'e':
        esc[0] = escape_code(optarg);
        break;
      case 'M':
        CON_flag = 3;
        break;
      case 'v':
        show_version();
        exit(3);
        break;
      case 'z':
        AZ_flag = 0;
        break;
      case 'x':
        if(convert_mode(optarg) == EOF) {
          fprintf(stderr,"%s: invalid mode\n", optarg);
          exit(1);
        }
        break;
      default:
        usage(*argv);
        exit(3);
    }
  }
  if(! com_port_use_check(com_port)) {
    fprintf(stderr, "\nDevice %s is busy\n",com_port);
  if (! Bin_flag ) exit(1);
  }

  signal(SIGALRM, dev_timeout);
  alarm(10);
  if (! net_flag) {
  #ifdef O_NDELAY
    if((com_port_fd = open(com_port, O_RDWR|O_NDELAY)) < 0)
  #else
    if((com_port_fd = open(com_port, O_RDWR)) < 0)
  #endif
  { perror(com_port);
    com_port_unuse(com_port);
    exit(4);
  }
  #ifdef O_NDELAY
    set_com_port_mode(speed);
   {
      int fd = com_port_fd;
      if((com_port_fd = open(com_port, O_RDWR)) < 0) {
        perror(com_port);
        com_port_unuse(com_port);
        exit(4);
      }
      close(fd);
    }
  #endif
  signal(SIGTERM, SIG_IGN);
  set_com_port_mode(speed);
  }

  alarm(0);
  signal(SIGALRM, SIG_DFL);
  signal(SIGINT, SIG_IGN);
  signal(SIGQUIT, SIG_IGN);
  signal(SIGTERM, end_process);
  console_fd = fileno(stdin);
  if (!SB_flag && !CM_flag && !Quiet_flag) {
    fprintf(stderr,"\r\n** Welcome to epicon Version-%s Copyright Isamu Yamauchi %s **",VER,DAY);
    if (!Bin_flag) fprintf(stderr,"\r\n      exec shell         ~! ");
    if (!Bin_flag) fprintf(stderr,"\r\n      send binary files  ~f");
    if (!Bin_flag) fprintf(stderr,"\r\n      send break         ~b");
    if (!Bin_flag) fprintf(stderr,"\r\n      call rz,sz,sx,rx   ~rz,~sz,~sx,~rx");
    if (!Bin_flag) fprintf(stderr,"\r\n      call kermit        ~sk,~rk");
    if (!Bin_flag) fprintf(stderr,"\r\n      external command   ~C ");
    if (!Bin_flag) fprintf(stderr,"\r\n      change speed       ~c ");
    if (!Bin_flag) fprintf(stderr,"\r\n      exit               ~. \r\n");
    if (Bin_flag)  fprintf(stderr,"\r\n      Do exit is kill own process or remote log out\r\n");
    fprintf(stderr,"      Connected %s \r\n",com_port);
  }
/* goto read & write process */
  epicon_main();
}

sigtype dev_timeout()
{
  perror("dev_timeout()");
  fprintf(stderr, "Cannot open port \n");
  end_process();
}


#if !defined(B38400) || (B38400 != 38400)
typedef struct {
  char *spc;
  int  spi;
} spd;
spd zspeed[] = {
#ifdef B50
  {"50",B50},
#endif
#ifdef B75
  {"75",B75},
#endif
#ifdef B110
  {"110",B110},
#endif
#ifdef B134
  {"134",B134},
#endif
#ifdef B150
  {"150",B150},
#endif
#ifdef B200
  {"200",B200},
#endif
#ifdef B300
  {"300",B300},
#endif
#ifdef B600
  {"600",B600},
#endif
#ifdef B1200
  {"1200",B1200},
#endif
#ifdef B1800
  {"1800",B1800},
#endif
#ifdef B2400
  {"2400",B2400},
#endif
#ifdef B4800
  {"4800",B4800},
#endif
#ifdef B9600
  {"9600",B9600},
#endif
#ifdef B19200
  {"19200", B19200},
#endif
#ifdef B38400
  {"38400", B38400},
#endif
#ifdef B57600
  {"57600", B57600},
#endif
#ifdef B115200
  {"115200", B115200},
#endif
#ifdef B230400
  {"230400", B230400},
#endif
#ifdef B460800
  {"460800", B460800},
#endif
#ifdef B500000
  {"500000", B500000},
#endif
#ifdef B576000
  {"576000", B576000},
#endif
#ifdef B921600
  {"921600", B921600},
#endif
#ifdef B1000000
  {"1000000", B1000000},
#endif
#ifdef B1152000
  {"1152000", B1152000},
#endif
#ifdef B1500000
  {"1500000", B1500000},
#endif
#ifdef B2000000
  {"2000000", B2000000},
#endif
#ifdef B2500000
  {"2500000", B2500000},
#endif
#ifdef B3000000
  {"3000000", B3000000},
#endif
#ifdef B3500000
  {"3500000", B3500000},
#endif
#ifdef B4000000
  {"4000000", B4000000},
#endif
#ifdef EXTA
  {"19200", B19200},
#endif
#ifdef EXTB
  {"38400", B38400},
#endif
  { 0, 0 }
};

int convert_speed(sp)
register char *sp;
{
  register spd *i = &zspeed[0];
  if(sp == NULL) return EOF;
  if(*sp == 0) return EOF;
  for(; i->spc != (char *)0; i++)
  if(strncmp(sp, i->spc, strlen(sp)) == 0)
    return i->spi;
  return EOF;
}
#else
int convert_speed(sp)
register char *sp;
{
  if(sp == NULL) return EOF;
  if(*sp == 0) return EOF;
  return atoi(sp);	
}
#endif

int convert_mode(arg)
register char *arg;
{
  register int i = 0;
  char c;
  int cs[] = {CS5, CS6, CS7, CS8};
  int sp[] = {0, CSTOPB};
  while(c = arg[i]) {
    if (i > 2) return EOF;
    if (c == '-') ;/* do nothing */
    else switch(i) {
    case 0:
      if (c < '5' || c > '8') return EOF;
      Datalen = cs[c - '5'];
      break;
    case 1:
      if (c == 'E' || c == 'e') Parity = PARENB;
      else if (c == 'O' || c == 'o') Parity = PARENB | PARODD;
      else if (c == 'N' || c == 'n') Parity = 0;
      else return EOF;
      break;
    case 2:
      if (c < '1' || c > '2') return EOF;
      Twostop = sp[c - '1'];
      break;
    }
    i++;
  }
  return 0;
}

int escape_code(c)
register char *c;
{
  register int i = 0;
  if(!isdigit(*c))
    return *c;
  do {
    if(*c > '7' || *c < '0') {
      fprintf(stderr,"escape char must be character octal digits\n");
      exit(4);
    }
    i = (i << 3) + (*c++ - '0');
  } while(isdigit(*c));
      return i;
}

int alarm_delay = 0;
jmp_buf timebuf;
int in_shell = 0;
sigtype alarm_int() {
 if( in_shell ) longjmp(timebuf, -1);
}
int paused = 0;

sigtype exit_shell() {
  paused = 0;
}

sigtype into_shell()
{
  alarm(0); in_shell = 0;
  int t_result = 0;
  t_result = write(2, "\r\nepicon wait\r\n",15);
  signal(SIGUSR1, SIG_IGN);
  signal(SIGUSR2, exit_shell);
  paused = 1;
  while(paused) pause();
  signal(SIGUSR2, SIG_IGN);
  signal(SIGUSR1, into_shell);
  t_result = write(2, "epicon run\r\n",12);
}

int usage(char *avg)
{
fprintf(stderr,"usage: [-options [argument]]\r\n");
fprintf(stderr,"        [-b ] <--escape no used\r\n");
fprintf(stderr,"        [-c external_command]\r\n");
fprintf(stderr,"        [-d send_charcacter_delay(ms)]\r\n");
fprintf(stderr,"        [-D send_CR_delay(ms)]\r\n");
fprintf(stderr,"        [-e escape_char]\r\n");
fprintf(stderr,"        [-f send_file]\r\n");
fprintf(stderr,"        [-F send_file_effective_delay]\r\n");
fprintf(stderr,"        [-m ] <--input echo mode\r\n");
fprintf(stderr,"        [-M ] <--line mode\r\n");
fprintf(stderr,"        [-l com_port]\r\n");
fprintf(stderr,"        [-L output_log_file]\r\n");
fprintf(stderr,"        [-n ip_address[:port]]\r\n");
fprintf(stderr,"        [-p [server_port]]\r\n");
fprintf(stderr,"        [-s speed]\r\n");
fprintf(stderr,"        [-v ] show version\r\n");
fprintf(stderr,"        [-x bit_length (5|6|7) parity(o|e|n) stop_bit (1|2)] \r\n");
fprintf(stderr,"        [-z ] <--auto rz prohibition\r\n%s",avg);
}

void show_version()
{
  fprintf(stderr,"\r\n\
** Welcome to epicon Version-%s Copyright Isamu Yamauchi %s **\r\n\
epicon is Easy Personal Interface CoNsole software.\r\n\r\n",VER,DAY);
}
