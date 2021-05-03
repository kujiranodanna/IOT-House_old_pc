/* epicon_main.c epicon main program
epicon is Copyright Isamu.Yamauchi 2002-2021.
o 2021.2.24 gcc version 7.5.0 Warning clean up.
  Mr. Sasano's suggestion specified character length, parity, stop bit, and transmission break.
o 2017.4.20 send_window_size() remove.
o 2010.11.19 warning: ignoring return value of ‘write’, declared with attribute warn_unused_result. clean up.
o 2009.10.31 add -q option quiet mode, clean up.
o 2009.6.23 telnet client bug fix.(corresponds to TELOPT_ENCRYPT,window size)
o 2008.1.9 bugs fix auto rz.
o 2008.1.4 easy telnet client is supported, -v option is changed to the line mode.
o 2007.9.30 bugs fix "/var/lock/epicon_socket"+.pid->Epicon_Socket,Auto rz
o 2006.8.17 bugs fix "/var/tmp/epicon_socket"->Epicon_Socket
o 2005.12.17 bugs fix (case of newline,ESC,ESC)
o 2005.9.10 many bugs fix (ESC,ip server)
o 2004.10.1 add "-F" send character file with delay
o 2004.9.20 add "-d" send charcacter delay,add "-D" send CR delay.
o 2003.10.2 clean up.
o 2003.9.13 oops select() using.
o 2002.11.6 add "-c" external command option.
o 2002.11.1 add "-f" send file option.
o 2002.10.27 gcc-2.96 available

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

#ifdef HAVE_TERMIOS_H
#  define HAVE_TERMIOS
#endif

#include "epicon.h"

extern char esc[2];             /* escape charctor */
pid_t ck_pid = 0;               /* check to process id */
extern int com_port_fd;         /* com_port file descriptor */
extern int console_fd;          /* console file descriptor */
extern int net_flag;            /* ip net connect flag */
extern int server_ip_flag;      /* ip server flag */
extern char *ip_addr;           /* ip net connect addres */
extern char *ip_port;           /* ip net connect port */
extern char *server_ip_port;    /* default server port */
extern char com_port[64];       /* tty_dev name */
extern int Bin_flag;            /* binary mode cannot escape */
extern int Echo_flag;           /* input echo flag mode */
extern int AZ_flag;             /* auto call rz flag default auto */
int ip_socket_bufsize = 1024*1024;  /* ip socket buffer size */
extern int SB_flag;             /* send binary file flag */
extern char *SB_file;           /* send binary file name */
extern int SF_flag;             /* send character file flag with delay*/
extern char *SF_file;           /* send character file name with delay */
extern int CM_flag;             /* external command option flag */
extern char *CM_file;           /* external command file */
extern unsigned int Char_delay; /* external send charcacter delay value */
extern unsigned int CR_delay;   /* external send CR delay value */
extern char Epicon_Socket[128]; /* AF_UNIX socket for Communication of parents & child */
void win_size_update();         /* windows size send for telnet */
void display_console();         /* open message */
long int incount,outcount,*mem_buff;
int speed;
int fp1,fp2;
int pg_flag,fp4;
int esc_flag = 0;
int result,result_fp,server_flag,read_block_flag_net;
int server_ip_sockfd, client_ip_sockfd;
int server_ip_len, client_ip_len;
int server_sockfd,client_sockfd;
int server_len,client_len;
int sockfd_ip;
int result_ip;
int zstep_flag = 0;       /* recive zmodem check step flag */
int iac_step_flag = 0;    /* recive telnet option check step flag */
int display_flag = 1;     /* enable display */
int dont_flag = 0;        /* Telnet'opt Don't flag */
int do_flag = 0;          /* Telnet'opt Do flag */
int will_flag = 0;        /* Telnet'opt Will flag */
int send_window_flag = 0; /* Telnet'opt windows flag */
int send_ttype_flag = 0;  /* Telnet'opt terminal type */
int sb_flag = 0;          /* Telnet'opt subnegotiation flag */
int my_width = 0;         /* Telnet'opt windows size width */
int my_height = 0;        /* Telnet'opt windows size height */
static char *c_result;    /* tmp char result */
static int t_result;      /* tmp int result */
unsigned int len_ip,ip1,ip2,ip3,ip4,ip_port_list;
unsigned long int ip_list;
char *f1,*f2,f3[128],ch,c,ch1,ch2,ch3,*ch4;
char ck_newline = 1;
fd_set   fpp1,fpp2;
void end_process();
void into_shell();
void sz();
void sx();
int sk();
void rz();
void rx();
int rk();
void exec_command();
void client_socket_write();
void display_ip_open_msg();
void set_console_mode();
int convert_speed();
void set_com_port_mode();
int shell_process();
void send_file();
void msleep();
void send_file_char();
void send_break();

// send telent option
void send_wont_char(int ch_send)
{
  sprintf(f3,"%c%c%c",IAC,WONT,ch_send);
  t_result = write(fp2,&f3,strlen(f3));
 return;
}

void send_dont_char(int ch_send)
{
  sprintf(f3,"%c%c%c",IAC,DONT,ch_send);
  t_result = write(fp2,&f3,strlen(f3));
 return;
}

void send_will_char(int ch_send)
{
  sprintf(f3,"%c%c%c",IAC,WILL,ch_send);
  t_result = write(fp2,&f3,strlen(f3));
 return;
}

void send_will_window()
{
  sprintf(f3,"%c%c%c",IAC,WILL,TELOPT_NAWS);
  t_result = write(fp2,&f3,strlen(f3));
  return;
}

void send_will_ttype()
{
  sprintf(f3,"%c%c%c",IAC,WILL,TELOPT_TTYPE);
  t_result = write(fp2,&f3,strlen(f3));
  return;
}

void send_ttype()
{
  int dummy = 0;
  sprintf(f3,"%c%c%c%c%s%c%c",IAC,SB,TELOPT_TTYPE,dummy,MY_TTYPE,IAC,SE);
  t_result = write(fp2,&f3, 6 + strlen(MY_TTYPE));
  return;
}

void send_window_size()
{
/* 2017.4.20 remove
  int dummy = 0;
  setupterm(NULL, fileno(stdout), (int *) 0);
  my_height = tigetnum("lines");
  my_width = tigetnum("cols");
  sprintf(f3,"%c%c%c%c%c%c%c%c%c",IAC,SB,TELOPT_NAWS,dummy,my_width,dummy,my_height,IAC,SE);
  t_result = write(fp2,&f3,9);
*/
  return;
}

sigtype win_size_update()
{
    send_window_size();
}

void zmodem_check()
{ /* zmodem check & telnet option */
  int chh;
  chh = ch & 0377;
  ch3 = chh;
  if (iac_step_flag != 0) {
    if (iac_step_flag == 1) {
      dont_flag = do_flag = will_flag = 0;
      iac_step_flag++;
      switch(chh) {
        case DONT:
          dont_flag = 1;
          break;
        case WILL:
          will_flag = 1;
          break;
        case DO:
          do_flag = 1;
          break;
        case SB:
          sb_flag = 1;
          break;
      }
      return;
    }
    if (iac_step_flag == 2) {
      iac_step_flag = 0;
      display_flag = 1;
      switch(chh) {
        case TELOPT_ECHO:
          if (dont_flag == 1) display_flag = 0;
          if (do_flag == 1) {
            display_flag = 1;
            send_wont_char(ch3);
          }
          if (will_flag == 1) {
            display_flag = 1;
          }
          break;
        case TELOPT_ENCRYPT:
          if (do_flag == 1) send_wont_char(ch3);
          if (will_flag == 1) send_dont_char(ch3);
          break;
        case TELOPT_AUTHENTICATION:
          if (do_flag == 1) send_wont_char(ch3);
          if (will_flag == 1) send_dont_char(ch3);
          break;
        case TELOPT_STATUS:
          if (will_flag == 1 || do_flag == 1) send_wont_char(ch3);
          if (dont_flag == 1) send_dont_char(ch3);
          break;
        case TELOPT_NAWS:
          if (send_window_flag == 1 && do_flag == 1) {
            send_window_size();
          }
          break;
        case TELOPT_TTYPE:
          if (send_ttype_flag == 1 && sb_flag == 1) {
            send_ttype();
          }
          break;
        default:
          if (will_flag == 1) send_dont_char(ch3);
          if (do_flag == 1) send_wont_char(ch3);
          if (send_window_flag == 0) {
            send_will_window();
            send_window_flag = 1;
            send_will_ttype();
            send_ttype_flag = 1;
          }
          break;
      }
      return;
    }
  }
/* AZ_flag auto_call rz flag -z option value is 0 */
  switch(chh) {  /* zmodem check **\030B00000 then rz call */
    case IAC:
      iac_step_flag = 1;
      display_flag = 0;
      zstep_flag = 0;
      break;
    case '*':
      if (zstep_flag == 0 || zstep_flag == 1) zstep_flag++;
      else  zstep_flag = 0;
      break;
    case '\030':
      if (zstep_flag == 2) zstep_flag++;
      else  zstep_flag = 0;
      break;
    case 'B':
      if (zstep_flag == 3) zstep_flag++;
      else  zstep_flag = 0;
      break;
    case '0':
      if (zstep_flag > 3 && zstep_flag < 15) zstep_flag++;
      if (zstep_flag == 15) {
        fprintf(stderr,"\n\repicon auto rz recive start!!\r\n");
        int status;
        pid_t pid;
        signal(SIGCHLD,  SIG_IGN);
        if ((pid = fork()) == -1) {
          perror("Cannot rzmodem");
          set_console_mode(2);
          return;
        }
        if (pid == 0) {
          dup2(fp2,0);
          dup2(fp2,1);
          execlp("rz","rz",(char *)0);
          perror("abnormal command end");
          exit(-1);
        }
        if (pid != 0) {
          signal(SIGCHLD, SIG_DFL);
          pid_t chid_pid;
          chid_pid = wait( &status );
          if ( net_flag ) {
            set_console_mode(1);
          }
          else {
            set_console_mode(2);
          }
          zstep_flag = 0;
           fprintf(stderr,"\r\n%s\r\n","auto rz normal end");
        }
      }
      break;
    default:break;
    }
  if (display_flag == 1) display_console(fp1,ch3);
  return;
}

void send_break()
{
  #ifndef HAVE_TERMIOS
    ioctl(com_port_fd, TCSBRK, 0);
  #else
    tcsendbreak(com_port_fd, 250);
  #endif
}

void check_char_1()
{
  ch &= (char)0377;
  switch (ch)  { /* which character is esc+? */
    case EOT:
    case '.':
    case '>':
      if (net_flag) {
        if (shutdown (fp2,2) < 0) {
          fprintf(stderr,"%d",errno);
          perror("shutdown socket error");
        }
      }
      close(fp1);
      close(fp2);
      exit(0);
    case 'r':
      esc_flag = 2;
      return;
    case 's':
      esc_flag = 3;
      return;
    case 'c':
      incount = incount - 2;
      if (!net_flag) {
        fprintf(stderr,"\n\rinput change speed:");
        set_console_mode(0);
        c_result = fgets(f3,64,stdin);
        ch4 = strtok(f3,"\n");
        set_console_mode(2);
        if ((speed = convert_speed(ch4)) == EOF) {
          fprintf(stderr,"%s: is invalid speed\r\n",ch4);
        }
         else {
          set_com_port_mode(speed);
          break;
        }
      }
      break;
    case '!':
      incount = incount - 2;
      /* into shell */
      kill(getppid(), SIGUSR1);
      shell_process();
      /* exit shell */
      if( kill(getppid(), SIGUSR2) < 0 ) exit(0);
      break;
      /* exec command line */
    case 'C':
      incount = incount - 2;
      client_socket_write('1');
      /* requet to commnand */
      set_console_mode(0);
      fprintf(stderr,"\n\rinput command_line:");
      c_result = fgets(f3,128,stdin);
      if (strlen(f3) < 2) {
        fprintf(stderr,"\r\nNo command!\r\n");
        set_console_mode(2);
        client_socket_write('0');
      }
      else {
        ch4 = strtok(f3,"\n");
        exec_command(fp1,fp2,ch4);
        ch = '\0'; esc_flag = 1;
        client_socket_write('0');
        set_console_mode(2);
        fprintf(stderr,"\r\nepicon commnad mode end\r\n");
      }
      break;
      /* send files binary */
    case 'f':
      incount = incount - 2;
      fprintf(stderr,"\n\rinput send_file_name :");
      set_console_mode(0);
      c_result = fgets(f3,64,stdin);
      set_console_mode(2);
      ch4 = strtok(f3,"\n");
      client_socket_write('1');
      send_file(fp2,ch4);
      client_socket_write('0');
      break;
      /* send charcter file with delay opiotion */
    case 'F':
      incount = incount - 2;
      fprintf(stderr,"\n\rinput send_file_name effective_delay :");
      set_console_mode(0);
      c_result = fgets(f3,64,stdin);
      set_console_mode(2);
      ch4 = strtok(f3,"\n");
      send_file_char(fp2,ch4);
      break;
    case 'b':
      incount = incount - 2;
      send_break();
      break;
    default:
      esc_flag = 0;
      break;
  }
  return;
}

void check_char_2()
{
  if ( esc_flag == 2 ) {  /* which character is esc+"r"+? */
    if (ch == 'x') {
      incount = incount - 3;
      client_socket_write('1');
      rx(fp1,fp2);
      client_socket_write('0');
    }
    if (ch == 'z') {
      incount = incount - 3;
      client_socket_write('1');
      rz(fp1,fp2);
      client_socket_write('0');
    }
    if (ch == 'k') {
      incount = incount - 3;
      client_socket_write('1');
      if (rk(fp1,fp2) == 0) {
        fprintf(stderr,"\r\nCall Kermit normal end.\r\n");
        client_socket_write('0');
      }
      else {
        fprintf(stderr,"\r\nCall Kermit Abnormal end.\r\n");
        client_socket_write('0');
      }

    }
  }
  if ( esc_flag == 3 ) {  /* character is esc+"s"+? */
    if (ch == 'x') {
      incount = incount - 3;
      client_socket_write('1');
      sx(fp1,fp2);
      client_socket_write('0');
    }
    if (ch == 'z') {
      incount = incount - 3;
      client_socket_write('1');
      sz(fp1,fp2);
      client_socket_write('0');
    }
    if (ch == 'k') {
      incount = incount - 3;
      client_socket_write('1');
      if (sk(fp1,fp2) == 0) {
        fprintf(stderr,"\r\nCall Kermit normal end.\r\n");
        client_socket_write('0');
      }
      else {
        fprintf(stderr,"\r\nCall Kermit Abnormal end.\r\n");
        client_socket_write('0');
      }
    }
  }
  esc_flag = 0;
  return;
}

void epicon_main()
{
  struct timeval timeout;
  struct sockaddr_in server_ip_address;
  struct sockaddr_in client_ip_address;
  struct sockaddr_un server_address;
  struct sockaddr_un client_address;
  struct sockaddr_in address_ip;
  struct linger ling;
  f1="/dev/stdin";
  f2=com_port;
  console_fd = fileno(stdin);
  fp1=console_fd;
  signal(SIGUSR1,into_shell);
/* AF_UNIX,SOCK_STREAM read & write nonblock set */
  server_sockfd = socket(AF_UNIX,SOCK_STREAM,0);
  server_address.sun_family = AF_UNIX;
  strcpy(server_address.sun_path,Epicon_Socket);
  server_len = sizeof(server_address);
  bind(server_sockfd,(struct sockaddr *)&server_address,server_len);
/* server process. waitting port */
  if (net_flag && server_ip_flag) {
    int on = 1;
    display_ip_open_msg();
    set_console_mode(1);
    server_ip_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (setsockopt(server_ip_sockfd, SOL_SOCKET, SO_REUSEADDR,(char *) &on ,sizeof(on))<0) perror("server_ip_sockfd,SOL_SOCKET,SO_REUSEADDR");
    setsockopt(server_ip_sockfd ,SOL_SOCKET,SO_SNDBUF,&ip_socket_bufsize,sizeof(ip_socket_bufsize));
    setsockopt(server_ip_sockfd ,SOL_SOCKET,SO_RCVBUF,&ip_socket_bufsize,sizeof(ip_socket_bufsize));
/*  set linger option for queued on socket and a close is performed  */
    ling.l_onoff = 1;
    ling.l_linger =60;
    if (setsockopt(server_ip_sockfd , SOL_SOCKET, SO_LINGER,(char *) &ling, sizeof(struct linger))<0) perror("setsockopt(server_ip_sockfd , SOL_SOCKET, SO_LINGER");
    if (setsockopt(server_ip_sockfd ,SOL_SOCKET,SO_KEEPALIVE, &on ,sizeof(on))<0) perror("server_ip_sockfd ,SOL_SOCKET,SO_KEEPALIVE");

    server_ip_address.sin_family = AF_INET;
    server_ip_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_ip_address.sin_port = htons(atoi(server_ip_port));
    server_ip_len = sizeof(server_ip_address);
    bind(server_ip_sockfd, (struct sockaddr *)&server_ip_address, server_ip_len);
    listen(server_ip_sockfd, 5);
    read_block_flag_net = fcntl(server_ip_sockfd,F_GETFL,0);
    fcntl(server_ip_sockfd,F_SETFL,O_NONBLOCK|read_block_flag_net);
    pg_flag = 0;
    fprintf(stderr,"\n\rServer waitting for client. Listen port = %d",atoi(server_ip_port));
    fprintf(stderr,"\n\rHit any key then end\n\r");
    while (1) {
      client_ip_len =  sizeof(client_ip_address);
      client_ip_sockfd = accept(server_ip_sockfd,(struct sockaddr *)&client_ip_address,&client_ip_len);
      if (client_ip_sockfd  > 0 ) break;
      if (read(fp1, &ch, 1) >0) {
        close(server_ip_sockfd);
        set_console_mode(0);
        end_process();
      }
    }
    set_console_mode(2);
    /* display client ip,port address */
    ip_list = ntohl(client_ip_address.sin_addr.s_addr);
    ip1 = (ip_list >> 24) & 0xff;
    ip2 = (ip_list >> 16) & 0xff;
    ip3 = (ip_list >> 8)  & 0xff;
    ip4 = ip_list         & 0xff;
    ip_port_list = ntohs(client_ip_address.sin_port);
    fprintf(stderr,"\r\nJust connected !\r\nClient ip:port --> %d.%d.%d.%d:%d\r\n",ip1,ip2,ip3,ip4,ip_port_list);
    fp2 = client_ip_sockfd;
    sprintf(f3,"\r\n***** Welcome to epicon ip_net server just connected ! ******\r\n");
    t_result = write(fp2,&f3,strlen(f3));
    sprintf(f3,"      Your ip:port --> %d.%d.%d.%d:%d\r\n",ip1,ip2,ip3,ip4,ip_port_list);
    t_result = write(fp2,&f3,strlen(f3));
  }
/* ip client process let' try connect to epicon server */
  if (net_flag  && ! server_ip_flag) {
    display_ip_open_msg();
    set_console_mode(2);
    sockfd_ip = socket(AF_INET, SOCK_STREAM, 0);
    address_ip.sin_family = AF_INET;
    address_ip.sin_addr.s_addr = inet_addr(ip_addr);
    address_ip.sin_port = htons(atoi(ip_port));
    len_ip = sizeof(address_ip);
    result_ip = connect(sockfd_ip, (struct sockaddr *)&address_ip, len_ip);
    if(result_ip == -1) {
      perror("\r\ncannot connect");
      fprintf(stderr,"\r%s:%s\r\n",ip_addr,ip_port);
      set_console_mode(0);
      end_process();
    }
    else {
      fp2 = sockfd_ip;
    }
  }
  incount = outcount = fp4 = 0;
  mem_buff = malloc(MSIZE*sizeof(char));
  memset( mem_buff, (int)'\0', sizeof( mem_buff));
  if ( mem_buff == NULL) {
    fprintf(stderr,"Memory not get!! about memory 10M less");
    end_process();
    exit(0);
  }
  if (! net_flag ) {
    if ((fp2 = open(f2,O_RDWR))<0) {
      fprintf(stderr,"%s: %s\n\n",f2,"file open error !!  ");
      end_process();
      exit(0);
    }
  }
  FD_ZERO(&fpp1);
  FD_SET(fp2,&fpp1);
  FD_SET(fp1,&fpp1);
  ck_pid = fork();
  if (ck_pid != 0) {
    pg_flag = 0;
    while (1) {/* check to child process call external program. exp:rz,sz  */
      signal(SIGCHLD,end_process);
      if (net_flag  && ! server_ip_flag) signal(SIGWINCH, win_size_update);
      listen(server_sockfd,5);
      server_flag = fcntl(server_sockfd,F_GETFL,0);
      fcntl(server_sockfd,F_SETFL,O_NONBLOCK|server_flag);
      client_sockfd = accept(server_sockfd,(struct sockaddr *)&client_address,&client_len);
      if ( client_sockfd >0 ) {
        if (read(client_sockfd,&ch3,1) >0) {
          ch3 &= (char)0377;
          if (ch3 == '1') {
            pg_flag = 1;
          }
          else {
            pg_flag = 0;
          }
          if (t_result = write(client_sockfd,"1",1) < 0) {
            perror("write error client socket AF_UNIX");
            raise(SIGTERM);
          }
        }
      }
      if ( pg_flag == 0 ) {
        timeout.tv_sec=0;
        timeout.tv_usec=SEL_TIME_OUT;
        fpp2=fpp1;result=0;
        result=select(fp2+1,&fpp2,(fd_set *)0,(fd_set *)0,&timeout);
        if (result == -1 && errno != EINTR) perror("\r\nselect");
        if (result>0) {
            if (FD_ISSET(fp2,&fpp2)) {
            if (read(fp2, &ch, 1) <= 0) {
              if (net_flag) {
                if (shutdown (fp2,2) < 0) {
                  perror("shutdown socket error");
                }
                if (server_ip_flag) {
                  close(server_ip_sockfd);
                  close(client_ip_sockfd);
                }
              }
              /* kill chid process */
              kill(ck_pid,SIGTERM);
              end_process();
            }
            else {
              if (AZ_flag) {
                zmodem_check(ch,zstep_flag);
              }
            }
          }
        }
      }
    }
  }
  if (ck_pid == 0) {
    if ( SB_flag ) {
      client_socket_write('1');
      send_file(fp2,SB_file); /* send binary file */
      client_socket_write('0');
      close(fp1);
      close(fp2);
      exit(0);
    }
    if ( CM_flag ) {
      client_socket_write('1');
      exec_command(fp1,fp2,CM_file); /* execute external commnad */
      client_socket_write('0');
      close(fp1);
      close(fp2);
      exit(0);
    }
    if ( SF_flag ) {
      send_file_char(fp2,SF_file);/* send charcter file with delay option */
    }
/* signal(SIGKILL,SIG_IGN); I wish catch to SIGKILL */
    while (1) {
      if (getppid() == 1) {/* check parent process die */
         unlink(Epicon_Socket);
         end_process();
      }
      timeout.tv_sec=0;
      timeout.tv_usec=SEL_TIME_OUT;
      fpp2=fpp1;result=0;
      result=select(fp1+1,&fpp2,(fd_set *)0,(fd_set *)0,&timeout);
      if (result == -1 && errno != EINTR) perror("\r\nselect");
      if (result>0) {
        if (FD_ISSET(fp1,&fpp2)) {
          if (read(fp1, &ch, 1) >0) {
            mem_buff[incount++] = ch;
            ch &= (char)0377;
            if ( Echo_flag ) {
              t_result = write(fp1, &ch ,1);
              if (ch == '\r') t_result = write(fp1, "\n" ,1);/* input echo mode */
            }
            if ( Bin_flag ) ck_newline = 0; /* cannot esacpe mode */
            switch( esc_flag ) {
              case 0:
                if ( ck_newline && ch==esc[0] ) {
                  esc_flag = 1;
                }
                break;
              case 1:
                if ( ch != esc[0] ) {
                  check_char_1(); /* check to esc character and function-1 */
                }
                else {
                  esc_flag = 0;
                }
                break;
              case 2:
              case 3:
                check_char_2(); /* check to esc character and function-2 */
                break;
              default:
                break;
            }
            if (ch == '\r'|| ch == '\n'|| ch == '\004' || ch == '\003' || \
                ch == '\377' || ch == '\0') {
                ck_newline = 1 ; esc_flag = 0;
            }
            else {
              ck_newline = 0 ;
            }
          }
        }
      }
      if (incount != outcount) {
        if (incount > MSIZE) {
          perror("\rBuffer Over follow going down !!");
          end_process();
        }
        if ( esc_flag == 0 ) {
          timeout.tv_sec=0;
          timeout.tv_usec=SEL_TIME_OUT;
          fpp2=fpp1;result=0;
          result=select(fp2+1,(fd_set *)0,&fpp2,(fd_set *)0,&timeout);
          if (result == -1 && errno != EINTR) perror("\r\nselect");
          if (result>0) {
            if (FD_ISSET(fp2,&fpp2)) {
          /* fprintf(stderr,"\n\r incount=%d outcount=%d esc_flag=%d\n\r",incount,outcount,esc_flag); */
              ch1 = mem_buff[outcount++];
              t_result = write(fp2,&ch1,1);
              if (ch1 == '\r') {
                if (CR_delay != 0) msleep(CR_delay);
              }
              else {
                if (Char_delay != 0) msleep(Char_delay);
              }
              if (outcount == incount) incount=outcount=0;
            }
          }
        }
      }
    }
  }
}
