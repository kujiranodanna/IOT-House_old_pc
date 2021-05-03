/* epicon_uty.c epicon utitlity program
epicon is Copyright Isamu.Yamauchi 2002-2011.
o 2021.2.24 gcc version 7.5.0 Warning clean up.
o 2017.4.20 read,write warn_unused_result. problem clean up. open with O_CREAT in second argument needs 3 arguments. problem clean up.
o 2011.10.16 Enhanced file locking, changed the function msleep.
o 2010.11.19 warning: ignoring return value of ‘write’, declared with attribute warn_unused_result. clean up.
o 2009.10.31 add -q option quiet mode, clean up.
o 2009.6.8 delete CBAUD IUCLC OLCUC
  add cfsetispeed(&com_port_set, speed) ,cfsetospeed(&com_port_set, speed)
  com_port_set.c_cflag |= CLOCAL|(speed) -> com_port_set.c_cflag |= CLOCAL
o 2008.1.4 easy telnet client is supported, -v option is changed to the line mode.
o 2007.9.29 bugs fix "/var/lock/epicon_socket"+.pid->Epicon_Socket,Auto rz
o 2006.8.17 bugs fix "/var/lock/epicon_socket"->Epicon_Socket
o 2005.12.23 clean up, com_port_use_check(com)--> /var/lock -> /var/tmp
o 2005.9.10 clean up.
o 2004.10.1 add "-v" vt100 imitate console mode
o 2004.10.1 add "-F" send character file with delay
o 2004.9.20 add msleep() for "-d" send charcacter delay,add "-D" send CR delay.
o 2003.10.3 gkermit packet 4000 -> 9000. & clean up
o 2002.12.24 clean up.
o 2002.11.6 clean up.
o 2002.11.2 add send_file process.
o 2002.10.30 oops sz process typo *c->c[64].
o 2002.10.25 add when execute external programs was none.

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
extern int com_port_fd;         /* com_port file descriptor */
extern int net_flag;            /* ip net connect flag */
extern int server_ip_flag;      /* ip server flag */
extern char com_port[64];       /* tty_dev name */
extern int Twostop;             /* 2 stop bit */
extern int Datalen;             /* data bit length */
extern int Parity;              /* parity setting */
extern int Bin_flag;            /* binary mode flag cannot escape */
extern int ip_socket_bufsize;   /* ip socket buffer size */
extern int SB_flag;             /* send binary file flag */
extern int CM_flag;             /* external command option flag */
extern int LOG_flag;            /* log flag */
extern FILE *LOG_fp;            /* log file descriptor */
extern char *argv_redirect;     /* redirect file */
void end_process();             /* call end process */
extern int CON_flag;            /* console flag */
extern unsigned int Char_delay; /* external send charcacter delay value */
extern unsigned int CR_delay;   /* external send CR delay value */
extern char Epicon_Socket[128]; /* external AF_Socket */
extern int Quiet_flag;          /* quiet flag */
int console_save_flag =0;       /* com_port save flag */
int redirect_flag = 0;          /* redirect flag */
static int t_result;            /* tmp int result */
static char *c_result;          /* tmp char result */
static FILE *f_result = 0;      /* tmp FILE result */
void set_console_mode();

int msleep(int msec)
{ /* wait milli second */
  struct timeval timeout;
  timeout.tv_sec = msec / 1000;
  timeout.tv_usec = (msec % 1000) * 1000;
  if (select(0, (fd_set *) 0, (fd_set *) 0, (fd_set *) 0, &timeout) < 0) {
    perror("msleep");
    return -1;
  }
}

void set_console_mode(int i)
{
  FILE *input;
  FILE *output;
  static struct termios initial_console_settings, new_console_settings;
  if (SB_flag || CM_flag) return;
  if (!isatty(fileno(stdout)))  {
    f_result = freopen(argv_redirect,"w",stdout);
    redirect_flag = 1;
  }
  if (!isatty(fileno(stderr)))  {
    f_result = freopen(argv_redirect,"w",stderr);
    redirect_flag = 1;
  }
  if (redirect_flag == 1)  {
    fprintf(stderr,"You are not a terminal.\n");
    return;
  }
  input = fopen("/dev/tty", "r");
  output = fopen("/dev/tty", "w");
  if (!input || !output)  {
    fprintf(stderr, "Unable to open console \n");
    return;
  }
  if (CON_flag == 3 && i == 2) i = 3;
  switch (i)  {
    case 0: /* set console initial_settings */
      tcsetattr(fileno(input), TCSANOW, &initial_console_settings);
      console_save_flag = 0;
      break;
    case 1: /* set raw console_settings */
      if (!console_save_flag)  {
          tcgetattr(fileno(input),&initial_console_settings);
          console_save_flag = 1;
          }
      new_console_settings = initial_console_settings;
      new_console_settings.c_lflag &= ~ICANON;
      new_console_settings.c_lflag &= ~ECHO;
      new_console_settings.c_cc[VMIN] = 0;
      new_console_settings.c_cc[VTIME] = 1;
      new_console_settings.c_lflag &= ~ISIG;
      if(tcsetattr(fileno(input), TCSANOW, &new_console_settings) != 0)  {
          fprintf(stderr,"could not set attributes\n");
      }
      break;
    case 2:
      if (!console_save_flag)  {
        tcgetattr(fileno(input),&initial_console_settings);
        console_save_flag = 1;
      }
      new_console_settings = initial_console_settings;
      new_console_settings.c_oflag &= ~OPOST;
      new_console_settings.c_iflag &= ~(ICRNL|IGNCR|INLCR);
      new_console_settings.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ICRNL|ISIG);
      new_console_settings.c_cc[VMIN]  = 1;
      new_console_settings.c_cc[VTIME] = 0; /* if VMIN == 0; 1/10 * VTIME for wait */
      if(tcsetattr(fileno(input), TCSANOW, &new_console_settings) != 0)  {
          fprintf(stderr,"could not set attributes\n");
      }
      break;
     case 3:
      if (!console_save_flag)  {
        tcgetattr(fileno(input),&initial_console_settings);
        console_save_flag = 1;
      }
      tcgetattr(fileno(input),&new_console_settings);
      new_console_settings.c_lflag &= ~(ISIG);
      if(tcsetattr(fileno(input), TCSANOW, &new_console_settings) != 0)  {
        fprintf(stderr,"could not set attributes\n");
      }
      break;
    }
}

int com_port_unuse(char *com)
{
  char file[128];
  char buf[16];
  char * p;
  struct flock fl;
  int fd,i;
  fl.l_type   = F_RDLCK;  /* F_RDLCK, F_WRLCK, F_UNLCK    */
  fl.l_whence = SEEK_SET; /* SEEK_SET, SEEK_CUR, SEEK_END */
  fl.l_start  = 0;        /* Offset from l_whence         */
  fl.l_len    = 0;        /* length, 0 = to EOF           */
  fl.l_pid    = getpid(); /* our PID                      */
  fd = open(VAR_LOCK, O_RDONLY);
  if( fd < 0 ) return 1;
  close(fd);
  p = strrchr(com, '/');
  strcpy(file, TTY_LOCK);
  if( p ) strcat(file, p + 1);
  else  {
    strcat(file, com);
  }
  for(p = file + 21; *p; p++) if( *p >= 'A' && *p <= 'Z' ) *p = *p - 'A' + 'a';
  if ((fd = open(file, O_RDONLY)) < 0 ) {
    return 1;
  }
  if (fcntl(fd, F_SETLK, &fl) < 0) {
    perror("fcntl");
    return 0;
  }
  int ret = read(fd, buf, sizeof(buf));
  i = atoi(buf);
  fl.l_type = F_UNLCK;
  if (fcntl(fd, F_SETLK, &fl) < 0) {
    perror("fcntl");
    return 0;
  }
  close(fd);
  if ( i == fl.l_pid || getppid() == 1 ) unlink(file);
  return 1;
}

#ifdef HAVE_TERMIOS
struct termios com_port_save, com_port_set;
#else
struct termio  com_port_save, com_port_set;
#endif
static int com_port_mode = -1;
int set_com_port_flag = 0;
void set_com_port_mode(speed)
  int speed;  {
  set_console_mode(2);
  if(!set_com_port_flag)  {
    set_com_port_flag = 1;
    #ifndef HAVE_TERMIOS
      ioctl(com_port_fd, TCGETA, &com_port_save);
    #else
      tcgetattr(com_port_fd, &com_port_save);
    #endif
  }
  com_port_set = com_port_save;
  com_port_set.c_iflag &= ~(IXON|IXANY|IXOFF|ISTRIP|INLCR|ICRNL|IGNCR);
  com_port_set.c_oflag &= ~(ONLCR|OCRNL|ONOCR|ONLRET);
  com_port_set.c_cflag &= ~(PARENB|PARODD|CSTOPB|CSIZE);
  com_port_set.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ISIG);
  com_port_set.c_cflag |=  (CLOCAL|CREAD|IGNPAR);
  com_port_set.c_cflag |=  (Parity|Twostop|Datalen);
  com_port_set.c_oflag = 0;
#ifdef CBAUDEX
  com_port_set.c_cflag &= ~(CBAUDEX);
#endif
  com_port_set.c_cflag &= ~(PARENB);
/*   com_port_set.c_iflag |= IXOFF;  */      /* X/Y modem will clear this */
//  com_port_set.c_cflag |= CLOCAL|(speed); 2009.6.9 delete
  com_port_set.c_cc[VMIN]  = 0;
  com_port_set.c_cc[VTIME] = 1;
#ifndef HAVE_TERMIOS
  ioctl(com_port_fd, TCSETA, &com_port_set);
#else
  cfsetispeed(&com_port_set, speed); // 2009.6.9 add
  cfsetospeed(&com_port_set, speed); // 2009.6.9 add
  tcsetattr(com_port_fd, TCSANOW, &com_port_set);
#endif
}

void initialize_com_port()
{
  set_console_mode(0);
  if(!set_com_port_flag) return;
  #ifndef HAVE_TERMIOS
    ioctl(com_port_fd, TCSETA, &com_port_save);
  #else
    tcsetattr(com_port_fd, TCSANOW, &com_port_save);
  #endif
}

int com_port_use_check(char *com)
{
  char file[128];
  char buf[16];
  char * p;
  struct flock fl;
  int fd;
  fl.l_type   = F_RDLCK;  /* F_RDLCK, F_WRLCK, F_UNLCK    */
  fl.l_whence = SEEK_SET; /* SEEK_SET, SEEK_CUR, SEEK_END */
  fl.l_start  = 0;        /* Offset from l_whence         */
  fl.l_len    = 0;        /* length, 0 = to EOF           */
  fl.l_pid    = getpid(); /* our PID                      */
  fd = open(VAR_LOCK, O_RDONLY);
  if( fd < 0 ) return 1;
  close(fd);
  p = strrchr(com, '/');
  strcpy(file, TTY_LOCK);
  if( p ) strcat(file, p + 1);
  else  {
    strcat(file, com);
  }
  for(p = file + 21; *p; p++) if( *p >= 'A' && *p <= 'Z' ) *p = *p - 'A' + 'a';
  if ((fd = open(file, O_RDONLY)) > 0 ) return 0;
  if ((fd = open(file, O_CREAT | O_RDWR , 0600)) < 0 ) perror("open");
  if (fcntl(fd, F_SETLK, &fl) < 0) {
    perror("fcntl");
    return 0;
  }
  sprintf(buf, "%d\n", getpid());
  int ret = write(fd, buf, strlen(buf));
  fl.l_type = F_UNLCK;
  if (fcntl(fd, F_SETLK, &fl) < 0) {
    perror("fcntl");
    return 0;
  }
  close(fd);
  return 1;
}

void shell_process()
{
  extern char *getenv();
  extern char **environ;
  register int pid, wpid;
  char *shell = (char *)0;
  int status,i;
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  if (shell == (char *)0)  {
    shell = getenv("SHELL");
    if (shell == (char *)0)
    shell = "/bin/sh";
  }
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
  set_console_mode(0);
  if ((pid=fork()) < 0)  {
    (void) signal(SIGQUIT, sig_q);
    (void) signal(SIGINT,  sig_i);
    (void) signal(SIGTERM, sig_t);
    set_console_mode(2);
    fprintf(stderr, "Cannot create lock_file \r\n");
    return;
  }
  if (pid == 0)  {
    signal(SIGQUIT, SIG_DFL);
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM,  SIG_DFL);
    i = execle("/bin/sh", "sh", "-i", (char *)0, environ);
    if (i< 0)  {
      perror("abnormal command end");
      raise(SIGKILL);
    }
    else  {
      set_console_mode(2);
      return;
    }
  }
  pid_t chid_pid;
  chid_pid = wait( &status );
  set_console_mode(2);
  return;
}

void rz(int fp1,int fp2)
{
  int pid,wpid,status,i;
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
  if ( net_flag  &&  server_ip_flag) {
    set_console_mode(1);
  }
  if ((pid = fork()) == -1)  {
    perror("Cannot rzmodem");
    set_console_mode(2);
    return;
  }
  if (pid == 0)  {
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    execlp("rz","rz",(char *)0);
      perror("abnormal command end");
      exit(-1);
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    fprintf(stderr,"\r\n%s\r\n","rz command end");
    if ( net_flag  &&  server_ip_flag) {
      set_console_mode(1);
    }
    else {
      set_console_mode(2);
    }
    return;
  }
}

void rx(int fp1,int fp2)
{
  int pid,wpid,status,i;
  char *filename,c[64];
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
/* requet to recive files name */
  fprintf(stderr,"\n\rinput rx_recive_file name:");
  set_console_mode(0);
  c_result = fgets(c,64,stdin);
  filename = strtok(c,"\n");
  set_console_mode(2);
  if ((pid = fork()) == -1)  {
    perror("Cannot rxmodem");
    set_console_mode(2);
    return;
  }
  if (pid == 0)  {
    signal(SIGQUIT, SIG_DFL);
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM,  end_process);
    set_console_mode(0);
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    execlp("rx","rx","-b",filename,(char *)0);
    perror("abnormal command end");
    exit(-1);
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    set_console_mode(2);
    (void) signal(SIGQUIT, sig_q);
    (void) signal(SIGINT,  sig_i);
    (void) signal(SIGTERM, sig_t);
    return;
  }
}

int rk(int fp1,int fp2)
{
  int pid,wpid,status,i;
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
  if ((pid = fork()) == -1)  {
    perror("Cannot gkermit");
    set_console_mode(2);
    return(-1);
  }
  if (pid == 0)  {
    signal(SIGQUIT, SIG_DFL);
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM,  end_process);
    set_console_mode(0);
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    if (net_flag)  {
      fprintf(stderr,"\n\rG-Kermit CU-1.00, Columbia University, 1999-12-25");
      fprintf(stderr,"\n\rG-Kermit CU-1.00+Display Recive & Send counter, Isamu.Yamauchi 2002-9-20: SYSV.");
      fprintf(stderr,"\n\rEscape back to your local Kermit and give a SEND command.");
      fprintf(stderr,"\n\rKERMIT READY TO RECIVE...");
      execlp("gkermit","gkermit","-r","-X",(char *)0);
      perror("abnormal command end");
      exit(-1);
    }
    else  {
      execlp("gkermit","gkermit","-r",(char *)0);
      perror("abnormal command end");
      exit(-1);
    }
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    set_console_mode(2);
    return 0;
  }
}

void sx(int fp1,int fp2)
{
  int pid,wpid,status,fp,i;
  char *filename,c[64];
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
/* requet to send files name */
  fprintf(stderr,"\n\rinput sx_send_file name:");
  set_console_mode(0);
  c_result = fgets(c,64,stdin);
  filename = strtok(c,"\n");
  set_console_mode(2);
  if ((fp=open(filename,O_RDONLY))<0)  {
    fprintf(stderr,"%s: %s\n\r",filename,"file open error !!  ");
    set_console_mode(2);
    return;
  }
  else  {
    close(fp);
  }
  if ((pid = fork()) == -1)  {
      perror("Cannot sxmodem");
      return;
  }
  if (pid == 0)  {
    signal(SIGQUIT, SIG_DFL);
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM,  end_process);
    set_console_mode(0);
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    execlp("sx","sx","-b","-f",filename,(char *)0);
    perror("abnormal command end");
    exit(-1);
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    set_console_mode(2);
    return;
  }
}

void sz(int fp1,int fp2)
{
  int pid,wpid,status,fp,i;
  char *filename,c[64];
  /* requet to send files name */
  fprintf(stderr,"\n\rinput sz_send_file name:");
  set_console_mode(0);
  c_result = fgets(c,64,stdin);
  filename = strtok(c,"\n");
  if ((fp=open(filename,O_RDONLY))<0)  {
    fprintf(stderr,"%s: %s\n\r",filename,"file open error !!  ");
    if ( net_flag  &&  server_ip_flag) {
      set_console_mode(1);
      return;
    }
    else {
      set_console_mode(2);
      return;
    }
  }
  else  {
           close(fp);
  }
  if ((pid = fork()) == -1)  {
      perror("Cannot szmodem");
      return;
  }
  if (pid == 0)  {
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    execlp("sz","sz",filename,(char *)0);
    perror("abnormal command end");
    exit(-1);
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    fprintf(stderr,"\r\n%s\r\n","sz command end");
    if ( net_flag  &&  server_ip_flag) {
      set_console_mode(1);
    }
    else {
      set_console_mode(2);
    }
    return;
  }
}

int sk(int fp1,int fp2)
{
  int pid,wpid,status,fp,i;
  char *filename,c[64];
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
  /* requet to send files name */
  set_console_mode(0);
  fprintf(stderr,"\n\rinput gkermit_send_file name:");
  c_result = fgets(c,64,stdin);
  filename = strtok(c,"\n");
  if ((fp=open(filename,O_RDONLY))<0)  {
    fprintf(stderr,"%s: %s\n\r",filename,"file open error !!  ");
    set_console_mode(2);
    return -1;
  }
  else  {
    close(fp);
  }
  if ((pid = fork()) == -1)  {
    perror("Cannot gkermit");
    return -1;
  }
  if (pid == 0)  {
    signal(SIGQUIT, SIG_DFL);
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM,  end_process);
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    if (net_flag)  {
      fprintf(stderr,"\n\rG-Kermit CU-1.00, Columbia University, 1999-12-25");
      fprintf(stderr,"\n\rEscape back to your local Kermit and give a RECIVE command.");
      fprintf(stderr,"\n\rKERMIT READY TO SEND...");
      execlp("gkermit","gkermit","-X","-b","5","-e","9000","-s",filename,(char *)0);
    }
    else  {
    execlp("gkermit","gkermit","-b","5","-s",filename,(char *)0);
    perror("abnormal command end");
    exit(-1);
    }
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    set_console_mode(2);
    return 0;
  }
}

void exec_command(int fp1,int fp2,char *command_line)
{
  int pid, wpid;
  int status,i;
  char *args[10],*c,*p;
  sigtype (*sig_q)(), (*sig_i)(), (*sig_t)();
  sig_q = signal(SIGQUIT, SIG_IGN);
  sig_i = signal(SIGINT,  SIG_IGN);
  sig_t = signal(SIGTERM,  SIG_IGN);
  for (i = 0;i < 10;i++) args[i] ='\0';
  c = strtok(command_line," ");i = 0;
  while((p = strtok(NULL," ")) != NULL)  {
    args[i++] = p;
  }
  if ((pid = fork()) == -1)  {
    perror("Cannot exec_command");
    return;
  }
  if (pid == 0)  {
    signal(SIGQUIT, SIG_DFL);
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM,  end_process);
    t_result = dup2(fp2,0);
    t_result = dup2(fp2,1);
    execlp(c,c,args[0],args[1],args[2],args[3],args[4],args[5],args[6],\
    args[7],args[8],args[9],(char *)0);
    perror("abnormal command end");
    exit(-1);
  }
  if (pid != 0)  {
    pid_t chid_pid;
    chid_pid = wait( &status );
    return;
  }
}

void send_file_char(fp2,s_file) /* send charcter file with delay opiotion */
int fp2;
char *s_file;
{
  long int outcount;
  int w_flag,fp3,fp4,ct,result,result_fp;
  char sch;
  struct timeval timeout;
  fd_set fpp1,fpp2;
  set_console_mode(2);
  if ((fp3=open(s_file,O_RDONLY))<0)  {
    if (! SB_flag)  {
      fprintf(stderr,"%s: %s\n\r",s_file,"file open error !!  ");
      return;
    }
    else  {
      set_console_mode(0);
      exit(1);
    }
  }
  if (! SB_flag) fprintf(stderr,"\n\r");
  outcount=fp4=0;
  FD_ZERO(&fpp1);
  FD_SET(fp2,&fpp1);
  while(read(fp3,&sch,1) > 0)  {
    outcount++;
    w_flag = 1;
    while(w_flag)  {
      timeout.tv_sec=0;
      timeout.tv_usec=SEL_TIME_OUT;
      fpp2 = fpp1;
      result=0;
      result=select(fp2+1,(fd_set *)0,&fpp2,(fd_set *)0,NULL );
      if (result == -1 && errno != EINTR) perror("\n\rselect");
      if (result>0)  {
        if (FD_ISSET(fp2,&fpp2))  {
          t_result = write(fp2,&sch,1);
          if (sch == '\r') {
            if (CR_delay != 0) msleep(CR_delay);
            }
          else {
            if (Char_delay != 0) msleep(Char_delay);
          }
          w_flag = 0;
        }
      }
    }
  }
  close(fp3);
  return;
}

void send_file(fp2,s_file) /* send file binary */
int fp2;
char *s_file;
{
  long int outcount;
  int w_flag,fp3,fp4,ct,result,result_fp;
  char sch,sendbuf[R_WSIZE+1];
  struct timeval timeout;
  fd_set fpp1,fpp2;
  set_console_mode(2);
  if ((fp3=open(s_file,O_RDONLY))<0)  {
    if (! SB_flag)  {
      fprintf(stderr,"%s: %s\n\r",s_file,"file open error !!  ");
      return;
    }
    else  {
      set_console_mode(0);
      exit(1);
    }
  }
  if (! SB_flag) fprintf(stderr,"\n\r");
  outcount=fp4=0;
  FD_ZERO(&fpp1);
  FD_SET(fp2,&fpp1);
  while((ct=read(fp3,sendbuf,R_WSIZE)) > 0)  {
    outcount+=ct;
    w_flag = 1;
    while(w_flag)  {
      timeout.tv_sec=0;
      timeout.tv_usec=SEL_TIME_OUT;
      fpp2 = fpp1;
      result=0;
      result=select(fp2+1,(fd_set *)0,&fpp2,(fd_set *)0,NULL );
      if (result == -1 && errno != EINTR) perror("\r\nselect");
      if (result>0)  {
        if (FD_ISSET(fp2,&fpp2))  {
          t_result = write(fp2,sendbuf,ct);
          fp4++;
          if (!SB_flag )  {
            if (fp4 == 1)  {
              fprintf(stderr,"Send_count=%ld       \r",outcount);
              fp4 = 0;
            }
          }
          w_flag = 0;
        }
      }
    }
  }
  close(fp3);
  if ( SB_flag ) sleep(2);/* Wait for send to remaing "/dev/ttySX"'s buffer */
  fprintf(stderr,"\r\nSend_file_binary_mode complete send bytes=%ld \r\n",outcount);
  return;
}

void client_socket_write(char ch)
{
  int check_sockfd,check_result = 1;
  int check_len,check_flag;
  char ch1;
  struct sockaddr_un check_address;
  check_sockfd = socket(AF_UNIX,SOCK_STREAM,0);
  check_address.sun_family = AF_UNIX;
  strcpy(check_address.sun_path,Epicon_Socket);
  check_len = sizeof(check_address);
  while (check_result != 0 ) {
    check_result = connect(check_sockfd,(struct sockaddr *)&check_address,check_len);
  }
  if (check_result < 0)  {
    perror("write error server socket AF_UNIX");
    exit(1);
  }
  if ( net_flag ) {
    t_result = write(check_sockfd,&ch,1);
    return;
  }
  else t_result = write(check_sockfd,&ch,1);
  while (check_flag=read(check_sockfd,&ch1,1) <= 0);
  ch1 &= (char)0377;
  if (ch1  != '1' )  {
    fprintf(stderr,"\n\rserver AF_UNIX NAK reply");
    end_process();
  }
  close(check_sockfd);
  return;
}

/* no print except for reading character */
void display_console(int fp,char ch)
{
  char c[10];
  ch &= 0xFF;
  if ( LOG_flag ) fputc(ch,LOG_fp);
  if ( Bin_flag )  {
    t_result = write(fp,&ch,1);
    return;
  }
  switch (ch)  {
    case '\377': t_result = write(fp, "\\d",1); break;
    case '\n': sprintf(c,"\n\r"); t_result = write(fp,&c,2); break;
    case '\r': t_result = write(fp,&ch,1); break;
    case '\t': t_result = write(fp,&ch,1); break;
    case '\b': t_result = write(fp,&ch,1); break;
    case '\033': t_result = write(fp,&ch,1); break;
    case '\014': t_result = write(fp,&ch,1); break;
    case '\007': t_result = write(fp,&ch,1); break;
    default:
    if (ch >= ' ' ) { t_result = write(fp,&ch,1); break; }
  }
}

void display_ip_open_msg()
{
if (!Quiet_flag) {
  fprintf(stderr,"\r\n** Welcome to epicon Version-%s Copyright Isamu Yamauchi %s **",VER,DAY);
  if (!Bin_flag) fprintf(stderr,"\r\n      exec shell         ~! ");
  if (!Bin_flag) fprintf(stderr,"\r\n      send binary files  ~f");
  if (!Bin_flag) fprintf(stderr,"\r\n      call rz,sz,sx,rx   ~rz,~sz,~sx,~rx");
  if (!Bin_flag) fprintf(stderr,"\r\n      call kermit        ~sk,~rk");
  if (!Bin_flag) fprintf(stderr,"\r\n      external command   ~C ");
  if (!Bin_flag) fprintf(stderr,"\r\n      exit               ~. \r\n\r\n");
  if (Bin_flag)  fprintf(stderr,"\r\n      Do exit is kill own process or remote log out\r\n");
 }
}
