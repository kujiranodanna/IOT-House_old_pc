/*
Copyright Isamu.Yamauchi 2013-2017. update 2015.8.21
This program uses the libpiface. thanks.
pepopiface.c is controls for Raspberry pi PiFace Digital I/O Expansion Board

o 2013.07.20 ver0.1, 1st release.
o 2013.9.5 ver0.2 , to match the input state of piface, modified to highlight the port data.
o 2014.12.21 ver0.3, add an exclusive processing.
o 2014.12.26 ver0.4, read & write command return value bug fixes.
o 2015.1.11 ver0.5 , add without writing timer option to continuous processing to read.
o 2015.8.21 Ver0.6 , Change the upper limit of 60 seconds timer to 5 minutes.
*/

/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/sem.h>
/* <sys/ipc.h> is include in sem.h
#include <sys/ipc.h>
*/
#include <libpiface-1.0/pfio.h>
#define DEBUG
#undef  DEBUG /* Comment out the case when debugging */
#define READ 'R'
#define WRITE 'W'
#define VER "0.6"
#define DAY "compiled:"__DATE__
#define LOCK -1
#define UNLOCK 1
#define PIFACE_SEMA "/var/run/pepopiface.semaphore"
#ifdef DEBUG
#define DEBUG_WAIT 10000 /* debug wait timer, ms */
#endif

int get_myval(int sid) {
  union semun {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
    struct seminfo *__buf;
    void *__pad;
  };
  union semun my_semun;
  uint16_t d_result = semctl(sid, 0, GETVAL, my_semun);
  if (d_result == -1) {
    perror("semctl: GETVAL failed");
    exit(1);
  }
#ifdef DEBUG
  printf("%s%d\n","val=",d_result);
#endif
  return(d_result);
}

int get_sempid(int sid) {
  union semun {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
    struct seminfo *__buf;
    void *__pad;
  };
  union semun my_semun;
  pid_t sem_pid;
  sem_pid = semctl(sid, 0, GETPID, my_semun);
  if (sem_pid == -1) {
    perror("semctl: GETPID failed");
    exit(1);
  }
  return(sem_pid);
}

void usage() {
  union semun {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
    struct seminfo *__buf;
    void *__pad;
  };
  union semun my_semun;
  FILE *fdsem;
  uint16_t d_result;
  int mysemun_id;
  key_t key;
#ifdef DEBUG
  pid_t my_pid, sem_pid;
  my_pid = getpid();
#endif
  fprintf(stderr,"\r\n** Welcome to pepopiface Version-%s Copyright Yamauchi.Isamu %s **",VER,DAY);
  fprintf(stderr,"\n\rusage:pepopiface port:0-8 [0|1] [timer:0-300000ms]\n\r");
  fdsem = fopen(PIFACE_SEMA,"r");
  if (fdsem != NULL) {
    if ((key = ftok(PIFACE_SEMA, 'S')) == -1) {
      perror("ftok: failed");
      exit(1);
    }
/* Creating of the semaphore */
  mysemun_id = semget(key, 1, 0666 | IPC_CREAT);
  if (mysemun_id == -1) {
    perror("semget: semget get failed");
    exit(1);
  }
#ifdef DEBUG
  sem_pid = get_sempid(mysemun_id);
  fprintf(stderr,"\r\nmy_pid:%d, sem_pid:%d\r\n",my_pid, sem_pid);
#endif
/* remove of the semaphore */
  my_semun.val = 1;
  if (semctl(mysemun_id , 0, IPC_RMID, my_semun) == -1) {
    perror("semctl: semaphore remove failed");
    exit(1);
  }
/* semaphore of the file delete */
  unlink(PIFACE_SEMA);
}
  fdsem = fopen(PIFACE_SEMA,"r");
  if (fdsem == NULL) {
/* File creation of semaphore */
    fdsem = fopen(PIFACE_SEMA,"w");
    if (fdsem == NULL) {
      perror("fopen: failed");
      exit(1);
    }
#ifdef DEBUG
    fprintf(stderr,"\r\n** %s file creation succeed of semaphore! **\r\n",PIFACE_SEMA);
#endif
    fclose(fdsem);
  }
  if ((key = ftok(PIFACE_SEMA, 'S')) == -1) {
    perror("ftok: failed");
    exit(1);
  }
/* Creating of the semaphore */
  mysemun_id = semget(key, 1, 0666 | IPC_CREAT);
  if (mysemun_id == -1) {
    perror("semget: semget Initialization failed");
    exit(1);
  }
  d_result = get_myval(mysemun_id);
  if (d_result == 0) {
/* Initialization of the semaphore */
    my_semun.val = 1;
    if (semctl(mysemun_id, 0, SETVAL, my_semun) == -1) {
      perror("semctl: Initialization failed");
      exit(1);
    }
#ifdef DEBUG
    fprintf(stderr,"\r\n** Initialization succeed of semaphore! **\r\n");
#endif
  }
}

/* The following are defined in the "<sys/sem.h>" */
//  struct sembuf {
//    unsigned short  sem_num;        /* semaphore index in array */
//    short           sem_op;         /* semaphore operation */
//    short           sem_flg;        /* operation flags */
//  };

void mysem_lock(int sid){
  struct sembuf mysemop[1];
  mysemop[0].sem_num = 0;
  mysemop[0].sem_op = LOCK;
  mysemop[0].sem_flg = SEM_UNDO;
  if(semop(sid, mysemop, 1) == -1){
    perror("semop: semop lock-1 failed");
    exit(1);
  }
#ifdef DEBUG
  printf("semop_lock:");get_myval(sid);
#endif
}

void mysem_unlock(int sid){
  struct sembuf mysemop[1];
  mysemop[0].sem_num = 0;
  mysemop[0].sem_op = UNLOCK;
  mysemop[0].sem_flg = SEM_UNDO;
  if(semop(sid, mysemop, 1) == -1){
    perror("semop: semop unlock failed");
    exit(1);
  }
#ifdef DEBUG
  printf("sem_unlock:");get_myval(sid);
#endif
}

int msleep(int ms) {
  if ( ms > 0 ) {
    struct timeval timeout;
    timeout.tv_sec = ms / 1000;
    timeout.tv_usec = (ms % 1000) * 1000;
    if (select(0, (fd_set *) 0, (fd_set *) 0, (fd_set *) 0, &timeout) < 0) {
      perror("msleep");
      return 1;
    }
  }
  return 0;
}

int main(int argc, char *argv[]) {
  int port = 0;
  int data = 0;
  int invert_data = 0;
  int wait_time = 1;
  int port_timer = 0;
  char rw_flag = READ;
  int i = 0x0000;
  int j = 0x0000;
  uint8_t s_result = 0x00;
  uint16_t d_result = 0x0000;
  int mysem_id = 0;
  key_t key;
  char patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
  if ( argc > 4 || argc < 2  ) {
    usage();
    exit(1);
  }
  else {
    port = atoi(argv[1]); 
    if ( port > 8 || port < 0 ) {
      usage();
      exit(1);
    }
  }
  
  if ( argc == 3 ||  argc == 4) {
    data = atoi(argv[2]);
    if ( data != 0 && data != 1 ) {
      usage();
      exit(1);
    }
    else {
      rw_flag = WRITE;
    }
  }
  if ( argc == 4 ) {
    port_timer = atoi(argv[3]);
    if ( port_timer < 0 || port_timer > 300000 ) {
      usage();
      exit(1);
    }
    rw_flag = WRITE;
  }
  if ((key = ftok(PIFACE_SEMA, 'S')) == -1) {
    perror("ftok: failed");
    fprintf(stderr,"\r\n** Please start first time is with no options **\r\n");
    exit(1);
  }  
/* Creating of the semaphore */
  mysem_id = semget(key, 1, 0666 | IPC_CREAT);
  if (mysem_id == -1) {
    perror("semget: semget Initialization failed");
    exit(1);
  }
  mysem_lock(mysem_id);
  if (pfio_init() < 0) exit(-1);
  mysem_unlock(mysem_id);
  if ( rw_flag == WRITE ) {
/* port write */
    mysem_lock(mysem_id);
    pfio_digital_write(port, data);
    if ( port_timer > 0 ) {
      mysem_unlock(mysem_id);
      if ( data == 0 ) {
        invert_data = 1;
      }
      else {
        invert_data = 0;
      }
      msleep(port_timer);
      mysem_lock(mysem_id);
      pfio_digital_write(port, invert_data);
    }
    msleep(wait_time);
    s_result = pfio_read_output() & patterns[port];
    mysem_unlock(mysem_id);
    s_result = s_result >> port;
    fprintf(stderr,"%1x",s_result);
  }
  if ( rw_flag == READ ) {
/* port read */
    mysem_lock(mysem_id);
    if ( port == 8 ) {
      i = pfio_read_input() & 0xffff;
      j = pfio_read_output() << 8;
      j = j & 0xffff;
      i = i ^ 0xffff;
      i = i & 0x00ff;
      d_result = j | i;
      fprintf(stderr,"%04x",d_result);
    }
    else {
      s_result = (pfio_read_input() & patterns[port]);
      s_result = s_result >> port;
      s_result = s_result ^ 0x01;
      fprintf(stderr,"%1x",s_result);
    }
    mysem_unlock(mysem_id);
  }
  msleep(wait_time);
  mysem_lock(mysem_id);
  pfio_deinit();
  mysem_unlock(mysem_id);
#ifdef DEBUG
  mysem_lock(mysem_id);
  fprintf(stderr,"lock & wait %d milliseconds\n",DEBUG_WAIT);
  msleep(DEBUG_WAIT);
  mysem_unlock(mysem_id);
#endif
  exit(0);
}
