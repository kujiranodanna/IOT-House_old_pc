/*
 * pepocp2112ctl.c  is written for cp2112_gpio and i2c device.
 * For example to used humidity & temperature sensor of am2320.
 * for CP2112 Single-Chip HID USB to SMBus Master Bridge see following
 * https://www.silabs.com/documents/public/data-sheets/cp2112-datasheet.pdf
 * Copyright (c) 2018 Isamu Yamauchi
 * Released under GPLv3 license, see http://www.gnu.org/licenses/gpl-3.0.html.
 *
 * Depends on the HID API multi-platform library, available from
 * http://www.signal11.us/oss/hidapi/.
 *
 * Build with
 *   gcc -Wall -g -o pepocp2112ctl pepocp2112ctl.c `pkg-config hidapi-hidraw --libs`
 *   or
 *   gcc -Wall -o pepocp2112ctl pepocp2112ctl.c -lhidapi-hidraw
 *
 * 2018.1.6 Ver0.1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include "hidapi/hidapi.h"
#include <sys/types.h>
#include <sys/time.h>
#include <sys/sem.h>

/* <sys/ipc.h> is include in sem.h
#include <sys/ipc.h>
*/
#define DEBUG
#undef  DEBUG /* Comment out the case when debugging */
#define DEMO
#undef  DEMO /* Comment out the case when DEMO */
#define READ 'R'
#define WRITE 'W'
#define VER "0.1"
#define DAY "compiled:"__DATE__
#define LOCK -1
#define UNLOCK 1
#define CP2112_SEMAPHORE "/var/run/pepocp2112.semaphore"
#ifdef DEBUG
#define DEBUG_WAIT 10000 /* debug wait timer, ms */
#endif

#define AM2320_SLAVE_ADDR 0x5C
#define CP2112_RESET 0x01
#define CP2112_GETSET_GPIO_CONFIG 0x02
#define CP2112_GET_GPIO 0x03
#define CP2112_SET_GPIO 0x04
#define CP2112_GETSET_SMBUS_CONFIG 0x06
#define CP2112_DATA_READ_REQ 0x10
#define CP2112_DATA_WRITE_READ_REQ 0x11
#define CP2112_DATA_READ_FORCE_SEND 0x12
#define CP2112_DATA_READ_RESPONS 0x13
#define CP2112_DATA_WRITE 0x14
#define CP2112_TRANSFER_STATUS_REQ 0x15
#define CP2112_TRANSFER_STATUS_RESP 0x16
#define CP2112_TRANSFER_CANCEL 0x17

/*
 *  msleep function
 */
int msleep(int ms){
	struct timeval timeout;
	timeout.tv_sec = ms / 1000;
	timeout.tv_usec = (ms % 1000) * 1000;
	if (select(0, (fd_set *) 0, (fd_set *) 0, (fd_set *) 0, &timeout) < 0) {
		perror("msleep");
		return -1;
	}
	return 0;
}

/*
 *  CRC16 function
 */ 
unsigned short crc16( unsigned char *ptr, unsigned char len ) {
  unsigned short crc = 0xFFFF;
  unsigned char i;
  
  while( len-- )
  {
	crc ^= *ptr++;
	for( i = 0; i < 8; i++ ) {
	  if( crc & 0x01 ) {
	crc >>= 1;
	crc ^= 0xA001;
	  } else {
	crc >>= 1;
	  }
	}
  }
  
  return crc;
}

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
	fprintf(stderr,"\r\n** Welcome to pepocp2112ctl Version-%s Copyright Yamauchi.Isamu %s **",VER,DAY);
	fprintf(stderr,"\n\rusage:pepocp2112ctl port:0-9 [0|1] [timer:0-300000ms]\n\r");
}

void create_semaphore() {
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
	fdsem = fopen(CP2112_SEMAPHORE,"r");
	if (fdsem != NULL) {
		if ((key = ftok(CP2112_SEMAPHORE, 'S')) == -1) {
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
	unlink(CP2112_SEMAPHORE);
}
	fdsem = fopen(CP2112_SEMAPHORE,"r");
	if (fdsem == NULL) {
/* File creation of semaphore */
	fdsem = fopen(CP2112_SEMAPHORE,"w");
	if (fdsem == NULL) {
		perror("fopen: failed");
		exit(1);
	}
#ifdef DEBUG
	fprintf(stderr,"\r\n** %s file creation succeed of semaphore! **\r\n",CP2112_SEMAPHORE);
#endif
	fclose(fdsem);
	}
	if ((key = ftok(CP2112_SEMAPHORE, 'S')) == -1) {
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
//	struct sembuf {
//		unsigned short	sem_num;				/* semaphore index in array */
//		short					 sem_op;				 /* semaphore operation */
//		short					 sem_flg;				/* operation flags */
//	};

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

/* reset cp2112 */
static int cp2112_reset(hid_device *hd) {
	unsigned char buf[2] = { CP2112_RESET ,0x01 ,};
	int ret = hid_send_feature_report(hd, buf, sizeof(buf));
/*	if (ret < 0) {
		fprintf(stderr, "reset hid_send_feature_report() failed: %ls\n",
			hid_error(hd));
		return -1;
	}
	msleep(5000); 
*/	return ret;
}


/* configure cp2112 GPIO pins */
static int cp2112_config_gpio(hid_device *hd) {
	unsigned char buf[5] = { CP2112_GETSET_GPIO_CONFIG, };
	int ret = hid_get_feature_report(hd, buf, sizeof(buf));
	if (ret < 0) {
		fprintf(stderr, "hid_get_feature_report() failed: %ls\n",
			hid_error(hd));
		return -1;
	}
// DEMO
#ifdef DEMO
	buf[1] = 0x7f; /* output direction for GPIOs 0, 1, 2, 3, 4, 5, 6 */
	buf[2] = 0x7f; /* push-pull for GPIOs 7 */
	buf[3] = 0x00; /* no special functions, i.e. use pins as GPIO */
	buf[4] = 0xff; /* clock driver value not use */
#else
	buf[1] = 0x0f; /* output direction for GPIOs 0, 1, 2, 3  input 4, 5, 6, 7 */
	buf[2] = 0x0f; /* push-pull for GPIOs 0, 1, 2, 3 */
	buf[3] = 0x00; /* no special functions, i.e. use pins as GPIO */
	buf[4] = 0xff; /* clock driver value not use */
#endif
/*	for example */ 
//	buf[1] = 0x8b; /* output direction for GPIOs 0, 1, 3, 7 */
//	buf[2] = 0x8b; /* push-pull for GPIOs 0, 1, 3, 7 */
//	buf[3] = 0x00; /* no special functions, i.e. use pins as GPIO */
	ret = hid_send_feature_report(hd, buf, sizeof(buf));
	if (ret < 0) {
		fprintf(stderr, "hid_send_feature_report() failed: %ls\n",
			hid_error(hd));
		return -1;
	}
	return 0;
}

/* set cp2112 GPIO pins */
static int cp2112_set_gpio(hid_device *hd, unsigned char mask, unsigned char value) {
	unsigned char buf[3] = { CP2112_SET_GPIO, };
	buf[1] = value; /* set GPIO pins: 0 .. turn LED on, 1 .. off */
	buf[2] = mask; /* mask of pins to set, ignore others */

	int ret = hid_send_feature_report(hd, buf, sizeof(buf));
	if (ret < 0) {
		fprintf(stderr, "hid_send_feature_report() failed: %ls\n",
			hid_error(hd));
		return -1;
	}

	return 0;
}

/* get cp2112 GPIO pins */
static unsigned char cp2112_get_gpio(hid_device *hd, unsigned char mask, unsigned char value) {
	unsigned char buf[3] = { CP2112_GET_GPIO, };
	buf[1] = value; /* set GPIO pins: 0 .. turn LED on, 1 .. off */
	buf[2] = mask; /* mask of pins to set, ignore others */

	int ret = hid_get_feature_report(hd, buf, sizeof(buf));
	if (ret < 0) {
		fprintf(stderr, "hid_get_feature_report() failed: %ls\n",
		hid_error(hd));
		return -1;
	}
	
	return buf[1] ;
}
unsigned char gpio_write(hid_device *hd, int port ,int value)  {
/* port no 0-7 write */
	char patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
	unsigned char mask = patterns[port];
	int write_value = value << port;
	int ret = cp2112_set_gpio(hd, mask ,write_value);
	if ( ret > 0 )  {
		ret = value;
	}
	return ret;
}

unsigned char gpio_read(hid_device *hd) {
/* port no 0-7 read */
	unsigned char mask = 0xff;
	unsigned char value = 0xff;
	int ret = cp2112_get_gpio(hd, mask ,value);
	if ( ret > 0 )  {
		ret = ret & mask;
	}
	return ret;
}

/* configures cp2112 to automatically send read data, see AN495 section 4.6 */
static int cp2112_set_auto_send_read(hid_device *hd, int on_off) {
	unsigned char buf[14] = { CP2112_GETSET_SMBUS_CONFIG, };
	int ret = hid_get_feature_report(hd, buf, sizeof(buf));
	if (ret < 0) {
		fprintf(stderr, "hid_get_feature_report() failed: %ls\n",
			hid_error(hd));
		return -1;
	}
	buf[6] = on_off;
	ret = hid_send_feature_report(hd, buf, sizeof(buf));
	if (ret < 0) {
		fprintf(stderr, "hid_send_feature_report() failed: %ls\n",
			hid_error(hd));
		return -1;
	}
	return 0;
}

/* see if the cp2112 is idle */
static int cp2112_is_idle(hid_device *hd) {
	unsigned char status_req[2] = { CP2112_TRANSFER_STATUS_REQ, 0x01, };
	if (hid_write(hd, status_req, sizeof(status_req)) < 0) {
		fprintf(stderr, "hid_write() failed: %ls\n",
			hid_error(hd));
		return -1;
	}

	unsigned char status_resp[7] = { 0x00, };
	if (hid_read(hd, status_resp, sizeof(status_resp)) < 0) {
		fprintf(stderr, "hid_read() failed: %ls\n",
			hid_error(hd));
		return -1;
	}

	if (status_resp[0] == CP2112_TRANSFER_STATUS_RESP) {
		if (status_resp[1] == 0x00) /* is idle? */
			return 1;
	}
	return 0;
}

int am2320_measured(hid_device *hd) {
	int i = 0;
	int ret = -1;
	int retry_cnt = 0;
	int timeout = 1500; // 1500 milisecons
	int wait_time = 50; // 50 milisecons
	unsigned short crc_m, crc_s;
	unsigned char crc_tmp[6];
	unsigned char buf_in[12];
	unsigned char buf_out[12];
	memset( buf_in, 0x00, sizeof(buf_in) );
	memset( buf_out, 0x00, sizeof(buf_out) );
	buf_out[0] = CP2112_DATA_WRITE;
/* 0x5c is the 7-bit SMBus slave address of the am2320 */
	buf_out[1] = AM2320_SLAVE_ADDR<<1;
/* These three steps can be completed by the sensor and writes reads
https://www.silabs.com/community/interface/knowledge-base.entry.html/2014/10/21/cp2112_returns_incor-Dbhn
*/
/* am2320 wake up */
	while ( retry_cnt < 5 )  {
		ret = cp2112_is_idle(hd);
		if ( ret != 1 )  {
			retry_cnt++;
			msleep(wait_time);
			continue;
		}
		ret = hid_write(hd, buf_out, 2);
		if ( ret != 2 ) {
			retry_cnt++;
			continue;
		}
		msleep(wait_time);
/* write a buf_out byte to the am2320 */
		buf_out[0] = CP2112_DATA_WRITE;
		buf_out[2] = 0x03; // writes length
		buf_out[3] = 0x03; // measured am2320 specification 1
		buf_out[4] = 0x00; // specification 2
		buf_out[5] = 0x04; // specification 3
//		ret = hid_write(hd, buf_out, 6);
		ret = hid_send_feature_report(hd, buf_out, 6);
		if ( ret != 6 ) {
			retry_cnt++;
			continue;
		}
		msleep (wait_time);
/* read a data byte from register of the am2320 */
		buf_out[0] = CP2112_DATA_READ_REQ;
		buf_out[2] = 0x00; // reads length_high
		buf_out[3] = 8; // reads length_low
//		ret = hid_write(hd, buf_out, 4);
		ret = hid_send_feature_report(hd, buf_out, 4);
		if (ret < 0) {
				fprintf(stderr, "hid_write() failed: %ls\n" ,
				hid_error(hd));
				return -1;
		}
		msleep (wait_time);
		buf_out[0] = CP2112_DATA_READ_FORCE_SEND;
		buf_out[2] = 0x00; // reads length_high
		buf_out[3] = 8; // reads length_low
//		ret = hid_write(hd, buf_out, 4);
		ret = hid_send_feature_report(hd, buf_out, 4);
		if (ret < 0) {
				fprintf(stderr, "hid_write() failed: %ls\n" ,
				hid_error(hd));
				return -1;
		}
/*
	fprintf(stderr, "\nCP2112_DATA_READ_FORCE_SEND ret:%d\n",ret);
	fprintf(stderr, "\nbuf_out dump start\n");
	for ( i = 0; i < sizeof(buf_out);  i++ )  {
		fprintf(stderr, "%d:%02x ",i ,buf_out[i]);
	}
	fprintf(stderr, "\nbuf_out dump end\n");
*/	
		msleep(wait_time);
		buf_in[0] = CP2112_DATA_READ_RESPONS;
		buf_in[1] = AM2320_SLAVE_ADDR<<1;
		buf_in[2] = 0x00; // reads length_high
		buf_in[3] = 8; // reads length_low
		ret = hid_read_timeout(hd, buf_in, 11, timeout);
//		ret = hid_get_feature_report(hd, buf_out, 11);
/*	fprintf(stderr, "\nbuf_in dump start\n");
	for ( i = 0; i < sizeof(buf_in);  i++ )  {
		fprintf(stderr, "%d:%02x ",i ,buf_in[i]);
	}
	fprintf(stderr, "\nbuf_in dump end\n");
*/		if (ret < 0) {
				fprintf(stderr, "hid_read() failed: %ls\n",
				hid_error(hd));
				return -1;
		}
		if (buf_in[1] == 02) break;
			msleep(wait_time);
			retry_cnt++;
	}
	if ( retry_cnt > 5 ) {
		fprintf(stderr, "-1");
		return -1;
	}
/*
buf_in[0]:hid_report ID->0x13
buf_in[1]:hid_status 0x00->Idle ,x01->Busy ,x020->Complete
,buf_in[2]:hid_read_length
Following_AM2320_Read_Data
buf_in[3]:Function Code ,buf_in[4]:data length ,buf_in[5]:high humidity ,buf_in[6]:low humidity ,
buf_in[7]:high temperature ,buf_in[8]:low temperature ,buf_in[9]:CRC checksum low byte ,
buf_in[10]:CRC checksum low byte
0x03(Function Code)+0x04(data length)+0x03(high humidity)+0x39(low humidity) + 
0x01 (high temperature) +0x15(low temperature)+0xE1(CRC checksum low byte) + 0xFE 
(CRC checksum high byte); 
Therefore: 0339H = 3×256 +3×16 +9 = 825 => humidity = 825÷10 = 82.5% RH; 
0115H = 1×256 +1×16 +5 = 277 => temperature = 277÷10 = 27.7 ℃
*/
	for ( i = 0; i < sizeof(crc_tmp);  i++ ) crc_tmp[i] = buf_in[i + 3];
	crc_m = crc16(crc_tmp, sizeof(crc_tmp));
	crc_s = (buf_in[10] << 8) + buf_in[9];
	if ( crc_m != crc_s ) {
		fprintf( stderr, "am2320: CRC16 does not match\n" );
		return -1;
	} else {
		int humidity = buf_in[5] * 256 + buf_in[6];
		int humidity_high = humidity / 10;
		int humidity_low = humidity % 10;
		int temperature = buf_in[7] * 256 + buf_in[8];
		int temperature_high = temperature / 10;
		int temperature_low = temperature % 10;
		fprintf(stderr ,"%d.%d %d.%d",temperature_high ,temperature_low ,humidity_high ,humidity_low);
		return 0;
	}
}

int main(int argc, char *argv[]) {
	int port = 0;
	int data = 0;
	int invert_data = 0;
	int port_timer = 0;
	char rw_flag = READ;
	unsigned char s_result = 0x00;
	int mysem_id = 0;
	key_t key;
	FILE *fdsem;
	char patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
	if ( argc > 4 || argc < 2	) {
	usage();
	exit(1);
	}
	else {
		port = atoi(argv[1]); 
		if ( port > 10 || port < 0 ) {
			usage();
			exit(1);
		}
	}
	
	if ( argc == 3 ||	argc == 4) {
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

	if (hid_init() < 0) {
		fprintf(stderr, "hid_init() failed, exit.\n");
		exit(EXIT_FAILURE);
	}
	/* open Silabs CP2112 USB reference design by USB product:vendor id */
	hid_device *hd = hid_open(0x10c4, 0xea90, NULL);
	if (hd == NULL) {
		fprintf(stderr, "hid_open() failed\n");
		unlink(CP2112_SEMAPHORE);
		exit(EXIT_FAILURE);
	} else {
		fdsem = fopen(CP2112_SEMAPHORE,"r");
		if (fdsem == NULL) {
			create_semaphore();
			cp2112_config_gpio(hd);
// DEMO
#ifdef DEMO
			unsigned char mask = 0x7f;
			unsigned char value = 0x7f;
#else
			unsigned char mask = 0x0f;
			unsigned char value = 0x00;
#endif
			cp2112_set_gpio(hd, mask, value);
	/* cp2112 auto_send_read disable */
			if (cp2112_set_auto_send_read(hd, 0) < 0)  exit(EXIT_FAILURE);
		}
		else fclose(fdsem);
	}
	if ((key = ftok(CP2112_SEMAPHORE, 'S')) == -1)  {
		perror("ftok: failed");
		exit(1);
	}
	if (mysem_id == 0) {
	/* Initialization of the semaphore */
		mysem_id = semget(key, 1, 0666 | IPC_CREAT);
	}
	if (mysem_id  < 0) {
		perror("semget: semget Initialization failed");
		exit(1);
	}
	if ( rw_flag == WRITE ) {
	/* port write */
		mysem_lock(mysem_id);
		gpio_write(hd, port, data);
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
			gpio_write(hd, port, invert_data);
		}
		s_result = gpio_read(hd) & patterns[port];
		mysem_unlock(mysem_id);
		s_result = s_result >> port;
		fprintf(stderr,"%1x",s_result);
	}
	if ( rw_flag == READ ) {
	/* port read */
		mysem_lock(mysem_id);
#ifdef DEMO
		if ( port == 9 ) {
			am2320_measured(hd);
		}
		else if ( port == 10 ) {
	/* cp2112 reset */
			cp2112_reset(hd)	;
			mysem_unlock(mysem_id);
			unlink(CP2112_SEMAPHORE);
			exit(EXIT_SUCCESS);
		}
#else
		if ( port == 5 ) {
			am2320_measured(hd);
		}
		else if ( port == 9 ) {
	/* cp2112 reset */
			cp2112_reset(hd)	;
			mysem_unlock(mysem_id);
			unlink(CP2112_SEMAPHORE);
			exit(EXIT_SUCCESS);
		}
#endif
		else if ( port == 8 ) {
			s_result = gpio_read(hd) & 0xff;
			fprintf(stderr,"%02x",s_result);
		}
		else {
			s_result = gpio_read(hd) & patterns[port];
			s_result = s_result >> port;
			fprintf(stderr,"%1x",s_result);
		}
		mysem_unlock(mysem_id);
	}
	mysem_lock(mysem_id);
	mysem_unlock(mysem_id);
#ifdef DEBUG
	mysem_lock(mysem_id);
	fprintf(stderr,"lock & wait %d milliseconds\n",DEBUG_WAIT);
	msleep(DEBUG_WAIT);
	mysem_unlock(mysem_id);
#endif
	hid_close(hd);
	hid_exit();
	return EXIT_SUCCESS;
}
