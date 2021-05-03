/*
The MIT License
Copyright (c) 2020-2027 Isamu.Yamauchi , 2019.5.13 Update 2021.4.23
read for AM2320 or BME680 temperature,humidity,presure,gas
*/

/*
 * pepocp2112ctl.c is written for cp2112_gpio and i2c device.
 * Measure the humidity and temperature of the AM2320 sensor with this example code.
 * Measure the temperature, humidity, pressure and gas of the BME680 sensor with this example code.
 * CP2112 Single-Chip HID USB to SMBus Master Bridge see following
 * https://www.silabs.com/documents/public/data-sheets/cp2112-datasheet.pdf
 *
 * Depends on the HID API multi-platform library, available from
 * http://www.signal11.us/oss/hidapi/
 * BME680 Drivers from https://github.com/BoschSensortec/BME680_driver
 * Downloads bme680.c bme680.h bme680_defs.h
 * Build with
 *   gcc -Wall -g -o pepocp2112ctl pepocp2112ctl.c bme680.c `pkg-config hidapi-hidraw --libs`
 *   or
 *   gcc -Wall -o pepocp2112ctl pepocp2112ctl.c bme680.c -lhidapi-hidraw
 *
 o 2021.4.23
   bug fix gpio output pin is reset when the command is executed
 o 2021.4.11
   bug fix BME680 and AM2320 cannot be operated at the same time
 o 2021.3.28 Ver0.2
 o Added support for some bug fixes and BME680
 o Existing bug
   BME680 and AM2320 cannot be operated at the same time
 o 2018.1.6 Ver0.1 1st release
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>  /* for O_RDWR */
#include <sys/ioctl.h>
#include <time.h>
#include <inttypes.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>  /* use signal handler */
#include <string.h>
#include <stdint.h>
#include "hidapi/hidapi.h"
#include <sys/types.h>
#include <sys/sem.h>
#ifdef __STDC__
#define sigtype void
#else
#define sigtype int
#endif
#define DEBUG
#undef  DEBUG /* Comment out the case when debugging */
#define DEMO
#undef  DEMO /* Comment out the case when DEMO */
#define READ 'R'
#define WRITE 'W'
#define VER "0.4"
#define DAY "compiled:"__DATE__
#define LOCK -1
#define UNLOCK 1
#define CP2112_SEMAPHORE "/var/run/pepocp2112.semaphore"
#ifdef  DEBUG
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
#define CP2112_VID 0x10c4
#define CP2112_PID 0xea90
#define CP2112_IS_OPEN 0x01
#define CP2112_IS_CLOSE 0x00
#define DELAY 3000  /* for measurement delay ms */
#define LOOP_TIME 5000  /* senser read period ms */
#define HID_WAIT 5  /* for hid delay ms */
#define DESTZONE    "TZ=Asia/Tokyo"  /* destination time zone */
#define SENSOR_DATA "/www/remote-hand/tmp/.pepocp2112bme680"  /* read sensor dta file */
#define SENSOR_DATA_TMP "/www/remote-hand/tmp/.pepocp2112bme680_tmp"  /* read sensor file data temporary */

#include "bme680.h"
/* BME680 I2C addresses defined bme680_def.h But can be changed here */
#define BME680_I2C_ADDR_PRIMARY    UINT8_C(0x76)
#define BME680_I2C_ADDR_SECONDARY  UINT8_C(0x77)
struct bme680_dev gas_sensor;
struct bme680_field_data data;
FILE *data_fd;
uint16_t meas_period;
int mysem_id = 0;
hid_device *hd;
int8_t is_hid = CP2112_IS_CLOSE;

void user_delay_ms(uint32_t period)
{
  struct timeval timeout;
  timeout.tv_sec = period / 1000;
  timeout.tv_usec = (period % 1000) * 1000;
  if (select(0, (fd_set *) 0, (fd_set *) 0, (fd_set *) 0, &timeout) < 0)
  {
    perror("user_delay_ms");
  }
}

void usage()
{
  fprintf(stderr,"\r\n** Welcome to pepocp2112ctl Version-%s Copyright Isamu.Yamauchi %s **",VER,DAY);
  fprintf(stderr,"\n\rusage:pepocp2112ctl port:0-8 [0|1] [timer:0-300000ms]");
  fprintf(stderr,"\n\rusage:pepocp2112ctl port:0-3 output, 4-7 input ");
  fprintf(stderr,"\n\rusage:pepocp2112ctl 5  <--AM2320 measured");
  fprintf(stderr,"\n\rusage:pepocp2112ctl 10  <--BME680 measured\n\r");
}

int get_myval(int sid)
{
  union semun
  {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
    struct seminfo *__buf;
    void *__pad;
  };
  union semun my_semun;
  uint16_t d_result = semctl(sid, 0, GETVAL, my_semun);
  if (d_result == -1)
  {
    perror("semctl: GETVAL failed");
    exit(EXIT_FAILURE);
  }
#ifdef DEBUG
  printf("%s%d\n","val=",d_result);
#endif
  return(d_result);
}

int get_sempid(int sid)
{
  union semun
  {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
    struct seminfo *__buf;
    void *__pad;
  };
  union semun my_semun;
  pid_t sem_pid;
  sem_pid = semctl(sid, 0, GETPID, my_semun);
  if (sem_pid == -1)
  {
    perror("semctl: GETPID failed");
    exit(EXIT_FAILURE);
  }
  return(sem_pid);
}

void create_semaphore()
{
  union semun
  {
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
  if (fdsem != NULL)
  {
    if ((key = ftok(CP2112_SEMAPHORE, 'S')) == -1)
    {
      perror("ftok: failed");
      exit(EXIT_FAILURE);
    }
/* Creating of the semaphore */
    mysemun_id = semget(key, 1, 0666 | IPC_CREAT);
    if (mysemun_id == -1)
    {
      perror("semget: semget get failed");
      exit(EXIT_FAILURE);
    }
#ifdef DEBUG
    sem_pid = get_sempid(mysemun_id);
    fprintf(stderr,"\r\nmy_pid:%d, sem_pid:%d\r\n",my_pid, sem_pid);
#endif
/* remove of the semaphore */
    my_semun.val = 1;
    if (semctl(mysemun_id , 0, IPC_RMID, my_semun) == -1)
    {
      perror("semctl: semaphore remove failed");
      exit(EXIT_FAILURE);
    }
/* semaphore of the file delete */
    unlink(CP2112_SEMAPHORE);
  }
  fdsem = fopen(CP2112_SEMAPHORE,"r");
  if (fdsem == NULL)
  {
/* File creation of semaphore */
    fdsem = fopen(CP2112_SEMAPHORE,"w");
    if (fdsem == NULL)
    {
      perror("fopen: failed");
      exit(EXIT_FAILURE);
    }
#ifdef DEBUG
    fprintf(stderr,"\r\n** %s file creation succeed of semaphore! **\r\n",CP2112_SEMAPHORE);
#endif
    fclose(fdsem);
  }
  if ((key = ftok(CP2112_SEMAPHORE, 'S')) == -1)
  {
    perror("ftok: failed");
    exit(EXIT_FAILURE);
  }
/* Creating of the semaphore */
  mysemun_id = semget(key, 1, 0666 | IPC_CREAT);
  if (mysemun_id == -1)
  {
    perror("semget: semget Initialization failed");
    exit(EXIT_FAILURE);
  }
  d_result = get_myval(mysemun_id);
  if (d_result == 0)
  {
/* Initialization of the semaphore */
    my_semun.val = 1;
    if (semctl(mysemun_id, 0, SETVAL, my_semun) == -1)
    {
      perror("semctl: Initialization failed");
      exit(EXIT_FAILURE);
    }
#ifdef DEBUG
  fprintf(stderr,"\r\n** Initialization succeed of semaphore! **\r\n");
#endif
  }
}

static int cp2112_close(hid_device *hd)
{
  int8_t rslt = 0;
  if (is_hid == CP2112_IS_OPEN)
  {
    hid_close(hd);
    hid_exit();
  }
  is_hid = CP2112_IS_CLOSE;
  return rslt;
}

static int cp2112_open(int cp2112_vid, int cp2112_pid)
{
/* open Silabs CP2112 USB reference design by USB product:vendor id */
//  hid_device *hd = hid_open(0x10c4, 0xea90, NULL);
 int8_t rslt = 0;
 if (is_hid == CP2112_IS_CLOSE)
  {
    if (hid_init() < 0)
    {
      fprintf(stderr, "hid_init() failed, exit.\n");
      raise(SIGTERM);
    }
    hd = hid_open(cp2112_vid, cp2112_pid, NULL);
    if (hd == NULL)
    {
      fprintf(stderr, "hid_open() failed\n");
      raise(SIGTERM);
    }
  }
  is_hid = CP2112_IS_OPEN;
  return rslt;
}

/* configure cp2112 GPIO pins */
static int cp2112_config_gpio(hid_device *hd)
{
  unsigned char buf[5] = { CP2112_GETSET_GPIO_CONFIG, };
  int ret = hid_get_feature_report(hd, buf, sizeof(buf));
  if (ret < 0)
  {
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
/*  for example */
//  buf[1] = 0x8b; /* output direction for GPIOs 0, 1, 3, 7 */
//  buf[2] = 0x8b; /* push-pull for GPIOs 0, 1, 3, 7 */
//  buf[3] = 0x00; /* no special functions, i.e. use pins as GPIO */
  ret = hid_send_feature_report(hd, buf, sizeof(buf));
  if (ret < 0)
  {
    fprintf(stderr, "hid_send_feature_report() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  return 0;
}

/* set cp2112 GPIO pins */
static int cp2112_set_gpio(hid_device *hd, unsigned char mask, unsigned char value)
{
  unsigned char buf[3] = { CP2112_SET_GPIO, };
  buf[1] = value; /* set GPIO pins: 0 .. turn LED on, 1 .. off */
  buf[2] = mask; /* mask of pins to set, ignore others */
  int ret = hid_send_feature_report(hd, buf, sizeof(buf));
  if (ret < 0)
  {
    fprintf(stderr, "hid_send_feature_report() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  return 0;
}

/* get cp2112 GPIO pins */
static unsigned char cp2112_get_gpio(hid_device *hd, unsigned char mask, unsigned char value)
{
  unsigned char buf[3] = { CP2112_GET_GPIO, };
  buf[1] = value; /* set GPIO pins: 0 .. turn LED on, 1 .. off */
  buf[2] = mask; /* mask of pins to set, ignore others */
  int ret = hid_get_feature_report(hd, buf, sizeof(buf));
  if (ret < 0)
  {
    fprintf(stderr, "hid_get_feature_report() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  return buf[1] ;
}
/* configures cp2112 to automatically send read data, see AN495 section 4.6 */
static int cp2112_set_auto_send_read(hid_device *hd, int on_off)
{
  unsigned char buf[14] = { CP2112_GETSET_SMBUS_CONFIG, };
  int ret = hid_get_feature_report(hd, buf, sizeof(buf));
  if (ret < 0)
  {
    fprintf(stderr, "hid_get_feature_report() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  buf[6] = on_off;
  ret = hid_send_feature_report(hd, buf, sizeof(buf));
  if (ret < 0)
  {
    fprintf(stderr, "hid_send_feature_report() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  return 0;
}

void mysem_lock(int sid)
{
  key_t key;
  FILE *fdsem;
  if (sid == 0)
  {
    fdsem = fopen(CP2112_SEMAPHORE,"r");
    if (fdsem == NULL)
    {
      create_semaphore();
      cp2112_open(CP2112_VID, CP2112_PID);
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
      if (cp2112_set_auto_send_read(hd, 0) < 0)
      {
        fprintf(stderr, "set_auto_send_read failed\n");
        is_hid = CP2112_IS_CLOSE;
        raise(SIGTERM);
      }
    }
    else
      fclose(fdsem);
    if ((key = ftok(CP2112_SEMAPHORE, 'S')) == -1)
    {
      perror("ftok: failed");
      exit(EXIT_FAILURE);
    }
    if (mysem_id == 0)
    {
  /* Initialization of the semaphore */
      mysem_id = semget(key, 1, 0666 | IPC_CREAT);
    }
    if (mysem_id  < 0)
    {
      perror("semget: semget Initialization failed");
      exit(EXIT_FAILURE);
    }
    sid = mysem_id;
  }
  if (is_hid == CP2112_IS_CLOSE) cp2112_open(CP2112_VID, CP2112_PID);
  struct sembuf mysemop[1];
  mysemop[0].sem_num = 0;
  mysemop[0].sem_op = LOCK;
  mysemop[0].sem_flg = SEM_UNDO;
  if (semop(sid, mysemop, 1) == -1)
  {
    perror("semop: semop lock-1 failed");
    exit(EXIT_FAILURE);
  }
#ifdef DEBUG
  printf("semop_lock:");get_myval(sid);
#endif
}

void mysem_unlock(int sid)
{
  struct sembuf mysemop[1];
  mysemop[0].sem_num = 0;
  mysemop[0].sem_op = UNLOCK;
  mysemop[0].sem_flg = SEM_UNDO;
  if (semop(sid, mysemop, 1) == -1)
  {
    perror("semop: semop unlock failed");
    exit(EXIT_FAILURE);
  }
#ifdef DEBUG
  printf("sem_unlock:");get_myval(sid);
#endif
}

sigtype close_fd()
{
  mysem_unlock(mysem_id);
  unlink(SENSOR_DATA);
  unlink(SENSOR_DATA_TMP);
  if (is_hid == CP2112_IS_OPEN)
  {
    hid_close(hd);
    hid_exit();
  }
  exit(EXIT_SUCCESS);
}

void move_file(const char* src_name, const char* dest_name)
{
  rename(src_name, dest_name);
}

/* CRC16 calculation */
unsigned short crc16( unsigned char *ptr, unsigned char len ) {
  unsigned short crc = 0xFFFF;
  unsigned char i;
  while( len-- )
  {
    crc ^= *ptr++;
    for(i = 0; i < 8; i++)
    {
      if(crc & 0x01)
      {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/* reset cp2112 */
static int cp2112_reset(hid_device *hd)
{
  unsigned char buf[2] = { CP2112_RESET ,0x01 ,};
  int ret = hid_send_feature_report(hd, buf, sizeof(buf));
  return ret;
}

/* port no 0-7 write */
unsigned char gpio_write(hid_device *hd, int port ,int value)
{
  char patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
  unsigned char mask = patterns[port];
  int write_value = value << port;
  int ret = cp2112_set_gpio(hd, mask ,write_value);
  if (ret > 0)
  {
    ret = value;
  }
  return ret;
}

unsigned char gpio_read(hid_device *hd)
{
/* port no 0-7 read */
  unsigned char mask = 0xff;
  unsigned char value = 0xff;
  int ret = cp2112_get_gpio(hd, mask ,value);
  if ( ret > 0 )
  {
    ret = ret & mask;
  }
  return ret;
}

/* see if the cp2112 is idle */
static int cp2112_is_idle(hid_device *hd)
{
  unsigned char status_req[2] = {CP2112_TRANSFER_STATUS_REQ, 0x01,};
  if (hid_write(hd, status_req, sizeof(status_req)) < 0)
  {
    fprintf(stderr, "hid_write() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  unsigned char status_resp[7] = { 0x00, };
  if (hid_read(hd, status_resp, sizeof(status_resp)) < 0)
  {
    fprintf(stderr, "hid_read() failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  if (status_resp[0] == CP2112_TRANSFER_STATUS_RESP)
  {
/* //debugging code in
    fprintf(stderr, "\ncp2112_is_idle dump start status_resp\n");
    for (int i = 0; i < sizeof(status_resp); i++)
    {
      fprintf(stderr, "%d:%02x ",i ,status_resp[i]);
    }
    fprintf(stderr, "\ndump end status_resp\n");
*/   //debugging code out
    if (status_resp[1] == 0x00)
      return 0; // Idle
    if (status_resp[1] == 0x01)
      return 1; // Busy
    if (status_resp[1] == 0x02)
      return 2; // Complete
    if (status_resp[1] == 0x03)
    {
      if (status_resp[2] == 0x00)
        return -1; // Timeout address NACKed
      if (status_resp[2] == 0x01)
        return -1; // Timeout bus not free (SCL Low Timeout)
      if (status_resp[2] == 0x02)
        return -1; // Arbitration lost
      if (status_resp[2] == 0x03)
        return -1; // Read incomplete
      if (status_resp[2] == 0x04)
        return -1; // Write incomplete
      if (status_resp[2] == 0x05)
        return 2; // Succeeded after Status 2 retries
    }
    else
      return status_resp[1]; // 0x00 is Idle, 0x01 is Busy
  }
  return -1;
}

int am2320_measured(hid_device *hd)
{
  int i = 0;
  int ret = -1;
  int retry_cnt = 0;
  int timeout = 1000; // 1000 milisecons
  unsigned short crc_m, crc_s;
  unsigned char crc_tmp[6];
  unsigned char buf_in[12] = { 0 };
  unsigned char buf_out[12] = { 0 };
  buf_out[0] = CP2112_DATA_WRITE;
/* 0x5c is the 7-bit SMBus slave address of the am2320 */
  buf_out[1] = AM2320_SLAVE_ADDR<<1;
/* These three steps can be completed by the sensor and writes reads
https://www.silabs.com/community/interface/knowledge-base.entry.html/2014/10/21/cp2112_returns_incor-Dbhn
*/
  while (retry_cnt < 5)
  {
    ret = cp2112_is_idle(hd);
 // ret: 0x01 is Busy
    if (ret == 1)
    {
      retry_cnt++;
      user_delay_ms(HID_WAIT);
      continue;
    }
// am2320 wake up
    ret = hid_write(hd, buf_out, 2);
// ret: 0x02 is Complete
    if (ret != 2)
    {
      retry_cnt++;
      user_delay_ms(HID_WAIT);
      continue;
    }
    user_delay_ms(HID_WAIT);
/* write a buf_out byte to the am2320 */
    buf_out[0] = CP2112_DATA_WRITE;
    buf_out[2] = 0x03; // writes length
    buf_out[3] = 0x03; // measured am2320 specification 1
    buf_out[4] = 0x00; // specification 2
    buf_out[5] = 0x04; // specification 3
    ret = hid_send_feature_report(hd, buf_out, 6);
    if (ret != 6)
    {
      retry_cnt++;
      continue;
    }
    user_delay_ms (HID_WAIT);
    buf_out[0] = CP2112_DATA_READ_REQ;
    buf_out[2] = 0x00; // reads length_high
    buf_out[3] = 8; // reads length_low
    ret = hid_send_feature_report(hd, buf_out, 4);
    if (ret < 0)
    {
      fprintf(stderr, "hid_write() failed: %ls\n" ,
      hid_error(hd));
     return -1;
    }
    user_delay_ms (HID_WAIT);
    buf_out[0] = CP2112_DATA_READ_FORCE_SEND;
    buf_out[2] = 0x00; // reads length_high
    buf_out[3] = 8; // reads length_low
    ret = hid_send_feature_report(hd, buf_out, 4);
    if (ret < 0)
    {
      fprintf(stderr, "hid_write() failed: %ls\n" ,
      hid_error(hd));
      return -1;
    }
    user_delay_ms(HID_WAIT);
    ret = hid_read_timeout(hd, buf_in, 11, timeout);
/*
    fprintf(stderr, "\nret: %d retry_cnt: %d\n",ret,retry_cnt);
    fprintf(stderr, "\nbuf_in dump start\n");

    for (i = 0; i < sizeof(buf_in); i++)
      {
        fprintf(stderr, "%d:%02x ",i ,buf_in[i]);
      }
    fprintf(stderr, "\nbuf_in dump end\n");
*/
    if (ret < 0)
    {
      fprintf(stderr, "hid_read() failed: %ls\n",
      hid_error(hd));
      return -1;
    }
    if (buf_in[0] == CP2112_DATA_READ_RESPONS && buf_in[1] == 0x02)
      break;
  }
/* Dummy reading */
  ret = cp2112_is_idle(hd);
  if ( retry_cnt > 5 )
  {
    fprintf(stderr, "-1");
    return -1;
  }
  if (buf_in[2] != 0x08)
  {
    /* read length not match */
    fprintf(stderr, "-1");
    return -1;
  }
/*
buf_in[0]:hid_report ID->0x13:CP2112_DATA_READ_RESPONS
buf_in[1]:hid_status 0x00->Idle ,x01->Busy ,x02->Complete
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
  for (i = 0; i < sizeof(crc_tmp); i++)
    crc_tmp[i] = buf_in[i + 3];
  crc_m = crc16(crc_tmp, sizeof(crc_tmp));
  crc_s = (buf_in[10] << 8) + buf_in[9];
  if (crc_m != crc_s)
  {
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

int bme680_is_idle(hid_device *hd)
{
  int ret = -1;
  int retry_cnt = 0;
  while (retry_cnt < 4)
  {
    ret = cp2112_is_idle(hd);
    if (ret == 0 || ret == 2)
      break;
    if (ret < 0)
    {
      retry_cnt++;
      user_delay_ms(HID_WAIT);
      continue;
    }
// rslt: 0x01 is Busy
    if (ret == 1)
    {
      retry_cnt++;
      user_delay_ms(HID_WAIT);
      continue;
    }
  }
  if (retry_cnt > 4)
  {
    fprintf(stderr, "bme680_is_idle()retry failed:%d %ls\n", retry_cnt,
    hid_error(hd));
    raise(SIGTERM);
  }
  return ret;
}

int bme680_is_connect(hid_device *hd)
{
  unsigned char buf_out[2] = { 0 };
  int ret = -1;
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
  buf_out[0] = CP2112_DATA_WRITE;
/* 0x77 is the 7-bit SMBus slave address of the BME680 */
  buf_out[1] = BME680_I2C_ADDR_PRIMARY<<1;
  ret = bme680_is_idle(hd);
  if (ret < 0)
  {
    fprintf(stderr, "bme680_is_connect() failed: %ls\n",
    hid_error(hd));
    raise(SIGTERM);
  }
  rslt = hid_write(hd, buf_out, 2);
  ret = bme680_is_idle(hd);
  if (ret < 0)
  {
    fprintf(stderr, "bme680_is_connect() failed: %ls\n",
    hid_error(hd));
    raise(SIGTERM);
  }
  if (rslt != 2)
  {
    return -1;
  }
  return 0;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int retry_cnt = 0;
  int8_t ret = -1;
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
  uint8_t reg[5] = { 0 };
  uint8_t buf_in[40] = { 0 };
  int timeout = 1000; // 1000 milisecons
  mysem_lock(mysem_id);
/* if cp2112_hid open ? */
  if (is_hid == CP2112_IS_CLOSE) cp2112_open(CP2112_VID, CP2112_PID);
//  fprintf(stderr,"\ndev_id: %02x reg_addr: %02x reg_data: %x len: %d\n",dev_id,reg_addr,sizeof(reg_data),len);
  ret = bme680_is_idle(hd);
  if (ret < 0)
  {
    fprintf(stderr, "user_i2c_read()0 failed: %ls\n",
    hid_error(hd));
    raise(SIGTERM);
  }
  reg[0] = CP2112_DATA_WRITE;
  reg[1] = BME680_I2C_ADDR_PRIMARY<<1;
  reg[2] = 1;
  reg[3] = reg_addr;
  rslt = hid_send_feature_report(hd, reg, 4);
  if (rslt < 0)
  {
    fprintf(stderr, "user_i2c_read()1 failed: %ls\n" ,
    hid_error(hd));
    raise(SIGTERM);
  }
  user_delay_ms(HID_WAIT);
  reg[0] = CP2112_DATA_READ_REQ;
  reg[1] = BME680_I2C_ADDR_PRIMARY<<1;
  reg[2] = 0x00; // reads length_high
  reg[3] = len; // reads length_low
  rslt = hid_send_feature_report(hd, reg, 4);
  if (rslt < 0)
  {
    fprintf(stderr, "user_i2c_read()2 failed: %ls\n" ,
    hid_error(hd));
    raise(SIGTERM);
  }
  user_delay_ms(HID_WAIT);
  while (retry_cnt < 3)
  {
/*  SMBus Polling */
    reg[0] = CP2112_DATA_READ_FORCE_SEND;
    reg[1] = 0x00; // reads length_high
    reg[2] = len; // reads length_low
    rslt = hid_send_feature_report(hd, reg, 3);
    if (rslt < 0)
    {
      fprintf(stderr, "user_i2c_read()3 failed: %ls\n" ,
      hid_error(hd));
      raise(SIGTERM);
    }
    user_delay_ms(HID_WAIT);
    memset(buf_in, 0x00, sizeof(buf_in));
    rslt = hid_read_timeout(hd, buf_in, len+3, timeout);
/*
    buf_in[0]:hid_report ID->0x13:CP2112_DATA_READ_RESPONS
    buf_in[1]:hid_status 0x00->Idle ,x01->Busy ,x02->Complete
    ,buf_in[2]:hid_read_length
    buf_in[3]-:Following_BME680_Read_Data
*/
/* // debugging code in
    fprintf(stderr, "user_i2c_read()hid_read_timeout rslt:%d retry_cnt:%d\n" ,rslt,retry_cnt);
    fprintf(stderr, "buf_in dump start\n");
    for (int8_t i = 0; i < rslt; i++)
    {
      fprintf(stderr, "%d:%02x ",i ,buf_in[i]);
    }
    fprintf(stderr, "\nbuf_in dump end\n");
*/  // debugging code out
    if (buf_in[0] == CP2112_DATA_READ_RESPONS)
    {
      if (buf_in[1] == 0x00 || buf_in[1] == 0x02)
      {
        ret = 0;
        break;
      }
      if (buf_in[1] == 0x03)
      {
        ret = -1;
        break;
      }
      user_delay_ms(HID_WAIT);
      continue;
    }
    retry_cnt++; // Busy or Other
    user_delay_ms(HID_WAIT);
    continue;
  }
  if (retry_cnt > 3)
  {
    fprintf(stderr, "user_i2c_read()4 retry failed: %ls\n",
    hid_error(hd));
    ret = -1;
  }
  if (buf_in[2] != len)
  {
    fprintf(stderr, "user_i2c_read()5 failed read length not match\n");
    ret = -1;
  }
  if (ret != -1)
  {
    if(buf_in[2] > 0 && rslt > 3 && buf_in[2] < rslt)
    {
      int8_t j = buf_in[2];
  //    fprintf(stderr, "\nbuf_in[2] dump start\n");
      for (int8_t i=0; i < j; i++)
      {
        reg_data[i] = buf_in[i+3];
  //      fprintf(stderr,"%d:%02x ",i,reg_data[i]);
      }
  //    fprintf(stderr, "\nbuf_in[2] end\n");
    }
  }
  /* Read as a dummy to make the status response idle */
  bme680_is_idle(hd);
  cp2112_close(hd);
  mysem_unlock(mysem_id);
  if (ret < 0)
  {
    fprintf(stderr, "user_i2c_read()6 timeout or others failed: %ls\n",
    hid_error(hd));
    return -1;
  }
  return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t ret = -1;
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
  uint8_t reg[30] = { 0 };
  mysem_lock(mysem_id);
/* if cp2112_hid open ? */
  if (is_hid == CP2112_IS_CLOSE) cp2112_open(CP2112_VID, CP2112_PID);
  ret = bme680_is_idle(hd);
  if (ret < 0)
  {
    fprintf(stderr, "user_i2c_write() failed: %ls\n",
    hid_error(hd));
    raise(SIGTERM);
  }
  reg[0] = CP2112_DATA_WRITE;
  reg[1] = BME680_I2C_ADDR_PRIMARY<<1;
  reg[2] = len+1;
  reg[3] = reg_addr;
  for (int8_t i=4; i<len+4; i++)
    reg[i] = reg_data[i-4];
  rslt = hid_send_feature_report(hd, reg, len+4);
  ret = bme680_is_idle(hd);
  if (ret < 0)
  {
    fprintf(stderr, "user_i2c_write() failed: %ls\n",
    hid_error(hd));
    raise(SIGTERM);
  }
/* // debugging code in
  fprintf(stderr, "user_i2c_write()1 rslt: %d len: %d\n" ,rslt,len);
  fprintf(stderr, "reg dump start\n");
  for (int8_t i = 0; i < rslt; i++)
    {
      fprintf(stderr, "%d:%02x ",i ,reg[i]);
    }
  fprintf(stderr, "\nreg dump end\n");
*/  // debugging code out
  cp2112_close(hd);
  mysem_unlock(mysem_id);
  if (rslt != len+4)
  {
    perror("user_i2c_write");
    return 1;
  }
   return 0;
}

int conf_bme680(hid_device *hd)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
  uint8_t set_required_settings;
  gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY<<1;
  gas_sensor.intf = BME680_I2C_INTF;
  gas_sensor.read = user_i2c_read;
  gas_sensor.write = user_i2c_write;
  gas_sensor.delay_ms = user_delay_ms;
  rslt = BME680_OK;
  rslt = bme680_init(&gas_sensor);
/* Set the temperature, pressure and humidity settings */
  gas_sensor.tph_sett.os_hum = BME680_OS_2X;
  gas_sensor.tph_sett.os_pres = BME680_OS_4X;
  gas_sensor.tph_sett.os_temp = BME680_OS_8X;
  gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
/* Set the remaining gas sensor settings and link the heating profile */
  gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
/* Create a ramp heat waveform in 3 steps */
  gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
  gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */
/* Select the power mode */
/* Must be set before writing the sensor configuration */
  gas_sensor.power_mode = BME680_FORCED_MODE;
/* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
    | BME680_GAS_SENSOR_SEL;
/* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
/* Set the power mode */
  rslt = bme680_set_sensor_mode(&gas_sensor);
/* Get the total measurement duration so as to sleep or wait till the
   measurement is complete */
  bme680_get_profile_dur(&meas_period, &gas_sensor);
/* Delay till the measurement is ready */
  user_delay_ms(meas_period + DELAY);
  return rslt;
}

int bme680_measured(hid_device *hd)
{
  int8_t rslt = 0;  /* Return 0 for Success, non-zero for failure */
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  putenv(DESTZONE);  // Switch to destination time zone
  if (bme680_is_connect(hd) < 0)
  {
    fprintf(stderr, "bme680_measured() bme680_is_connect() failed: %ls\n" ,hid_error(hd));
    raise(SIGTERM);
  }
  mysem_unlock(mysem_id);
/* Get sensor data Avoid using measurements from an unstable heating setup */
  if (conf_bme680(hd) != 0)
  {
    fprintf(stderr, "bme680_measured() conf_bme680() failed: %ls\n" ,hid_error(hd));
    raise(SIGTERM);
  }
  while(1)
  {
    rslt = bme680_get_sensor_data(&data, &gas_sensor);
    if(rslt != 0)
    {
      conf_bme680(hd);
      continue;
    }
    if(data.status & BME680_HEAT_STAB_MSK)
    {
      data_fd = fopen(SENSOR_DATA_TMP,"w");
      if(data_fd < 0)
      {
        raise(SIGTERM);
      }
      t = time(NULL);
      tm = *localtime(&t);
      fprintf(data_fd,"%d/%02d/%02d/%02d:%02d:%02d,", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
      fprintf(data_fd,"%.1f,%.1f,%.1f,%d", data.temperature / 100.0f, data.humidity / 1000.1f,data.pressure / 100.0f, data.gas_resistance);
      fclose(data_fd);
      move_file(SENSOR_DATA_TMP,SENSOR_DATA);
    }
/* Trigger a meausurement */
    rslt = bme680_set_sensor_mode(&gas_sensor);
/* Wait for a measurement to complete */
    user_delay_ms(meas_period + DELAY);
/* Wait for the Measurement interval */
    user_delay_ms(LOOP_TIME);
  }
  return rslt;
}

int main(int argc, char *argv[])
{
  signal(SIGTERM,close_fd);
  signal(SIGQUIT,close_fd);
  signal(SIGINT,close_fd);
  signal(SIGHUP,close_fd);
  signal(SIGFPE,SIG_IGN);
  int port = 0;
  int data = 0;
  int invert_data = 0;
  int port_timer = 0;
  char rw_flag = READ;
  unsigned char s_result = 0x00;
  char patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
  if ( argc > 4 || argc < 2  )
  {
    usage();
    exit(EXIT_FAILURE);
  }
  else {
    port = atoi(argv[1]);
    if (port > 10 || port < 0)
    {
      usage();
      exit(EXIT_FAILURE);
    }
  }
  if ( argc == 3 ||  argc == 4)
  {
    data = atoi(argv[2]);
    if ( data != 0 && data != 1 )
    {
      usage();
      exit(EXIT_FAILURE);
    }
    else {
      rw_flag = WRITE;
    }
  }
  if ( argc == 4 )
  {
    port_timer = atoi(argv[3]);
    if ( port_timer < 0 || port_timer > 300000 )
    {
      usage();
      exit(EXIT_FAILURE);
    }
    rw_flag = WRITE;
  }
  mysem_lock(mysem_id);
  if (rw_flag == WRITE)
  {
  /* port write */
    gpio_write(hd, port, data);
    if (port_timer > 0)
    {
      mysem_unlock(mysem_id);
      if (data == 0)
      {
        invert_data = 1;
      }
      else {
        invert_data = 0;
      }
      user_delay_ms(port_timer);
      mysem_lock(mysem_id);
      gpio_write(hd, port, invert_data);
    }
    s_result = gpio_read(hd) & patterns[port];
    s_result = s_result >> port;
    fprintf(stderr,"%1x",s_result);
  }
  if (rw_flag == READ)
  {
  /* port read */
#ifdef DEMO
    if (port == 9)
    {
      am2320_measured(hd);
    }
    else if (port == 10)
    {
  /* cp2112 reset */
      cp2112_reset(hd);
      mysem_unlock(mysem_id);
      unlink(CP2112_SEMAPHORE);
      exit(EXIT_SUCCESS);
    }
#else
    if (port == 5)
    {
      am2320_measured(hd);
    }
    else if (port == 9)
    {
  /* cp2112 reset */
      cp2112_reset(hd);
      cp2112_close(hd);
      mysem_unlock(mysem_id);
      unlink(CP2112_SEMAPHORE);
      exit(EXIT_SUCCESS);
    }
#endif
    else if (port == 8)
    {
      s_result = gpio_read(hd) & 0xff;
      fprintf(stderr,"%02x",s_result);
    }
    else if (port == 10)
    {
      bme680_measured(hd);
    }
    else
    {
      s_result = gpio_read(hd) & patterns[port];
      s_result = s_result >> port;
      fprintf(stderr,"%1x",s_result);
    }
  }
  mysem_unlock(mysem_id);
#ifdef DEBUG
  mysem_lock(mysem_id);
  fprintf(stderr,"lock & wait %d milliseconds\n",DEBUG_WAIT);
  user_delay_ms(DEBUG_WAIT);
  mysem_unlock(mysem_id);
#endif
  cp2112_close(hd);
  exit(EXIT_SUCCESS);
}
