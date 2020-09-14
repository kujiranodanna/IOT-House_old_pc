/*
    pepoam2320.c read am2320 temperature & humidity
    Copyright (C) 2017.12.31 Isamu.Yamauchi
    
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
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>              /* for O_RDWR */
#include <sys/ioctl.h>
#include <string.h>             /* for memcpy */
#include <linux/i2c-dev.h>      /* for I2C_SLAVE */
#include <time.h>
#include <inttypes.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
/* I2C character device */
// #define I2C_DEVICE "/dev/i2c-6"
#define I2C_DEVICE "/dev/i2c-cp2112"

/* I2C address of AM2320 sensor in 7 bits
 * - the first 7 bits should be passed to ioctl system call
 *   because the least 1 bit of the address represents read/write
 *   and the i2c driver takes care of it
 */
#define AM2320_ADDR (0xB8 >> 1)

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


/*
 *  st_am2320 Structure
 */

typedef struct {
  unsigned char data[8];
} st_am2320;

void __check_crc16( st_am2320 measured ) {
  unsigned short crc_m, crc_s;

  crc_m = crc16( measured.data, 6 );
  crc_s = (measured.data[7] << 8) + measured.data[6];
  if ( crc_m != crc_s ) {
    fprintf( stderr, "am2320: CRC16 does not match\n" );
    exit( 1 );
  }
  
  return;
}

st_am2320 __st_am2320( unsigned char* data ) {
  st_am2320 result;
  memcpy( result.data, data, 8 );
  __check_crc16( result );
  return result;
}

short __am2320_temperature( st_am2320 measured ) {
  return (measured.data[4] << 8) + measured.data[5];
}

short am2320_temperature_integral( st_am2320 measured ) {
  return __am2320_temperature( measured ) / 10;
}

short am2320_temperature_fraction( st_am2320 measured ) {
  return __am2320_temperature( measured ) % 10;
}

short __am2320_humidity( st_am2320 measured ) {
  return (measured.data[2] << 8) + measured.data[3];
}

short am2320_humidity_integral( st_am2320 measured ) {
  return __am2320_humidity( measured ) / 10;
}

short am2320_humidity_fraction( st_am2320 measured ) {
  return __am2320_humidity( measured ) % 10;
}


/*
 *  Measurement function
 */

st_am2320 am2320() {
  int fd;
  int ret = -1;
  int retry_cnt = 0;
  unsigned char data[8];

  /* open I2C device */
  fd = open( I2C_DEVICE, O_RDWR );
  if ( fd < 0 ) {
    perror( "am2320(1)" );
    exit( 1 );
  }

  /* set address of I2C device in 7 bits */
  ret = ioctl( fd, I2C_SLAVE, AM2320_ADDR );
  if ( ret < 0 ) {
    perror( "am2320(2)" );
    exit( 2 );
  }
 
  while ( retry_cnt < 5 ) {
  /* wake I2C device up */
    write( fd, NULL, 0);
    msleep( 50 );
  /* write measurement request */
    data[0] = 0x03; data[1] = 0x00; data[2] = 0x04;
    ret = write( fd, data, 3 );
    if ( ret < 0 ) retry_cnt++ ; else break;
    msleep( 50 );
  }
  if ( retry_cnt > 5 ) {
    perror( "am2320(3)" );
    exit( 3 );
  }
  
  /* wait for having measured */
  msleep( 50 );
  
  /* read measured result */
  memset( data, 0x00, 8 );
  ret = read( fd, data, 8 );
  if ( ret < 0 ) {
    perror( "am2320(4)" );
    exit( 4 );
  }
  
  /* close I2C device */
  close( fd );
  
  return __st_am2320( data );
}

/*
 *  Print functions
 */

void print_am2320( st_am2320 measured ) {
  printf( "%d.%d %d.%d\n",
          am2320_temperature_integral( measured ),
          am2320_temperature_fraction( measured ),
          am2320_humidity_integral( measured ),
          am2320_humidity_fraction( measured ) );
  return;
}


/*
 *  Main
 */

#define OPT_HUMAN_READABLE 0x1
#define OPT_STUB 0x2

int main( int argc, char* argv[] ) {
  st_am2320 measured; 
  /* measure temperature and humidity */
  measured = am2320();

  /* print measured temperature and humidity */
    print_am2320( measured );  
  return 0;
}
