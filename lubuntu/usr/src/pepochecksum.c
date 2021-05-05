/* 
Copyright Isamu.Yamauchi 2015-2023. update 2021.5.5
This program is intended for making the checksum of the transmitted data of TWE-Lite DIP(Tocos wireless engine) .
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
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/types.h>
#define VER "0.1"
#define DAY "compiled:"__DATE__

void usage(){
  fprintf(stderr,"\r\n** Welcome to pepochecksum Version-%s Copyright Isamu.Yamauchi %s **",VER,DAY);
  fprintf(stderr, "\r\nusage: pepochecksum HEX_ascii_strings\r\n") ;
  exit (-1) ;
}

int char2hex(unsigned char ch)
{
  if ( ch >= '0' && ch <= '9' )
    return (ch - '0') ;
  else if ( ch >= 'a' && ch <= 'f' )
    return (ch - 'a' + 10) ;
  else if ( ch >= 'A' && ch <= 'F' )
    return (ch - 'A' + 10) ;
  else usage() ;
}

int main(int argc, char *argv[]) {
  int i = 0 ;
  int j = 0 ;
  int len = 0 ;
  char *ch = argv[1] ;
  unsigned char s[3] ;
  unsigned char result = 0x00 ;
  if ( argc != 2 ) usage() ;
  len = strlen(argv[1]) ;
  if ( len < 2 ) usage() ;
  for ( i = 0 ; i < len - 2 ; i = i + 2 )  {
    strncpy( s , ch + i , 1 ) ;
    s[1] = '\0' ;
    result = result + (char2hex(*s) * 16) ;
	j = i + 1;
    strncpy( s , ch + j , 1 ) ;
    s[1] = '\0' ;
    result = result + char2hex(*s) ;
  }
  result = ( result ^ 0xff ) + 1 ;
  sprintf( s , "%02x" , result );
  printf("%c%c\n", toupper(s[0]) , toupper(s[1]) ) ;  
}
