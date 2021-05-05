/*
Copyright (c) 2020-2027 Isamu.Yamauchi , 2011.6.25 update 2021.5.5
milliseconds to sleep
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
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
int msleep(int ms)
{
  struct timeval timeout;
  timeout.tv_sec = ms / 1000;
  timeout.tv_usec = (ms % 1000) * 1000;
  if (select(0, (fd_set *) 0, (fd_set *) 0, (fd_set *) 0, &timeout) < 0) {
    perror("msleep");
      return -1;
   }
  return 0;
}
int main(int argc, char *argv[]){
  unsigned int msec;
  if (argc < 2) {
    printf("Usage: msleep ,miliseconds\n");
    exit (1);
  }
  msec = atoi(argv[1]);
  msleep(msec);
  exit(0);
}
