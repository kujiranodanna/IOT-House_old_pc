/*
 Copyright Isamu.Yamauchi 2011.11.5 update 2021.5.5
 pepodioexec.c help diod for daemon contorl digital-Input to ANDDIO
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
#include <unistd.h>
void exec_cmd(char *cmd) {
  char *cmds[4];
  cmds[0] = "/bin/sh";
  cmds[1] = cmd;
  cmds[2] = 0;
  execv(cmds[0], cmds);
}

int main (int argc, char *argv[]) {
  int pid;
  switch(pid = fork()) {
    case 0:
      exec_cmd(argv[1]);
    break;
    case 1:
      return 0;
    break;
  }
}
