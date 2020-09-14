## Makefile.am --- automake input file for epicon
# Copyright (C) 2002-2017
# Author:
#   Isamu.Yamauchi
#   The PepoLinux Project, Only Me.
#   https://ja.osdn.net/projects/pepolinux/ 
#   pepolinux@users.osdn.me
# Other targets:
#   clean:     Remove object files
#   install:   Install epicon
#   uninstall: Uninstall epicon
#
# Default compiler is cc.  To force gcc use:
#   make "CC=gcc" [ <target> ]
#
# See README and COPYING for further information.
## process this file with automake to produce Makefile.in

ACLOCAL_AMFLAGS = -I m4
AM_CTAGSFLAGS = --globals -l c
## AM_LDFLAGS = -I /usr/include/ncurses -lncurses *2017.4.4 remove
bin_PROGRAMS = epicon
epicon_SOURCES = epicon.c epicon_main.c epicon_uty.c
mandir = /usr/share/man
man1_MANS = epicon.nr
