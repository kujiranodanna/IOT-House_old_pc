#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

DIRD=/www/remote-hand/tmp/.di_read_data
JSONRD=/www/remote-hand/tmp/.di_read_data.json
[ -e "$JSONRD" ] && rm -f $JSONRD && touch $JSONRD || touch $JSONRD
echo { >>$JSONRD
[ -e "$DIRD" ] && . "$DIRD"
for n in 0 1 2 3 4 5 6 7;do
case "$n" in
  0)
   [ "${di[$n]}" = "1" ] && di0="high"
   [ "${di[$n]}" = "0" ] && di0="low"
   [ "${di[$n]}" = "-1" ] && di0="none"
   echo '"'di0'"':'"'${di0}'",' >>$JSONRD
  ;;
  1)
   [ "${di[$n]}" = "1" ] && di1="high"
   [ "${di[$n]}" = "0" ] && di1="low"
   [ "${di[$n]}" = "-1" ] && di1="none"
   echo '"'di1'"':'"'${di1}'",' >>$JSONRD
  ;;
  2)
   [ "${di[$n]}" = "1" ] && di2="high"
   [ "${di[$n]}" = "0" ] && di2="low"
   [ "${di[$n]}" = "-1" ] && di2="none"
   echo '"'di2'"':'"'${di2}'",' >>$JSONRD
  ;;
  3)
   [ "${di[$n]}" = "1" ] && di3="high"
   [ "${di[$n]}" = "0" ] && di3="low"
   [ "${di[$n]}" = "-1" ] && di3="none"
   echo '"'di3'"':'"'${di3}'",' >>$JSONRD
  ;;
  4)
   [ "${di[$n]}" = "1" ] && di4="high"
   [ "${di[$n]}" = "0" ] && di4="low"
   [ "${di[$n]}" = "-1" ] && di4="none"
   echo '"'di4'"':'"'${di4}'",' >>$JSONRD
  ;;
  5)
   [ "${di[$n]}" = "1" ] && di5="high"
   [ "${di[$n]}" = "0" ] && di5="low"
   [ "${di[$n]}" = "-1" ] && di5="none"
   echo '"'di5'"':'"'${di5}'",' >>$JSONRD
  ;;
  6)
   [ "${di[$n]}" = "1" ] && di6="high"
   [ "${di[$n]}" = "0" ] && di6="low"
   [ "${di[$n]}" = "-1" ] && di6="none"
   echo '"'di6'"':'"'${di6}'",' >>$JSONRD
  ;;
  7)
   [ "${di[$n]}" = "1" ] && di7="high"
   [ "${di[$n]}" = "0" ] && di7="low"
   [ "${di[$n]}" = "-1" ] && di7="none"
   echo '"'di7'"':'"'${di7}'",' >>$JSONRD
  ;;
  esac
done
NOWDATE=`date +%x%X`
echo '"date":"'$NOWDATE'"' >>$JSONRD
echo "}" >>$JSONRD
#cat $JSONRD
chown apache.apache $JSONRD
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.jpn.org">
<META NAME="Build" content="2012.3.1">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>di2json</TITLE>
'
cat $JSONRD
echo -n '
</HEAD>
</BODY>
</HTML>'
