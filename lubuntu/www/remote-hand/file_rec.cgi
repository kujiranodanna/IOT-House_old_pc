#!/bin/bash
# The MIT License
# Copyright (c) 2024-2027 Isamu.Yamauchi ,2024.7.24 update 2024.7.24

PATH=$PATH:/usr/local/bin
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2024.7.24">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Upload Sound File settings</TITLE>
<script type="text/javascript">
function blink() {
  // if (!document.all) { return; }
  for (i = 0; i < document.all.length; i++) {
    obj = document.all(i);
    if (obj.className == "blink") {
      if (obj.style.visibility == "visible") {
        obj.style.visibility = "hidden";
      } else {
        obj.style.visibility = "visible";
      }
    }
  }
  setTimeout("blink()",1000);
}
</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<HR>
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Processing Upload Image File</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024-2026 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
prog=file_rec
tFILE_NAME=$DIR/.${prog}.$$
IMAGE_FILE=$DIR/.${prog}_file.$$
tIMAGE_FILE=$DIR/.${prog}_file_tmp.$$
CMD=$DIR/${prog}_$$.pepocmd
LOCK=$DIR/file_rec.lock
MAXFILESIZE=$((2048 * 1024))
error(){
  [ -e ${tFILE_NAME} ] && rm ${tFILE_NAME}
  [ -e ${IMAGE_FILE} ] && rm ${IMAGE_FILE}
  [ -e ${tIMAGE_FILE} ] && rm ${tIMAGE_FILE}
  exit 0
}
trap error TERM HUP KILL INT QUIT
cat >$tIMAGE_FILE
if [ -e $tIMAGE_FILE ];then
  dd if=$tIMAGE_FILE bs=256 count=1 |mawk '/^[a-z]/{gsub("\r","",$0);print}' >$tFILE_NAME
  . $tFILE_NAME  
  if [ ! -z $filesize ];then
    if [ $filesizee -gt $MAXFILESIZE ];then
      [ -e $tFILE_NAME ] && rm $tFILE_NAME
      [ -e $IMAGE_FILE ] && rm $IMAGE_FILE
      [ -e $tIMAGE_FILE ] && rm $tIMAGE_FILE
      echo -n '</HTML>'
      exit
    fi
  fi
  cat $tIMAGE_FILE | sed -n 6,6p|mawk '{gsub("\r","",$0);gsub(";","",$0);printf("%s\n%s\n",$3,$4)}' >$tFILE_NAME
  . $tFILE_NAME
  SIZE=`cat $tIMAGE_FILE | mawk 'NR == 4{gsub("\r","",$0);printf $1}'`
else
  exit
fi
tSIZE=$(wc -c $tIMAGE_FILE | mawk '{print $1}')
while [ $tSIZE -lt $filesize ];do
  msleep 1000
  cat >> $tIMAGE_FILE
  tSIZE=$(wc -c $tIMAGE_FILE | mawk '{print $1}')
done
FILENAME=$DIR/$filename
if [ ! -e $LOCK ];then
  touch $LOCK
  cat $tIMAGE_FILE | sed '1,8d' >$tFILE_NAME
  dd if=$tFILE_NAME of=$FILENAME bs=$filesize count=1
  [ -e $LOCK ] && rm $LOCK
else
  [ -e ${tFILE_NAME} ] && rm ${tFILE_NAME}
  [ -e ${IMAGE_FILE} ] && rm ${IMAGE_FILE}
  [ -e ${tIMAGE_FILE} ] && rm ${tIMAGE_FILE}
fi
echo -n '
</HTML>'
[ -e $tFILE_NAME ] && rm $tFILE_NAME
[ -e $IMAGE_FILE ] && rm $IMAGE_FILE
[ -e $tIMAGE_FILE ] && rm $tIMAGE_FILE
