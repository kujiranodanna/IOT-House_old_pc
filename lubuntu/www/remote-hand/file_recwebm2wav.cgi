#!/bin/bash
# The MIT License
# Copyright (c) 2024-2027 Isamu.Yamauchi ,2024.10.2 update 2024.10.3

PATH=$PATH:/usr/local/bin
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2024.10.3">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Upload Sound File settings</TITLE>
<script type="text/javascript">
function blink() {
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
prog=file_rec_webm2wav
IMAGE_FILE=$DIR/${prog}_file.$$
TMP=$DIR/${prog}_tmp.$$
LOCK=$DIR/${prog}.lock
MAXFILESIZE=$((4096 * 1024))
TRIMSIZE=48
TRIMSIZE1=46
error(){
  [ -e ${IMAGE_FILE} ] && rm ${IMAGE_FILE}
  [ -e ${TMP} ] && rm ${TMP}
  exit
}
trap error TERM HUP KILL INT QUIT ERR
cat >$IMAGE_FILE
msleep 2000
if [ -e $IMAGE_FILE ];then
  tSIZE=$(wc -c $IMAGE_FILE |mawk '{print $1}')
  if [ $tSIZE -gt $MAXFILESIZE ];then
    rm $IMAGE_FILE
    exit
  fi
  dd if=$IMAGE_FILE bs=256 count=1 |mawk '{gsub("\r|-","",$0);print}' >$TMP
else
  exit
fi
filename1=$(cat $TMP |mawk '/filename/{gsub("filename|=|\"","",$4);print $4}')
if [ ! -z $filename1 ];then
  [ $filename1 = blob ] && unset filename1
fi
filename2=$(cat $TMP |mawk 'NR == 4{print $0}')
if [ -z "$filename1" -a -z "$filename2" ];then
  [ -e ${IMAGE_FILE} ] && rm ${IMAGE_FILE}
  [ -e ${TMP} ] && rm ${TMP}
  exit
fi
if [ -e $LOCK ];then
  touch $LOCK
  [ -e ${IMAGE_FILE} ] && rm ${IMAGE_FILE}
  [ -e ${TMP} ] && rm ${TMP}
  exit
fi
if [ ! -z $filename1 ];then
  FILENAME=$DIR/$filename1
  SIZE=$(($tSIZE - $TRIMSIZE))
  dd if=$IMAGE_FILE bs=$SIZE count=1 |sed '1,4d' >$FILENAME
elif [ ! -z $filename2 ];then
  FILENAME=$DIR/$filename2
  SIZE=$(($tSIZE - $TRIMSIZE1))
  dd if=$IMAGE_FILE bs=$SIZE count=1 |sed '1,8d' >$FILENAME
  if [ $filename2 = "voice.webm" ];then
    conv_file="voice.wav"
    CONV_FILE=$DIR/$conv_file
    ffmpeg -i $FILENAME -y $CONV_FILE
    tmp="sound_file[9]"
    SOUND_FILE=$DIR/.sound_file_name
    tSOUNF_FILE=$DIR/.sound_file_name.tmp
    cat $SOUND_FILE| grep -F -v $tmp >$tSOUNF_FILE
    echo "${tmp}"="$conv_file" >>$tSOUNF_FILE
    mv $tSOUNF_FILE $SOUND_FILE
    rm $FILENAME
  fi
fi
[ -e ${IMAGE_FILE} ] && rm ${IMAGE_FILE}
[ -e ${TMP} ] && rm ${TMP}
[ -e $LOCK ] && rm $LOCK
echo -n '
</HTML>'
