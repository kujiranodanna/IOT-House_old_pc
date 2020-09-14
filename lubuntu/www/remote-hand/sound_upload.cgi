#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="2;URL=/remote-hand/pi_int.html">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Upload Sound File settings</TITLE>
<script type="text/javascript">
function blink() {
//  if (!document.all) { return; }
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
<TR ALIGN=CENTER class="blink"><TD>Processing Upload Sound File settings</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
FILE_NAME=$DIR/.sound_file_name
tFILE_NAME=$DIR/.sound_file_name_tmp
SOUND_FILE=$DIR/.sound_file
tSOUND_FILE=$DIR/.sound_file_tmp
[ -e $tSOUND_FILE ] && exit
cat >$tSOUND_FILE
cat $tSOUND_FILE | sed -n 6,6p|awk '{gsub("\r","",$0);gsub(";","",$0);printf("%s\n%s\n",$3,$4)}' >$tFILE_NAME
. $tFILE_NAME
SIZE=`cat $tSOUND_FILE | awk 'NR == 4{gsub("\r","",$0);printf $1}'`
case $name in
  "sound_file_0")
     tmp="sound_file[0]"
     n=0
  ;;
  "sound_file_1")
     tmp="sound_file[1]"
     n=1
  ;;
  "sound_file_2")
     tmp="sound_file[2]"
     n=2
  ;;
  "sound_file_3")
     tmp="sound_file[3]"
     n=3
  ;;
  "sound_file_4")
     tmp="sound_file[4]"
     n=4
  ;;
esac
if [ -e $FILE_NAME ];then
  .  $FILE_NAME
  [ -e $DIR/${sound_file[$n]} ] && rm -f $DIR/${sound_file[$n]}
  cat $FILE_NAME |grep -F -v $tmp >$tFILE_NAME
  echo "$tmp"="$filename" >> $tFILE_NAME
  mv $tFILE_NAME $FILE_NAME
else 
  echo "$tmp"="$filename" > $FILE_NAME
fi
FILE=$DIR/$filename
cat $tSOUND_FILE | sed '1,8d' >$SOUND_FILE
dd if=$SOUND_FILE of=$FILE bs=1 count=$SIZE
rm -f $tSOUND_FILE $tFILE_NAME $SOUND_FILE
echo -en '
</HTML>'
