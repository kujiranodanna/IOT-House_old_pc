#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2024.11.28

PATH=$PATH:/usr/local/bin
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2024.11.28">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Sound delete file</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Processing Sound File settings</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>'

CONV=./conv_get.cgi
. $CONV
DIR=/www/remote-hand/tmp
prog=sound_del
LOCK=$DIR/${prog}_lock
LOCK_OTHER=$DIR/sound_upload_lock
[ -e $LOCK -o -e $LOCK_OTHER ] && exit
echo -en $$ >$LOCK
FILE_NAME=$DIR/.sound_file_name
tFILE_NAME=$DIR/.sound_file_name_tmp
if [ ! -e $FILE_NAME ];then
  exit
else
  . $FILE_NAME
fi
for n in 0 1 2 3 4 5 6 7 8 9;do
if [ -n "${disp_sound[$n]}" ];then
  if [ "${sound_file[$n]}" = "${disp_sound[$n]}" ];then
    cat $FILE_NAME | grep -F -v ${sound_file[$n]} > $tFILE_NAME
    mv $tFILE_NAME $FILE_NAME
    rm -f $DIR/${sound_file[$n]}
  fi
fi
lengthFILE_NAME=`cat $FILE_NAME | wc -l`
if [ $lengthFILE_NAME -eq 0 ];then
  rm $FILE_NAME
  exit
fi
done
[ -e $LOCK ] && rm $LOCK
echo -n '
</HTML>'
