#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2017.7.13

PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2017.7.13">
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

CONV=./conv_get.cgi
. $CONV
DIR=/www/remote-hand/tmp
FILE_NAME=$DIR/.sound_file_name
tFILE_NAME=$DIR/.sound_file_name_tmp

if [ ! -e $FILE_NAME ];then
  exit
else
  . $FILE_NAME
fi
for n in 0 1 2 3 4;do
if [ -n ${sound_file[$n]} ];then
  if [ ${reg_sound[$n]} = "del" ];then
    cat $FILE_NAME | grep -F -v ${sound_file[$n]} > $tFILE_NAME
    mv $tFILE_NAME $FILE_NAME
    rm -f $DIR/${sound_file[$n]}
  fi
fi
done
echo -en '
</HTML>'
