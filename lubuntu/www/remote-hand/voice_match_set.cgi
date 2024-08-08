#!/bin/bash
# The MIT License
# Copyright (c) 2024-2027 Isamu.Yamauchi ,2024.7.5 update 2024.7.7

echo -n '
<HTML>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<HEAD>
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="build" content="2024.7.7">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>command of execution</TITLE>
<script type="text/javascript">
<!--
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
<TR ALIGN=CENTER class="blink"><TD>Voice match extension command creating</TD></TR>
</TABLE>
<BR>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
CONV=./conv_get.cgi
. $CONV
DIR=/www/remote-hand/tmp
VOM_DI=$DIR/.vomdi
tVOM_DI=$DIR/.vomdi.tmp
for n in 0 1 2 3 4 5 6 7 8 9 10; do
  [ -z "${vom_reg[$n]}" ] && continue
  if [ "${vom_reg[$n]}" != "none" ];then
    if [ -e "$VOM_DI" ];then
      cat "$VOM_DI" | grep -F -v [$n] > "$tVOM_DI"
      mv "$tVOM_DI" "$VOM_DI"
    fi
    echo "vom_val[$n]=""${vom_val[$n]}" >> "$VOM_DI"
    echo "vom_ans[$n]=""${vom_ans[$n]}" >> "$VOM_DI"
    echo "vom_var[$n]=""${vom_var[$n]}" >> "$VOM_DI"
  fi
  if [ "${vom_reg[$n]}" = "del" ];then
    if [ -e "$VOM_DI" ];then
      cat "$VOM_DI" | grep -F -v [$n] > "$tVOM_DI"
      mv "$tVOM_DI" "$VOM_DI"
    fi
  fi
done