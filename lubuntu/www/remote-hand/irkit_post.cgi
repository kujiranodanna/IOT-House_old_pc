#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2024.7.12
# irkit_post.cgi,Post of IR data for IRKit and Nature Remo
PATH=$PATH:/usr/local/bin
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.jpn.org">
<META NAME="Build" content="2024.7.12">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Post of IR data IRKit</TITLE>
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
}</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>It is in the IR data output of IRKit</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024-2027 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>
'
DIR=/www/remote-hand/tmp
IRKIT_IP=$DIR/.IRKit_IP
DOCFILE=$DIR/irkit_out_document
CONV=./conv_get.cgi
. $CONV
IRNUM=$ir_num
TIMER=$ir_timer

CMD=$DIR/irkit_data.pepocmd
cat>${CMD}<<END
#!/bin/sh
pepoirkitpost ${IRNUM} ${TIMER}
END
fi