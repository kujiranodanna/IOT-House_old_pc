#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2020.3.20 update 2020.6.29

PATH=$PATH:/usr/local/bin
# irkit_search.cgi, search or set IP Address of IRKit
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.jpn.org">
<META NAME="Build" content="2020.6.29">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>search IP Address of IRKit</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>This is the IP address during the acquisition of IRKit</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>
'
CONV=./conv_get.cgi
. $CONV
IP=$ip
DIR=/www/remote-hand/tmp
CMD=$DIR/irkit_search.pepocmd
IRKITSERCH=/usr/local/bin/pepoirkitsearch
IRKIT_IP=$DIR/.IRKit_IP
if [ "$IP" != "none" ];then
# set IRkit IP
  cat>${CMD}<<END
#!/bin/sh
echo -n $IP >$IRKIT_IP
END
else
# get IRkit IP
  cat>${CMD}<<END
#!/bin/sh
rm -f $IRKIT_IP
IP=\`${IRKITSERCH}\`
if [ \${IP} != 0 ];then
  echo -n \${IP} >${IRKIT_IP}
else
  echo -n "none" >${IRKIT_IP}
fi
END
fi
