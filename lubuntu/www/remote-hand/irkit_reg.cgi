#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2024.2.10 update 2024.8.17
# 2024.7.12 add Nature Remo proc
PATH=$PATH:/usr/local/bin
# irkit_reg.cgi,Registration of IR data for IRKit, Nature Remo
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.jpn.org">
<META NAME="Build" content="2024.7.14">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Registration of IR data IRKit</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>IRKit It is of IR data registration</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>
'

DIR=/www/remote-hand/tmp
IRKIT_IP=$DIR/.IRKit_IP
CONV=./conv_get.cgi
. $CONV
IRNUM=$ir_num
IRFILE=$DIR/.irdata_${IRNUM}
USERAGENT="Chrome/126.0.6478.127"
RETRYTIME=2
RETRY=1
if [ -e ${IRKIT_IP} ];then
  IP=`cat ${IRKIT_IP}`
else
  rm -f $IRFILE
  exit
fi
[ -e /.dockerenv ] && IS_CONTAINER="YES" || IS_CONTAINER="NO"
if [ $IS_CONTAINER = "YES" ];then
  tREMO3_MAC=1
  tIRKIT_MAC=0
else
  REMO3_MAC="0c:8b:95"
  IRKIT_MAC="20:f8:5e"
  ping -c 1 $IP >${DOCFILE}
  tREMO3_MAC=$(arp $IP|grep $REMO3_MAC|wc -l)
  tIRKIT_MAC=$(arp $IP|grep $IRKIT_MAC|wc -l)
fi
if [ $tREMO3_MAC != 0 ];then
  tMAC=REMO
elif [ $tIRKIT_MAC != 0 ];then
  tMAC=IRKIT
else
  echo $IP Neither IRKit or Nature Remo
  exit
fi

CMD=$DIR/irkit_data.pepocmd
if [ $tMAC = "IRKIT" ];then
# get IRkit IR data
cat>${CMD}<<END
#!/bin/sh
curl -s -m $RETRYTIME --retry $RETRY --user-agent ${USERAGENT} http://${IP}/messages --header "X-Requested-With: PepoLinux" >${IRFILE}
END
elif [ $tMAC = "REMO" ];then
# get Remo IR data
cat>${CMD}<<END
#!/bin/sh
curl -s http://${IP}/messages -H "X-Requested-With: local" >${IRFILE}
END
fi

