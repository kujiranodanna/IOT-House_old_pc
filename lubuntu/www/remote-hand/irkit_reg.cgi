#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

PATH=$PATH:/usr/local/bin
# irkit_reg.cgi,Registration of IR data for IRKit
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2018.2.24">
<META NAME="reply-to" content="izamu@pepolinux.com">
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'

DIR=/www/remote-hand/tmp
IRKIT_IP=$DIR/.IRKit_IP
CONV=./conv_get.cgi
. $CONV
IRNUM=$ir_num
IRFILE=$DIR/.irdata_${IRNUM}
if [ -e ${IRKIT_IP} ];then
  IP=`cat ${IRKIT_IP}`
else 
  rm -f $IRFILE
  exit
fi
CMD=$DIR/irkit_data.pepocmd

# get IRkit IR data
cat>${CMD}<<END
#!/bin/bash
wget http://${IP}/messages --header="X-Requested-With: PepoLinux" --output-document=${IRFILE}
END
