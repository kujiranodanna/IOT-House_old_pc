#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Remote-Hand interface change</TITLE>
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
<TR class="blink" ALIGN=CENTER><TD>During interface changes</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>'

CMD=/www/remote-hand/tmp/ex_int.pepocmd
IP=`echo "$QUERY_STRING"|awk 'BEGIN{FS="&"};$1~/ip=/{gsub("ip=","");printf("%s",$1)}'`
MASK=`echo "$QUERY_STRING"|awk 'BEGIN{FS="&"};$2~/mask=/{gsub("mask=","");printf("%s",$2)}'`
ETH=`echo "$QUERY_STRING"|sed -e "s/&int=eth0/eth0/" -e "s/&int=eth1/eth1/" -e "s/&int=wlan0/wlan0/" -e "s/&mask=[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*//" -e "s/ip=[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*//"`
if [ $IP"X" != "X" ];then
cat>$CMD<<EOF
#!/bin/bash
/sbin/ifconfig $ETH $IP netmask $MASK
EOF
fi
