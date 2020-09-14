#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

echo -en '
<html>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<HEAD>
</script>
<TITLE>During change routing table</TITLE>
<script type="text/javascript">
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
<TR ALIGN=CENTER class="blink"><TD>During change routing table</TD></TR>
</TABLE>
<BR>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>'

CMD=/www/remote-hand/tmp/ex_route.pepocmd

C_ACT=`echo $QUERY_STRING |awk 'BEGIN{FS="&"};/add/{print "add"};/del/{print "del"}'`
ROUTE=`echo $QUERY_STRING |awk 'BEGIN{FS="&"};$1~/route=/{gsub("route=","");printf("%s",$1)}'`
MASK=`echo $QUERY_STRING |awk 'BEGIN{FS="&"};$2~/mask=/{gsub("mask=","");printf("%s",$2)}'`
GWIP=`echo $QUERY_STRING |awk 'BEGIN{FS="&"};$3~/gw=/{gsub("gw=","");printf("%s",$3)}'`

if [ $MASK"X" == "255.255.255.255X" ];then
    C_NET="-host"
    C_MASK=""
    MASK=""
else
    C_NET="-net"
    C_MASK="netmask"
fi

if [ $ROUTE"X" != "X" ];then
  if [ $GWIP"X" != "X" ];then
    C_GW="gw"
  else
    C_GW=""
    GWIP=""
  fi
cat>$CMD<<EOF
#!/bin/bash
/sbin/route $C_ACT $C_NET $ROUTE $C_MASK $MASK $C_GW $GWIP
EOF
fi
