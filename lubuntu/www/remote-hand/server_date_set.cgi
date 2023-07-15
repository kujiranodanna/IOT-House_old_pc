#!/bin/bash
# The MIT License
# Copyright (c) 2021-2028 Isamu.Yamauchi , update 2018.2.24

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.osdn.jp">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.osdn.jp">
<TITLE>Date being set</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Date being set</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2023 pepolinux.osdn.jp</TD><TR></TABLE>
</body>
</HTML>
'
CONV=./conv_get.cgi
. $CONV
CMD=/www/remote-hand/tmp/server_date.pepocmd
YY=`echo $server_date|awk 'BEGIN{FS="/"};{print $1}'`
DD=`echo $server_date|awk 'BEGIN{FS="/"};{print $2"/"$3}'`
HH=`echo $server_time|awk 'BEGIN{FS=":"};{print $1":"$2}'`
cat>$CMD<<EOF
#!/bin/bash
date -s "$DD $HH $YY"
EOF
