#!/bin/sh
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2025.8.9
# cp2112bme680_amb_temp.cgi, BME680 The ambient temperature can be set to before reconfiguring the gas sensor
echo -n '
<HTML>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<HEAD>
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META http-equiv="Refresh" content="2;URL=/remote-hand/pi_int.html">
<META NAME="build" content="2025.8.9">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>command of execution</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>CP2112 BME680 amb_temp set</TD></TR>
</TABLE>
<BR>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2025-2027 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
CMD=/www/remote-hand/tmp/cp2112bme680_amb_temp.pepocmd
cat>$CMD<<EOF
#!/bin/sh
pkill -USR1 -f "/usr/local/bin/pepocp2112ctl 10"
EOF
