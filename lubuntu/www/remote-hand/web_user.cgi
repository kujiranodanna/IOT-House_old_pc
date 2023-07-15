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
<TITLE>Web user settings in</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Web user settings in</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2023 pepolinux.osdn.jp</TD><TR></TABLE>
</BODY>
</HTML>
'
CONV=./conv_get.cgi
. $CONV
CMD=/www/remote-hand/tmp/web_user.pepocmd
PASSWD_FILE=/etc/rc.pepo/password
HTPASSWD=/usr/bin/htpasswd
WEB_USER=$user
WEB_PASS=$password
cat>$CMD<<EOF
#!/bin/bash
$HTPASSWD -bc $PASSWD_FILE $WEB_USER $WEB_PASS
EOF
