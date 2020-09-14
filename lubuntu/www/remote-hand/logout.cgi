#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="5;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Remote-Hand interface change</TITLE>
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
<TR class="blink" ALIGN=CENTER><TD>Logout process is in</TD></TR>
</TABLE>
<BR>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>'

CMD=/www/remote-hand/tmp/logout.pepocmd
cat>$CMD<<EOF
#!/bin/bash
HTPPASS_ORG=/etc/rc.pepo/password
HTPPASS_TMP=/www/remote-hand/tmp/.htpasswd.tmp
RAND=`echo -e \$RANDOM`
mv "\$HTPPASS_ORG" "\$HTPPASS_TMP"
echo -en "remote:\$RAND" > "\$HTPPASS_ORG"
msleep 10000
mv \$HTPPASS_TMP \$HTPPASS_ORG
EOF
