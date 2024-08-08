#!/bin/bash
# The MIT License
# Copyright (c) 2021-2027 Isamu.Yamauchi , update 2023.2.13
PATH=$PATH:/usr/local/bin
# for raspberry pi
DIR=/www/remote-hand/tmp
CMD=$DIR/update_for_wait.pepocmd
HOMEPAGE=/www/remote-hand/pi_int.html
RMHOMEPAGE="YES"
if [ $RMHOMEPAGE = "YES" ];then
  cat>$CMD<<END
#!/bin/sh
  rm -f $HOMEPAGE
END
fi
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2023.2.15">
<META http-equiv="Refresh" content="1;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Only update process</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Setting server initial data</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2023-2026 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
