#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

PATH=$PATH:/usr/local/bin:/usr/local/sbin
DIR=/www/remote-hand/tmp
DIOCMD=$DIR/do_ajax.pepocmd
ALIAS_DI=$DIR/.alias_di
[ -e $ALIAS_DI ] && . $ALIAS_DI
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="0;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Digital -out is being modified</TITLE>
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
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Digital -out is being modified</TD></TR>
</TABLE>
</BODY>
</HTML>
'
CONV=./conv_get.cgi
. $CONV
if [ $ch -gt 13 ];then
  if [ $TOCOS_TTY = "none" ];then
    cmd=/usr/local/bin/pepotocoshelp
  else
    ch=$(($ch - 13))
    cmd=/usr/local/bin/pepotocoshelp_local
  fi
else
  cmd=/usr/local/bin/pepopiface
fi
cat>$DIOCMD<<END
#!/bin/bash
$cmd $ch $val $time >/dev/null 2>&1 
END
