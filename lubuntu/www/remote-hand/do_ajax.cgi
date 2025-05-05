#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2024.2.10

PATH=$PATH:/usr/local/bin:/usr/local/sbin
DIR=/www/remote-hand/tmp
DIOCMD=$DIR/$$do_ajax.pepocmd
EXEC_CMD=/usr/local/bin/pepodiodexec
ALIAS_DI=$DIR/.alias_di
[ -e $ALIAS_DI ] && . $ALIAS_DI
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2024.2.10">
<META http-equiv="Refresh" content="0;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
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
if [[ "$ch" =~ ^dio ]];then
  CH0=$(echo "$ch"|tr [:alpha:] ' ')
  CH0=$(($CH0 + 0))
  CH1=$(($CH0 + 11))
  [ $val = "0" ] && val="low" || val="high"
  DIO0=/usr/bin/dio$CH0${val}
  DIO1=/usr/bin/dio$CH1${val}
  cat>$DIOCMD<<END
#!/bin/sh
[ -e $DIO0 ] && $EXEC_CMD $DIO0 >/dev/null 2>&1
[ -e $DIO1 ] && $EXEC_CMD $DIO1 >/dev/null 2>&1
END
else
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
#!/bin/sh
$cmd $ch $val $time >/dev/null 2>&1 
END
fi

