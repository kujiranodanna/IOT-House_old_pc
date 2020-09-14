#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2017.7.27 update 2020.5.14

# ip_do_ajax ; For raspberry pi , and scripts to run the Remote DIO and TOCOS .
PATH=$PATH:/usr/local/bin:/usr/local/sbin
DIOCMD=/www/remote-hand/tmp/ip_do_ajax.pepocmd
LOCAL_DIR=/www/remote-hand/tmp
ALIAS_DI=$LOCAL_DIR/.alias_di
VOICEREQ=$LOCAL_DIR/.voice_req
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2020.5.14">
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'
#QUERY_CMD=/www/remote-hand/tmp/.QUERY_STRING.cmd
#cat >$QUERY_CMD
#QUERY_STRING=`cat $QUERY_CMD`
QUERY_STRING=`cat `
CONV=./conv_get.cgi
. $CONV
[ -e $ALIAS_DI ] && . $ALIAS_DI
if [ "$ch" = "voice_req" ];then
  cat>$DIOCMD<<END
#!/bin/bash
  echo -en "$val" >$VOICEREQ
END
  exit
fi
if [ $ch -gt 7 -a $ch -lt 13 ];then
  cmd=/usr/local/bin/pepoirkitpost
  ch=$(($ch - 8))
  if [ ! -z $time ];then
    val=$time
    time=""
  else
    val="" 
  fi
elif [ $ch -gt 13 -a $ch -lt 17 ];then
  ch=$(($ch - 13))
  cmd=/usr/local/bin/pepotocoshelp_local
else
  if [ $DI_TTY = "gpio" ];then
    cmd=/usr/local/bin/pepogpiohelp
  elif  [ $DI_TTY = "piface" ];then
    cmd=/usr/local/bin/pepopiface_local
  else
    exit
  fi
fi
cat>$DIOCMD<<END
#!/bin/bash
$cmd $ch $val $time >/dev/null 2>&1 
END
