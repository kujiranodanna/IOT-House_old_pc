#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2020.7.21

# streming_start_stop.cgi
PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2020.7.21">
<META NAME="reply-to" content="izamu@pepolinux.com">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<TITLE>Streming start_stop</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Streming start_stop</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>'

DIR=/www/remote-hand/tmp
CONV=./conv_get.cgi
. $CONV
START_STOP_CMD=$DIR/streming_start_stop.cmd
START_STOP="$start_stop"
if [ "$START_STOP" != "stop" ];then
cat >${START_STOP_CMD}<<END
#/bin/bash
raspivid -vf -hf -o - -t 0 -w 640 -h 480 | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8554}' :demux=h264 >/dev/null 2>&1 &
#raspivid -o - -t 0 -w 640 -h 480 | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8554}' :demux=h264 >/dev/null 2>&1 &
END
else
cat >${START_STOP_CMD}<<END
#/bin/bash
killall vlc
killall raspivid
END
fi
rm -f $CMD
CMD=$DIR/streming_start_stop.pepocmd
cat >${CMD}<<END
#/bin/bash
sudo -u pi sh $START_STOP_CMD
rm -f $START_STOP_CMD
END
