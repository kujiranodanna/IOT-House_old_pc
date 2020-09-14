#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2020.3.20 update 2020.6.29

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2018.2.24">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Ping monitoring and digital output</TITLE>
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
<TABLE ALIGN=CENTER BGCOLOR="#E0FFFF" BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Ping monitoring and digital output</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'
DIR=/www/remote-hand/tmp
CMD=$DIR/ping_watch_don.pepocmd
PING_DON=$DIR/.ping_don_list
PING_TMP=$DIR/.ping_don_tmp
PING_WATCH=/pepolinux/ping_watch_don
PING_CRON=$DIR/.ping_watch.cron
tPING_CRON=$DIR/.ping_watch.cron.tmp

CONV=./conv_get.cgi
. $CONV

for n in 0 1 2 3 ; do
  if [ -n "${ip[$n]}" ] && [ "${reg[$n]}" = "reg" ];then
    if [ "${ping_don[$n]}" != "none" ];then
      if [ -e "$PING_DON" ];then
        cat "$PING_DON" | grep -F -v "${ip[$n]}" > "$PING_TMP"
        mv "$PING_TMP" "$PING_DON"
      fi
      [ -z "${ping_don_time[$n]}" ] && ping_don_wtime[$n]="*" || ping_don_wtime[$n]=ping_don_time[$n]
      echo "$PING_WATCH" "${ip[$n]}" "${ping_don[$n]}" "${ping_don_wtime[$n]}" "${interval[0]}" >> "$PING_DON"
      if [ -e "$PING_CRON" ];then
        cat "$PING_CRON" | grep -F -v "$PING_WATCH" > "$tPING_CRON"
        cat "$PING_CRON" | grep -F "$PING_WATCH" | grep -F -v "${ip[$n]}" > "$PING_TMP"
        cat "$PING_TMP" >> "$tPING_CRON"
        mv "$tPING_CRON" "$PING_CRON"
      fi
      cat >> "$PING_CRON" <<END
0-59/${interval[0]} * * * * $PING_WATCH ${ip[$n]} ${ping_don[$n]} ${ping_don_time[$n]}
END
    fi
  fi
  if [ "${reg[$n]}" = "del" ];then
    if [ -e "$PING_CRON" ];then
      cat "$PING_CRON" | grep -F -v "$PING_WATCH" > "$tPING_CRON"
      cat "$PING_CRON" | grep -F "$PING_WATCH" | grep -F -v "${ip[$n]}" > "$PING_TMP"
      cat "$PING_TMP" >> "$tPING_CRON"
      mv "$tPING_CRON" "$PING_CRON"
    fi
    if [ -e "$PING_DON" ];then
      cat "$PING_DON" | grep -F -v "${ip[$n]}" > "$PING_TMP"
      mv "$PING_TMP" "$PING_DON"
    fi
  fi
done
if [ -e "$PING_DON" ];then
  if [ `cat "$PING_DON" | wc -l` = "0" ]; then
    rm -f "$PING_DON"
  fi
fi
if [ -e "$PING_CRON" ];then
  LEN=`cat $PING_CRON | wc -l`
  if [ $LEN != 0 ];then 
    crontab $PING_CRON
  else
    crontab -r
    rm -f $PING_CRON
  fi  
fi
