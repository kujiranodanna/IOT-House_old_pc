#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2022.8.9

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.osdn.jp">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="Build" content="2018.2.24">
<META NAME="reply-to" content="izamu@pepolinux.osdn.jp">
<TITLE>Setting in Ping monitoring and phone</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Setting in Ping monitoring and phone</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.osdn.jp</TD><TR></TABLE>
</BODY>
</HTML>'

CMD=/www/remote-hand/tmp/ping_watch_phone.pepocmd
PING_TEL=/www/remote-hand/tmp/.ping_phone_list
PING_TMP=/www/remote-hand/tmp/.ping_phone_list.tmp
PING_WATCH=/www/pepolinux/phone/ping_watch_phone
PING_CRON=/www/remote-hand/tmp/.ping_watch.cron
tPING_CRON=/www/remote-hand/tmp/.ping_watch.cron.tmp

CONV=./conv_get.cgi
. $CONV

for n in 0 1 2 3 ; do
  if [ -n "${ip[$n]}" ] && [ "${reg[$n]}" = "reg" ] && [ -n "${tel[$n]}" ];then
    if [ -e "$PING_TEL" ];then
      cat "$PING_TEL" | grep -F -v "${ip[$n]}" > "$PING_TMP"
      mv "$PING_TMP" "$PING_TEL"
    fi
    echo "$PING_WATCH" "${ip[$n]}" "${tel[$n]}" "${interval[0]}" >> "$PING_TEL"
    if [ -e "$PING_CRON" ];then
      cat "$PING_CRON" | grep -F -v "$PING_WATCH" > "$tPING_CRON"
      cat "$PING_CRON" | grep -F "$PING_WATCH" | grep -F -v "${ip[$n]}" > "$PING_TMP"
      cat "$PING_TMP" >> "$tPING_CRON"
      mv "$tPING_CRON" "$PING_CRON"
    fi
    cat >> "$PING_CRON" <<END
0-59/${interval[0]} * * * * $PING_WATCH ${ip[$n]} ${tel[$n]}
END
  fi
  if [ "${reg[$n]}" = "del" ];then
    if [ -e "$PING_CRON" ];then
      cat "$PING_CRON" | grep -F -v "$PING_WATCH" > "$tPING_CRON"
      cat "$PING_CRON" | grep -F "$PING_WATCH" | grep -F -v "${ip[$n]}" > "$PING_TMP"
      cat "$PING_TMP" >> "$tPING_CRON"
      mv "$tPING_CRON" "$PING_CRON"
    fi
    if [ -e "$PING_TEL" ];then
      cat "$PING_TEL" | grep -F -v "${ip[$n]}" > "$$PING_TMP"
      mv "$$PING_TMP" "$PING_TEL"
    fi
  fi
done
if [ -e "$PING_TEL" ];then
  if [ `cat "$PING_TEL" | wc -l` = "0" ]; then
    rm -f "$PING_TEL"
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
