#!/bin/bash
# The MIT License
# Copyright (c) 2021-2028 Isamu.Yamauchi , update 2022.9.28

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.osdn.jp">
<META NAME="build" content="2022.9.28">
<META http-equiv="Refresh" content="10;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.osdn.jp">
<TITLE>Server initial settings</TITLE>
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2023 pepolinux.osdn.jp</TD><TR></TABLE>
</BODY>
</HTML>'

CONV=./conv_get.cgi
. $CONV
DIR=/www/remote-hand/tmp
CMD=${DIR}/server_set.pepocmd
STARTUP=/usr/src/pepolinux/startup.s
tSTARTUP=${DIR}/.t_startup.s.tmp
BACKUP=/www/remote-hand/backup.tar.gz
DIO_BACKUP=/www/remote-hand/dio_sh.tar.gz
LINENOTIFY_FILE=/etc/rc.pepo/linenotify
SET_WEBUSER="${server_val[0]}"
SET_WEBPASSWORD="${server_val[1]}"
SET_LINENOTIFY="${server_val[2]}"
cat>$tSTARTUP<<EOF
#!/bin/bash
SET_WEBUSER="${SET_WEBUSER}"
SET_WEBPASSWORD="${SET_WEBPASSWORD}"
[ ! -z "${SET_WEBUSER}" ] && [ ! -z "${SET_WEBPASSWORD}" ] && htpasswd -bc /etc/rc.pepo/password ${SET_WEBUSER} ${SET_WEBPASSWORD} >/dev/null 2>&1
SET_LINENOTIFY=\`[ -e $LINENOTIFY_FILE ] && echo -en "OK" || echo -en ""\`
EOF
chmod +x $tSTARTUP
cat>$CMD<<END
#!/bin/bash
if [ ! -z "${SET_LINENOTIFY}" ];then
  echo -en $SET_LINENOTIFY >${LINENOTIFY_FILE}
else
  rm ${LINENOTIFY_FILE}
fi
. ${tSTARTUP}
mv ${tSTARTUP} ${STARTUP}
cd /usr/bin
DIO_SH=\`ls |grep -E 'dio(0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|22)[low|high]+\$'\`
if [ \`echo \${DIO_SH} | wc -w\` != 0 ];then
  tar cfz ${DIR}/dio_sh.tar.gz \`echo \${DIO_SH}\`
  mv ${DIR}/dio_sh.tar.gz /www/remote-hand/
fi
msleep 1000
cd /www/remote-hand/
tar cfz /www/tmp/back_up.tar.gz ./
mv /www/tmp/back_up.tar.gz /usr/src/pepolinux/
END
