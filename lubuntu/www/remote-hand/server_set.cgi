#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2019.6.21

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2019.6.21">
<META http-equiv="Refresh" content="10;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2019-2022 pepolinux.com</TD><TR></TABLE>
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
SET_WEBUSER="${server_val[0]}"
SET_WEBPASSWORD="${server_val[1]}"
cat>$tSTARTUP<<EOF
#!/bin/bash
SET_WEBUSER="${SET_WEBUSER}"
SET_WEBPASSWORD="${SET_WEBPASSWORD}"
[ ! -z "${SET_WEBUSER}" ] && [ ! -z "${SET_WEBPASSWORD}" ] && htpasswd -bc /etc/rc.pepo/password ${SET_WEBUSER} ${SET_WEBPASSWORD} >/dev/null 2>&1
EOF
chmod +x $tSTARTUP
cat>$CMD<<END
#!/bin/bash
cd /usr/bin
DIO_SH=\`ls |grep -E 'dio(0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|22)[low|high]+\$'\`
if [ \`echo \${DIO_SH} | wc -w\` != 0 ];then
  tar cfz ${DIR}/dio_sh.tar.gz \`echo \${DIO_SH}\`
  mv ${DIR}/dio_sh.tar.gz /www/remote-hand/
fi
cd /www/remote-hand/
svc -d /www/pepolinux/tocosd
svc -d /www/pepolinux/diod
msleep 1000
tar cfz /usr/src/pepolinux/back_up.tar.gz ./
svc -u /www/pepolinux/tocosd
svc -u /www/pepolinux/diod
. ${tSTARTUP}
mv ${tSTARTUP} ${STARTUP}
END
