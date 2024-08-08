#!/bin/bash
# The MIT License
# Copyright (c) 2021-2027 Isamu.Yamauchi , update 2023.8.15
PATH=$PATH:/usr/local/bin
# for i386
if [ ! -e pi_int_gpio.cgi ];then
  ln -s pi_int_cp2112.cgi pi_int_gpio.cgi
fi
DIR=/www/remote-hand/tmp
CMD=$DIR/wait_for.pepocmd
PPP_USR=$DIR/.ppp_user
PPP_SEC=/etc/ppp/chap-secrets
PPP_MODE=$DIR/.ppp_mode
PPP_DIAL_MODE=/etc/ppp/ppp-dialer-mode
LOCKFILE="$DIR/LCK..wait_for.cgi"
LOCKPID="$DIR/LCK..wait_for.cgi.pid"
LOCKCGI="$DIR/LCK..pi_int_cp2112.cgi"
LOCKCGIPID="$DIR/LCK..pi_int_cp2112.cgi.pid"
HOMEPAGE=./pi_int_cp2112.html
RMHOMEPAGE="NO"
NOWTIME=`date +%s`
JITTER=5
ALIAS_DI=$DIR/.alias_di
if [ -e $ALIAS_DI ];then
  timeSTAMP=`date +%s -r $ALIAS_DI`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
ALIAS_DO=$DIR/.alias_do
if [ -e $ALIAS_DO ];then
  timeSTAMP=`date +%s -r $ALIAS_DO`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
DICHANG1=$DIR/.di_change1
if [ -e $DICHANG1 ];then
  timeSTAMP=`date +%s -r $DICHANG1`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
DICHANG2=$DIR/.di_change2
if [ -e $DICHANG1 ];then
  timeSTAMP=`date +%s -r $DICHANG2`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
AUTOACT_LIST=$DIR/.auto_act.list
if [ -e $AUTOACT_LIST ];then
  timeSTAMP=`date +%s -r $AUTOACT_LIST`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
STARTUP=$DIR/.startup.s.tmp
if [ -e $STARTUP ];then
  timeSTAMP=`date +%s -r $STARTUP`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
SOUND=$DIR/.sound_file_name
if [ -e $SOUND ];then
  timeSTAMP=`date +%s -r $SOUND`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
GMAIL=$DIR/.pepogmail4dio.conf
if [ -e $GMAIL ];then
  timeSTAMP=`date +%s -r $GMAIL`
  [ $(($NOWTIME - $timeSTAMP)) -lt $JITTER ] && RMHOMEPAGE="YES"
fi
[ -e $ALIAS_DI ] && . $ALIAS_DI
if [ $DI_TTY != "gpio" ];then
   PI_INT=pi_int_cp2112.cgi
   cat>$CMD<<END
#!/bin/bash
rm -f /www/remote-hand/pepocp2112help
END
else
   PI_INT=pi_int_cp2112.cgi
   cat>$CMD<<END
#!/bin/bash
rm -f /www/remote-hand/pepocp2112help
ln -s /usr/local/bin/pepocp2112help /www/remote-hand/pepocp2112help
END
fi

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.jpn.org">
<META NAME="Build" content="2019.12.15">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Remote-hand wait for process</TITLE>
<script type="text/javascript">
<!--
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
}'
# wait proceed
BUSY=`ls $DIR/|grep -E ".pepocmd$"|wc -w`
while [ "$BUSY" != 0 ]
do
  msleep 500
  BUSY=`ls $DIR/|grep -E ".pepocmd$"|wc -w`
done
while [ -e ${LOCKFILE} ]
do
  msleep 5000
  rm -rf ${LOCKFILE}
done
lockfile -3 -r 5 ${LOCKFILE} >/dev/null 2>&1
if [ $? != 0 ];then
echo -en '
var jump_url = setTimeout("jump_href()", 10000);
function jump_href() {
  var jump_location = "/index.html?" + (new Date().getTime());
  location.href=jump_location;
  clearTimeout(jump_url);
}
</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<HR>
<TABLE ALIGN=CENTER BGCOLOR="#E0FFFF" BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Server busy</TD></TR>
<TR ALIGN=CENTER><TD>Please wait</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
exit -1
else
while [ -e ${LOCKCGI} ]
do
  echo -en '
var jump_url = setTimeout("jump_href()", 10000);
function jump_href() {
  var jump_location = "/index.html?" + (new Date().getTime());
  location.href=jump_location;
  clearTimeout(jump_url);
}
</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<HR>
<TABLE ALIGN=CENTER BGCOLOR="#E0FFFF" BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Server busy</TD></TR>
<TR ALIGN=CENTER><TD>Please wait</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
  msleep 20000
  rm -rf ${LOCKCGI}
  rm -rf ${LOCKCGIPID}
  exit -1
done
  echo -en $$ >${LOCKPID}
fi

if [ -e $PPP_SEC ]; then
cat>$CMD<<EOF
#!/bin/bash
ppp_user=\`cat $PPP_SEC |grep -F -v "#"|awk 'BEGIN{FS="\""};{print \$2}'\`
echo \$ppp_user > $PPP_USR
chown www-data.www-data $PPP_USR
ppp_mode=\`cat $PPP_DIAL_MODE |grep -E "64k=" |awk '{
split(\$1,I,"=")
if (I[2] == "yes")
  printf("yes 64kDigital communications")
else if (I[2]  == "no")
  printf("no Packet communications")
else if (I[2] == "NONE")
  printf("NONE none")
}'\`

echo \$ppp_mode >$PPP_MODE
chown www-data.www-data $PPP_MODE
EOF
fi

# get web user name
WEBPASS=/etc/rc.pepo/password
TMPWEB=$DIR/.htpasswd
if [ -e $WEBPASS ]; then
cat>>$CMD<<EOF
WEB_USER=\`cat $WEBPASS |grep -F -v "#"|awk 'BEGIN{FS=":"};{print \$1}'\`
echo "\$WEB_USER" > $TMPWEB
chown www-data.www-data $TMPWEB
EOF
fi

cat>>$CMD<<EOF
# get server setting data
STARTUP=/usr/src/pepolinux/startup.s
tSTARTUP=$DIR/.startup.s.tmp
rm -f \$tSTARTUP
if [ -e \$STARTUP ];then
  cat \$STARTUP |awk '/^SET_/{print \$0}' >\$tSTARTUP
  . \$tSTARTUP
  echo "vWEBUSER=\$SET_WEBUSER" >> \$tSTARTUP
  echo "vWEBPASSWORD=\$SET_WEBPASSWORD" >> \$tSTARTUP
  echo "vLINENOTIFY=\$SET_LINENOTIFY" >> \$tSTARTUP
  chown www-data.www-data \$tSTARTUP
fi
EOF

# wait proceed
BUSY=`ls /$DIR|grep -E ".pepocmd$"|wc -w`
while [ "$BUSY" != 0 ]
do
  msleep 500
  BUSY=`ls $DIR/|grep -E ".pepocmd$"|wc -w`
done
[ $RMHOMEPAGE = "YES" ] && rm $HOMEPAGE

while [ -e ${LOCKCGI} ]
do
  msleep 20000
  rm -rf ${LOCKCGI}
  rm -rf ${LOCKCGIPID}
done
if [ -e "./pi_int_cp2112.html" ];then
  echo -en '
var jump_url = setTimeout("jump_href()", 500);
function jump_href() {
  var jump_location = "./pi_int_cp2112.html?" + (new Date().getTime());
  location.href=jump_location;
  clearTimeout(jump_url);
}
</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<HR>
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Now processing</TD></TR>
<TR ALIGN=CENTER><TD>Please wait</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024 pepolinux.jpn.org</TD></TR></TABLE>
</BODY>
</HTML>'
  msleep 1000
  ./${PI_INT} >/dev/null 2>&1
  chown www-data.www-data ./pi_int_cp2112.html
else
  if [ $DI_TTY = "cp2112" ];then
    echo -en '
var jump_url = setTimeout("jump_href()", 1500);
function jump_href() {
  var jump_location = "/remote-hand/pi_int_cp2112.cgi?" + (new Date().getTime());
'
  else
    echo -en '
var jump_url = setTimeout("jump_href()", 1500);
function jump_href() {
  var jump_location = "/remote-hand/pi_int_cp2112.cgi?" + (new Date().getTime());
'
  fi
  echo -en '
  location.href=jump_location;
  clearTimeout(jump_url);
}
</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<HR>
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Now processing</TD></TR>
<TR ALIGN=CENTER><TD>Please wait</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024 pepolinux.jpn.org</TD></TR></TABLE>
</BODY>
</HTML>'
fi
if [ -e ${LOCKFILE} ];then
  [ $$ = `cat ${LOCKPID}` ] && rm ${LOCKFILE} && rm ${LOCKPID}
fi
