#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2019.12.15
PATH=$PATH:/usr/local/bin
# get ppp_user name & ppp mode
DIR=/www/remote-hand/tmp
CMD=$DIR/wait_for.pepocmd
PPP_USR=$DIR/.ppp_user
PPP_SEC=/etc/ppp/chap-secrets
PPP_MODE=$DIR/.ppp_mode
PPP_DIAL_MODE=/etc/ppp/ppp-dialer-mode
LOCKFILE="$DIR/LCK..wait_for.cgi"
LOCKPID="$DIR/LCK..wait_for.cgi.pid"
LOCKCGI="$DIR/LCK..pi_int.cgi"
LOCKCGIPID="$DIR/LCK..pi_int.cgi.pid"
ALIAS_DI=$DIR/.alias_di
[ -e $ALIAS_DI ] && . $ALIAS_DI
if [ $DI_TTY != "gpio" ];then
   PI_INT=pi_int.cgi
else
   PI_INT=pi_int_gpio.cgi
fi
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2019.12.15">
<META NAME="reply-to" content="izamu@pepolinux.com">
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2019-2022 pepolinux.com</TD><TR></TABLE>
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
while [ -e ${LOCKCGI} ]
do
  msleep 20000
  rm -rf ${LOCKCGI}
  rm -rf ${LOCKCGIPID}
done
if [ -e "./pi_int.html" ];then
  echo -en '
var jump_url = setTimeout("jump_href()", 500);
function jump_href() {
  var jump_location = "./pi_int.html?" + (new Date().getTime());
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2019-2022 pepolinux.com</TD></TR></TABLE>
</BODY>
</HTML>'
  msleep 1000
  ./${PI_INT} >/dev/null 2>&1
  chown www-data.www-data ./pi_int.html
else
  if [ $DI_TTY = "gpio" ];then
    echo -en '
var jump_url = setTimeout("jump_href()", 1500);
function jump_href() {
  var jump_location = "/remote-hand/pi_int_gpio.cgi?" + (new Date().getTime());
'
  else
    echo -en '
var jump_url = setTimeout("jump_href()", 1500);
function jump_href() {
  var jump_location = "/remote-hand/pi_int.cgi?" + (new Date().getTime());
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
<TABLE ALIGN=RIGHT><TR><TD>&copy;2019-2022 pepolinux.com</TD></TR></TABLE>
</BODY>
</HTML>'
fi
if [ -e ${LOCKFILE} ];then
  [ $$ = `cat ${LOCKPID}` ] && rm ${LOCKFILE} && rm ${LOCKPID}
fi
