#!/bin/bash
PATH=$PATH:/usr/local/bin
STOP_FOMA="svc -d /service/ppp-foma"
RUN_FOMA="svc -u /service/ppp-foma"
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="1;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>PPP set in</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>PPP set in</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2018-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'
CONV=./conv_get.cgi
. $CONV
CMD=/www/remote-hand/tmp/ppp_user.pepocmd
PPP_CAHP=/etc/ppp/chap-secrets
PPP_MODE=/etc/ppp/ppp-dialer-mode
PPP_USER=$user
PPP_PASS=$password
PPP_64K='64k='$pppmode
MODEM_DEV=/www/remote-hand/tmp/.modem
MODEM=$modem
cat>$CMD<<EOF
#!/bin/bash
$STOP_FOMA
msleep 500
cat >$PPP_CAHP<<END
# Secrets for authentication using CHAP
# client	server	secret	IP addresses
"$PPP_USER"	"*"	"$PPP_PASS"	"*" 
END
cat >$PPP_MODE<<END
# ppp-dail-mode
# 64k-digital or the packet is chosen here. 64k yes or no
$PPP_64K
END
cat >$MODEM_DEV<<END
modem_dev=$MODEM
END
msleep 500
PPPRUN=`ps ax |awk 'BEGIN{I="NO"};/pppd$/{I="YES"};END{printf I}'`
if [ \$PPPRUN = "YES" ];then
  killall pppd
fi
$RUN_FOMA
EOF
