#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.10.14

# gamil_set.cgi 
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META http-equiv="Refresh" content="0;URL=/remote-hand/wait_for.cgi">
<META NAME="Build" content="2018.10.14">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Setting in DIO operation in Gmail</TITLE>
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
<TABLE ALIGN=CENTER BGCOLOR="#E0FFFF" BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Setting in DIO operation in Gmail</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD></TR></TABLE>
</BODY>
</HTML>'
CONV=./conv_get.cgi
. $CONV
GMAILUSER="$gmailuser"
GMAILPASSWORD="$gmailpassword"
PERMITMAIL="$permitmail"
KEYWORD="$keyword"
JITTER="$jitter"
#// pepo 2018.10.14 "&reg" "&REG" IE & Firefox not use!
#REG="$reg"
REG="$peporeg"
LOOPTIME="$looptime"
CONF="/www/remote-hand/tmp/.pepogmail4dio.conf"
CMD="/www/remote-hand/tmp/pepogmail4dio.pepocmd"
MAIL_CONF_ORG="/etc/exim4/passwd.client.org"
MAIL_CONF="/etc/exim4/passwd.client"
tMAIL_CONF="/www/remote-hand/tmp/.gmail.cgi.tmp"
if [ $REG = "del" ];then
cat>$CMD<<END
  [ -e "$CONF" ] && rm "$CONF"
  rm $MAIL_CONF
  msleep 100
  update-exim4.conf
END
  exit 0
fi
cat >$CONF<<END
GMAILUSER="$gmailuser"
GMAILPASSWORD="$gmailpassword"
PERMITMAIL="$permitmail"
KEYWORD="$keyword"
JITTER="$jitter"
LOOPTIME="$looptime"
END
cat>$CMD<<END
#!/bin/bash
if [ ! -e ${MAIL_CONF_ORG} ];then
cat >${MAIL_CONF_ORG}<<EOF
# password file used when the local exim is authenticating to a remote
# host as a client.
#
# see exim4_passwd_client(5) for more documentation
#
# Example:
### target.mail.server.example:login:password
gmail-smtp.l.google.com:YOUR-USER-NAME@gmail.com:YOUR-USER-PASSWORD
*.google.com:YOUR-USER-NAME@gmail.com:YOUR-USER-PASSWORD
smtp.gmail.com:YOUR-USER-NAME@gmail.com:YOUR-USER-PASSWORD
EOF
fi
cat $MAIL_CONF_ORG | awk '
/^gmail-smtp.l.google.com:/{print "gmail-smtp.l.google.com:${GMAILUSER}@gmail.com:${GMAILPASSWORD}";next}
/^*.google.com:/{print "*.google.com:${GMAILUSER}@gmail.com:${GMAILPASSWORD}";next}
/^smtp.gmail.com:/{print "smtp.gmail.com:${GMAILUSER}@gmail.com:${GMAILPASSWORD}";next}
  {print \$0}' >$tMAIL_CONF
[ -e $tMAIL_CONF ] && mv $tMAIL_CONF $MAIL_CONF
update-exim4.conf
#cp $MAIL_CONF_ORG $MAIL_CONF
END
