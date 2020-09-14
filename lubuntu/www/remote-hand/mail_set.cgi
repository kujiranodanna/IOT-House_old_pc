#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.2.24

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META http-equiv="Refresh" content="0;URL=/remote-hand/wait_for.cgi">
<META NAME="Build" content="2018.2.24">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Setting in the system e-mail</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Setting in the system e-mail</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>'

CONV=./conv_get.cgi
. $CONV
CMD=/www/remote-hand/tmp/mail_set.pepocmd
MAIL_SET=/www/remote-hand/tmp/.mail_set.list
MAIL_ADDRESS=$mail_from
MAIL_SERVER=$mail_server
PASSWORD=$password
WEBPASSWORD=$webpassword
WEBUSER=$webuser
MAIL_SELECT=$mail_select
echo "MAIL_ADDRESS="$MAIL_ADDRESS >$MAIL_SET
echo "MAIL_SERVER="$MAIL_SERVER >>$MAIL_SET
echo "PASSWORD="$PASSWORD >>$MAIL_SET
echo "WEBUSER="$WEBUSER >>$MAIL_SET
echo "WEBPASSWORD="$WEBPASSWORD >>$MAIL_SET
echo "MAIL_SELECT="$MAIL_SELECT >>$MAIL_SET
cat >$CMD<<END
#!/bin/bash
if [ "$MAIL_SELECT" = "sendmail" ];then
  SENDMAIL_CF=/etc/mail/sendmail.cf
  tSENDMAIL_CF=/www/remote-hand/tmp/.sendmail.cf.tmp
  if [ "$MAIL_SERVER" != "not.foward" ];then
    cat \$SENDMAIL_CF | awk '/^DS/{sub(/DS.+/,"DS${MAIL_SERVER}\n",\$0)};{print \$0}' > \$tSENDMAIL_CF
  else
    cat \$SENDMAIL_CF | awk '/^DS/{sub(/DS.+/,"DS",$0)}};{print \$0}' > \$tSENDMAIL_CF
  fi
  mv \$tSENDMAIL_CF \$SENDMAIL_CF
  (cd /pepolinux/mail ;rm ping_watch_mail; ln -s  ping_watch_mail.sendmail ping_watch_mail)
#  (cd /www/remote-hand/ ; rm di_contorl1.cgi ; ln -s di_contorl1_sendmail.cgi di_contorl1.cgi)
#  (cd /www/remote-hand/ ; rm di_contorl2.cgi ; ln -s di_contorl2_sendmail.cgi di_contorl2.cgi)
makemap hash /etc/aliases.db </etc/aliases
service sendmail restart
elif [ "$MAIL_SELECT" = "wget" ];then
  (cd /pepolinux/mail ;rm ping_watch_mail ; ln -s ping_watch_mail.wget ping_watch_mail)
#  (cd /www/remote-hand/ ; rm di_contorl1.cgi; ln -s di_contorl1_wget.cgi di_contorl1.cgi)
#  (cd /www/remote-hand/ ; rm di_contorl2.cgi; ln -s di_contorl2_wget.cgi di_contorl2.cgi)
fi
END

