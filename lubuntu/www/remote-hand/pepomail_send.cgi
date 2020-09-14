#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2015.3.15

# pepomail_send.cgi ; Attach the image file and send mail, use the mutt
# licence GPLv3 ; this scripts designed by Yamauchi Isamu 2015.3.16 update 2018.2.24
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2015.3.15">
<META http-equiv="Refresh" content="10;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>メール送信中です。</TITLE>
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
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>メール送信中です。</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'
CONV=./conv_get.cgi
. $CONV

WORKDIR=/www/remote-hand/tmp
MAIL_CMD=$WORKDIR/mail_test_pepocmd
WGETMAIL=/usr/local/bin/peposendmail
MAIL_TO=$mail_to
SUBJECT=$subject
MESSAGE=$msg
IMAGE=$image_file

cat>$MAIL_CMD<<END
#!/bin/bash
error(){
  exit 0
}
trap error SIGINT SIGTERM SIGHUP SIGKILL
$WGETMAIL $MAIL_TO $SUBJECT $MESSAGE $IMAGE
rm -f $MAIL_CMD
END
