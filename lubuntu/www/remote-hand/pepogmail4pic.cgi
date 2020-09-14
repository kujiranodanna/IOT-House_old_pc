#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2012.7.11 update 2018.2.24

# pepogmail4jpg.cgi ; get video or jpeg & send mail
# pepogmail4jpg mail_to subject message
# /usr/local/bin/peposendmail $1:mail_to, $2:subject, $3:message ,$4:image.mp4

PATH=$PATH:/usr/local/bin:/usr/local/sbin
DIOCMD=/www/remote-hand/tmp/pepogmail4pic_ajax.pepocmd
FFMPEGCTL=/usr/local/bin/pepomp4ctl
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2018.2.24">
<META http-equiv="Refresh" content="0;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Video is being retrieved</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Video is being retrieved</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'
CONV=./conv_get.cgi
. $CONV

cat>$DIOCMD<<END
#!/bin/bash
#IMAGE=remote_hand.jpg
IMAGE=remote_hand.mp4
WORKDIR=/www/remote-hand/tmp
WORKIMAGE=\${WORKDIR}/\${IMAGE}
WGETMAIL=/usr/local/bin/peposendmail
MAIL_TO=$mail_address
SUBJECT="remote_hand+picture"
MESSAGE="remote_hand+picture"
error(){
  exit 0
}

trap error SIGINT SIGTERM SIGHUP SIGKILL

VIDEO=/dev/video0
$FFMPEGCTL \$VIDEO \$IMAGE \$$
if [ ! -e \$WORKIMAGE ];then
  exit -1
fi
if [ -e \$WORKIMAGE ];then
  \$WGETMAIL \$MAIL_TO \$SUBJECT \$MESSAGE \$IMAGE
  rm -f \$WORKIMAGE
fi
rm -f $DIOCMD
END

