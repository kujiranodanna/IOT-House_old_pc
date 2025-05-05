#!/bin/bash
# The MIT License
# Copyright (c) 2024-2027 Isamu.Yamauchi ,2024.8.20 update 2024.8.20

PATH=$PATH:/usr/local/bin
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2024.8.20">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>File delete</TITLE>
<script type="text/javascript">
function blink() {
  // if (!document.all) { return; }
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
<TR ALIGN=CENTER class="blink"><TD>Processing Upload Image File</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2024-2026 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
prog=file_del
CONV=./conv_get.cgi
. $CONV
if [ -n $file_name ];then
FILE_NAME=$DIR/$file_name
CMD=$DIR/${prog}_$$.pepocmd
cat>$CMD<<END
#!/bin/sh
rm $FILE_NAME
END
fi
echo -n '
</HTML>'
