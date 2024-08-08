#!/bin/sh
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2024.2.25

WORKDIR=/www/remote-hand/tmp
DSFILE=${WORKDIR}/.gpio_temp_hum.rrd
FETCHDATA=${WORKDIR}/gpio_rrdfetch.txt
CMD=${WORKDIR}/gpiorrdtoolfetch.pepocmd
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.jpn.org">
<META NAME="Build" content="2023.11.10">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<META http-equiv="Refresh" content="15;URL=/remote-hand/tmp/gpio_rrdfetch.txt">
<TITLE>GPIO Last hour CSV data Create</TITLE>
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
<TABLE ALIGN=CENTER BORDER=0 CELLPADDING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>GPIO rrdtool fetch create</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
if [ -e ${DSFILE} ];then
cat >${CMD}<<EOF
#!/bin/sh
[ -e $FETCHDATA ] && rm -f $FETCHDATA
rrdtool fetch ${DSFILE} MAX -r 60 -s -1h | awk 'BEGIN{print "Date,Temp,Hum,Pres,Gas,IAQ"};!/nan|gpio/{if(length(\$0)==0){next};gsub(/:/,"",\$0);printf("%s,%2.1f,%2.1f,%d,%d,%d\\n",strftime("%Y/%m/%d %H:%M:%S",\$1),\$2,\$3,\$4,\$5,\$6)}' > ${FETCHDATA}
EOF
fi