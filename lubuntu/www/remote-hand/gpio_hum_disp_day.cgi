#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2018.10.25

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2018.10.25">
<META NAME="reply-to" content="izamu@pepolinux.com">
<META http-equiv="Refresh" content="10;URL=/remote-hand/tmp/gpio_hum.png">
<TITLE>GPIO humature Graph Create</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>GPIO Humidity graph create</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>'
WORKDIR=/www/remote-hand
DSFILE=${WORKDIR}/tmp/.gpio_temp_hum.rrd
RRDSTART=${WORKDIR}/tmp/.gpio_temp_hum_start_time
GRAP_TEMPFILE=${WORKDIR}/tmp/gpio_hum.png
END=`date "+%Y%m%d %H:%M:00" -d "-10 minute"`
END=`date -d "${END}" +%s`
CMD=${WORKDIR}/tmp/gpio_hum.pepocmd
if [ -e ${DSFILE} ];then
  [ -e ${RRDSTART} ] && . ${RRDSTART}
  TEMPSTART=`date "+%Y%m%d %H:%M:00" -d "-1 days"`
  TEMPSTART=`date -d "${TEMPSTART}" +%s`
  if [ ${TEMPSTART} -gt ${START} ];then
    START=${TEMPSTART}
  fi
  [ -e $GRAP_TEMPFILE ] && rm -f $GRAP_TEMPFILE
cat >${CMD}<<EOF
#!/bin/bash
rrdtool graph ${GRAP_TEMPFILE} --width 600 --height 120 --start ${START} --end ${END} --upper-limit 100 --lower-limit 0 DEF:a=${DSFILE}:gpio_hum:MAX LINE1:a#FF0000:"GPIO Humidity"
EOF
fi
