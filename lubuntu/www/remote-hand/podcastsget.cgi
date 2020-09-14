#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2015.5.17 update 2017.9.1

# podcastgets.cgi for podacasts auto get contorol 
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2017.9.1">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Podcast get set in</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Podcast get set in</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>
</HTML>
'
DIR=/www/remote-hand/tmp
CMD=$DIR/podcasts.pepocmd
EXEC_PODCAST=$DIR/podcasts.sh
WGET_LIST=$DIR/.podcasts_list
WGET_CRON=$DIR/.ping_watch.cron
tWGET_CRON=$DIR/.ping_watch.cron.tmp
WGET_SH=/usr/local/bin/pepopodcastget
#XML=http://www3.nhk.or.jp/rj/podcast/rss/english.xml
CONV=./conv_get.cgi
. $CONV

if [ "${wget_val[2]}" = "reg" ];then
  cat>$EXEC_PODCAST<<END
#!/bin/bash
$WGET_SH ${wget_val[0]} ${wget_val[1]} ${wget_val[8]} ${wget_val[9]}
END
  chmod +x $EXEC_PODCAST
  if [ -e "$WGET_CRON" ];then
    cat "$WGET_CRON" | grep -F -v "$EXEC_PODCAST" > "$tWGET_CRON"
    mv "$tWGET_CRON" "$WGET_CRON"
  fi
  [ -e $WGET_LIST ] && rm -f $WGET_LIST
  for n in 0 1 2 3 4 5 6 7 8 9;do
    echo wget_val[$n]="${wget_val[$n]}" >> $WGET_LIST
  done
  cat>>"$WGET_CRON"<<END
${wget_val[3]} ${wget_val[4]} ${wget_val[5]} ${wget_val[6]} ${wget_val[7]} cp -f $EXEC_PODCAST $CMD
END
fi
if [ "${wget_val[2]}" = "del" ];then
      cat "$WGET_CRON" | grep -F -v "$EXEC_PODCAST" > "$tWGET_CRON"
      mv "$tWGET_CRON" "$WGET_CRON"
      [ -e $WGET_LIST ] && rm -f $WGET_LIST
fi
if [ -e "$WGET_CRON" ];then
  WLEN=`cat $WGET_CRON | wc -l`
  if [ $WLEN != 0 ];then
    crontab "$WGET_CRON"
  else 
    crontab -r
	rm -f $WGET_CRON
  fi
fi
