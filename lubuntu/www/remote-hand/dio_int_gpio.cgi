#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2020.7.9

PATH=$PATH:/usr/local/bin
DIR=/www/remote-hand/tmp
LOCKFILE="$DIR/LCK..pi_int.cgi"
LOCKPID="$DIR/LCK..pi_int.cgi.pid"
DATE="2020.7.9"
VERSION="ver:0.13&nbsp;$DATE"
ZEROW=`gpio readall|grep "Pi ZeroW"|wc -w`
[ $ZEROW != 0 ] && ZEROW_YES_NO="YES" || ZEROW_YES_NO="NO"
if [ $ZEROW_YES_NO = "YES" ];then
  DIST_NAME="IOT-House_zero_w"
else
  DIST_NAME="IOT-House_pi"
fi
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=utf-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="$DATE">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>$DIST_NAME command execution</TITLE>
<script type="text/javascript">
<!--
function blink() {
  for (var i = 0; i < document.all.length; i++) {
    obj = document.all(i);
    if (obj.className == "blink") {
      if (obj.style.visibility == "visible") {
        obj.style.visibility = "hidden";
      }
      else {
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
lockfile -3 -r 5 ${LOCKFILE} >/dev/null 2>&1
if [ $? != 0 ];then
echo -en '
var jump_url = setTimeout("jump_href()", 20000);
function jump_href() {
  var jump_location = "./pi_int.html?" + (new Date().getTime());
  location.href=jump_location;
  clearTimeout(jump_url);
}
</script>
</HEAD>
<BODY onload="blink()" BGCOLOR="#E0FFFF">
<HR>
<TABLE ALIGN=CENTER BGCOLOR="#E0FFFF" BORDER=0 CELLPADING=6 CELLSPACING=2>
<TR ALIGN=CENTER class="blink"><TD>Server busy</TD></TR>
<TR ALIGN=CENTER><TD>Please wait</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2019-2022 pepolinux.com</TD></TR></TABLE>
</BODY>
</HTML>'
  exit -1
else
  echo -en $$ >${LOCKPID}
fi
PAGE1=pi_int.html.tmp
PAGE2=pi_int.html
PAGE3=setup.html.tmp
PAGE4=setup.html
PAGE5=temp_hum.html
echo_f() {
  local DT FL
  DT=$1
  FL=$PAGE1
  echo -en $DT |cat >>$FL
}
disp_int() {
  local INT
  INT="$1"
  echo_f "$INT:"
  echo_f `ip addr show $INT |awk '/inet/{printf $2}'`
}
DIRD=$DIR/.di_read_data
DOWD=$DIR/.do_write_data
ALIAS_DI=$DIR/.alias_di
ALIAS_DO=$DIR/.alias_do
ALIAS_VDO=$DIR/.alias_vdo
[ -e "$DIRD" ] && . "$DIRD"
[ -e "$DOWD" ] && . "$DOWD"
[ -e "$ALIAS_DI" ] && . "$ALIAS_DI"
[ -e "$ALIAS_DO" ] && . "$ALIAS_DO"
[ -e "$ALIAS_VDO" ] && . "$ALIAS_VDO" 
for n in 0 1 2 3 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25;do
  DI[$n]="none"
  DO[$n]="none"
  if [ "${di[$n]}" = "1" ];then
    DI[$n]="high"
  elif [ "${di[$n]}" = "0" ];then
    DI[$n]="low" 
  fi
  if [ "${do[$n]}" = "1" ];then
    DO[$n]="high" 
  elif [ "${do[$n]}" = "0" ];then
    DO[$n]="low"
  fi
  if [ "${ai[$n]}" = "1" ];then
    ai[$n]="high" 
  elif [ "${ai[$n]}" = "0" ];then
    ai[$n]="low"
  fi
  [ -n "${don_time[$n]}" ] && DON_TIME[$n]="${don_time[$n]}" || DON_TIME[$n]=""
  [ -n "${alias_di[$n]}" ] && ALIAS_DI[$n]="${alias_di[$n]}" || ALIAS_DI[$n]="Input"$(($n + 1))
  [ -n "${alias_do[$n]}" ] && ALIAS_DO[$n]="${alias_do[$n]}" || ALIAS_DO[$n]="Output"$(($n + 1))
  [ -n "${slice_ai[$n]}" ] && SLICE_AI[$n]="${slice_ai[$n]}" || SLICE_AI[$n]=""
  done
  [ ${ALIAS_DI[11]} = "Input12" ] && ALIAS_DI[11]="" || ALIAS_DI[11]="*"
  [ ${ALIAS_DI[24]} = "Input25" ] && ALIAS_DI[24]="" || ALIAS_DI[24]="*"
  [ -n "${slice_ai[25]}" ] && SLICE_AI[25]="${slice_ai[25]}" || SLICE_AI[25]="Auto"
n=0
while [ $n -lt 34 ];do
  [ -n "${alias_vdo[$n]}" ] && ALIAS_VDO[$n]="${alias_vdo[$n]}" || ALIAS_VDO[$n]=""
  n=$(($n + 1))
done

SMART_PHONE=`echo "$HTTP_USER_AGENT" |awk 'BEGIN{S_PHONE="NO"};/(iPhone|Android)/{S_PHONE="YES"};END{printf S_PHONE}'`
if [ $SMART_PHONE = "YES" ];then
  cat >$PAGE1<<END
<?xml version="1.0" encoding="UTF-8"?>
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META name="Auther" content="yamauchi.isamu">
<META name="Copyright" content="pepolinux">
<META name="Build" content="$DATE">
<META name="reply-to" content="izamu@pepolinux.com">
<META http-equiv="content-style-type" content="text/css" />
<META http-equiv="content-script-type" content="text/javascript" />
<link rel="stylesheet" href="rasp_phone.css" type="text/css" media="print, projection, screen">
<script src="jquery-1.8.1.min.js" type="text/javascript"></script>
<script src="remote-hand_pi_gpio.min.js" type="text/javascript"></script>
<TITLE>$DIST_NAME Smart Phone Control</TITLE>
</HEAD>
<BODY BGCOLOR="#e0ffff" onload="update_di('onload')" onunload="update_di('onunload')>
<META http-equiv="Refresh" content="120;URL=/remote-hand/pi_int.cgi">
<DIV style="text-align:center"><FONT size="5" color="green">$DIST_NAME<FONT size="2">&nbsp;$VERSION</FONT></FONT></DIV>
<BR>
<FORM NAME="menu5" id="menu5_form"  ACTION="./dio_set.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="+1"><B>Digital output information</B></FONT>
<BR>
<span id="s_phone_do0">
</span>
<span id="s_phone_do1">
</span>
<span id="s_phone_do2">
</span>
<span id="s_phone_do3">
</span>
<span id="s_phone_do8">
</span>
<span id="s_phone_do9">
</span>
<span id="s_phone_do10">
</span>
<span id="s_phone_do11">
</span>
<span id="s_phone_do12">
</span>
<span id="s_phone_do13">
</span>
<span id="s_phone_do14">
</span>
<span id="s_phone_do15">
</span>
<span id="s_phone_do16">
</span>
<HR>
<FONT SIZE="+1"><B>Digital input information</B></FONT>
<BR>
<span id="s_phone_di0">
</span>
<span id="s_phone_di1">
</span>
<span id="s_phone_di2">
</span>
<span id="s_phone_di3">
</span>
<span id="s_phone_di8">
</span>
<span id="s_phone_di9">
</span>
<span id="s_phone_di10">
</span>
<span id="s_phone_di11">
</span>
<span id="s_phone_di12">
</span>
<span id="s_phone_di13">
</span>
<span id="s_phone_di14">
</span>
<span id="s_phone_di15">
</span>
<span id="s_phone_di16">
</span>
<span id="s_phone_di17">
</span>
<span id="s_phone_di18">
</span>
<span id="s_phone_di19">
</span>
<span id="s_phone_di20">
</span>
</span>
<span id="s_phone_di21">
</span>
</span>
<span id="s_phone_di22">
</span>
</span>
<span id="s_phone_di23">
</span>
</FORM>
<HR>
<img border="0" src="./google-microphone.png" width="300" height="300" alt="microphone" onclick="startWebVoiceRecognition();"/>
<BR>
<span id="voice_sel">
Voice control
<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do('voice_sel')" placeholder="Command" autofocus />
</span>
<SELECT NAME="voice_lang" id="voice_lang">
<OPTION VALUE="ja" SELECTED>Japanese
<OPTION VALUE="en">English
</SELECT>
<HR>
<span id="s_phone_cpu_temp_graph"></span>
<span id="s_phone_gpio_temp_graph"></span>
<span id="s_phone_gpio_hum_graph"></span>
<span id="s_phone_gpio_pres_graph"></span>
<span id="s_phone_gpio_gas_graph"></span>
<span id="s_phone_gpio_iaq_graph"></span>
<span id="s_phone_gpio_csv"></span>
<span id="s_phone_i2c_temp_disp"></span>
<span id="s_phone_i2c_hum_disp"></span>
<span id="s_phone_vai_1_graph"></span>
<span id="s_phone_vai_2_graph"></span>
<span id="s_phone_vai_3_graph"></span>
<span id="s_phone_vai_4_graph"></span>
<INPUT style="text-align:center" TYPE="button" VALUE="Temp&Hum Disp" onclick="location.href='./temp_hum.html'";>
<BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Update" onclick="clearTimeout(Update_di_Timer);location.href='./wait_for.cgi'">&nbsp;
<BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Setup" onclick="location.href='./setup.html'";>
<BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Logout" onclick="logout()" ;>
<BR>
<BR>
&copy;2019-2022 pepolinux.com&nbsp;
</H1>
</BODY>
</HTML>
END
  mv ${PAGE1} ${PAGE2}
  PAGE1=$PAGE3
  PAGE2=$PAGE4
  cat >$PAGE5<<END
<?xml version="1.0" encoding="UTF-8"?>
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META name="Auther" content="yamauchi.isamu">
<META name="Copyright" content="pepolinux">
<META name="Build" content="$DATE">
<META name="reply-to" content="izamu@pepolinux.com">
<META http-equiv="content-style-type" content="text/css" />
<META http-equiv="content-script-type" content="text/javascript" />
<link rel="stylesheet" href="rasp_phone.css" type="text/css" media="print, projection, screen">
<script src="jquery-1.8.1.min.js" type="text/javascript"></script>
<script src="remote-hand_pi_gpio.min.js" type="text/javascript"></script>
<TITLE>IOT-House Temperature&Humidity</TITLE>
</HEAD>
<BODY BGCOLOR="#e0ffff" onload="update_di('onload')" onunload="update_di('onunload')>
<META http-equiv="Refresh" content="120;URL=/remote-hand/pi_int.cgi">
<DIV style="text-align:center"><FONT size="5" color="green">$DIST_NAME<FONT size="2">&nbsp;$VERSION</FONT></FONT></DIV>
<span id="s_phone_temp_hum"></span>
<span id="s_phone_gpio_temp_graph"></span>
<span id="s_phone_gpio_hum_graph"></span>
<BR>
<span id="s_phone_gpio_pres_graph"></span>
<span id="s_phone_gpio_gas_graph"></span>
<BR>
<span id="s_phone_gpio_iaq_graph"></span>
<span id="gpio_iaq_val"></span>
<BR>
<span id="s_phone_gpio_csv"></span>
<span id="s_phone_tocos_temp_hum"></span>
<BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Home" onclick="location.href='./pi_int.html'";/>
<BR>
<BR>
&copy;2019-2022 pepolinux.com&nbsp;
<span id="server_time" style="text-align:left"></span>
</H1>
</BODY>
</HTML>
END
fi

# Not Smart Phone
cat >$PAGE1<<END
<?xml version="1.0" encoding="UTF-8"?>
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META name="Auther" content="yamauchi.isamu">
<META name="Copyright" content="pepolinux">
<META name="Build" content="$DATE">
<META name="reply-to" content="izamu@pepolinux.com">
<META http-equiv="content-style-type" content="text/css" />
<META http-equiv="content-script-type" content="text/javascript" />
<link rel="stylesheet" href="ui.tabs.css" type="text/css" media="print, projection, screen">
<script src="jquery-1.8.1.min.js" type="text/javascript"></script>
<script src="ui.core.js" type="text/javascript"></script>
<script src="ui.tabs.js" type="text/javascript"></script>
<script src="remote-hand_pi_gpio.min.js" type="text/javascript"></script>
<script type="text/javascript">
<!--
\$(function() {
  \$('#tab_cont_div > ul').tabs({selected: 1});
  });
// -->
</script>

<TITLE>$DIST_NAME Control Panel</TITLE>
</HEAD>
<BODY id="tab_cont_body" BGCOLOR="#e0ffff" onload="update_di('onload')" onunload="update_di('onunload')>
<META http-equiv="Refresh" content="120;URL=/remote-hand/pi_int.cgi">
<DIV  style="text-align:center"><FONT size="5" color="green">$DIST_NAME</FONT><FONT size="2">&nbsp;$VERSION</FONT></DIV>
<DIV id="tab_cont_div">
<UL id="tab">
<LI><a href="#menu4dl" title="Sound Setting"><span>Sound Setting</span></a></LI>
<LI><a href="#menu5dl" title="DIO Setting"><span>DIO Setting</span></a></LI>
<LI><a href="#menu6dl" title="ping_DO Setting"><span>ping_DO Setting</span></a></LI>
<LI><a href="#menu7dl" title="ping_mail Setting"><span>ping_mail Setting</span></a></LI>
<LI><a href="#menu8dl" title="ping_tel Setting"><span>ping_tel Setting</span></a></LI>
<LI><a href="#menu9dl" title="DIO Control-1"><span>DIO Control1</span></a></LI>
<LI><a href="#menu10dl" title="DIO Control-2"><span>DIO Control2</span></a></LI>
<LI><a href="#menu11dl" title="Mail Setting"><span>Mail Setting</span></a></LI>
<LI><a href="#menu12dl" title="Auto Process"><span>Auto Process</span></a></LI>
<LI><a href="#menu13dl" title="Server Control"><span>Server Control</span></a></LI>
</UL>
END

cat >>$PAGE1<<END
<DL id="menu4dl">
<DT><FONT SIZE="+1"><B>Setting Sound</B></FONT></DT>
<DD>
<FONT SIZE="3"><B>Sound Upload</B></FONT><BR>
<FORM style="display: inline" id="menu4_form" NAME="menu4" ACTION="./sound_set.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
Sound1
<INPUT TYPE="file" id="sound_file_0" style="width:240px;" NAME="sound_file_0" VALUE="">&nbsp;
<INPUT style="text-align:center" TYPE="button" id="menu4_sound_0" VALUE="Upload" onClick="return menu4_ck('menu4_sound_0','disp_sound_0');"/>
&nbsp;
<span id="disp_sound_0">
</span>&nbsp;
<SELECT id="reg_sound_0" NAME="reg_sound_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
Sound2
<INPUT TYPE="file" id="sound_file_1" style="width:240px;" NAME="sound_file_1" VALUE="">&nbsp;
<INPUT style="text-align:center" TYPE="button" id="menu4_sound_1" VALUE="Upload" onClick="return menu4_ck('menu4_sound_1','disp_sound_1');"/>
&nbsp;
<span id="disp_sound_1">
</span>&nbsp;
<SELECT id="reg_sound_1" NAME="reg_sound_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
Sound3
<INPUT TYPE="file" id="sound_file_2" style="width:240px;" NAME="sound_file_2" VALUE="">&nbsp;
<INPUT style="text-align:center" TYPE="button" id="menu4_sound_2" VALUE="Upload" onClick="return menu4_ck('menu4_sound_2','disp_sound_2');"/>
&nbsp;
<span id="disp_sound_2">
</span>&nbsp;
<SELECT id="reg_sound_2" NAME="reg_sound_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
Sound4
<INPUT TYPE="file" id="sound_file_3" style="width:240px;" NAME="sound_file_3" VALUE="">&nbsp;
<INPUT style="text-align:center" TYPE="button" id="menu4_sound_3" VALUE="Upload" onClick="return menu4_ck('menu4_sound_3','disp_sound_3');"/>
&nbsp;
<span id="disp_sound_3">
</span>&nbsp;
<SELECT id="reg_sound_3" NAME="reg_sound_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
Sound5
<INPUT TYPE="file" id="sound_file_4" style="width:240px;" NAME="sound_file_4" VALUE="">&nbsp;
<INPUT style="text-align:center" TYPE="button" id="menu4_sound_4" VALUE="Upload" onClick="return menu4_ck('menu4_sound_4','disp_sound_4');"/>
&nbsp;
<span id="disp_sound_4">
</span>&nbsp;
<SELECT id="reg_sound_4" NAME="reg_sound_4">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<INPUT style="text-align:center" TYPE="button" id="menu4_jikkou" VALUE="Run" onClick="return menu4_ck('menu4_jikkou','disp_jikkou');"/>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
</DD>
</DL>
END

if [ ! -z "${TOCOS_TTY}" ];then
  case ${TOCOS_TTY} in
    ttyUSBTWE-Lite) vTOCOS_TTY="ttyUSBTWE-Lite" ;;
    none) vTOCOS_TTY="none" ;;
  esac
else 
  vTOCOS_TTY="none" ; TOCOS_TTY="none"
fi
if [ -n "${DI_TTY}" ];then
  case ${DI_TTY} in
    gpio) vTTY="gpio" ;;
    piface) vTTY="piface" ;;
    none) vTTY="none" ;;
  esac
else 
   DI_TTY="gpio" ; vTTY="gpio"
fi
MODEM=$DIR/.modem
[ -e $MODEM ] && . $MODEM || modem_dev=none
LIVE_SERVER=`hostname -I`
LIVE_SERVER=`echo -en $LIVE_SERVER`
cat >>$PAGE1<<END
<DL id="menu5dl">
<DT><FONT SIZE="+1"><B>Setting DIO & IRKit & Tocos</B></FONT></DT>
<DD>
<FORM NAME="menu5" id="menu5_form" ACTION="./dio_set.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<B>Setting digital output terminal name</B>
&nbsp;&nbsp;<INPUT style="text-align:center" TYPE="reset" VALUE="Reload" onClick="update_di(onload);"/>
<div id="disp_menu5">Server Synchronized</div>
<span id="do_0">
Output1<INPUT TYPE="text" size="3" id="vdo_0" name="vdo_0" readonly style="width:36px;text-align:center;" VALUE="${DO[0]}">&nbsp;
</span>
<span id="dosel_0">
<SELECT onChange="update_do('dosel_0')" NAME="do_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_0" style="width:36px;text-align:right;" VALUE="${DON_TIME[0]}" NAME="don_time_0">ms&nbsp;
</span>
<INPUT TYPE="text" id="alias_do_0" style="width:100px;" NAME="alias_do_0" VALUE="${ALIAS_DO[0]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_0" style="width:100px;" NAME="alias_vdo_0" VALUE="${ALIAS_VDO[0]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_1" style="width:100px;" NAME="alias_vdo_1" VALUE="${ALIAS_VDO[1]}">&nbsp;
<SELECT NAME="alias_do_reg_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="do_1">
Output2<INPUT TYPE="text"  size="3" id="vdo_1" name="vdo_1" readonly style="width:36px;text-align:center;" VALUE="${DO[1]}">&nbsp;
</span>
<span id="dosel_1">
<SELECT onChange="update_do('dosel_1')" NAME="do_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_1" style="width:36px;text-align:right;" VALUE="${DON_TIME[1]}" NAME="don_time_1">ms&nbsp;
<INPUT TYPE="text" id="alias_do_1" style="width:100px;" NAME="alias_do_1" VALUE="${ALIAS_DO[1]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_2" style="width:100px;" NAME="alias_vdo_2" VALUE="${ALIAS_VDO[2]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_3" style="width:100px;" NAME="alias_vdo_3" VALUE="${ALIAS_VDO[3]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
<span id="do_2">
Output3<INPUT TYPE="text" size="3" id="vdo_2" name="vdo_2" readonly style="width:36px;text-align:center;" VALUE="${DO[2]}">&nbsp;
</span>
<span id="dosel_2">
<SELECT onChange="update_do('dosel_2')" NAME="do_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_2" style="width:36px;text-align:right;" VALUE="${DON_TIME[2]}" NAME="don_time_2">ms&nbsp;
<INPUT TYPE="text" id="alias_do_2" style="width:100px;" NAME="alias_do_2" VALUE="${ALIAS_DO[2]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_4" style="width:100px;" NAME="alias_vdo_4" VALUE="${ALIAS_VDO[4]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_5" style="width:100px;" NAME="alias_vdo_5" VALUE="${ALIAS_VDO[5]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
<span id="do_3">
Output4<INPUT TYPE="text" size="3" id="vdo_3" name="vdo_3" readonly style="width:36px;text-align:center;" VALUE="${DO[3]}">&nbsp;
</span>
<span id="dosel_3">
<SELECT onChange="update_do('dosel_3')" NAME="do_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_3" style="width:36px;text-align:right;" VALUE="${DON_TIME[3]}" NAME="don_time_3">ms&nbsp;
<INPUT TYPE="text" id="alias_do_3" style="width:100px;" NAME="alias_do_3" VALUE="${ALIAS_DO[3]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_6" style="width:100px;" NAME="alias_vdo_6" VALUE="${ALIAS_VDO[6]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_7" style="width:100px;" NAME="alias_vdo_7" VALUE="${ALIAS_VDO[7]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<HR>
<B>Setting IRKit</B>
<BR>
IR1<span id="dosel_8">
<SELECT onChange="update_do('irkitdo_0')" NAME="irkitdo_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="Send">Send
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_8" style="width:36px;text-align:right;" VALUE="${DON_TIME[8]}" NAME="don_time_8">ms&nbsp;
<INPUT TYPE="text" id="alias_do_8" style="width:100px;" NAME="alias_do_8" VALUE="${ALIAS_DO[8]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_16" style="width:100px;" NAME="alias_vdo_16" VALUE="${ALIAS_VDO[16]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_17" style="width:100px;" NAME="alias_vdo_17" VALUE="${ALIAS_VDO[17]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_8">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;IR_data<span id="irdata_0"><INPUT TYPE="text" size="3" name="irdata_0" readonly style="width:50px;text-align:Left;" VALUE="">&nbsp;
</span>
<SELECT NAME="ir_reg_0" id="ir_reg_0" onChange="irkit_reg('irdata_0','#ir_reg_0');"/>
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">reg
</SELECT>
<BR>
IR2<span id="dosel_9">
<SELECT onChange="update_do('irkitdo_1')" NAME="irkitdo_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="Send">Send
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_9" style="width:36px;text-align:right;" VALUE="${DON_TIME[9]}" NAME="don_time_9">ms&nbsp;
<INPUT TYPE="text" id="alias_do_9" style="width:100px;" NAME="alias_do_9" VALUE="${ALIAS_DO[9]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_18" style="width:100px;" NAME="alias_vdo_18" VALUE="${ALIAS_VDO[18]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_19" style="width:100px;" NAME="alias_vdo_19" VALUE="${ALIAS_VDO[19]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;IR_data<span id="irdata_1"><INPUT TYPE="text" size="3" name="irdata_1" readonly style="width:50px;text-align:Left;" VALUE="">&nbsp;
</span>
<SELECT NAME="ir_reg_1" id="ir_reg_1" onChange="irkit_reg('irdata_1','#ir_reg_1');"/>
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">reg
</SELECT>
<BR>
IR3<span id="dosel_10">
<SELECT onChange="update_do('irkitdo_2')" NAME="irkitdo_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="Send">Send
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_10" style="width:36px;text-align:right;" VALUE="${DON_TIME[10]}" NAME="don_time_10">ms&nbsp;
<INPUT TYPE="text" id="alias_do_10" style="width:100px;" NAME="alias_do_10" VALUE="${ALIAS_DO[10]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_20" style="width:100px;" NAME="alias_vdo_20" VALUE="${ALIAS_VDO[20]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_21" style="width:100px;" NAME="alias_vdo_21" VALUE="${ALIAS_VDO[21]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_10">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;IR_data<span id="irdata_2"><INPUT TYPE="text" size="3" name="irdata_2" readonly style="width:50px;text-align:Left;" VALUE="">&nbsp;
</span>
<SELECT NAME="ir_reg_2" id="ir_reg_2" onChange="irkit_reg('irdata_2','#ir_reg_2');"/>
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">reg
</SELECT>
<BR>
IR4<span id="dosel_11">
<SELECT onChange="update_do('irkitdo_3')" NAME="irkitdo_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="Send">Send
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_11" style="width:36px;text-align:right;" VALUE="${DON_TIME[11]}" NAME="don_time_11">ms&nbsp;
<INPUT TYPE="text" id="alias_do_11" style="width:100px;" NAME="alias_do_11" VALUE="${ALIAS_DO[11]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_22" style="width:100px;" NAME="alias_vdo_22" VALUE="${ALIAS_VDO[22]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_23" style="width:100px;" NAME="alias_vdo_23" VALUE="${ALIAS_VDO[23]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_11">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;IR_data<span id="irdata_3"><INPUT TYPE="text" size="3" name="irdata_3" readonly style="width:50px;text-align:Left;" VALUE="">&nbsp;
</span>
<SELECT NAME="ir_reg_3" id="ir_reg_3" onChange="irkit_reg('irdata_3','#ir_reg_3');"/>
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">reg
</SELECT>
<BR>
IR5<span id="dosel_12">
<SELECT onChange="update_do('irkitdo_4')" NAME="irkitdo_4">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="Send">Send
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_12" style="width:36px;text-align:right;" VALUE="${DON_TIME[12]}" NAME="don_time_12">ms&nbsp;
<INPUT TYPE="text" id="alias_do_12" style="width:100px;" NAME="alias_do_12" VALUE="${ALIAS_DO[12]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_24" style="width:100px;" NAME="alias_vdo_24" VALUE="${ALIAS_VDO[24]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_25" style="width:100px;" NAME="alias_vdo_25" VALUE="${ALIAS_VDO[25]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_12">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;IR_data<span id="irdata_4"><INPUT TYPE="text" size="3" name="irdata_4" readonly style="width:50px;text-align:Left;" VALUE="">&nbsp;
</span>
<SELECT NAME="ir_reg_4" id="ir_reg_4" onChange="irkit_reg('irdata_4','#ir_reg_4');"/>
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">reg
</SELECT>
<BR>
IR6<span id="dosel_13">
<SELECT onChange="update_do('irkitdo_5')" NAME="irkitdo_5">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="Send">Send
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_13" style="width:36px;text-align:right;" VALUE="${DON_TIME[13]}" NAME="don_time_13">ms&nbsp;
<INPUT TYPE="text" id="alias_do_13" style="width:100px;" NAME="alias_do_13" VALUE="${ALIAS_DO[13]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_26" style="width:100px;" NAME="alias_vdo_26" VALUE="${ALIAS_VDO[26]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_27" style="width:100px;" NAME="alias_vdo_27" VALUE="${ALIAS_VDO[27]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_13">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;IR_data<span id="irdata_5"><INPUT TYPE="text" size="3" name="irdata_5" readonly style="width:50px;text-align:Left;" VALUE="">&nbsp;
</span>
<SELECT NAME="ir_reg_5" id="ir_reg_5" onChange="irkit_reg('irdata_5','#ir_reg_5');"/>
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">reg
</SELECT>
<BR>
IRKit_IP<span id="irkit_ip"></span><INPUT TYPE="text" name="irkit_ip" style="width:100px;text-align:Left;" VALUE="none">&nbsp;<input type="button" value="Search_Set" onclick="irkit_search();"/>
<HR>
<B>Setting Twlite</B>
<SELECT NAME="TOCOS_TTY">
<OPTION VALUE="${TOCOS_TTY}" SELECTED>${vTOCOS_TTY}
<OPTION VALUE="none">none
<OPTION VALUE="ttyUSBTWE-Lite">ttyUSBTWE-Lite
</SELECT>&nbsp;&nbsp;
tocos_ip<INPUT TYPE="text" style="width:120px;" NAME="tocos_ip" VALUE="${tocos_ip}">&nbsp;
&nbsp;I2C_Temperature&Humidity
<span id="i2ctemp"></span>
<BR>
<span id="do_14">
TO1<INPUT TYPE="text" size="3" id="vdo_14" name="vdo_14" readonly style="width:36px;text-align:center;" VALUE="${DO[14]}">&nbsp;
</span>
<span id="dosel_14">
<SELECT onChange="update_do('tocosdo_1')" NAME="tocosdo_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_14" style="width:36px;text-align:right;" VALUE="${DON_TIME[14]}" NAME="don_time_14">ms&nbsp;
<INPUT TYPE="text" id="alias_do_14" style="width:100px;" NAME="alias_do_14" VALUE="${ALIAS_DO[14]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_28" style="width:100px;" NAME="alias_vdo_28" VALUE="${ALIAS_VDO[28]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_29" style="width:100px;" NAME="alias_vdo_29" VALUE="${ALIAS_VDO[29]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_14">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="do_15">
TO2<INPUT TYPE="text" size="3" id="vdo_15" name="vdo_15" readonly style="width:36px;text-align:center;" VALUE="${DO[15]}">&nbsp;
</span>
<span id="dosel_15">
<SELECT onChange="update_do('tocosdo_2')" NAME="tocosdo_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_15" style="width:36px;text-align:right;" VALUE="${DON_TIME[15]}" NAME="don_time_15">ms&nbsp;
<INPUT TYPE="text" id="alias_do_15" style="width:100px;" NAME="alias_do_15" VALUE="${ALIAS_DO[15]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_30" style="width:100px;" NAME="alias_vdo_30" VALUE="${ALIAS_VDO[30]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_31" style="width:100px;" NAME="alias_vdo_31" VALUE="${ALIAS_VDO[31]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_15">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="do_16">
TO3<INPUT TYPE="text" size="3" id="vdo_16" name="vdo_16" readonly style="width:36px;text-align:center;" VALUE="${DO[16]}">&nbsp;
</span>
<span id="dosel_16">
<SELECT onChange="update_do('tocosdo_3')" NAME="tocosdo_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="1">high
<OPTION VALUE="0">low
</SELECT>&nbsp;
<INPUT TYPE="text" id="don_time_16" style="width:36px;text-align:right;" VALUE="${DON_TIME[16]}" NAME="don_time_16">ms&nbsp;
<INPUT TYPE="text" id="alias_do_16" style="width:100px;" NAME="alias_do_16" VALUE="${ALIAS_DO[16]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_32" style="width:100px;" NAME="alias_vdo_32" VALUE="${ALIAS_VDO[32]}">&nbsp;
<INPUT TYPE="text" id="alias_vdo_33" style="width:100px;" NAME="alias_vdo_33" VALUE="${ALIAS_VDO[33]}">&nbsp;
</span>
<SELECT NAME="alias_do_reg_16">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<HR>
<B>Setting USB Modem Device</B>
<SELECT NAME="modem">
<OPTION VALUE="$modem_dev" SELECTED>$modem_dev
<OPTION VALUE="none">none
<OPTION VALUE="ttyUSBMODEM">ttyUSBMODEM
</SELECT>
<HR>
<span id="voice_sel">
Voice control
<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do('voice_sel')" placeholder="Command" autofocus />
<SELECT NAME="voice_lang" id="voice_lang">
<OPTION VALUE="ja" SELECTED>Japanese
<OPTION VALUE="en">English
</SELECT>
</span>
&nbsp;Mail:
<INPUT TYPE="text" id="alias_di_24" style="width:140px;" NAME="alias_di_24" VALUE="${ALIAS_DI[24]}">&nbsp;
<SELECT NAME="alias_di_reg_24">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<input type="button" value="Recognition start" onclick="startWebVoiceRecognition();"/>
<input type="button" value="Recognition stop" onclick="stopWebVoiceRecognition();"/>
State<span id="recognition_state" >Stop</span>
<BR>
<HR>
<input type="button" value="Camera_1 photo" onclick="start_photo('video0');"/>&nbsp
<input type="button" value="Camera_2 photo" onclick="start_photo('video1');"/>&nbsp
<input type="button" value="Camera_3 photo" onclick="start_photo('video2');"/>&nbsp
<BR>
<input type="button" value="Web Camera1" onclick="start_video('video0');"/>
<input id="live_timer0" type="text" style="width:20px;" NAME="live_timer0" VALUE="10">Sec&nbsp;
<input type="button" value="Web Camera2" onclick="start_video('video1');"/>
<input id="live_timer1" type="text" style="width:20px;" NAME="live_timer1" VALUE="10">Sec&nbsp;
<input type="button" value="Web Camera3" onclick="start_video('video1');"/>
<input id="live_timer2" type="text" style="width:20px;" NAME="live_timer2" VALUE="10">Sec&nbsp
<input type="button" value="Module Camera" onclick="start_video('vchiq');"/>
<input id="live_timer3" type="text" style="width:20px;" NAME="live_timer3" VALUE="10">Sec&nbsp;
<BR>
<input type="button" value="Streaming start" onclick="streaming_start_stop('vchiq','start');"/>
<input type="button" value="Streaming stop" onclick="streaming_start_stop('vchiq','stop');"/>
Server<input type="text" style="width:100px;" id="live_server" NAME="Server" VALUE="${LIVE_SERVER}">
<HR>
<B>Setting digital input terminal name</B>
&nbsp;&nbsp;<INPUT style="text-align:center" TYPE="reset" VALUE="Reload" onClick="update_di(onload);"/>
<BR>
<span id="di_0">
Input1<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center" VALUE="${DI[0]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_0" VALUE="${ALIAS_DI[0]}">&nbsp;
<SELECT NAME="alias_di_reg_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="di_1">
Input2<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[1]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_1" VALUE="${ALIAS_DI[1]}">&nbsp;
<SELECT NAME="alias_di_reg_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="di_2">
Input3<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[2]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_2" VALUE="${ALIAS_DI[2]}">&nbsp;
<SELECT NAME="alias_di_reg_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>

<span id="di_3">
Input4<INPUT TYPE="text"  size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[3]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_3" VALUE="${ALIAS_DI[3]}">&nbsp;
<SELECT NAME="alias_di_reg_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<HR>
<B>Setting Twlite DI & AI</B>
<BR>
<span id="di_8">
TI1<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[8]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_8" VALUE="${ALIAS_DI[8]}">&nbsp;
<SELECT NAME="alias_di_reg_8">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="di_9">
TI2<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[9]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_9" VALUE="${ALIAS_DI[9]}">&nbsp;
<SELECT NAME="alias_di_reg_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="di_10">
TI3<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[10]}">&nbsp;
</span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_10" VALUE="${ALIAS_DI[10]}">&nbsp;
<SELECT NAME="alias_di_reg_10">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<span id="di_12">
AI1<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[12]}">&nbsp;
</span>
<span id="vai_1"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_12" VALUE="${ALIAS_DI[12]}">&nbsp;
Slice<INPUT TYPE="text" style="width:36px;" NAME="slice_ai_12" VALUE="${SLICE_AI[12]}">&nbsp;
<SELECT NAME="alias_di_reg_12">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="vai_1_graph"></span>
<BR>
<span id="di_13">
AI2<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[13]}">&nbsp;
</span>
<span id="vai_2"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_13" VALUE="${ALIAS_DI[13]}">&nbsp;
Slice<INPUT TYPE="text" style="width:36px;" NAME="slice_ai_13" VALUE="${SLICE_AI[13]}">&nbsp;
<SELECT NAME="alias_di_reg_13">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="vai_2_graph"></span>
<BR>
<span id="di_14">
AI3<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[14]}">&nbsp;
</span>
<span id="vai_3"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_14" VALUE="${ALIAS_DI[14]}">&nbsp;
Slice<INPUT TYPE="text" style="width:36px;" NAME="slice_ai_14" VALUE="${SLICE_AI[14]}">&nbsp;
<SELECT NAME="alias_di_reg_14">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="vai_3_graph"></span>
<BR>
<span id="di_15">
AI4<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[15]}">&nbsp;
</span>
<span id="vai_4"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_15" VALUE="${ALIAS_DI[15]}">&nbsp;
Slice<INPUT TYPE="text" style="width:36px;" NAME="slice_ai_15" VALUE="${SLICE_AI[15]}">&nbsp;
<SELECT NAME="alias_di_reg_15">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="vai_4_graph"></span>
<HR>
<span id="di_16">
CPU Temperature<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[16]}">
</span>
<span id="cpu_temp"></span>
&nbsp;
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_16" VALUE="${ALIAS_DI[16]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_16" VALUE="${SLICE_AI[16]}">&nbsp;
<SELECT NAME="alias_di_reg_16">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<TD><FONT SIZE="-1"><input type="button" value="CPU Temperature Graph" onClick="window.open('./cpu_temp_disp.cgi','','width=600,height=200');"/>
</FONT></TD>
<BR>
<span id="di_17">
GPIO Temperature<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[17]}">
</span>
<span id="gpio_temp_val"></span>
&nbsp;
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_17" VALUE="${ALIAS_DI[17]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_17" VALUE="${SLICE_AI[17]}">&nbsp;
<SELECT NAME="alias_di_reg_17">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="gpio_temp_graph"></span>
<BR>
<span id="di_18">
GPIO Humidity<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[18]}">
</span>
<span id="gpio_hum_val"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_18" VALUE="${ALIAS_DI[18]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_18" VALUE="${SLICE_AI[18]}">&nbsp;
<SELECT NAME="alias_di_reg_18">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
&nbsp;
<span id="gpio_hum_graph"></span>
<BR>
<span id="di_19">
Twlite Temperature<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[19]}">
</span>
<span id="i2c_temp_val"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_19" VALUE="${ALIAS_DI[19]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_19" VALUE="${SLICE_AI[19]}">&nbsp;
<SELECT NAME="alias_di_reg_19">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="i2c_temp_disp"></span>
<BR>
<span id="di_20">
Twlite Humidity<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[20]}">
</span>
<span id="i2c_hum_val"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_20" VALUE="${ALIAS_DI[20]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_20" VALUE="${SLICE_AI[20]}">&nbsp;
<SELECT NAME="alias_di_reg_20">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="i2c_hum_disp"></span>
<BR>
<span id="di_21">
GPIO Pressure<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[21]}">
</span>
<span id="gpio_pres_val"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_21" VALUE="${ALIAS_DI[21]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_21" VALUE="${SLICE_AI[21]}">&nbsp;
<SELECT NAME="alias_di_reg_21">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="gpio_pres_graph"></span>
<BR>
<span id="di_22">
GPIO Gas<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[22]}">
</span>
<span id="gpio_gas_val"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_22" VALUE="${ALIAS_DI[22]}">&nbsp;
Slice<INPUT TYPE="text" style="width:50px;" NAME="slice_ai_22" VALUE="${SLICE_AI[22]}">&nbsp;
<SELECT NAME="alias_di_reg_22">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
Base<INPUT TYPE="text" style="width:50px;" NAME="slice_ai_24" VALUE="${SLICE_AI[24]}">&nbsp;
<SELECT NAME="alias_di_reg_24">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<SELECT NAME="slice_ai_25">
<OPTION VALUE="${SLICE_AI[25]}" SELECTED>${SLICE_AI[25]}
<OPTION VALUE="Auto">Auto
<OPTION VALUE="Fix">Fix
</SELECT>
<span id="gpio_gas_graph"></span>
<BR>
<span id="di_23">
IAQ Sample<INPUT TYPE="text" size="1" readonly style="width:36px;text-align:center;" VALUE="${DI[23]}">
</span>
<span id="gpio_iaq_val"></span>
<INPUT TYPE="text" style="width:120px;" NAME="alias_di_23" VALUE="${ALIAS_DI[23]}">&nbsp;
Slice<INPUT TYPE="text" style="width:24px;" NAME="slice_ai_23" VALUE="${SLICE_AI[23]}">&nbsp;
<SELECT NAME="alias_di_reg_23">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<span id="gpio_iaq_graph"></span>
<BR>
<FONT SIZE="-2">IAQ is Sample 0(Good) to 500(Hazardous) Temperature(17-28℃):10% Humidity(40-70%):10% Gas(Gas±Gas_base/Gas_base):80%
</FONT><BR>
<HR>
Interface<SELECT NAME="DI_TTY">
<OPTION VALUE="${DI_TTY}" SELECTED>${vTTY}
<OPTION VALUE="gpio">gpio
<OPTION VALUE="piface">piface
<OPTION VALUE="none">none
</SELECT>&nbsp;&nbsp;
piface_ip<INPUT TYPE="text" style="width:120px;" NAME="piface_ip" VALUE="${piface_ip}">
<BR>
<INPUT style="text-align:center" TYPE="button" id="menu5_jikkou" VALUE="Run" onClick="return menu5_ck();"/>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
</DD>
</DL>
END

cat >>$PAGE1<<END
<DL id="menu6dl">
<DT><FONT SIZE="+1"><B>Setting ping monitoring and digital output action</B></FONT></DT>
<DD>
<FORM NAME="menu6" id="menu6_form" ACTION="./ping_watch_don.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="3"><B>Setting IP address and the digital Output for monitoring</B></FONT><BR>
IP1<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_0">&nbsp;
&nbsp;DO<SELECT NAME="ping_don_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:48px;text-align:left;" NAME="ping_don_time_0">ms&nbsp;
&nbsp;<SELECT NAME="reg_0">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP2<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_1">&nbsp;
&nbsp;DO<SELECT NAME="ping_don_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low

</SELECT>&nbsp;
<INPUT TYPE="text" style="width:48px;text-align:left;" NAME="ping_don_time_1">ms&nbsp;
&nbsp;<SELECT NAME="reg_1">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP3<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_2">&nbsp;
&nbsp;DO<SELECT NAME="ping_don_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low

</SELECT>&nbsp;
<INPUT TYPE="text" style="width:48px;text-align:left;" NAME="ping_don_time_2">ms&nbsp;
&nbsp;<SELECT NAME="reg_2">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP4<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_3">&nbsp;
&nbsp;DO<SELECT NAME="ping_don_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low

</SELECT>&nbsp;
<INPUT TYPE="text" style="width:48px;text-align:left;" NAME="ping_don_time_3">ms&nbsp;
&nbsp;<SELECT NAME="reg_3">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
<HR>
Ping monitoring interval<SELECT NAME="interval_0">
<OPTION VALUE="1">1 Min
<OPTION VALUE="2">2 Min
<OPTION VALUE="3">3 Min
<OPTION VALUE="4">4 Min
<OPTION VALUE="5" SELECTED>5 Min
</SELECT><BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu6_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
END
PING_DON=$DIR/.ping_don_list
PING_DON_HTML=.ping_don.html
if [ -e $PING_DON ];then
cat >$PING_DON_HTML<<END
<HTML><HEAD><style type="text/css">
TD{
font-size:15px;
}
</style>
<BODY BGCOLOR="#CCCCCC">
<TITLE>ping digital-out diplay</TITLE></HEAD>
-------- ping watch & digital out list --------
<TABLE BORDER="0">
<TR><TD WIDTH="100">IP-ADRESS</TD>
<TD WIDTH="100">DO-NO</TD>
<TD WIDTH="100">ON-TIMER</TD>
<TD WIDTH="100">INTERVAL</TD>
</TR>
END
cat $PING_DON | awk 'BEGIN{FS=" "};{print "<TR><TD WIDTH=\"100\">"$2"</TD><TD WIDTH=\"100\">"$3"<TD WIDTH=\"100\">"$4"</TD><TD WIDTH=\"100\">"$5"</TD></TR>"}' >>$PING_DON_HTML
echo '</TABLE></BODY></HTML>' >>$PING_DON_HTML
cat >>$PAGE1<<END
<IFRAME SRC="$PING_DON_HTML" WIDTH="600" HEIGHT="200" SCROLLING="yes"
 FRAMEBODER="1" ALIGN="center" ALT="It has been displayed in the frame.">
</IFRAME>
END
fi
cat >>$PAGE1<<END
</DD>
</DL>
END

cat >>$PAGE1<<END
<DL id="menu7dl">
<DT><FONT SIZE="+1"><B>Setting ping monitoring and e-mail</B></FONT></DT>
<DD>
<FORM NAME="menu7" id="menu7_form" ACTION="./ping_watch_mail.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="3"><B>Setting IP address and FAIL at which monitoring</B></FONT><BR>
IP1<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_0">&nbsp;
Email1<INPUT TYPE="text" NAME="mail_0">
&nbsp;<SELECT NAME="reg_0">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP2<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_1">&nbsp;
Email2<INPUT NAME="mail_1">
&nbsp;<SELECT NAME="reg_1">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP3<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_2">&nbsp;
Email3<INPUT TYPE="text" NAME="mail_2">
&nbsp;<SELECT NAME="reg_2">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP4<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_3">&nbsp;
Email4<INPUT TYPE="text" NAME="mail_3">
&nbsp;<SELECT NAME="reg_3">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
<HR>
Ping monitoring interval<SELECT NAME="interval_0">
<OPTION VALUE="1">1 Min
<OPTION VALUE="2">2 Min
<OPTION VALUE="3">3 Min
<OPTION VALUE="4">4 Min
<OPTION VALUE="5" SELECTED>5 Min
</SELECT><BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu7_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
END
PING_MAIL=$DIR/.ping_mail_list
PING_MAIL_HTML=.ping_mail.html
if [ -e $PING_MAIL ];then
cat >$PING_MAIL_HTML<<END
<HTML><HEAD><BODY BGCOLOR="#CCCCCC">
<TITLE>ping mail diplay</TITLE><style type="text/css">
TD{
font-size:15px;
}
</style>
<BODY BGCOLOR="#CCCCCC">
<TITLE>ping mail diplay</TITLE></HEAD>
-------- ping watch & mail list --------
<TABLE BORDER="0">
<TR><TD WIDTH="100">IP-ADRESS</TD>
<TD WIDTH="160">MAIL-ADRESS</TD>
<TD WIDTH="100">INTERVAL</TD>
</TR>
END
cat $PING_MAIL | awk 'BEGIN{FS=" "};{print "<TR><TD WIDTH=\"100\">"$2"</TD><TD WIDTH=\"160\">"$3"</TD><TD WIDTH=\"100\">"$4"</TD></TR>"}' >>$PING_MAIL_HTML
echo '</TABLE></BODY></HTML>' >>$PING_MAIL_HTML
cat >>$PAGE1<<END
<IFRAME SRC="$PING_MAIL_HTML" WIDTH="600" HEIGHT="200" SCROLLING="yes"
 FRAMEBODER="1" ALIGN="center" ALT="It has been displayed in the frame.">
</IFRAME>
END
fi
cat >>$PAGE1<<END
</DD>
</DL>
END


cat >>$PAGE1<<END
<DL id="menu8dl">
<DT><FONT SIZE="+1"><B>Setting ping monitoring and phone number</B></FONT></DT>
<DD>
<FORM NAME="menu8" id="menu8_form" ACTION="./ping_watch_phone.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="3"><B>Setting IP address and at which monitoring</B></FONT><BR>
IP1<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_0">&nbsp;
Phone1<INPUT TYPE="text" size="22" style="width:110px;" NAME="tel_0">
&nbsp;<SELECT NAME="reg_0">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP2<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_1">&nbsp;
Phone2<INPUT TYPE="text" size="22" style="width:110px;" NAME="tel_1">
&nbsp;<SELECT NAME="reg_1">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP3<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_2">&nbsp;
Phone3<INPUT TYPE="text" size="22" style="width:110px;" NAME="tel_2">
&nbsp;<SELECT NAME="reg_2">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
IP4<INPUT TYPE="text" size="22" style="width:110px;" NAME="ip_3">&nbsp;
Phone4<INPUT TYPE="text" size="22" style="width:110px;" NAME="tel_3">
&nbsp;<SELECT NAME="reg_3">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT><BR>
<HR>
Ping monitoring interval<SELECT NAME="interval_0">
<OPTION VALUE="1">1 Min
<OPTION VALUE="2">2 Min
<OPTION VALUE="3">3 Min
<OPTION VALUE="4">4 Min
<OPTION VALUE="5" SELECTED>5 Min
</SELECT><BR>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu8_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
END
PING_PHONE=$DIR/.ping_phone_list
PING_PHONE_HTML=.ping_phone.html
if [ -e $PING_PHONE ];then
cat >$PING_PHONE_HTML<<END
<HTML><HEAD>
<style type="text/css">
TD{
font-size:15px;
}
</style>
<BODY BGCOLOR="#CCCCCC">
<TITLE>ping phone diplay</TITLE></HEAD>
-------- ping watch & phone call list --------
<TABLE BORDER="0">
<TR><TD WIDTH="100">IP-ADRESS</TD>
<TD WIDTH="100">TEL-NO</TD>
<TD WIDTH="100">INTERVAL</TD>
</TR>
END
cat $PING_PHONE | awk 'BEGIN{FS=" "};{print "<TR><TD WIDTH=\"100\">"$2"</TD><TD WIDTH=\"100\">"$3"</TD><TD WIDTH=\"100\">"$4"</TD></TR>"}' >>$PING_PHONE_HTML
echo '</TABLE></BODY></HTML>' >>$PING_PHONE_HTML
cat >>$PAGE1<<END
<IFRAME SRC="$PING_PHONE_HTML" WIDTH="600" HEIGHT="200" SCROLLING="yes"
 FRAMEBODER="1" ALIGN="center" ALT="It has been displayed in the frame.">
</IFRAME>
END
fi
cat >>$PAGE1<<END
</DD>
</DL>
END


DICH=$DIR/.di_change1
[ -e "$DICH" ] && . "$DICH"
n=0
while [ $n -lt 22 ];do
  case "${di_change[$n]}" in
  "low2high")
     vdi_change[$n]="low→high" ;;
  "high2low")
     vdi_change[$n]="high→low" ;;
  *)
    di_change[$n]="low2high"
    vdi_change[$n]="low→high"
  esac
  case "${di_act[$n]}" in
    "DON_0")
      vdi_act[$n]="${ALIAS_DO[0]}high" ;;
    "DOFF_0")
      vdi_act[$n]="${ALIAS_DO[0]}low" ;;
    "DON_1")
      vdi_act[$n]="${ALIAS_DO[1]}high" ;;
    "DOFF_1")
      vdi_act[$n]="${ALIAS_DO[1]}low" ;;
    "DON_2")
      vdi_act[$n]="${ALIAS_DO[2]}high" ;;
    "DOFF_2")
      vdi_act[$n]="${ALIAS_DO[2]}low" ;;
    "DON_3")
      vdi_act[$n]="${ALIAS_DO[3]}high" ;;
    "DOFF_3")
      vdi_act[$n]="${ALIAS_DO[3]}low" ;;
    "IREXEC_0")
      vdi_act[$n]="${ALIAS_DO[8]}" ;;
    "IREXEC_1")
      vdi_act[$n]="${ALIAS_DO[9]}" ;;
    "IREXEC_2")
      vdi_act[$n]="${ALIAS_DO[10]}" ;;
    "IREXEC_3")
      vdi_act[$n]="${ALIAS_DO[11]}" ;;
    "IREXEC_4")
      vdi_act[$n]="${ALIAS_DO[12]}" ;;
    "IREXEC_5")
      vdi_act[$n]="${ALIAS_DO[13]}" ;;
    "TON_0")
      vdi_act[$n]="${ALIAS_DO[14]}high" ;;
    "TOFF_0")
      vdi_act[$n]="${ALIAS_DO[14]}low" ;;
    "TON_1")
      vdi_act[$n]="${ALIAS_DO[15]}high" ;;
    "TOFF_1")
      vdi_act[$n]="${ALIAS_DO[15]}low" ;;
    "TON_2")
      vdi_act[$n]="${ALIAS_DO[16]}high" ;;
    "TOFF_2")
      vdi_act[$n]="${ALIAS_DO[16]}low" ;;
    "phone")
      vdi_act[$n]="Phone" ;;
    "mail")
      vdi_act[$n]="Email" ;;
    "mail_message")
      vdi_act[$n]="Email_messageage" ;;
    "web_camera_video")
      vdi_act[$n]="Web_camera Video" ;;
    "mod_camera_still")
      vdi_act[$n]="Mod_camera Still" ;;
    "mod_camera_video")
      vdi_act[$n]="Mod_camera Video" ;;
    "SOUND_0")
      vdi_act[$n]="Sound_1" ;;
    "SOUND_1")
      vdi_act[$n]="Sound_2" ;;
    "SOUND_2")
      vdi_act[$n]="Sound_3" ;;
    "SOUND_3")
      vdi_act[$n]="Sound_4" ;;
    "SOUND_4")
      vdi_act[$n]="Sound_5" ;;
    *)
      di_act[$n]="none"
      vdi_act[$n]="none" ;;
  esac
  [ -n "${don_time[$n]}" ] && DON_TIME[$n]="${don_time[$n]}" || DON_TIME[$n]=""
  [ -n "${di_tel[$n]}" ] && DI_TELNO[$n]="${di_tel[$n]}" || DI_TELNO[$n]=""
  [ -n "${di_mail[$n]}" ] && DI_MAIL[$n]="${di_mail[$n]}" || DI_MAIL[$n]=""
  if [ -n "${di_mail_message[$n]}" ];then
    DI_MESS[$n]="`echo "${di_mail_message[$n]}" |tr "+" " "`"
  else
    DI_MESS[$n]=""
  fi
  [ -n "${di_act_alt[$n]}" ] && DI_ACT_ALT[$n]="${di_act_alt[$n]}" || DI_ACT_ALT[$n]="none"
  n=$(($n + 1))
done

cat >>$PAGE1<<END
<DL id="menu9dl">
<DT><FONT SIZE="+1"><B>Management DI(Digital Input)-1</B></FONT></DT>
<DD>
<FORM NAME="menu9" id="menu9_form" ACTION="./di_contorl_pi1.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="3"><B>Setting first action to the digital input</B></FONT>
<BR>
<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[0]}">
<span id="menu90di_0">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[0]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_0" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_0">
<OPTION VALUE="${di_act[0]}" SELECTED>${vdi_act[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_0">
<OPTION VALUE="${DI_ACT_ALT[0]}" SELECTED>${DI_ACT_ALT[0]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[0]}" NAME="don_time_0">ms&nbsp;
<SELECT NAME="di_change_reg_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio0high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[0]}" NAME="di_tel_0">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[0]}" NAME="di_mail_0">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[0]}" NAME="di_mail_message_0">
&nbsp;
<span id="menu90ct_0">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[1]}">
<span id="menu90di_1">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[1]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_1" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_1">
<OPTION VALUE="${di_act[1]}" SELECTED>${vdi_act[1]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_1">
<OPTION VALUE="${DI_ACT_ALT[1]}" SELECTED>${DI_ACT_ALT[1]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[1]}" NAME="don_time_1">ms&nbsp;
<SELECT NAME="di_change_reg_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio1high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[1]}" NAME="di_tel_1">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[1]}" NAME="di_mail_1">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[1]}" NAME="di_mail_message_1">
&nbsp;
<span id="menu90ct_1">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[2]}">
<span id="menu90di_2">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[2]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_2" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_2">
<OPTION VALUE="${di_act[2]}" SELECTED>${vdi_act[2]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_2">
<OPTION VALUE="${DI_ACT_ALT[2]}" SELECTED>${DI_ACT_ALT[2]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[2]}" NAME="don_time_2">ms&nbsp;
<SELECT NAME="di_change_reg_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio2high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[2]}" NAME="di_tel_2">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[2]}" NAME="di_mail_2">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[2]}" NAME="di_mail_message_2">
&nbsp;
<span id="menu90ct_2">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[3]}">
<span id="menu90di_3">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[3]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_3" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_3">
<OPTION VALUE="${di_act[3]}" SELECTED>${vdi_act[3]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_3">
<OPTION VALUE="${DI_ACT_ALT[3]}" SELECTED>${DI_ACT_ALT[3]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[3]}" NAME="don_time_3">ms&nbsp;
<SELECT NAME="di_change_reg_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio3high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[3]}" NAME="di_tel_3">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[3]}" NAME="di_mail_3">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[3]}" NAME="di_mail_message_3">
&nbsp;
<span id="menu90ct_3">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[8]}">
<span id="menu90di_8">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[8]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_8" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_8">
<OPTION VALUE="${di_act[8]}" SELECTED>${vdi_act[8]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_8">
<OPTION VALUE="${DI_ACT_ALT[8]}" SELECTED>${DI_ACT_ALT[8]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[8]}" NAME="don_time_8">ms&nbsp;
<SELECT NAME="di_change_reg_8">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio8high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[8]}" NAME="di_tel_8">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[8]}" NAME="di_mail_8">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[8]}" NAME="di_mail_message_8">
&nbsp;
<span id="menu90ct_8">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[9]}">
<span id="menu90di_9">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[9]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_9" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_9">
<OPTION VALUE="${di_act[9]}" SELECTED>${vdi_act[9]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_9">
<OPTION VALUE="${DI_ACT_ALT[9]}" SELECTED>${DI_ACT_ALT[9]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[9]}" NAME="don_time_9">ms&nbsp;
<SELECT NAME="di_change_reg_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio9high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[9]}" NAME="di_tel_9">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[9]}" NAME="di_mail_9">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[9]}" NAME="di_mail_message_9">
&nbsp;
<span id="menu90ct_9">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[10]}">
<span id="menu90di_10">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[10]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_10" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_10">
<OPTION VALUE="${di_act[10]}" SELECTED>${vdi_act[10]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_10">
<OPTION VALUE="${DI_ACT_ALT[10]}" SELECTED>${DI_ACT_ALT[10]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[10]}" NAME="don_time_10">ms&nbsp;
<SELECT NAME="di_change_reg_10">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio10high">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[10]}" NAME="di_tel_10">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[10]}" NAME="di_mail_10">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[10]}" NAME="di_mail_message_10">
&nbsp;
<span id="menu90ct_10">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[0]}">
<span id="menu91di_0">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[0]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_11" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_11">
<OPTION VALUE="${di_act[11]}" SELECTED>${vdi_act[11]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_11">
<OPTION VALUE="${DI_ACT_ALT[11]}" SELECTED>${DI_ACT_ALT[11]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[11]}" NAME="don_time_11">ms&nbsp;
<SELECT NAME="di_change_reg_11">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio0low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[11]}" NAME="di_tel_11">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[11]}" NAME="di_mail_11">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[11]}" NAME="di_mail_message_11">
&nbsp;
<span id="menu91ct_0">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[1]}">
<span id="menu91di_1">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[1]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_12" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_12">
<OPTION VALUE="${di_act[12]}" SELECTED>${vdi_act[12]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_12">
<OPTION VALUE="${DI_ACT_ALT[12]}" SELECTED>${DI_ACT_ALT[12]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[12]}" NAME="don_time_12">ms&nbsp;
<SELECT NAME="di_change_reg_12">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio1low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[12]}" NAME="di_tel_12">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[12]}" NAME="di_mail_12">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[12]}" NAME="di_mail_message_12">
&nbsp;
<span id="menu91ct_1">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[2]}">
<span id="menu91di_2">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[2]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_13" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_13">
<OPTION VALUE="${di_act[13]}" SELECTED>${vdi_act[13]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_13">
<OPTION VALUE="${DI_ACT_ALT[13]}" SELECTED>${DI_ACT_ALT[13]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[13]}" NAME="don_time_13">ms&nbsp;
<SELECT NAME="di_change_reg_13">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio2low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[13]}" NAME="di_tel_13">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[13]}" NAME="di_mail_13">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[13]}" NAME="di_mail_message_13">
&nbsp;
<span id="menu91ct_2">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[3]}">
<span id="menu91di_3">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[3]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_14" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_14">
<OPTION VALUE="${di_act[14]}" SELECTED>${vdi_act[14]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_14">
<OPTION VALUE="${DI_ACT_ALT[14]}" SELECTED>${DI_ACT_ALT[14]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[14]}" NAME="don_time_14">ms&nbsp;
<SELECT NAME="di_change_reg_14">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio3low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[14]}" NAME="di_tel_14">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[14]}" NAME="di_mail_14">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[14]}" NAME="di_mail_message_14">
&nbsp;
<span id="menu91ct_3">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[8]}">
<span id="menu91di_8">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[8]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_19" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_19">
<OPTION VALUE="${di_act[19]}" SELECTED>${vdi_act[19]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_19">
<OPTION VALUE="${DI_ACT_ALT[19]}" SELECTED>${DI_ACT_ALT[19]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[19]}" NAME="don_time_19">ms&nbsp;
<SELECT NAME="di_change_reg_19">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio8low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[19]}" NAME="di_tel_19">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[19]}" NAME="di_mail_19">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[19]}" NAME="di_mail_message_19">
&nbsp;
<span id="menu91ct_8">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[9]}">
<span id="menu91di_9">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[9]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_20" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_20">
<OPTION VALUE="${di_act[20]}" SELECTED>${vdi_act[20]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_20">
<OPTION VALUE="${DI_ACT_ALT[20]}" SELECTED>${DI_ACT_ALT[20]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[20]}" NAME="don_time_20">ms&nbsp;
<SELECT NAME="di_change_reg_20">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio9low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[20]}" NAME="di_tel_20">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[20]}" NAME="di_mail_20">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[20]}" NAME="di_mail_message_20">
&nbsp;
<span id="menu91ct_9">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[10]}">
<span id="menu91di_10">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[10]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_21" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_21">
<OPTION VALUE="${di_act[21]}" SELECTED>${vdi_act[21]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_21">
<OPTION VALUE="${DI_ACT_ALT[21]}" SELECTED>${DI_ACT_ALT[21]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[21]}" NAME="don_time_21">ms&nbsp;
<SELECT NAME="di_change_reg_21">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
<OPTION VALUE="clr">reset
</SELECT>
<span id="dio10low">
</span>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[21]}" NAME="di_tel_21">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[21]}" NAME="di_mail_21">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[21]}" NAME="di_mail_message_21">
&nbsp;
<span id="menu91ct_10">
</span>
<HR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu9_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
</DD>
</DL>
END

# unset di_chang1 val
n=0
while [ $n -lt 22 ];do
  unset di_change[$n]
  unset di_act[$n]
  unset don_time[$n]
  unset di_mail[$n]
  unset di_mail_message[$n]  
  unset di_tel[$n]
  unset vdi_change[$n]
  unset vdi_act[$n]
  unset di_act_alt[$n]
  n=$(($n + 1))
done

DICH=$DIR/.di_change2
[ -e "$DICH" ] && . "$DICH"
n=0
while [ $n -lt 22 ];do
  case "${di_change[$n]}" in
  "low2high")
     vdi_change[$n]="low→high" ;;
  "high2low")
     vdi_change[$n]="high→low" ;;
  *)
    di_change[$n]="low2high"
    vdi_change[$n]="low→high"

  esac
  case "${di_act[$n]}" in
    "DON_0")
      vdi_act[$n]="${ALIAS_DO[0]}high" ;;
    "DOFF_0")
      vdi_act[$n]="${ALIAS_DO[0]}low" ;;
    "DON_1")
      vdi_act[$n]="${ALIAS_DO[1]}high" ;;
    "DOFF_1")
      vdi_act[$n]="${ALIAS_DO[1]}low" ;;
    "DON_2")
      vdi_act[$n]="${ALIAS_DO[2]}high" ;;
    "DOFF_2")
      vdi_act[$n]="${ALIAS_DO[2]}low" ;;
    "DON_3")
      vdi_act[$n]="${ALIAS_DO[3]}high" ;;
    "DOFF_3")
      vdi_act[$n]="${ALIAS_DO[3]}low" ;;
    "IREXEC_0")
      vdi_act[$n]="${ALIAS_DO[8]}" ;;
    "IREXEC_1")
      vdi_act[$n]="${ALIAS_DO[9]}" ;;
    "IREXEC_2")
      vdi_act[$n]="${ALIAS_DO[10]}" ;;
    "IREXEC_3")
      vdi_act[$n]="${ALIAS_DO[11]}" ;;
    "IREXEC_4")
      vdi_act[$n]="${ALIAS_DO[12]}" ;;
    "IREXEC_5")
      vdi_act[$n]="${ALIAS_DO[13]}" ;;
    "TON_0")
      vdi_act[$n]="${ALIAS_DO[14]}high" ;;
    "TOFF_0")
      vdi_act[$n]="${ALIAS_DO[14]}low" ;;
    "TON_1")
      vdi_act[$n]="${ALIAS_DO[15]}high" ;;
    "TOFF_1")
      vdi_act[$n]="${ALIAS_DO[15]}low" ;;
    "TON_2")
      vdi_act[$n]="${ALIAS_DO[16]}high" ;;
    "TOFF_2")
      vdi_act[$n]="${ALIAS_DO[16]}low" ;;
    "phone")
      vdi_act[$n]="Phone" ;;
    "mail")
      vdi_act[$n]="Email" ;;
    "mail_message")
      vdi_act[$n]="Email_messageage" ;;
    "web_camera_video")
      vdi_act[$n]="Web_camera Video" ;;
    "mod_camera_still")
      vdi_act[$n]="Mod_camera Still" ;;
    "mod_camera_video")
      vdi_act[$n]="Mod_camera Video" ;;
    "SOUND_0")
      vdi_act[$n]="Sound_1" ;;
    "SOUND_1")
      vdi_act[$n]="Sound_2" ;;
    "SOUND_2")
      vdi_act[$n]="Sound_3" ;;
    "SOUND_3")
      vdi_act[$n]="Sound_4" ;;
    "SOUND_4")
      vdi_act[$n]="Sound_5" ;;
    *)
      di_act[$n]="none"
      vdi_act[$n]="none" ;;
  esac
  [ -n "${don_time[$n]}" ] && DON_TIME[$n]="${don_time[$n]}" || DON_TIME[$n]=""
  [ -n "${di_tel[$n]}" ] && DI_TELNO[$n]="${di_tel[$n]}" || DI_TELNO[$n]=""
  [ -n "${di_mail[$n]}" ] && DI_MAIL[$n]="${di_mail[$n]}" || DI_MAIL[$n]=""
  if [ -n "${di_mail_message[$n]}" ];then
    DI_MESS[$n]="`echo "${di_mail_message[$n]}" |tr "+" " "`"
  else
    DI_MESS[$n]=""
  fi
  [ -n "${di_act_alt[$n]}" ] && DI_ACT_ALT[$n]="${di_act_alt[$n]}" || DI_ACT_ALT[$n]="none"
  n=$(($n + 1))
done

cat >>$PAGE1<<END
<DL id="menu10dl">
<DT><FONT SIZE="+1"><B>Management DI(Digital Input)-2</B></FONT></DT>
<DD>
<FORM NAME="menu10" id="menu10_form" ACTION="./di_contorl_pi2.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="3"><B>Setting second action to the digital input</B></FONT>
<BR>
<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[0]}">
<span id="menu100di_0"><INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[0]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_0" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_0">
<OPTION VALUE="${di_act[0]}" SELECTED>${vdi_act[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_0">
<OPTION VALUE="${DI_ACT_ALT[0]}" SELECTED>${DI_ACT_ALT[0]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[0]}" NAME="don_time_0">ms&nbsp;
<SELECT NAME="di_change_reg_0">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[0]}" NAME="di_tel_0">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[0]}" NAME="di_mail_0">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[0]}" NAME="di_mail_message_0">
&nbsp;
<span id="menu100ct_0">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[1]}">
<span id="menu100di_1">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[1]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_1" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_1">
<OPTION VALUE="${di_act[1]}" SELECTED>${vdi_act[1]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_1">
<OPTION VALUE="${DI_ACT_ALT[1]}" SELECTED>${DI_ACT_ALT[1]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[1]}" NAME="don_time_1">ms&nbsp;
<SELECT NAME="di_change_reg_1">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[1]}" NAME="di_tel_1">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[1]}" NAME="di_mail_1">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[1]}" NAME="di_mail_message_1">
&nbsp;
<span id="menu100ct_1">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[2]}">
<span id="menu100di_2">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[2]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_2" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_2">
<OPTION VALUE="${di_act[2]}" SELECTED>${vdi_act[2]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_2">
<OPTION VALUE="${DI_ACT_ALT[2]}" SELECTED>${DI_ACT_ALT[2]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[2]}" NAME="don_time_2">ms&nbsp;
<SELECT NAME="di_change_reg_2">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[2]}" NAME="di_tel_2">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[2]}" NAME="di_mail_2">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[2]}" NAME="di_mail_message_2">
&nbsp;
<span id="menu100ct_2">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[3]}">
<span id="menu100di_3">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[3]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_3" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_3">
<OPTION VALUE="${di_act[3]}" SELECTED>${vdi_act[3]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_3">
<OPTION VALUE="${DI_ACT_ALT[3]}" SELECTED>${DI_ACT_ALT[3]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[3]}" NAME="don_time_3">ms&nbsp;
<SELECT NAME="di_change_reg_3">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[3]}" NAME="di_tel_3">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[3]}" NAME="di_mail_3">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[3]}" NAME="di_mail_message_3">
&nbsp;
<span id="menu100ct_3">
</span>
<HR>
<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[8]}">
<span id="menu100di_8">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[8]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_8" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_8">
<OPTION VALUE="${di_act[8]}" SELECTED>${vdi_act[8]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_8">
<OPTION VALUE="${DI_ACT_ALT[8]}" SELECTED>${DI_ACT_ALT[8]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[8]}" NAME="don_time_8">ms&nbsp;
<SELECT NAME="di_change_reg_8">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[8]}" NAME="di_tel_8">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[8]}" NAME="di_mail_8">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[8]}" NAME="di_mail_message_8">
&nbsp;
<span id="menu100ct_8">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[9]}">
<span id="menu100di_9">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[9]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_9" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_9">
<OPTION VALUE="${di_act[9]}" SELECTED>${vdi_act[9]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_9">
<OPTION VALUE="${DI_ACT_ALT[9]}" SELECTED>${DI_ACT_ALT[9]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[9]}" NAME="don_time_9">ms&nbsp;
<SELECT NAME="di_change_reg_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[9]}" NAME="di_tel_9">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[9]}" NAME="di_mail_9">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[9]}" NAME="di_mail_message_9">
&nbsp;
<span id="menu100ct_9">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[10]}">
<span id="menu100di_10">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[10]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_10" VALUE="low2high">
Action:low→high
<SELECT NAME="di_act_10">
<OPTION VALUE="${di_act[10]}" SELECTED>${vdi_act[10]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_10">
<OPTION VALUE="${DI_ACT_ALT[10]}" SELECTED>${DI_ACT_ALT[10]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[10]}" NAME="don_time_10">ms&nbsp;
<SELECT NAME="di_change_reg_10">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[10]}" NAME="di_tel_10">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[10]}" NAME="di_mail_10">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[10]}" NAME="di_mail_message_10">
&nbsp;
<span id="menu100ct_10">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[0]}">
<span id="menu101di_0">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[0]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_11" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_11">
<OPTION VALUE="${di_act[11]}" SELECTED>${vdi_act[11]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_11">
<OPTION VALUE="${DI_ACT_ALT[11]}" SELECTED>${DI_ACT_ALT[11]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[11]}" NAME="don_time_11">ms&nbsp;
<SELECT NAME="di_change_reg_11">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[11]}" NAME="di_tel_11">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[11]}" NAME="di_mail_11">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[11]}" NAME="di_mail_message_11">
&nbsp;
<span id="menu101ct_0">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[1]}">
<span id="menu101di_1">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[12]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_12" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_12">
<OPTION VALUE="${di_act[12]}" SELECTED>${vdi_act[12]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_12">
<OPTION VALUE="${DI_ACT_ALT[12]}" SELECTED>${DI_ACT_ALT[12]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[12]}" NAME="don_time_12">ms&nbsp;
<SELECT NAME="di_change_reg_12">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[12]}" NAME="di_tel_12">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[12]}" NAME="di_mail_12">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[12]}" NAME="di_mail_message_12">
&nbsp;
<span id="menu101ct_1">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[2]}">
<span id="menu101di_2">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[13]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_13" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_13">
<OPTION VALUE="${di_act[13]}" SELECTED>${vdi_act[13]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_13">
<OPTION VALUE="${DI_ACT_ALT[13]}" SELECTED>${DI_ACT_ALT[13]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[13]}" NAME="don_time_13">ms&nbsp;
<SELECT NAME="di_change_reg_13">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[13]}" NAME="di_tel_13">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[13]}" NAME="di_mail_13">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[13]}" NAME="di_mail_message_13">
&nbsp;
<span id="menu101ct_2">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[3]}">
<span id="menu101di_3">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[14]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_14" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_14">
<OPTION VALUE="${di_act[14]}" SELECTED>${vdi_act[14]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_"14>
<OPTION VALUE="${DI_ACT_ALT[14]}" SELECTED>${DI_ACT_ALT[14]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[14]}" NAME="don_time_14">ms&nbsp;
<SELECT NAME="di_change_reg_14">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[14]}" NAME="di_tel_14">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[14]}" NAME="di_mail_14">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[14]}" NAME="di_mail_message_14">
&nbsp;
<span id="menu101ct_3">
</span>
<HR>
<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[8]}">
<span id="menu101di_8">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[8]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_19" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_19">
<OPTION VALUE="${di_act[19]}" SELECTED>${vdi_act[19]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_19">
<OPTION VALUE="${DI_ACT_ALT[19]}" SELECTED>${DI_ACT_ALT[19]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[19]}" NAME="don_time_19">ms&nbsp;
<SELECT NAME="di_change_reg_19">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[19]}" NAME="di_tel_19">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[19]}" NAME="di_mail_19">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[19]}" NAME="di_mail_message_19">
&nbsp;
<span id="menu101ct_8">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[9]}">
<span id="menu101di_9">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[9]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_20" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_20">
<OPTION VALUE="${di_act[20]}" SELECTED>${vdi_act[20]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_20">
<OPTION VALUE="${DI_ACT_ALT[20]}" SELECTED>${DI_ACT_ALT[20]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[20]}" NAME="don_time_20">ms&nbsp;
<SELECT NAME="di_change_reg_20">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[20]}" NAME="di_tel_20">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[20]}" NAME="di_mail_20">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[20]}" NAME="di_mail_message_20">
&nbsp;
<span id="menu101ct_9">
</span>
<HR>

<INPUT TYPE="text" readonly style="width:100px;" VALUE="${ALIAS_DI[10]}">
<span id="menu101di_10">
<INPUT TYPE="text" readonly style="width:36px;text-align:center;" VALUE="${DI[10]}">&nbsp;
</span>
<INPUT TYPE="hidden" NAME="di_change_21" VALUE="high2low">
Action:high→low
<SELECT NAME="di_act_21">
<OPTION VALUE="${di_act[21]}" SELECTED>${vdi_act[21]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="phone">Phone
<OPTION VALUE="mail">Email
<OPTION VALUE="mail_message">Email_messageage
<OPTION VALUE="web_camera_video">Web_camera Video
<OPTION VALUE="mod_camera_still">Mod_camera Still
<OPTION VALUE="mod_camera_video">Mod_camera Video
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>
Alt
<SELECT NAME="di_act_alt_21">
<OPTION VALUE="${DI_ACT_ALT[21]}" SELECTED>${DI_ACT_ALT[21]}
<OPTION VALUE="none">none
<OPTION VALUE="alt">alt
</SELECT>
&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${DON_TIME[21]}" NAME="don_time_21">ms&nbsp;
<SELECT NAME="di_change_reg_21">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
Phone<INPUT TYPE="text" style="width:100px;text-align:left;" VALUE="${DI_TELNO[21]}" NAME="di_tel_21">
&nbsp;
Email<INPUT TYPE="text" style="width:120px;text-align:left;" VALUE="${DI_MAIL[21]}" NAME="di_mail_21">
&nbsp;
Message<INPUT TYPE="text" style="width:50px;text-align:left;" VALUE="${DI_MESS[21]}" NAME="di_mail_message_21">
&nbsp;
<span id="menu101ct_10">
</span>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu10_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
</DD>
</DL>
END

MAIL_SET=$DIR/.mail_set.list
CONF="$DIR/.pepogmail4dio.conf"
[ -e $CONF ] && . $CONF
[ ! -z "$GMAILUSER" ] && vGMAILUSER="$GMAILUSER"
[ ! -z "$GMAILPASSWORD" ] && vGMAILPASSWORD="*"
[ ! -z "$PERMITMAIL" ] && vPERMITMAIL="$PERMITMAIL" || vPERMITMAIL="all@pepolinux.local"
[ ! -z "$KEYWORD" ] && vKEYWORD="$KEYWORD"
[ ! -z "$JITTER" ] && vJITTER="$JITTER"
[ ! -z "$LOOPTIME" ] && vLOOPTIME="$LOOPTIME"
cat >>$PAGE1<<END
<DL id="menu11dl">
<DT><FONT SIZE="+1"><B>Setting system Email</B></FONT></DT>
<DD>
<B>Setting operation in Gmail</B>
<BR>
<FORM NAME="menu11" id="menu11_form" ACTION="gmail_set.cgi" METHOD="post" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
Gmail User<INPUT TYPE="text" style="width:80px;text-align:right;" VALUE="$vGMAILUSER" NAME="gmailuser">@gmail.com<BR>
Gmail Password<INPUT TYPE="password" style="width:80px;text-align:left;" VALUE="$vGMAILPASSWORD" NAME="gmailpassword"><BR>
Mail Address<INPUT TYPE="text" style="width:180px;text-align:left;" VALUE="$vPERMITMAIL" NAME="permitmail">&nbsp;Allow Email address<BR>
Key Word<INPUT TYPE="text" style="width:80px;text-align:left;" VALUE="$vKEYWORD" NAME="keyword">&nbsp;Subject(keyword)<BR>
Mail Check <INPUT TYPE="text" style="width:20px;text-align:left;" VALUE="$vLOOPTIME" NAME="looptime">&nbsp;New Email check interval(Sec)<BR>
Jitter<INPUT TYPE="text" style="width:20px;text-align:left;" VALUE="$vJITTER" NAME="jitter">&nbsp;Email Arrival correction(Sec)<BR>
<SELECT NAME="reg">
<option VALUE="reg" SELECTED>Entry
<option VALUE="del">Delete
</SELECT>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu11_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
</DD>
</DL>
END

WGET_LIST=$DIR/.podcasts_list
[ -e "$WGET_LIST" ] && . "$WGET_LIST"
for n in 0 1 2 3 4 5 6 7 8 9;do
  [ ! -z "${wget_val[$n]}" ] && vWGET_VAL[$n]="${wget_val[$n]}" || vWGET_VAL[$n]='*'
  [ "${wget_val[$n]}" = "*" ] && vWGET_VAL[$n]="*"
  [ "${wget_val[$n]}" = "*" ] && vWGET_VAL[$n]="*"
  if [ $n = 7 ];then
    case "${wget_val[$n]}" in
      0) vWGET_VAL[$n]="Sun" ;;
      1) vWGET_VAL[$n]="M" ;;
      2) vWGET_VAL[$n]="Tue" ;;
      3) vWGET_VAL[$n]="Wed" ;;
      4) vWGET_VAL[$n]="Thu" ;;
      5) vWGET_VAL[$n]="Fri" ;;
      6) vWGET_VAL[$n]="Sat" ;; 
    esac
  fi
done
[ "${vWGET_VAL[0]}" = '*' ] &&  vWGET_VAL[0]=""
[ "${vWGET_VAL[1]}" = '*' ] &&  vWGET_VAL[1]="1"
[ "${vWGET_VAL[8]}" = '*' ] &&  vWGET_VAL[8]="/dev/sdb1"
[ "${vWGET_VAL[9]}" = '*' ] &&  vWGET_VAL[9]="Podcasts"

cat >>$PAGE1<<END
<DL id="menu12dl">
<DT><FONT SIZE="3"><B>Setting automated processing</B></FONT></DT>
<DD>
<FONT SIZE="2"><B>Setting MP3 file from URL</B></FONT>
<FORM NAME="menu12" id="menu12_form" ACTION="podcastsget.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
URL<INPUT TYPE="text" style="width:400px;text-align:left;" VALUE="${vWGET_VAL[0]}" NAME="wget_val_0">
http://www3.nhk.or.jp/rj/podcast/rss/english.xml
<BR>
Generation management<SELECT NAME="wget_val_1">
<OPTION VALUE="${vWGET_VAL[1]}" SELECTED>${vWGET_VAL[1]}<OPTION VALUE="1">1<OPTION VALUE="2">2<OPTION VALUE="3">3<OPTION VALUE="4">4<OPTION VALUE="5">5<OPTION VALUE="6">6<OPTION VALUE="7">7<OPTION VALUE="8">8<OPTION VALUE="9">9<OPTION VALUE="10">10
</SELECT>
&nbsp;
Start<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vWGET_VAL[3]}" NAME="wget_val_3">00-59&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vWGET_VAL[4]}" NAME="wget_val_4">00-23&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vWGET_VAL[5]}" NAME="wget_val_5">1-31&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vWGET_VAL[6]}" NAME="wget_val_6">1-12&nbsp;
<SELECT NAME="wget_val_7">
<OPTION VALUE="${vWGET_VAL[7]}" SELECTED>${vWGET_VAL[7]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>&nbsp;*:Every time
<BR>
Device Name:
<SELECT NAME="wget_val_8"><OPTION VALUE="${vWGET_VAL[8]}" SELECTED>${vWGET_VAL[8]}
<OPTION VALUE="/dev/sdb1">/dev/sdb1
<OPTION VALUE="/dev/sdc1">/dev/sdc1
<OPTION VALUE="/dev/sda1">/dev/sda1
</SELECT>&nbsp;
Folder name<INPUT TYPE="text" style="width:80px;text-align:left;" VALUE="${vWGET_VAL[9]}" NAME="wget_val_9">
&nbsp;
<SELECT NAME="wget_val_2">
<OPTION VALUE="reg" SELECTED>Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu12_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
<HR>
END

AUTO_ACT_LIST=$DIR/.auto_act.list
[ -e "$AUTO_ACT_LIST" ] && . "$AUTO_ACT_LIST"
for n in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20;do
  vAUTO_ACT_CON[$n]="Enable"
  [ ! -z "${auto_act_con[$n]}" ] && vAUTO_ACT_CON[$n]=${auto_act_con[$n]} || auto_act_con[$n]="Enable"
  case ${auto_act_con[$n]} in
    "DI_ON_0")  vAUTO_ACT_CON[$n]=${ALIAS_DI[0]}high  ;; 
    "DI_OFF_0")  vAUTO_ACT_CON[$n]=${ALIAS_DI[0]}low  ;;
    "DI_ON_1")  vAUTO_ACT_CON[$n]=${ALIAS_DI[1]}high  ;;
    "DI_OFF_1")  vAUTO_ACT_CON[$n]=${ALIAS_DI[1]}low  ;;
    "DI_ON_2")  vAUTO_ACT_CON[$n]=${ALIAS_DI[2]}high  ;;
    "DI_OFF_2")  vAUTO_ACT_CON[$n]=${ALIAS_DI[2]}low  ;;
    "DI_ON_3")  vAUTO_ACT_CON[$n]=${ALIAS_DI[3]}high  ;;
    "DI_OFF_3")  vAUTO_ACT_CON[$n]=${ALIAS_DI[3]}low  ;;
    "DI_ON_8")  vAUTO_ACT_CON[$n]=${ALIAS_DI[8]}high  ;;
    "DI_OFF_8")  vAUTO_ACT_CON[$n]=${ALIAS_DI[8]}low  ;;
    "DI_ON_9")  vAUTO_ACT_CON[$n]=${ALIAS_DI[9]}high  ;;
    "DI_OFF_9")  vAUTO_ACT_CON[$n]=${ALIAS_DI[9]}low  ;;
    "DI_ON_10")  vAUTO_ACT_CON[$n]=${ALIAS_DI[10]}high  ;;
    "DI_OFF_10")  vAUTO_ACT_CON[$n]=${ALIAS_DI[10]}low  ;;
    "DI_ON_12")  vAUTO_ACT_CON[$n]=${ALIAS_DI[12]}high  ;;
    "DI_OFF_12")  vAUTO_ACT_CON[$n]=${ALIAS_DI[12]}low  ;;
    "DI_ON_13")  vAUTO_ACT_CON[$n]=${ALIAS_DI[13]}high  ;;
    "DI_OFF_13")  vAUTO_ACT_CON[$n]=${ALIAS_DI[13]}low  ;;
    "DI_ON_14")  vAUTO_ACT_CON[$n]=${ALIAS_DI[14]}high  ;;
    "DI_OFF_14")  vAUTO_ACT_CON[$n]=${ALIAS_DI[14]}low  ;;
    "DI_ON_15")  vAUTO_ACT_CON[$n]=${ALIAS_DI[15]}high  ;;
    "DI_OFF_15")  vAUTO_ACT_CON[$n]=${ALIAS_DI[15]}low  ;;
    "DI_ON_16")  vAUTO_ACT_CON[$n]=${ALIAS_DI[16]}high  ;;
    "DI_OFF_16")  vAUTO_ACT_CON[$n]=${ALIAS_DI[16]}low  ;;
    "DI_ON_17")  vAUTO_ACT_CON[$n]=${ALIAS_DI[17]}high  ;;
    "DI_OFF_17")  vAUTO_ACT_CON[$n]=${ALIAS_DI[17]}low  ;;
    "DI_ON_18")  vAUTO_ACT_CON[$n]=${ALIAS_DI[18]}high  ;;
    "DI_OFF_18")  vAUTO_ACT_CON[$n]=${ALIAS_DI[18]}low  ;;
    "DI_ON_19")  vAUTO_ACT_CON[$n]=${ALIAS_DI[19]}high  ;;
    "DI_OFF_19")  vAUTO_ACT_CON[$n]=${ALIAS_DI[19]}low  ;;
    "DI_ON_20")  vAUTO_ACT_CON[$n]=${ALIAS_DI[20]}high  ;;
    "DI_OFF_20")  vAUTO_ACT_CON[$n]=${ALIAS_DI[20]}low  ;;
    "DI_ON_21")  vAUTO_ACT_CON[$n]=${ALIAS_DI[21]}high  ;;
    "DI_OFF_21")  vAUTO_ACT_CON[$n]=${ALIAS_DI[21]}low  ;;
    "DI_ON_22")  vAUTO_ACT_CON[$n]=${ALIAS_DI[22]}high  ;;
    "DI_OFF_22")  vAUTO_ACT_CON[$n]=${ALIAS_DI[22]}low  ;;
    "DI_ON_23")  vAUTO_ACT_CON[$n]=${ALIAS_DI[23]}high  ;;
    "DI_OFF_23")  vAUTO_ACT_CON[$n]=${ALIAS_DI[23]}low  ;;
  esac  
  case $n in
    0)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act0_val[${m}]}" ] && vAUTO_ACT0_VAL[${m}]="${auto_act0_val[${m}]}" || vAUTO_ACT0_VAL[${m}]="*"
      done
      [ -z "${auto_act0_val[0]}" ] && auto_act0_val[0]="none"
      case "${auto_act0_val[0]}" in
        DON_0)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[8]}low ;;
        IREXEC_0)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[8]}low ;;
        IREXEC_1)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[9]}low ;;
        IREXEC_2)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[10]}low ;;
        IREXEC_3)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[11]}low ;;
        IREXEC_4)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[12]}low ;;
        IREXEC_5)  
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[13]}low ;;
        TON_0)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT0_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT0_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT0_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT0_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT0_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT0_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT0_VAL[0]="none";;
      esac
      [ -z "${auto_act0_val[1]}" ] && vAUTO_ACT0_VAL[1]=""
      [ -z "${auto_act0_val[8]}" ] && auto_act0_val[8]="*"
      case "${auto_act0_val[8]}" in
        0) vAUTO_ACT0_VAL[8]="Sun" ;;
        1) vAUTO_ACT0_VAL[8]="Mon" ;;
        2) vAUTO_ACT0_VAL[8]="Tue" ;;
        3) vAUTO_ACT0_VAL[8]="Wed" ;;
        4) vAUTO_ACT0_VAL[8]="Thu" ;;
        5) vAUTO_ACT0_VAL[8]="Fri" ;;
        6) vAUTO_ACT0_VAL[8]="Sat" ;;
        *) vAUTO_ACT0_VAL[8]="*" ;;
      esac
    ;;
    1)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act1_val[${m}]}" ] && vAUTO_ACT1_VAL[${m}]="${auto_act1_val[${m}]}" || vAUTO_ACT1_VAL[${m}]="*"
      done
      [ -z "${auto_act1_val[0]}" ] && auto_act1_val[0]="none"
      case "${auto_act1_val[0]}" in
        DON_0)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[8]}low ;;
        IREXEC_0)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[8]}low ;;
        IREXEC_1)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[9]}low ;;
        IREXEC_2)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[10]}low ;;
        IREXEC_3)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[11]}low ;;
        IREXEC_4)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[12]}low ;;
        IREXEC_5)  
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[13]}low ;;
        TON_0)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT1_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT1_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT1_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT1_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT1_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT1_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT1_VAL[0]="none";;
      esac
      [ -z "${auto_act1_val[1]}" ] && vAUTO_ACT1_VAL[1]=""
      [ -z "${auto_act1_val[8]}" ] && auto_act1_val[8]="*"
      case "${auto_act1_val[8]}" in
        0) vAUTO_ACT1_VAL[8]="Sun" ;;
        1) vAUTO_ACT1_VAL[8]="Mon" ;;
        2) vAUTO_ACT1_VAL[8]="Tue" ;;
        3) vAUTO_ACT1_VAL[8]="Wed" ;;
        4) vAUTO_ACT1_VAL[8]="Thu" ;;
        5) vAUTO_ACT1_VAL[8]="Fri" ;;
        6) vAUTO_ACT1_VAL[8]="Sat" ;;
        *) vAUTO_ACT1_VAL[8]="*" ;;
      esac
    ;;
    2)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act2_val[${m}]}" ] && vAUTO_ACT2_VAL[${m}]="${auto_act2_val[${m}]}" || vAUTO_ACT2_VAL[${m}]="*"
      done
      [ -z "${auto_act2_val[0]}" ] && auto_act2_val[0]="none"
      case "${auto_act2_val[0]}" in
        DON_0)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[8]}low ;;
        IREXEC_0)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[8]}low ;;
        IREXEC_1)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[9]}low ;;
        IREXEC_2)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[10]}low ;;
        IREXEC_3)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[11]}low ;;
        IREXEC_4)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[12]}low ;;
        IREXEC_5)  
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[13]}low ;;
        TON_0)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT2_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT2_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT2_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT2_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT2_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT2_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT2_VAL[0]="none";;
      esac
      [ -z "${auto_act2_val[1]}" ] && vAUTO_ACT2_VAL[1]=""
      [ -z "${auto_act2_val[8]}" ] && auto_act2_val[8]="*"
      case "${auto_act2_val[8]}" in
        0) vAUTO_ACT2_VAL[8]="Sun" ;;
        1) vAUTO_ACT2_VAL[8]="Mon" ;;
        2) vAUTO_ACT2_VAL[8]="Tue" ;;
        3) vAUTO_ACT2_VAL[8]="Wed" ;;
        4) vAUTO_ACT2_VAL[8]="Thu" ;;
        5) vAUTO_ACT2_VAL[8]="Fri" ;;
        6) vAUTO_ACT2_VAL[8]="Sat" ;;
        *) vAUTO_ACT2_VAL[8]="*" ;;
      esac
    ;;
    3)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act3_val[${m}]}" ] && vAUTO_ACT3_VAL[${m}]="${auto_act3_val[${m}]}" || vAUTO_ACT3_VAL[${m}]="*"
      done
      [ -z "${auto_act3_val[0]}" ] && auto_act3_val[0]="none"
      case "${auto_act3_val[0]}" in
        DON_0)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT3_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT3_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT3_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT3_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT3_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT3_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT3_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT3_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT3_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT3_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT3_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT3_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT3_VAL[0]="none";;
      esac
      [ -z "${auto_act3_val[1]}" ] && vAUTO_ACT3_VAL[1]=""
      [ -z "${auto_act3_val[8]}" ] && auto_act3_val[8]="*"
      case "${auto_act3_val[8]}" in
        0) vAUTO_ACT3_VAL[8]="Sun" ;;
        1) vAUTO_ACT3_VAL[8]="Mon" ;;
        2) vAUTO_ACT3_VAL[8]="Tue" ;;
        3) vAUTO_ACT3_VAL[8]="Wed" ;;
        4) vAUTO_ACT3_VAL[8]="Thu" ;;
        5) vAUTO_ACT3_VAL[8]="Fri" ;;
        6) vAUTO_ACT3_VAL[8]="Sat" ;;
        *) vAUTO_ACT3_VAL[8]="*" ;;
      esac
    ;;
    4)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act4_val[${m}]}" ] && vAUTO_ACT4_VAL[${m}]="${auto_act4_val[${m}]}" || vAUTO_ACT4_VAL[${m}]="*"
      done
      [ -z "${auto_act4_val[0]}" ] && auto_act4_val[0]="none"
      case "${auto_act4_val[0]}" in
        DON_0)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT4_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT4_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT4_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT4_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT4_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT4_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT4_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT4_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT4_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT4_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT4_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT4_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT4_VAL[0]="none";;
      esac
      [ -z "${auto_act4_val[1]}" ] && vAUTO_ACT4_VAL[1]=""
      [ -z "${auto_act4_val[8]}" ] && auto_act4_val[8]="*"
      case "${auto_act4_val[8]}" in
        0) vAUTO_ACT4_VAL[8]="Sun" ;;
        1) vAUTO_ACT4_VAL[8]="Mon" ;;
        2) vAUTO_ACT4_VAL[8]="Tue" ;;
        3) vAUTO_ACT4_VAL[8]="Wed" ;;
        4) vAUTO_ACT4_VAL[8]="Thu" ;;
        5) vAUTO_ACT4_VAL[8]="Fri" ;;
        6) vAUTO_ACT4_VAL[8]="Sat" ;;
        *) vAUTO_ACT4_VAL[8]="*" ;;
      esac
    ;;
    5)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act5_val[${m}]}" ] && vAUTO_ACT5_VAL[${m}]="${auto_act5_val[${m}]}" || vAUTO_ACT5_VAL[${m}]="*"
      done
      [ -z "${auto_act5_val[0]}" ] && auto_act5_val[0]="none"
      case "${auto_act5_val[0]}" in
        DON_0)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT5_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT5_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT5_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT5_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT5_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT5_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT5_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT5_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT5_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT5_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT5_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT5_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT5_VAL[0]="none";;
      esac
      [ -z "${auto_act5_val[1]}" ] && vAUTO_ACT5_VAL[1]=""
      [ -z "${auto_act5_val[8]}" ] && auto_act5_val[8]="*"
      case "${auto_act5_val[8]}" in
        0) vAUTO_ACT5_VAL[8]="Sun" ;;
        1) vAUTO_ACT5_VAL[8]="Mon" ;;
        2) vAUTO_ACT5_VAL[8]="Tue" ;;
        3) vAUTO_ACT5_VAL[8]="Wed" ;;
        4) vAUTO_ACT5_VAL[8]="Thu" ;;
        5) vAUTO_ACT5_VAL[8]="Fri" ;;
        6) vAUTO_ACT5_VAL[8]="Sat" ;;
        *) vAUTO_ACT5_VAL[8]="*" ;;
      esac
    ;;
    6)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act6_val[${m}]}" ] && vAUTO_ACT6_VAL[${m}]="${auto_act6_val[${m}]}" || vAUTO_ACT6_VAL[${m}]="*"
      done
      [ -z "${auto_act6_val[0]}" ] && auto_act6_val[0]="none"
      case "${auto_act6_val[0]}" in
        DON_0)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT6_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT6_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT6_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT6_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT6_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT6_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT6_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT6_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT6_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT6_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT6_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT6_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT6_VAL[0]="none";;
      esac
      [ -z "${auto_act6_val[1]}" ] && vAUTO_ACT6_VAL[1]=""
      [ -z "${auto_act6_val[8]}" ] && auto_act6_val[8]="*"
      case "${auto_act6_val[8]}" in
        0) vAUTO_ACT6_VAL[8]="Sun" ;;
        1) vAUTO_ACT6_VAL[8]="Mon" ;;
        2) vAUTO_ACT6_VAL[8]="Tue" ;;
        3) vAUTO_ACT6_VAL[8]="Wed" ;;
        4) vAUTO_ACT6_VAL[8]="Thu" ;;
        5) vAUTO_ACT6_VAL[8]="Fri" ;;
        6) vAUTO_ACT6_VAL[8]="Sat" ;;
        *) vAUTO_ACT6_VAL[8]="*" ;;
      esac
    ;;
    7)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act7_val[${m}]}" ] && vAUTO_ACT7_VAL[${m}]="${auto_act7_val[${m}]}" || vAUTO_ACT7_VAL[${m}]="*"
      done
      [ -z "${auto_act7_val[0]}" ] && auto_act7_val[0]="none"
      case "${auto_act7_val[0]}" in
        DON_0)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT7_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT7_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT7_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT7_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT7_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT7_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT7_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT7_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT7_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT7_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT7_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT7_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT7_VAL[0]="none";;
      esac
      [ -z "${auto_act7_val[1]}" ] && vAUTO_ACT7_VAL[1]=""
      [ -z "${auto_act7_val[8]}" ] && auto_act7_val[8]="*"
      case "${auto_act7_val[8]}" in
        0) vAUTO_ACT7_VAL[8]="Sun" ;;
        1) vAUTO_ACT7_VAL[8]="Mon" ;;
        2) vAUTO_ACT7_VAL[8]="Tue" ;;
        3) vAUTO_ACT7_VAL[8]="Wed" ;;
        4) vAUTO_ACT7_VAL[8]="Thu" ;;
        5) vAUTO_ACT7_VAL[8]="Fri" ;;
        6) vAUTO_ACT7_VAL[8]="Sat" ;;
        *) vAUTO_ACT7_VAL[8]="*" ;;
      esac
    ;;
    8)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act8_val[${m}]}" ] && vAUTO_ACT8_VAL[${m}]="${auto_act8_val[${m}]}" || vAUTO_ACT8_VAL[${m}]="*"
      done
      [ -z "${auto_act8_val[0]}" ] && auto_act8_val[0]="none"
      case "${auto_act8_val[0]}" in
        DON_0)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT8_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT8_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT8_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT8_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT8_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT8_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT8_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT8_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT8_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT8_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT8_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT8_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT8_VAL[0]="none";;
      esac
      [ -z "${auto_act8_val[1]}" ] && vAUTO_ACT8_VAL[1]=""
      [ -z "${auto_act8_val[8]}" ] && auto_act8_val[8]="*"
      case "${auto_act8_val[8]}" in
        0) vAUTO_ACT8_VAL[8]="Sun" ;;
        1) vAUTO_ACT8_VAL[8]="Mon" ;;
        2) vAUTO_ACT8_VAL[8]="Tue" ;;
        3) vAUTO_ACT8_VAL[8]="Wed" ;;
        4) vAUTO_ACT8_VAL[8]="Thu" ;;
        5) vAUTO_ACT8_VAL[8]="Fri" ;;
        6) vAUTO_ACT8_VAL[8]="Sat" ;;
        *) vAUTO_ACT8_VAL[8]="*" ;;
      esac
    ;;
    9)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act9_val[${m}]}" ] && vAUTO_ACT9_VAL[${m}]="${auto_act9_val[${m}]}" || vAUTO_ACT9_VAL[${m}]="*"
      done
      [ -z "${auto_act9_val[0]}" ] && auto_act9_val[0]="none"
      case "${auto_act9_val[0]}" in
        DON_0)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT9_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT9_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT9_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT9_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT9_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT9_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT9_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT9_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT9_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT9_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT9_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT9_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT9_VAL[0]="none";;
      esac
      [ -z "${auto_act9_val[1]}" ] && vAUTO_ACT9_VAL[1]=""
      [ -z "${auto_act9_val[8]}" ] && auto_act9_val[8]="*"
      case "${auto_act9_val[8]}" in
        0) vAUTO_ACT9_VAL[8]="Sun" ;;
        1) vAUTO_ACT9_VAL[8]="Mon" ;;
        2) vAUTO_ACT9_VAL[8]="Tue" ;;
        3) vAUTO_ACT9_VAL[8]="Wed" ;;
        4) vAUTO_ACT9_VAL[8]="Thu" ;;
        5) vAUTO_ACT9_VAL[8]="Fri" ;;
        6) vAUTO_ACT9_VAL[8]="Sat" ;;
        *) vAUTO_ACT9_VAL[8]="*" ;;
      esac
    ;;
    10)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act10_val[${m}]}" ] && vAUTO_ACT10_VAL[${m}]="${auto_act10_val[${m}]}" || vAUTO_ACT10_VAL[${m}]="*"
      done
      [ -z "${auto_act10_val[0]}" ] && auto_act10_val[0]="none"
      case "${auto_act10_val[0]}" in
        DON_0)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT10_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT10_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT10_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT10_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT10_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT10_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT10_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT10_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT10_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT10_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT10_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT10_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT10_VAL[0]="none";;
      esac
      [ -z "${auto_act10_val[1]}" ] && vAUTO_ACT10_VAL[1]=""
      [ -z "${auto_act10_val[8]}" ] && auto_act10_val[8]="*"
      case "${auto_act10_val[8]}" in
        0) vAUTO_ACT10_VAL[8]="Sun" ;;
        1) vAUTO_ACT10_VAL[8]="Mon" ;;
        2) vAUTO_ACT10_VAL[8]="Tue" ;;
        3) vAUTO_ACT10_VAL[8]="Wed" ;;
        4) vAUTO_ACT10_VAL[8]="Thu" ;;
        5) vAUTO_ACT10_VAL[8]="Fri" ;;
        6) vAUTO_ACT10_VAL[8]="Sat" ;;
        *) vAUTO_ACT10_VAL[8]="*" ;;
      esac
    ;;
    11)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act11_val[${m}]}" ] && vAUTO_ACT11_VAL[${m}]="${auto_act11_val[${m}]}" || vAUTO_ACT11_VAL[${m}]="*"
      done
      [ -z "${auto_act11_val[0]}" ] && auto_act11_val[0]="none"
      case "${auto_act11_val[0]}" in
        DON_0)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT11_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT11_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT11_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT11_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT11_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT11_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT11_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT11_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT11_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT11_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT11_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT11_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT11_VAL[0]="none";;
      esac
      [ -z "${auto_act11_val[1]}" ] && vAUTO_ACT11_VAL[1]=""
      [ -z "${auto_act11_val[8]}" ] && auto_act11_val[8]="*"
      case "${auto_act11_val[8]}" in
        0) vAUTO_ACT11_VAL[8]="Sun" ;;
        1) vAUTO_ACT11_VAL[8]="Mon" ;;
        2) vAUTO_ACT11_VAL[8]="Tue" ;;
        3) vAUTO_ACT11_VAL[8]="Wed" ;;
        4) vAUTO_ACT11_VAL[8]="Thu" ;;
        5) vAUTO_ACT11_VAL[8]="Fri" ;;
        6) vAUTO_ACT11_VAL[8]="Sat" ;;
        *) vAUTO_ACT11_VAL[8]="*" ;;
      esac
    ;;
    12)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act12_val[${m}]}" ] && vAUTO_ACT12_VAL[${m}]="${auto_act12_val[${m}]}" || vAUTO_ACT12_VAL[${m}]="*"
      done
      [ -z "${auto_act12_val[0]}" ] && auto_act12_val[0]="none"
      case "${auto_act12_val[0]}" in
        DON_0)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT12_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT12_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT12_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT12_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT12_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT12_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT12_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT12_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT12_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT12_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT12_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT12_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT12_VAL[0]="none";;
      esac
      [ -z "${auto_act12_val[1]}" ] && vAUTO_ACT12_VAL[1]=""
      [ -z "${auto_act12_val[8]}" ] && auto_act12_val[8]="*"
      case "${auto_act12_val[8]}" in
        0) vAUTO_ACT12_VAL[8]="Sun" ;;
        1) vAUTO_ACT12_VAL[8]="Mon" ;;
        2) vAUTO_ACT12_VAL[8]="Tue" ;;
        3) vAUTO_ACT12_VAL[8]="Wed" ;;
        4) vAUTO_ACT12_VAL[8]="Thu" ;;
        5) vAUTO_ACT12_VAL[8]="Fri" ;;
        6) vAUTO_ACT12_VAL[8]="Sat" ;;
        *) vAUTO_ACT12_VAL[8]="*" ;;
      esac
    ;;
    13)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act13_val[${m}]}" ] && vAUTO_ACT13_VAL[${m}]="${auto_act13_val[${m}]}" || vAUTO_ACT13_VAL[${m}]="*"
      done
      [ -z "${auto_act13_val[0]}" ] && auto_act13_val[0]="none"
      case "${auto_act13_val[0]}" in
        DON_0)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT13_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT13_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT13_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT13_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT13_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT13_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT13_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT13_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT13_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT13_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT13_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT13_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT13_VAL[0]="none";;
      esac
      [ -z "${auto_act13_val[1]}" ] && vAUTO_ACT13_VAL[1]=""
      [ -z "${auto_act13_val[8]}" ] && auto_act13_val[8]="*"
      case "${auto_act13_val[8]}" in
        0) vAUTO_ACT13_VAL[8]="Sun" ;;
        1) vAUTO_ACT13_VAL[8]="Mon" ;;
        2) vAUTO_ACT13_VAL[8]="Tue" ;;
        3) vAUTO_ACT13_VAL[8]="Wed" ;;
        4) vAUTO_ACT13_VAL[8]="Thu" ;;
        5) vAUTO_ACT13_VAL[8]="Fri" ;;
        6) vAUTO_ACT13_VAL[8]="Sat" ;;
        *) vAUTO_ACT13_VAL[8]="*" ;;
      esac
    ;;
    14)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act14_val[${m}]}" ] && vAUTO_ACT14_VAL[${m}]="${auto_act14_val[${m}]}" || vAUTO_ACT14_VAL[${m}]="*"
      done
      [ -z "${auto_act14_val[0]}" ] && auto_act14_val[0]="none"
      case "${auto_act14_val[0]}" in
        DON_0)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT14_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT14_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT14_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT14_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT14_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT14_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT14_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT14_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT14_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT14_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT14_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT14_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT14_VAL[0]="none";;
      esac
      [ -z "${auto_act14_val[1]}" ] && vAUTO_ACT14_VAL[1]=""
      [ -z "${auto_act14_val[8]}" ] && auto_act14_val[8]="*"
      case "${auto_act14_val[8]}" in
        0) vAUTO_ACT14_VAL[8]="Sun" ;;
        1) vAUTO_ACT14_VAL[8]="Mon" ;;
        2) vAUTO_ACT14_VAL[8]="Tue" ;;
        3) vAUTO_ACT14_VAL[8]="Wed" ;;
        4) vAUTO_ACT14_VAL[8]="Thu" ;;
        5) vAUTO_ACT14_VAL[8]="Fri" ;;
        6) vAUTO_ACT14_VAL[8]="Sat" ;;
        *) vAUTO_ACT14_VAL[8]="*" ;;
      esac
    ;;
    15)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act15_val[${m}]}" ] && vAUTO_ACT15_VAL[${m}]="${auto_act15_val[${m}]}" || vAUTO_ACT15_VAL[${m}]="*"
      done
      [ -z "${auto_act15_val[0]}" ] && auto_act15_val[0]="none"
      case "${auto_act15_val[0]}" in
        DON_0)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT15_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT15_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT15_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT15_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT15_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT15_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT15_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT15_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT15_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT15_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT15_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT15_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT15_VAL[0]="none";;
      esac
      [ -z "${auto_act15_val[1]}" ] && vAUTO_ACT15_VAL[1]=""
      [ -z "${auto_act15_val[8]}" ] && auto_act15_val[8]="*"
      case "${auto_act15_val[8]}" in
        0) vAUTO_ACT15_VAL[8]="Sun" ;;
        1) vAUTO_ACT15_VAL[8]="Mon" ;;
        2) vAUTO_ACT15_VAL[8]="Tue" ;;
        3) vAUTO_ACT15_VAL[8]="Wed" ;;
        4) vAUTO_ACT15_VAL[8]="Thu" ;;
        5) vAUTO_ACT15_VAL[8]="Fri" ;;
        6) vAUTO_ACT15_VAL[8]="Sat" ;;
        *) vAUTO_ACT15_VAL[8]="*" ;;
      esac
    ;;
    16)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act16_val[${m}]}" ] && vAUTO_ACT16_VAL[${m}]="${auto_act16_val[${m}]}" || vAUTO_ACT16_VAL[${m}]="*"
      done
      [ -z "${auto_act16_val[0]}" ] && auto_act16_val[0]="none"
      case "${auto_act16_val[0]}" in
        DON_0)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT16_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT16_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT16_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT16_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT16_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT16_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT16_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT16_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT16_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT16_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT16_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT16_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT16_VAL[0]="none";;
      esac
      [ -z "${auto_act16_val[1]}" ] && vAUTO_ACT16_VAL[1]=""
      [ -z "${auto_act16_val[8]}" ] && auto_act16_val[8]="*"
      case "${auto_act16_val[8]}" in
        0) vAUTO_ACT16_VAL[8]="Sun" ;;
        1) vAUTO_ACT16_VAL[8]="Mon" ;;
        2) vAUTO_ACT16_VAL[8]="Tue" ;;
        3) vAUTO_ACT16_VAL[8]="Wed" ;;
        4) vAUTO_ACT16_VAL[8]="Thu" ;;
        5) vAUTO_ACT16_VAL[8]="Fri" ;;
        6) vAUTO_ACT16_VAL[8]="Sat" ;;
        *) vAUTO_ACT16_VAL[8]="*" ;;
      esac
    ;;
    17)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act17_val[${m}]}" ] && vAUTO_ACT17_VAL[${m}]="${auto_act17_val[${m}]}" || vAUTO_ACT17_VAL[${m}]="*"
      done
      [ -z "${auto_act17_val[0]}" ] && auto_act17_val[0]="none"
      case "${auto_act17_val[0]}" in
        DON_0)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT17_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT17_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT17_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT17_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT17_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT17_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT17_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT17_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT17_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT17_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT17_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT17_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT17_VAL[0]="none";;
      esac
      [ -z "${auto_act17_val[1]}" ] && vAUTO_ACT17_VAL[1]=""
      [ -z "${auto_act17_val[8]}" ] && auto_act17_val[8]="*"
      case "${auto_act17_val[8]}" in
        0) vAUTO_ACT17_VAL[8]="Sun" ;;
        1) vAUTO_ACT17_VAL[8]="Mon" ;;
        2) vAUTO_ACT17_VAL[8]="Tue" ;;
        3) vAUTO_ACT17_VAL[8]="Wed" ;;
        4) vAUTO_ACT17_VAL[8]="Thu" ;;
        5) vAUTO_ACT17_VAL[8]="Fri" ;;
        6) vAUTO_ACT17_VAL[8]="Sat" ;;
        *) vAUTO_ACT17_VAL[8]="*" ;;
      esac
    ;;
    18)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act18_val[${m}]}" ] && vAUTO_ACT18_VAL[${m}]="${auto_act18_val[${m}]}" || vAUTO_ACT18_VAL[${m}]="*"
      done
      [ -z "${auto_act18_val[0]}" ] && auto_act18_val[0]="none"
      case "${auto_act18_val[0]}" in
        DON_0)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT18_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT18_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT18_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT18_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT18_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT18_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT18_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT18_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT18_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT18_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT18_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT18_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT18_VAL[0]="none";;
      esac
      [ -z "${auto_act18_val[1]}" ] && vAUTO_ACT18_VAL[1]=""
      [ -z "${auto_act18_val[8]}" ] && auto_act18_val[8]="*"
      case "${auto_act18_val[8]}" in
        0) vAUTO_ACT18_VAL[8]="Sun" ;;
        1) vAUTO_ACT18_VAL[8]="Mon" ;;
        2) vAUTO_ACT18_VAL[8]="Tue" ;;
        3) vAUTO_ACT18_VAL[8]="Wed" ;;
        4) vAUTO_ACT18_VAL[8]="Thu" ;;
        5) vAUTO_ACT18_VAL[8]="Fri" ;;
        6) vAUTO_ACT18_VAL[8]="Sat" ;;
        *) vAUTO_ACT18_VAL[8]="*" ;;
      esac
    ;;
    19)
      for m in 0 1 2 3 4 5 6 7 8 9;do
        [ ! -z "${auto_act19_val[${m}]}" ] && vAUTO_ACT19_VAL[${m}]="${auto_act19_val[${m}]}" || vAUTO_ACT19_VAL[${m}]="*"
      done
      [ -z "${auto_act19_val[0]}" ] && auto_act19_val[0]="none"
      case "${auto_act19_val[0]}" in
        DON_0)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[0]}high ;;
        DOFF_0)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[0]}low ;;
        DON_1)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[1]}high ;;
        DOFF_1)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[1]}low ;;
        DON_2)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[2]}high ;;
        DOFF_2)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[2]}low ;;
        DON_3)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[3]}high ;;
        DOFF_3)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[3]}low ;;
        IREXEC_0)
          vAUTO_ACT19_VAL[0]="${ALIAS_DO[8]}" ;;
        IREXEC_1)
          vAUTO_ACT19_VAL[0]="${ALIAS_DO[9]}" ;;
        IREXEC_2)
          vAUTO_ACT19_VAL[0]="${ALIAS_DO[10]}" ;;
        IREXEC_3)
          vAUTO_ACT19_VAL[0]="${ALIAS_DO[11]}" ;;
        IREXEC_4)
          vAUTO_ACT19_VAL[0]="${ALIAS_DO[12]}" ;;
        IREXEC_5)
          vAUTO_ACT19_VAL[0]="${ALIAS_DO[13]}" ;;
        TON_0)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[14]}high ;; 
        TOFF_0)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[14]}low ;;
        TON_1)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[15]}high ;; 
        TOFF_1)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[15]}low ;;
        TON_2)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[16]}high ;; 
        TOFF_2)
          vAUTO_ACT19_VAL[0]=${ALIAS_DO[16]}low ;;
        SOUND_0)
          vAUTO_ACT19_VAL[0]=Sound_1 ;;
        SOUND_1)
          vAUTO_ACT19_VAL[0]=Sound_2 ;;
        SOUND_2)
          vAUTO_ACT19_VAL[0]=Sound_3 ;;
        SOUND_3)
          vAUTO_ACT19_VAL[0]=Sound_4 ;;
        SOUND_4)
          vAUTO_ACT19_VAL[0]=Sound_5 ;;
        none)
          vAUTO_ACT19_VAL[0]="none";;
      esac
      [ -z "${auto_act19_val[1]}" ] && vAUTO_ACT19_VAL[1]=""
      [ -z "${auto_act19_val[8]}" ] && auto_act19_val[8]="*"
      case "${auto_act19_val[8]}" in
        0) vAUTO_ACT19_VAL[8]="Sun" ;;
        1) vAUTO_ACT19_VAL[8]="Mon" ;;
        2) vAUTO_ACT19_VAL[8]="Tue" ;;
        3) vAUTO_ACT19_VAL[8]="Wed" ;;
        4) vAUTO_ACT19_VAL[8]="Thu" ;;
        5) vAUTO_ACT19_VAL[8]="Fri" ;;
        6) vAUTO_ACT19_VAL[8]="Sat" ;;
        *) vAUTO_ACT19_VAL[8]="*" ;;
      esac
    esac
done
 
cat >>$PAGE1<<END
<FORM NAME="menu12sub" id="menu12sub_form" ACTION="auto_porc_pi.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<FONT SIZE="2"><B>Setting digital Output</B></FONT>
<BR>
1&nbsp;
<SELECT NAME="auto_act_con_0">
<OPTION VALUE="${auto_act_con[0]}" SELECTED>${vAUTO_ACT_CON[0]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act0_val_0">
<OPTION VALUE="${auto_act0_val[0]}" SELECTED>${vAUTO_ACT0_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low

<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT0_VAL[1]}" NAME="auto_act0_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT0_VAL[2]}" NAME="auto_act0_val_2">min
<SELECT NAME="auto_act0_val_3">
<OPTION VALUE="${vAUTO_ACT0_VAL[3]}" SELECTED>${vAUTO_ACT0_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT0_VAL[4]}" NAME="auto_act0_val_4">hour
<SELECT NAME="auto_act0_val_5">
<OPTION VALUE="${vAUTO_ACT0_VAL[5]}" SELECTED>${vAUTO_ACT0_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT0_VAL[6]}" NAME="auto_act0_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT0_VAL[7]}" NAME="auto_act0_val_7">Month&nbsp;
<SELECT NAME="auto_act0_val_8">
<OPTION VALUE="${auto_act0_val[8]}" SELECTED>${vAUTO_ACT0_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act0_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
2&nbsp;
<SELECT NAME="auto_act_con_1">
<OPTION VALUE="${auto_act_con[1]}" SELECTED>${vAUTO_ACT_CON[1]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act1_val_0">
<OPTION VALUE="${auto_act1_val[0]}" SELECTED>${vAUTO_ACT1_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low

<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT1_VAL[1]}" NAME="auto_act1_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT1_VAL[2]}" NAME="auto_act1_val_2">min
<SELECT NAME="auto_act1_val_3">
<OPTION VALUE="${vAUTO_ACT1_VAL[3]}" SELECTED>${vAUTO_ACT1_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT1_VAL[4]}" NAME="auto_act1_val_4">hour
<SELECT NAME="auto_act1_val_5">
<OPTION VALUE="${vAUTO_ACT1_VAL[5]}" SELECTED>${vAUTO_ACT1_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT1_VAL[6]}" NAME="auto_act1_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT1_VAL[7]}" NAME="auto_act1_val_7">Month&nbsp;
<SELECT NAME="auto_act1_val_8">
<OPTION VALUE="${auto_act1_val[8]}" SELECTED>${vAUTO_ACT1_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act1_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
3&nbsp;
<SELECT NAME="auto_act_con_2">
<OPTION VALUE="${auto_act_con[2]}" SELECTED>${vAUTO_ACT_CON[2]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act2_val_0">
<OPTION VALUE="${auto_act2_val[0]}" SELECTED>${vAUTO_ACT2_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low

<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT2_VAL[1]}" NAME="auto_act2_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT2_VAL[2]}" NAME="auto_act2_val_2">min
<SELECT NAME="auto_act2_val_3">
<OPTION VALUE="${vAUTO_ACT2_VAL[3]}" SELECTED>${vAUTO_ACT2_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT2_VAL[4]}" NAME="auto_act2_val_4">hour
<SELECT NAME="auto_act2_val_5">
<OPTION VALUE="${vAUTO_ACT2_VAL[5]}" SELECTED>${vAUTO_ACT2_VAL[5]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT2_VAL[6]}" NAME="auto_act2_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT2_VAL[7]}" NAME="auto_act2_val_7">Month&nbsp;
<SELECT NAME="auto_act2_val_8">
<OPTION VALUE="${auto_act2_val[8]}" SELECTED>${vAUTO_ACT2_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act2_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
4&nbsp;
<SELECT NAME="auto_act_con_3">
<OPTION VALUE="${auto_act_con[3]}" SELECTED>${vAUTO_ACT_CON[3]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act3_val_0">
<OPTION VALUE="${auto_act3_val[0]}" SELECTED>${vAUTO_ACT3_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT3_VAL[1]}" NAME="auto_act3_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT3_VAL[2]}" NAME="auto_act3_val_2">min
<SELECT NAME="auto_act3_val_3">
<OPTION VALUE="${vAUTO_ACT3_VAL[3]}" SELECTED>${vAUTO_ACT3_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT3_VAL[4]}" NAME="auto_act3_val_4">hour
<SELECT NAME="auto_act3_val_5">
<OPTION VALUE="${vAUTO_ACT3_VAL[5]}" SELECTED>${vAUTO_ACT3_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT3_VAL[6]}" NAME="auto_act3_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT3_VAL[7]}" NAME="auto_act3_val_7">Month&nbsp;
<SELECT NAME="auto_act3_val_8">
<OPTION VALUE="${auto_act3_val[8]}" SELECTED>${vAUTO_ACT3_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act3_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
5&nbsp;
<SELECT NAME="auto_act_con_4">
<OPTION VALUE="${auto_act_con[4]}" SELECTED>${vAUTO_ACT_CON[4]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act4_val_0">
<OPTION VALUE="${auto_act4_val[0]}" SELECTED>${vAUTO_ACT4_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT4_VAL[1]}" NAME="auto_act4_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT4_VAL[2]}" NAME="auto_act4_val_2">min
<SELECT NAME="auto_act4_val_3">
<OPTION VALUE="${vAUTO_ACT4_VAL[3]}" SELECTED>${vAUTO_ACT4_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT4_VAL[4]}" NAME="auto_act4_val_4">hour
<SELECT NAME="auto_act4_val_5">
<OPTION VALUE="${vAUTO_ACT4_VAL[5]}" SELECTED>${vAUTO_ACT4_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT4_VAL[6]}" NAME="auto_act4_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT4_VAL[7]}" NAME="auto_act4_val_7">Month&nbsp;
<SELECT NAME="auto_act4_val_8">
<OPTION VALUE="${auto_act4_val[8]}" SELECTED>${vAUTO_ACT4_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act4_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>&nbsp;
<BR>
6&nbsp;
<SELECT NAME="auto_act_con_5">
<OPTION VALUE="${auto_act_con[5]}" SELECTED>${vAUTO_ACT_CON[5]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act5_val_0">
<OPTION VALUE="${auto_act5_val[0]}" SELECTED>${vAUTO_ACT5_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT5_VAL[1]}" NAME="auto_act5_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT5_VAL[2]}" NAME="auto_act5_val_2">min
<SELECT NAME="auto_act5_val_3">
<OPTION VALUE="${vAUTO_ACT5_VAL[3]}" SELECTED>${vAUTO_ACT5_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT5_VAL[4]}" NAME="auto_act5_val_4">hour
<SELECT NAME="auto_act5_val_5">
<OPTION VALUE="${vAUTO_ACT5_VAL[5]}" SELECTED>${vAUTO_ACT5_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT5_VAL[6]}" NAME="auto_act5_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT5_VAL[7]}" NAME="auto_act5_val_7">Month&nbsp;
<SELECT NAME="auto_act5_val_8">
<OPTION VALUE="${auto_act5_val[8]}" SELECTED>${vAUTO_ACT5_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act5_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
7&nbsp;
<SELECT NAME="auto_act_con_6">
<OPTION VALUE="${auto_act_con[6]}" SELECTED>${vAUTO_ACT_CON[6]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act6_val_0">
<OPTION VALUE="${auto_act6_val[0]}" SELECTED>${vAUTO_ACT6_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT6_VAL[1]}" NAME="auto_act6_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT6_VAL[2]}" NAME="auto_act6_val_2">min
<SELECT NAME="auto_act6_val_3">
<OPTION VALUE="${vAUTO_ACT6_VAL[3]}" SELECTED>${vAUTO_ACT6_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT6_VAL[4]}" NAME="auto_act6_val_4">hour
<SELECT NAME="auto_act6_val_5">
<OPTION VALUE="${vAUTO_ACT6_VAL[5]}" SELECTED>${vAUTO_ACT6_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT6_VAL[6]}" NAME="auto_act6_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT6_VAL[7]}" NAME="auto_act6_val_7">Month&nbsp;
<SELECT NAME="auto_act6_val_8">
<OPTION VALUE="${auto_act6_val[8]}" SELECTED>${vAUTO_ACT6_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act6_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
8&nbsp;
<SELECT NAME="auto_act_con_7">
<OPTION VALUE="${auto_act_con[7]}" SELECTED>${vAUTO_ACT_CON[7]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act7_val_0">
<OPTION VALUE="${auto_act7_val[0]}" SELECTED>${vAUTO_ACT7_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT7_VAL[1]}" NAME="auto_act7_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT7_VAL[2]}" NAME="auto_act7_val_2">min
<SELECT NAME="auto_act7_val_3">
<OPTION VALUE="${vAUTO_ACT7_VAL[3]}" SELECTED>${vAUTO_ACT7_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT7_VAL[4]}" NAME="auto_act7_val_4">hour
<SELECT NAME="auto_act7_val_5">
<OPTION VALUE="${vAUTO_ACT7_VAL[5]}" SELECTED>${vAUTO_ACT7_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT7_VAL[6]}" NAME="auto_act7_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT7_VAL[7]}" NAME="auto_act7_val_7">Month&nbsp;
<SELECT NAME="auto_act7_val_8">
<OPTION VALUE="${auto_act7_val[8]}" SELECTED>${vAUTO_ACT7_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act7_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
9&nbsp;
<SELECT NAME="auto_act_con_8">
<OPTION VALUE="${auto_act_con[8]}" SELECTED>${vAUTO_ACT_CON[8]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act8_val_0">
<OPTION VALUE="${auto_act8_val[0]}" SELECTED>${vAUTO_ACT8_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT8_VAL[1]}" NAME="auto_act8_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT8_VAL[2]}" NAME="auto_act8_val_2">min
<SELECT NAME="auto_act8_val_3">
<OPTION VALUE="${vAUTO_ACT8_VAL[3]}" SELECTED>${vAUTO_ACT8_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT8_VAL[4]}" NAME="auto_act8_val_4">hour
<SELECT NAME="auto_act8_val_5">
<OPTION VALUE="${vAUTO_ACT8_VAL[5]}" SELECTED>${vAUTO_ACT8_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT8_VAL[6]}" NAME="auto_act8_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT8_VAL[7]}" NAME="auto_act8_val_7">Month&nbsp;
<SELECT NAME="auto_act8_val_8">
<OPTION VALUE="${auto_act8_val[8]}" SELECTED>${vAUTO_ACT8_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act8_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
10&nbsp;
<SELECT NAME="auto_act_con_9">
<OPTION VALUE="${auto_act_con[9]}" SELECTED>${vAUTO_ACT_CON[9]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act9_val_0">
<OPTION VALUE="${auto_act9_val[0]}" SELECTED>${vAUTO_ACT9_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT9_VAL[1]}" NAME="auto_act9_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT9_VAL[2]}" NAME="auto_act9_val_2">min
<SELECT NAME="auto_act9_val_3">
<OPTION VALUE="${vAUTO_ACT9_VAL[3]}" SELECTED>${vAUTO_ACT9_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT9_VAL[4]}" NAME="auto_act9_val_4">hour
<SELECT NAME="auto_act9_val_5">
<OPTION VALUE="${vAUTO_ACT9_VAL[5]}" SELECTED>${vAUTO_ACT9_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT9_VAL[6]}" NAME="auto_act9_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT9_VAL[7]}" NAME="auto_act9_val_7">Month&nbsp;
<SELECT NAME="auto_act9_val_8">
<OPTION VALUE="${auto_act9_val[8]}" SELECTED>${vAUTO_ACT9_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act9_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
11&nbsp;
<SELECT NAME="auto_act_con_10">
<OPTION VALUE="${auto_act_con[10]}" SELECTED>${vAUTO_ACT_CON[10]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act10_val_0">
<OPTION VALUE="${auto_act10_val[0]}" SELECTED>${vAUTO_ACT10_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT10_VAL[1]}" NAME="auto_act10_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT10_VAL[2]}" NAME="auto_act10_val_2">min
<SELECT NAME="auto_act10_val_3">
<OPTION VALUE="${vAUTO_ACT10_VAL[3]}" SELECTED>${vAUTO_ACT10_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT10_VAL[4]}" NAME="auto_act10_val_4">hour
<SELECT NAME="auto_act10_val_5">
<OPTION VALUE="${vAUTO_ACT10_VAL[5]}" SELECTED>${vAUTO_ACT10_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT10_VAL[6]}" NAME="auto_act10_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT10_VAL[7]}" NAME="auto_act10_val_7">Month&nbsp;
<SELECT NAME="auto_act10_val_8">
<OPTION VALUE="${auto_act10_val[8]}" SELECTED>${vAUTO_ACT10_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act10_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
12&nbsp;
<SELECT NAME="auto_act_con_11">
<OPTION VALUE="${auto_act_con[11]}" SELECTED>${vAUTO_ACT_CON[11]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act11_val_0">
<OPTION VALUE="${auto_act11_val[0]}" SELECTED>${vAUTO_ACT11_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT11_VAL[1]}" NAME="auto_act11_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT11_VAL[2]}" NAME="auto_act11_val_2">min
<SELECT NAME="auto_act11_val_3">
<OPTION VALUE="${vAUTO_ACT11_VAL[3]}" SELECTED>${vAUTO_ACT11_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT11_VAL[4]}" NAME="auto_act11_val_4">hour
<SELECT NAME="auto_act11_val_5">
<OPTION VALUE="${vAUTO_ACT11_VAL[5]}" SELECTED>${vAUTO_ACT11_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT11_VAL[6]}" NAME="auto_act11_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT11_VAL[7]}" NAME="auto_act11_val_7">Month&nbsp;
<SELECT NAME="auto_act11_val_8">
<OPTION VALUE="${auto_act11_val[8]}" SELECTED>${vAUTO_ACT11_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act11_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
13&nbsp;
<SELECT NAME="auto_act_con_12">
<OPTION VALUE="${auto_act_con[12]}" SELECTED>${vAUTO_ACT_CON[12]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act12_val_0">
<OPTION VALUE="${auto_act12_val[0]}" SELECTED>${vAUTO_ACT12_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT12_VAL[1]}" NAME="auto_act12_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT12_VAL[2]}" NAME="auto_act12_val_2">min
<SELECT NAME="auto_act12_val_3">
<OPTION VALUE="${vAUTO_ACT12_VAL[3]}" SELECTED>${vAUTO_ACT12_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT12_VAL[4]}" NAME="auto_act12_val_4">hour
<SELECT NAME="auto_act12_val_5">
<OPTION VALUE="${vAUTO_ACT12_VAL[5]}" SELECTED>${vAUTO_ACT12_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT12_VAL[6]}" NAME="auto_act12_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT12_VAL[7]}" NAME="auto_act12_val_7">Month&nbsp;
<SELECT NAME="auto_act12_val_8">
<OPTION VALUE="${auto_act12_val[8]}" SELECTED>${vAUTO_ACT12_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act12_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
14&nbsp;
<SELECT NAME="auto_act_con_13">
<OPTION VALUE="${auto_act_con[13]}" SELECTED>${vAUTO_ACT_CON[13]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act13_val_0">
<OPTION VALUE="${auto_act13_val[0]}" SELECTED>${vAUTO_ACT13_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT13_VAL[1]}" NAME="auto_act13_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT13_VAL[2]}" NAME="auto_act13_val_2">min
<SELECT NAME="auto_act13_val_3">
<OPTION VALUE="${vAUTO_ACT13_VAL[3]}" SELECTED>${vAUTO_ACT13_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT13_VAL[4]}" NAME="auto_act13_val_4">hour
<SELECT NAME="auto_act13_val_5">
<OPTION VALUE="${vAUTO_ACT13_VAL[5]}" SELECTED>${vAUTO_ACT13_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT13_VAL[6]}" NAME="auto_act13_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT13_VAL[7]}" NAME="auto_act13_val_7">Month&nbsp;
<SELECT NAME="auto_act13_val_8">
<OPTION VALUE="${auto_act13_val[8]}" SELECTED>${vAUTO_ACT13_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act13_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
15&nbsp;
<SELECT NAME="auto_act_con_14">
<OPTION VALUE="${auto_act_con[14]}" SELECTED>${vAUTO_ACT_CON[14]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act14_val_0">
<OPTION VALUE="${auto_act14_val[0]}" SELECTED>${vAUTO_ACT14_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT14_VAL[1]}" NAME="auto_act14_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT14_VAL[2]}" NAME="auto_act14_val_2">min
<SELECT NAME="auto_act14_val_3">
<OPTION VALUE="${vAUTO_ACT14_VAL[3]}" SELECTED>${vAUTO_ACT14_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT14_VAL[4]}" NAME="auto_act14_val_4">hour
<SELECT NAME="auto_act14_val_5">
<OPTION VALUE="${vAUTO_ACT14_VAL[5]}" SELECTED>${vAUTO_ACT14_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT14_VAL[6]}" NAME="auto_act14_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT14_VAL[7]}" NAME="auto_act14_val_7">Month&nbsp;
<SELECT NAME="auto_act14_val_8">
<OPTION VALUE="${auto_act14_val[8]}" SELECTED>${vAUTO_ACT14_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act14_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
16&nbsp;
<SELECT NAME="auto_act_con_15">
<OPTION VALUE="${auto_act_con[15]}" SELECTED>${vAUTO_ACT_CON[15]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act15_val_0">
<OPTION VALUE="${auto_act15_val[0]}" SELECTED>${vAUTO_ACT15_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT15_VAL[1]}" NAME="auto_act15_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT15_VAL[2]}" NAME="auto_act15_val_2">min
<SELECT NAME="auto_act15_val_3">
<OPTION VALUE="${vAUTO_ACT15_VAL[3]}" SELECTED>${vAUTO_ACT15_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT15_VAL[4]}" NAME="auto_act15_val_4">hour
<SELECT NAME="auto_act15_val_5">
<OPTION VALUE="${vAUTO_ACT15_VAL[5]}" SELECTED>${vAUTO_ACT15_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT15_VAL[6]}" NAME="auto_act15_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT15_VAL[7]}" NAME="auto_act15_val_7">Month&nbsp;
<SELECT NAME="auto_act15_val_8">
<OPTION VALUE="${auto_act15_val[8]}" SELECTED>${vAUTO_ACT15_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act15_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
17&nbsp;
<SELECT NAME="auto_act_con_16">
<OPTION VALUE="${auto_act_con[16]}" SELECTED>${vAUTO_ACT_CON[16]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act16_val_0">
<OPTION VALUE="${auto_act16_val[0]}" SELECTED>${vAUTO_ACT16_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT16_VAL[1]}" NAME="auto_act16_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT16_VAL[2]}" NAME="auto_act16_val_2">min
<SELECT NAME="auto_act16_val_3">
<OPTION VALUE="${vAUTO_ACT16_VAL[3]}" SELECTED>${vAUTO_ACT16_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT16_VAL[4]}" NAME="auto_act16_val_4">hour
<SELECT NAME="auto_act16_val_5">
<OPTION VALUE="${vAUTO_ACT16_VAL[5]}" SELECTED>${vAUTO_ACT16_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT16_VAL[6]}" NAME="auto_act16_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT16_VAL[7]}" NAME="auto_act16_val_7">Month&nbsp;
<SELECT NAME="auto_act16_val_8">
<OPTION VALUE="${auto_act16_val[8]}" SELECTED>${vAUTO_ACT16_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act16_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
18&nbsp;
<SELECT NAME="auto_act_con_17">
<OPTION VALUE="${auto_act_con[17]}" SELECTED>${vAUTO_ACT_CON[17]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act17_val_0">
<OPTION VALUE="${auto_act17_val[0]}" SELECTED>${vAUTO_ACT17_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT17_VAL[1]}" NAME="auto_act17_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT17_VAL[2]}" NAME="auto_act17_val_2">min
<SELECT NAME="auto_act17_val_3">
<OPTION VALUE="${vAUTO_ACT17_VAL[3]}" SELECTED>${vAUTO_ACT17_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT17_VAL[4]}" NAME="auto_act17_val_4">hour
<SELECT NAME="auto_act17_val_5">
<OPTION VALUE="${vAUTO_ACT17_VAL[5]}" SELECTED>${vAUTO_ACT17_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT17_VAL[6]}" NAME="auto_act17_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT17_VAL[7]}" NAME="auto_act17_val_7">Month&nbsp;
<SELECT NAME="auto_act17_val_8">
<OPTION VALUE="${auto_act17_val[8]}" SELECTED>${vAUTO_ACT17_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act17_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
19&nbsp;
<SELECT NAME="auto_act_con_18">
<OPTION VALUE="${auto_act_con[18]}" SELECTED>${vAUTO_ACT_CON[18]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act18_val_0">
<OPTION VALUE="${auto_act18_val[0]}" SELECTED>${vAUTO_ACT18_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT18_VAL[1]}" NAME="auto_act18_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT18_VAL[2]}" NAME="auto_act18_val_2">min
<SELECT NAME="auto_act18_val_3">
<OPTION VALUE="${vAUTO_ACT18_VAL[3]}" SELECTED>${vAUTO_ACT18_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT18_VAL[4]}" NAME="auto_act18_val_4">hour
<SELECT NAME="auto_act18_val_5">
<OPTION VALUE="${vAUTO_ACT18_VAL[5]}" SELECTED>${vAUTO_ACT18_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT18_VAL[6]}" NAME="auto_act18_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT18_VAL[7]}" NAME="auto_act18_val_7">Month&nbsp;
<SELECT NAME="auto_act18_val_8">
<OPTION VALUE="${auto_act18_val[8]}" SELECTED>${vAUTO_ACT18_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act18_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
20&nbsp;
<SELECT NAME="auto_act_con_19">
<OPTION VALUE="${auto_act_con[19]}" SELECTED>${vAUTO_ACT_CON[19]}
<OPTION VALUE="Disable">Disable
<OPTION VALUE="Enable">Enable
<OPTION VALUE="DI_ON_0">${ALIAS_DI[0]}high
<OPTION VALUE="DI_OFF_0">${ALIAS_DI[0]}low
<OPTION VALUE="DI_ON_1">${ALIAS_DI[1]}high
<OPTION VALUE="DI_OFF_1">${ALIAS_DI[1]}low
<OPTION VALUE="DI_ON_2">${ALIAS_DI[2]}high
<OPTION VALUE="DI_OFF_2">${ALIAS_DI[2]}low
<OPTION VALUE="DI_ON_3">${ALIAS_DI[3]}high
<OPTION VALUE="DI_OFF_3">${ALIAS_DI[3]}low
<OPTION VALUE="DI_ON_8">${ALIAS_DI[8]}high
<OPTION VALUE="DI_OFF_8">${ALIAS_DI[8]}low
<OPTION VALUE="DI_ON_9">${ALIAS_DI[9]}high
<OPTION VALUE="DI_OFF_9">${ALIAS_DI[9]}low
<OPTION VALUE="DI_ON_10">${ALIAS_DI[10]}high
<OPTION VALUE="DI_OFF_10">${ALIAS_DI[10]}low
<OPTION VALUE="DI_ON_12">${ALIAS_DI[12]}high
<OPTION VALUE="DI_OFF_12">${ALIAS_DI[12]}low
<OPTION VALUE="DI_ON_13">${ALIAS_DI[13]}high
<OPTION VALUE="DI_OFF_13">${ALIAS_DI[13]}low
<OPTION VALUE="DI_ON_14">${ALIAS_DI[14]}high
<OPTION VALUE="DI_OFF_14">${ALIAS_DI[14]}low
<OPTION VALUE="DI_ON_15">${ALIAS_DI[15]}high
<OPTION VALUE="DI_OFF_15">${ALIAS_DI[15]}low
<OPTION VALUE="DI_ON_16">${ALIAS_DI[16]}high
<OPTION VALUE="DI_OFF_16">${ALIAS_DI[16]}low
<OPTION VALUE="DI_ON_17">${ALIAS_DI[17]}high
<OPTION VALUE="DI_OFF_17">${ALIAS_DI[17]}low
<OPTION VALUE="DI_ON_18">${ALIAS_DI[18]}high
<OPTION VALUE="DI_OFF_18">${ALIAS_DI[18]}low
<OPTION VALUE="DI_ON_19">${ALIAS_DI[19]}high
<OPTION VALUE="DI_OFF_19">${ALIAS_DI[19]}low
<OPTION VALUE="DI_ON_20">${ALIAS_DI[20]}high
<OPTION VALUE="DI_OFF_20">${ALIAS_DI[20]}low
<OPTION VALUE="DI_ON_21">${ALIAS_DI[21]}high
<OPTION VALUE="DI_OFF_21">${ALIAS_DI[21]}low
<OPTION VALUE="DI_ON_22">${ALIAS_DI[22]}high
<OPTION VALUE="DI_OFF_22">${ALIAS_DI[22]}low
<OPTION VALUE="DI_ON_23">${ALIAS_DI[23]}high
<OPTION VALUE="DI_OFF_23">${ALIAS_DI[23]}low
</SELECT>&nbsp;
<SELECT NAME="auto_act19_val_0">
<OPTION VALUE="${auto_act19_val[0]}" SELECTED>${vAUTO_ACT19_VAL[0]}
<OPTION VALUE="DON_0">${ALIAS_DO[0]}high
<OPTION VALUE="DOFF_0">${ALIAS_DO[0]}low
<OPTION VALUE="DON_1">${ALIAS_DO[1]}high
<OPTION VALUE="DOFF_1">${ALIAS_DO[1]}low
<OPTION VALUE="DON_2">${ALIAS_DO[2]}high
<OPTION VALUE="DOFF_2">${ALIAS_DO[2]}low
<OPTION VALUE="DON_3">${ALIAS_DO[3]}high
<OPTION VALUE="DOFF_3">${ALIAS_DO[3]}low
<OPTION VALUE="IREXEC_0">${ALIAS_DO[8]}
<OPTION VALUE="IREXEC_1">${ALIAS_DO[9]}
<OPTION VALUE="IREXEC_2">${ALIAS_DO[10]}
<OPTION VALUE="IREXEC_3">${ALIAS_DO[11]}
<OPTION VALUE="IREXEC_4">${ALIAS_DO[12]}
<OPTION VALUE="IREXEC_5">${ALIAS_DO[13]}
<OPTION VALUE="TON_0">${ALIAS_DO[14]}high
<OPTION VALUE="TOFF_0">${ALIAS_DO[14]}low
<OPTION VALUE="TON_1">${ALIAS_DO[15]}high
<OPTION VALUE="TOFF_1">${ALIAS_DO[15]}low
<OPTION VALUE="TON_2">${ALIAS_DO[16]}high
<OPTION VALUE="TOFF_2">${ALIAS_DO[16]}low
<OPTION VALUE="SOUND_0">Sound_1
<OPTION VALUE="SOUND_1">Sound_2
<OPTION VALUE="SOUND_2">Sound_3
<OPTION VALUE="SOUND_3">Sound_4
<OPTION VALUE="SOUND_4">Sound_5
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:36px;text-align:right;" VALUE="${vAUTO_ACT19_VAL[1]}" NAME="auto_act19_val_1">ms&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT19_VAL[2]}" NAME="auto_act19_val_2">min
<SELECT NAME="auto_act19_val_3">
<OPTION VALUE="${vAUTO_ACT19_VAL[3]}" SELECTED>${vAUTO_ACT19_VAL[3]}
<OPTION VALUE="*">*
<OPTION VALUE="2">2
<OPTION VALUE="4">4
<OPTION VALUE="5">5
<OPTION VALUE="6">6
<OPTION VALUE="10">10
<OPTION VALUE="15">15
<OPTION VALUE="20">20
<OPTION VALUE="30">30
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:28px;text-align:center;" VALUE="${vAUTO_ACT19_VAL[4]}" NAME="auto_act19_val_4">hour
<SELECT NAME="auto_act19_val_5">
<OPTION VALUE="${vAUTO_ACT19_VAL[5]}" SELECTED>${vAUTO_ACT19_VAL[5]}
<OPTION VALUE="*">*<OPTION VALUE="2">2<OPTION VALUE="4">4
<OPTION VALUE="6">6
<OPTION VALUE="8">8
<OPTION VALUE="12">12
</SELECT>&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT19_VAL[6]}" NAME="auto_act19_val_6">Day&nbsp;
<INPUT TYPE="text" style="width:20px;text-align:center;" VALUE="${vAUTO_ACT19_VAL[7]}" NAME="auto_act19_val_7">Month&nbsp;
<SELECT NAME="auto_act19_val_8">
<OPTION VALUE="${auto_act19_val[8]}" SELECTED>${vAUTO_ACT19_VAL[8]}
<OPTION VALUE="*">*
<OPTION VALUE="0">Sun
<OPTION VALUE="1">Mon
<OPTION VALUE="2">Tue
<OPTION VALUE="3">Wed
<OPTION VALUE="4">Thu
<OPTION VALUE="5">Fri
<OPTION VALUE="6">Sat
</SELECT>
<SELECT NAME="auto_act19_val_9">
<OPTION VALUE="none" SELECTED>none
<OPTION VALUE="reg">Entry
<OPTION VALUE="del">Delete
</SELECT>
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu12sub_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
</DD>
</DL>
END

tSTARTUP=$DIR/.startup.s.tmp
[ -e $tSTARTUP ] && . $tSTARTUP
[ ! -z $vWEBPASSWORD ] && vWEBPASSWORD="*" 
cat >>$PAGE1<<END
<DL id="menu13dl">
<DT><FONT SIZE="+1"><B>Server configuration and save</B></FONT></DT>
<DD>
<FORM NAME="menu15" id="menu15_form" ACTION="server_date_set.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<SPAN id="menu15_server_date">
Date<INPUT TYPE="text" style="width:70px;text-align:left;" VALUE="" NAME="server_date" onClick="return menu15_ck()">
&nbsp;</SPAN>
<SPAN id="menu15_server_time">
Time<INPUT TYPE="text" style="width:60px;text-align:left;" VALUE="" NAME="server_time" onClick="return menu15_ck()">
</SPAN>
<INPUT style="text-align:center" TYPE="button" VALUE="Set" onClick="return menu15_ck('Set')" ;>
</FORM>
<HR>
<FORM NAME="menu13" id="menu13_form" ACTION="server_set.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
web user
<INPUT TYPE="text" size="22" style="width:110px;" VALUE="$vWEBUSER" NAME="server_val_0">
<BR>
web password
<INPUT TYPE="password" size="22" style="width:110px;" VALUE="$vWEBPASSWORD" NAME="server_val_1">
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu13_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</FORM>
<HR>
<FORM NAME="menu14" id="menu14_form" ACTION="exec_cmd.cgi" METHOD="get" onsubmit="this.disabled=true;" ENCTYPE="multipart/form-data">
<INPUT TYPE="radio" NAME="cmd" VALUE="poweroff" CHECKED>Stop&nbsp;
<INPUT TYPE="radio" NAME="cmd" VALUE="reboot">Restart
<INPUT TYPE="radio" NAME="cmd" VALUE="init">Initial setting&Poweoff
<BR>
<INPUT style="text-align:center" TYPE="button" VALUE="Run" onClick="return menu14_ck()" ;>
<INPUT style="text-align:center" TYPE="reset" VALUE="Clear">
</DD>
</DL>

<INPUT style="text-align:center" TYPE="button" VALUE="Update" onclick="clearTimeout(Update_di_Timer);location.href='./wait_for.cgi'">&nbsp;
<INPUT style="text-align:center" TYPE="button" VALUE="Logout" onclick="logout()" ;>
<TABLE ALIGN=RIGHT>
<TR><TD><FONT SIZE="-1">&copy;2019-2022 pepolinux.com&nbsp;
<span id="server_time" style="text-align:left"></span>&nbsp;
</TR>
</TABLE>
</DIV>
</BODY>
</HTML>
END
mv ${PAGE1} ${PAGE2}
echo -en '
var jump_url = setTimeout("jump_href()", 1000);
function jump_href() {
  var  jump_location = "/remote-hand/pi_int.html?" + (new Date().getTime());
  location.href=jump_location;
  clearTimeout(jump_url);
}
// -->
</script>
</HEAD>
<BODY BGCOLOR="#E0FFFF">
</BODY>
</HTML>'
if [ -e ${LOCKFILE} ];then
  [ $$ = `cat ${LOCKPID}` ] && rm ${LOCKFILE} && rm ${LOCKPID}
fi
