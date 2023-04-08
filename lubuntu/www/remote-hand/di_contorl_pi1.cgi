#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2022.10.5
# di_control_pi1.cgi

PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/HTML; charset=UTF-8">
<META NAME="Auther" content="yamauchi.isamu">
<META NAME="Copyright" content="pepolinux.com">
<META NAME="Build" content="2022.10.5">
<META NAME="reply-to" content="izamu@pepolinux.com">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<TITLE>DI in the action setting for( digital-in)</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>DIO action-1 settings</TD></TR>
</TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.com</TD></TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
DICH=$DIR/.di_change1
tDICH=$DIR/.di_change1.tmp
sDICH=$DIR/.di_change1.set
DI_ALIAS=$DIR/.alias_di
MAIL_SET=$DIR/.mail_set.list
DO_WRITE_DATA=$DIR/.do_write_data
LOGCOUNT=1000
IRKITPOST=/usr/local/bin/pepoirkitpost
CONV=./conv_get.cgi
. $CONV
MAIL="di_sendmail"
[ -e "$MAIL_SET" ] && . $MAIL_SET
[ "$MAIL_SELECT" != "sendmail" ] && MAIL="di_wgetmail"

di_count() {
# di_count file_name
  local file count log
  file=$DIR/."$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  if [ ! -e $count ];then
    echo Reset `date '+%Y/%m/%d %T'` >$count ;  echo '#0' >>$count
    cat $count | head -n 1 >$log
  fi
}

di_clear() {
# di_clear file_name
  local file log count
  file=$DIR/"$1"
  bin=/usr/bin/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  echo Reset `date '+%Y/%m/%d %T'` >$count ;  echo '#0' >>$count
  cat $count | head -n 1 >$log
  if [ ! -e $bin ];then
    cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
fi
rm \$LOCK
EOF
    chmod +x $file
  fi
}

irkit_exec() {
# irkit_exec file_name ir_ch timer
  local file ir_num time log
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  ir_num=$2
  time=$3
cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
$IRKITPOST $ir_num $timer
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
fi
rm \$LOCK
EOF
  chmod +x $file
}

tocos_high_low() {
# tocos_high_low file_name do_ch high_low timer invert
  local file ch high_low time log invert
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  ch=$2
  high_low=$3
  time=$4
  invert=$5
  do_ch=$(($ch + 7))
  cmd=/usr/local/bin/pepotocoshelp
  cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
high_low=$high_low
if [ $invert != "none" ];then
  [ -e $DO_WRITE_DATA ] && . $DO_WRITE_DATA
  [ \${do[$do_ch]} -eq 0 ] && high_low="1" || high_low="0"
fi
$cmd $ch \$high_low $time
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
fi
rm \$LOCK
EOF
  chmod +x $file
}

do_high_low() {
# di_high_low file_name do_ch high_low timer invert
  local file ch high_low time log invert
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  ch=$2
  high_low=$3
  time=$4
  invert=$5
  cmd=/usr/local/bin/pepodioctl
  cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
high_low=$high_low
if [ $invert != "none" ];then
  [ -e $DO_WRITE_DATA ] && . $DO_WRITE_DATA
  [ \${do[$ch]} -eq 0 ] && high_low="1" || high_low="0"
fi
$cmd $ch \$high_low $time
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
fi
rm \$LOCK
EOF
  chmod +x $file
}

di_tel() {
# di_tel file_name tel_number tel_lock_file
  local file tel tel_file log
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  tel="$2"
  tel_file="$3"
  cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
echo $tel >$tel_file
if [ -e $count ];then
  LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
  if [ -e \$LOCK ];then
    exit
  else
    echo -en \$\$ >\$LOCK
  fi
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
fi
rm \$LOCK
EOF
  chmod +x $file
}

di_wgetmail() {
# di_mail file_name mail_address message1 message2 message3 message4
  local file mail msg log act msg_box
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  mail_to="$2"
  msg="$3"+"$4"
  act="$5"
  msg_box="$6"
  FFMPEGCTL=/usr/local/bin/pepomp4ctl
  cat >$file<<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
WGETMAIL=/usr/local/bin/peposendmail
if [ $act = "mail" ];then
  WGETMAIL=/usr/local/bin/peposendmail
  SUBJECT=$msg
elif [ $act = "mail_message" ];then
  WGETMAIL=/usr/local/bin/pepomsgsend
  SUBJECT=`echo -en $msg_box |awk '{gsub(/ /,"+",$0);printf $0}'`
elif [ $act = "web_camera_still" ];then
  IMAGE=remote_hand.jpg
  $FFMPEGCTL /dev/video0 \$IMAGE \$$
  if [ ! -z "$msg_box" ];then
    SUBJECT=`echo -en $msg_box |awk '{gsub(/ /,"+",$0);printf $0}'`
  else
    SUBJECT=$msg
  fi
elif [ $act = "web_camera_video" ];then
  IMAGE=remote_hand.mp4
  $FFMPEGCTL /dev/video0 \$IMAGE \$$
  if [ ! -z "$msg_box" ];then
    SUBJECT=`echo -en $msg_box |awk '{gsub(/ /,"+",$0);printf $0}'`
  else
    SUBJECT=$msg
  fi
elif [ $act = "mod_camera_still" ];then
  IMAGE=remote_hand.jpg
  $FFMPEGCTL /dev/vchiq \$IMAGE \$$
  if [ ! -z "$msg_box" ];then
    SUBJECT=`echo -en $msg_box |awk '{gsub(/ /,"+",$0);printf $0}'`
  else
    SUBJECT=$msg
  fi
elif [ $act = "mod_camera_video" ];then
  IMAGE=remote_hand.mp4
  \$FFMPEGCTL /dev/vchiq \$IMAGE \$$
  if [ ! -z "$msg_box" ];then
    SUBJECT=`echo -en $msg_box |awk '{gsub(/ /,"+",$0);printf $0}'`
  else
    SUBJECT=$msg
  fi
fi
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
  WTMP=$DIR/.dio_low_high.tmp.\$$
# .dio0high.count --> dio0high
  DIO=$count
  echo Count=\"\`cat \$DIO |awk '/^#[0-9]+/{N=\$1;gsub(/\#/,"",N);print N }'\`\" >\$WTMP
  echo Reset=\"\`cat \$DIO |grep -E "Reset " |awk '{gsub(/Reset /,"",\$0);print \$0}'\`\" >>\$WTMP
  echo Event=\"\`cat \$DIO |grep -E "Update "|awk '{gsub(/Update /,"",\$0);split(\$0,yy," ");split(yy[1],mm,"/");m=mm[2]"/"mm[3];print m,yy[2]}'\`\" >>\$WTMP
  MESSAGE=\`cat \$WTMP|awk '{gsub(/ /,"+",\$0);printf \$0"+++"}'\`
  rm \$WTMP
  unset WTMP
  \$WGETMAIL "$mail_to" \$SUBJECT \$MESSAGE \$IMAGE
fi
rm \$LOCK
EOF
  chmod +x $file
}

di_sendmail() {
# di_mail file_name mail_address message1 message2 message3 message4
  local file mail msg log
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  mail="$2"
  msg="$3""$4""$5"
  hostname=`hostname`
  cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
msg_file="$file".mailmsg
cat >\$msg_file<<END
To:"$mail"
Subject:$msg
END
nkf -j --overwrite \$msg_file
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
  cat $count >>\$msg_file
fi
/usr/sbin/sendmail -i \"$mail\" <\$msg_file
rm \$LOCK
EOF
  chmod +x $file
}

di_sound(){
#di_sound file_name sound_no timer
  local file ch time log
  di_count $1
  file=$DIR/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  ch=$2
  time=$3
  cmd=/usr/local/bin/peposound
  cat > $file <<EOF
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
$cmd $ch $time
if [ -e $count ];then
  cat $count |grep -E "Reset" >${count}.tmp
  echo Update \`date '+%Y/%m/%d %T'\` >>${count}.tmp
  cat $count |awk '/^\#[0-9]+/{N=\$1;gsub(/\#/,"",N); N++ ;print "#"N }' >>${count}.tmp
  mv ${count}.tmp $count
  cat $count | awk '{if(/Update/)printf \$0" ";if(/#/)print \$0}' >>$log
  cat $log | awk '{if(!/Reset/)print \$0}' |sort -r |head -n $LOGCOUNT >${log}.tmp
  cat $count | awk '{if(/Reset/)print \$0}' >>${log}.tmp
  mv ${log}.tmp $log
  chown www-data.www-data $count $log
fi
rm \$LOCK
EOF
  chmod +x $file
}

del_all() {
# delete command
  local file log
  di_count $1
#  file=$DIR/"$1"
  file=/usr/bin/"$1"
  count=$DIR/."$1".count
  log=$DIR/."$1".log
  CMD=$DIR/dio_control_del_$1.pepocmd
  cat >$CMD<<END
#!/bin/bash
LOCK=${DIR}/`echo $file |awk 'BEGIN{FS="/"};{print $NF}'`.lock
if [ -e \$LOCK ];then
  exit
else
  echo -en \$\$ >\$LOCK
fi
if [ -e $count ];then
  rm -f $file $count $log
fi
rm \$LOCK
END
}

rm -f "$tDICH" "$sDICH"
n=0
while [ $n -lt 22 ];do
  if [ "${di_change_reg[$n]}" != "none" ];then
    if [ -e "$DICH" ];then
      cat "$DICH" | grep -F -v [$n] >"$tDICH"
      mv "$tDICH" "$DICH"
    fi
    if [ "${di_change_reg[$n]}" = "reg" -o "${di_change_reg[$n]}" = "clr" ] ;then
      echo "di_change_reg[$n]=""${di_change_reg[$n]}" >>"$sDICH"
      echo "di_change[$n]=""${di_change[$n]}" >>"$DICH"
      echo "di_change[$n]=""${di_change[$n]}" >>"$sDICH"
      echo "di_act[$n]=""${di_act[$n]}" >>"$DICH"
      echo "di_act[$n]=""${di_act[$n]}" >>"$sDICH"
      if [ "${di_act[$n]}" = "phone" ];then
        echo "di_tel[$n]=""${di_tel[$n]}" >>"$DICH"
        echo "di_tel[$n]=""${di_tel[$n]}" >>"$sDICH"
      elif [ "${di_act[$n]}" = "mail_message" ];then
        echo "di_mail_message[$n]=""${di_mail_message[$n]}" >>"$DICH"
        echo "di_mail_message[$n]=""${di_mail_message[$n]}" >>"$sDICH"
        echo "di_mail[$n]="\"${di_mail[$n]}\" >>"$DICH"
        echo "di_mail[$n]="\"${di_mail[$n]}\" >>"$sDICH"
      elif [ "${di_act[$n]}" = "mail" -o "${di_act[$n]}" = "web_camera_still" -o "${di_act[$n]}" = "web_camera_video" -o "${di_act[$n]}" = "mod_camera_still" -o "${di_act[$n]}" = "mod_camera_video" ];then
        echo "di_mail_message[$n]=""${di_mail_message[$n]}" >>"$DICH"
        echo "di_mail_message[$n]=""${di_mail_message[$n]}" >>"$sDICH"
        echo "di_mail[$n]="\"${di_mail[$n]}\" >>"$DICH"
        echo "di_mail[$n]="\"${di_mail[$n]}\" >>"$sDICH"
      else
        echo "di_act_alt[$n]=""${di_act_alt[$n]}" >>"$DICH"
        echo "di_act_alt[$n]=""${di_act_alt[$n]}" >>"$sDICH"
        echo "don_time[$n]=""${don_time[$n]}" >>"$DICH"
        echo "don_time[$n]=""${don_time[$n]}" >>"$sDICH"
      fi
    elif [ "${di_change_reg[$n]}" = "del" ];then
      echo "di_change_reg[$n]=""${di_change_reg[$n]}" >>"$sDICH"
    fi
  fi
  n=$(($n + 1))
done

if [ -e "$sDICH" ];then
  chmod +x $sDICH
. $sDICH
  n=0
  while [ $n -lt 22 ]; do
    case "$n" in
      0 | 11)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio0high" || FIL="dio0low"
      ;;
      1 | 12)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio1high" || FIL="dio1low"
      ;;
      2 | 13)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio2high" || FIL="dio2low"
      ;;
      3 | 14)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio3high" || FIL="dio3low"
      ;;
      4 | 15)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio4high" || FIL="dio4low"
      ;;
      5 | 16)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio5high" || FIL="dio5low"
      ;;
      6 | 17)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio6high" || FIL="dio6low"
      ;;
      7 | 18)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio7high" || FIL="dio7low"
      ;;
      8 | 19)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio8high" || FIL="dio8low"
      ;;
      9 | 20)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio9high" || FIL="dio9low"
      ;;
      10 | 21)
        [ "${di_change[$n]}" = "low2high" ] && FIL="dio10high" || FIL="dio10low"
      ;;
    esac
    if [ "${di_change_reg[$n]}" = "reg" ];then
      case "${di_act[$n]}" in
        "DON_0")
          ARG1="0"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_0")
           ARG1="0"
           ARG2="0"
           ARG3="${don_time[$n]}"
           ARG4="${di_act_alt[$n]}"
           do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
         ;;
        "DON_1")
          ARG1="1"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_1")
          ARG1="1"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DON_2")
          ARG1="2"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_2")
          ARG1="2"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DON_3")
          ARG1="3"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_3")
          ARG1="3"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DON_4")
          ARG1="4"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_4")
          ARG1="4"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DON_5")
          ARG1="5"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_5")
          ARG1="5"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DON_6")
          ARG1="6"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_6")
          ARG1="6"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DON_7")
          ARG1="7"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "DOFF_7")
          ARG1="7"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          do_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "IREXEC_0")
          ARG1="0"
          ARG2="${don_time[$n]}"
          irkit_exec "$FIL" "$ARG1" "$ARG2"
        ;;
        "IREXEC_1")
          ARG1="1"
          ARG2="${don_time[$n]}"
          irkit_exec "$FIL" "$ARG1" "$ARG2"
        ;;
        "IREXEC_2")
          ARG1="2"
          ARG2="${don_time[$n]}"
          irkit_exec "$FIL" "$ARG1" "$ARG2"
        ;;
        "IREXEC_3")
          ARG1="3"
          ARG2="${don_time[$n]}"
          irkit_exec "$FIL" "$ARG1" "$ARG2"
        ;;
        "IREXEC_4")
          ARG1="4"
          ARG2="${don_time[$n]}"
          irkit_exec "$FIL" "$ARG1" "$ARG2"
        ;;
        "IREXEC_5")
          ARG1="5"
          ARG2="${don_time[$n]}"
          irkit_exec "$FIL" "$ARG1" "$ARG2"
        ;;
        "TON_0")
          ARG1="1"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          tocos_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "TOFF_0")
          ARG1="1"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          tocos_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "TON_1")
          ARG1="2"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          tocos_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "TOFF_1")
          ARG1="2"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          tocos_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "TON_2")
          ARG1="3"
          ARG2="1"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          tocos_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "TOFF_2")
          ARG1="3"
          ARG2="0"
          ARG3="${don_time[$n]}"
          ARG4="${di_act_alt[$n]}"
          tocos_high_low "$FIL" "$ARG1" "$ARG2" "$ARG3" "$ARG4"
        ;;
        "phone")
          don_time[$n]=""
          di_mail[$n]=""
          di_tel "$FIL" "${di_tel[$n]}" "$DIR/.di_tel$n.tel_lock"
        ;;
        "mail" | "mail_message" | "web_camera_still" |  "web_camera_video" | "mod_camera_still" | "mod_camera_video")
          don_time[$n]=""
          di_tel[$n]=""
          case "$n" in
            0 | 11) m=0 ;;
            1 | 12) m=1 ;;
            2 | 13) m=2 ;;
            3 | 14) m=3 ;;
            4 | 15) m=4 ;;
            5 | 16) m=5 ;;
            6 | 17) m=6 ;;
            7 | 18) m=7 ;;
            8 | 19) m=8 ;;
            9 | 20) m=9 ;;
           10 | 21) m=10 ;;
          esac
         if [ -e "$DI_ALIAS" ];then
            . $DI_ALIAS
            DI_NAME="${alias_di[$m]}"
          else
            for i in  0 1 2 3 4 5 6 7 8 9 10;do
              j=`expr $i + 1`
              alias_di[$i]='入力'"$j"
              echo "alias_di[$i]=""${alias_di[$i]}" >>"$DI_ALIAS"
            done
            DI_NAME="${alias_di[$m]}"
          fi
          if [ ${di_act[$n]} = "mail" ];then
            $MAIL "$FIL" "${di_mail[$n]}" "$DI_NAME" "$FIL" "mail"
          elif [ ${di_act[$n]} = "mail_message" ];then
            $MAIL "$FIL" "${di_mail[$n]}" "$DI_NAME" "$FIL" "mail_message" "${di_mail_message[$n]}"
          elif [ ${di_act[$n]} = "web_camera_still" ];then
            $MAIL "$FIL" "${di_mail[$n]}" "$DI_NAME" "$FIL" "web_camera_still" "${di_mail_message[$n]}"
          elif [ ${di_act[$n]} = "web_camera_video" ];then
            $MAIL "$FIL" "${di_mail[$n]}" "$DI_NAME" "$FIL" "web_camera_video" "${di_mail_message[$n]}"
          elif [ ${di_act[$n]} = "mod_camera_still" ];then
            $MAIL "$FIL" "${di_mail[$n]}" "$DI_NAME" "$FIL" "mod_camera_still" "${di_mail_message[$n]}"
          elif [ ${di_act[$n]} = "mod_camera_video" ];then
            $MAIL "$FIL" "${di_mail[$n]}" "$DI_NAME" "$FIL" "mod_camera_video" "${di_mail_message[$n]}"
          fi
         ;;
        "SOUND_0")
          ARG1="0"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_1")
          ARG1="1"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_2")
          ARG1="2"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_3")
          ARG1="3"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_4")
          ARG1="4"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_5")
          ARG1="5"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_6")
          ARG1="6"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_7")
          ARG1="7"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_8")
          ARG1="8"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
        "SOUND_9")
          ARG1="9"
          ARG2="${don_time[$n]}"
          di_sound "$FIL" "$ARG1" "$ARG2"
        ;;
      esac
    fi
    if [ "${di_change_reg[$n]}" = "del" ];then
      del_all "$FIL"
    fi
    if [ "${di_change_reg[$n]}" = "clr" ];then
      di_clear "$FIL"
    fi
    n=$(($n + 1))
  done
  CMD=$DIR/dio_control.pepocmd
  cat >$CMD<<END
#!/bin/bash
  CT=\`ls $DIR/|grep -E 'dio(0|1|2|3|4|5|6|7|8|9|10)[low|high]+$'\`
  if [ -n \`echo \$CT | wc -w\` ];then
    for DIO in \$CT ; do
      MVCMD=$DIR/\$DIO
      chown root.root \$MVCMD
      mv -f \$MVCMD /usr/bin/
    done
  fi
END
fi
echo -en '
</HTML>'
if [ $DI_TTY = "gpio" ];then
  ./pi_int_gpio.cgi
elif [ $DI_TTY = "piface" ];then
  ./pi_int.cgi
elif [ $DI_TTY = "cp2112" ];then
  ./pi_int_cp2112.cgi
fi
