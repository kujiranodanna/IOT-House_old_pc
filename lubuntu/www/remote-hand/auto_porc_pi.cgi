#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2022.4.26

echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.osdn.jp">
<META NAME="build" content="2022.4.26">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.osdn.jp">
<TITLE>Automatic process settings</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Digital output automatic processing</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.osdn.jp</TD><TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
ACT_DO=/usr/local/bin/pepodioctl
ACT_IRKIT=/usr/local/bin/pepoirkitpost
ACT_TOCOS=/usr/local/bin/pepotocoshelp
SOUND_DO=/usr/local/bin/peposound
CMD=$DIR/auto_act.pepocmd
AUTO_ACT_LIST=$DIR/.auto_act.list
tAUTO_ACT_LIST=$DIR/.auto_act.list.tmp
ttAUTO_ACT_LIST=$DIR/.auto_act.list.tmp.tmp
VAUTO_ACT_LIST=$DIR/.vauto_act.list
tVAUTO_ACT_LIST=$DIR/.vauto_act.list
DIRD=$DIR/.di_read_data
PING_CRON=$DIR/.ping_watch.cron
tPING_CRON=$DIR/.ping_watch.cron.tmp
QUERY=/www/remote-hand/tmp/.QUERY_STRING.cmd
CONV=./conv_get.cgi
. $CONV
auto_cron_reg() {
  TM="${WORK[1]}"
  case "${WORK[0]}" in
    DON_0)
      CH=0
      DO=1 ;;
    DOFF_0)
      CH=0
      DO=0 ;;
    DON_1)
      CH=1
      DO=1 ;;
    DOFF_1)
      CH=1
      DO=0 ;;
    DON_2)
      CH=2
      DO=1 ;;
    DOFF_2)
      CH=2
      DO=0 ;;
    DON_3)
      CH=3
      DO=1 ;;
    DOFF_3)
      CH=3
      DO=0 ;;
    DON_4)
      CH=4
      DO=1 ;;
    DOFF_4)
      CH=4
      DO=0 ;;
    DON_5)
      CH=5
      DO=1 ;;
    DOFF_5)
      CH=5
      DO=0 ;;
    DON_6)
      CH=6
      DO=1 ;;
    DOFF_6)
      CH=6
      DO=0 ;;
    DON_7)
      CH=7
      DO=1 ;;
    DOFF_7)
      CH=7
      DO=0 ;;
    IREXEC_0)
      CH=8 ;;
    IREXEC_1)
      CH=9 ;;
    IREXEC_2)
      CH=10 ;;
    IREXEC_3)
      CH=11 ;;
    IREXEC_4)
      CH=12 ;;
    IREXEC_5)
      CH=13 ;;
    "TON_0")
      CH=14
      DO=1 ;;
    "TOFF_0")
      CH=14
      DO=0 ;;
    "TON_1")
      CH=15
      DO=1 ;;
    "TOFF_1")
      CH=15
      DO=0 ;;
    "TON_2")
      CH=16
      DO=1 ;;
    "TOFF_2")
      CH=16
      DO=0 ;;
    "SOUND_0")
      CH=17
      DO=0 ;;
    "SOUND_1")
      CH=18
      DO=1 ;;
    "SOUND_2")
      CH=19
      DO=2 ;;
    "SOUND_3")
      CH=20
      DO=3 ;;
    "SOUND_4")
      CH=21
      DO=4 ;;
    "SOUND_5")
      CH=22
      DO=5 ;;
    "SOUND_6")
      CH=23
      DO=6 ;;
    "SOUND_7")
      CH=24
      DO=7 ;;
    "SOUND_8")
      CH=25
      DO=8 ;;
    "SOUND_9")
      CH=26
      DO=9 ;;
  esac
  YES_NO="ENABLE"
  DI_CH="-1"
  DI="-1"
  case "$CRON_COND" in
    "Disable")
      YES_NO="DISABLE" ;;
    "DI_ON_0")
      DI_CH=0
      DI=1 ;;
    "DI_OFF_0")
      DI_CH=0
      DI=0 ;;
    "DI_ON_1")
      DI_CH=1
      DI=1 ;;
    "DI_OFF_1")
      DI_CH=1
      DI=0 ;;
    "DI_ON_2")
      DI_CH=2
      DI=1 ;;
    "DI_OFF_2")
      DI_CH=2
      DI=0 ;;
    "DI_ON_3")
      DI_CH=3
      DI=1 ;;
    "DI_OFF_3")
      DI_CH=3
      DI=0 ;;
    "DI_ON_4")
      DI_CH=4
      DI=1 ;;
    "DI_OFF_4")
      DI_CH=4
      DI=0 ;;
    "DI_ON_5")
      DI_CH=5
      DI=1 ;;
    "DI_OFF_5")
      DI_CH=5
      DI=0 ;;
    "DI_ON_6")
      DI_CH=6
      DI=1 ;;
    "DI_OFF_6")
      DI_CH=6
      DI=0 ;;
    "DI_ON_7")
      DI_CH=7
      DI=1 ;;
    "DI_OFF_7")
      DI_CH=7
      DI=0 ;;
    "DI_ON_8")
      DI_CH=8
      DI=1 ;;
    "DI_OFF_8")
      DI_CH=8
      DI=0 ;;
    "DI_ON_9")
      DI_CH=9
      DI=1 ;;
    "DI_OFF_9")
      DI_CH=9
      DI=0 ;;
    "DI_ON_10")
      DI_CH=10
      DI=1 ;;
    "DI_OFF_10")
      DI_CH=10
      DI=0 ;;
    "DI_ON_11")
      DI_CH=11
      DI=1 ;;
    "DI_OFF_11")
      DI_CH=11
      DI=0 ;;
    "DI_ON_12")
      DI_CH=12
      DI=1 ;;
    "DI_OFF_12")
      DI_CH=12
      DI=0 ;;
    "DI_ON_13")
      DI_CH=13
      DI=1 ;;
    "DI_OFF_13")
      DI_CH=13
      DI=0 ;;
    "DI_ON_14")
      DI_CH=14
      DI=1 ;;
    "DI_OFF_14")
      DI_CH=14
      DI=0 ;;
    "DI_ON_15")
      DI_CH=15
      DI=1 ;;
    "DI_OFF_15")
      DI_CH=15
      DI=0 ;;
    "DI_ON_16")
      DI_CH=16
      DI=1 ;;
    "DI_OFF_16")
      DI_CH=16
      DI=0 ;;
    "DI_ON_17")
      DI_CH=17
      DI=1 ;;
    "DI_OFF_17")
      DI_CH=17
      DI=0 ;;
    "DI_ON_18")
      DI_CH=18
      DI=1 ;;
    "DI_OFF_18")
      DI_CH=18
      DI=0 ;;
    "DI_ON_19")
      DI_CH=19
      DI=1 ;;
    "DI_OFF_19")
      DI_CH=19
      DI=0 ;;
    "DI_ON_20")
      DI_CH=20
      DI=1 ;;
    "DI_OFF_20")
      DI_CH=20
      DI=0 ;;
    "DI_ON_21")
      DI_CH=21
      DI=1 ;;
    "DI_OFF_21")
      DI_CH=21
      DI=0 ;;
    "DI_ON_22")
      DI_CH=22
      DI=1 ;;
    "DI_OFF_22")
      DI_CH=22
      DI=0 ;;
    "DI_ON_23")
      DI_CH=23
      DI=1 ;;
    "DI_OFF_23")
      DI_CH=23
      DI=0 ;;
  esac
  CMD=$DIR/${CRON_NAME}.pepocmd
  DO_EXEC=$DIR/$CRON_NAME
  [ $CH -gt 7 -a $CH -lt 14 ] && J=$((CH - 8))
  [ $CH -gt 13 -a $CH -lt 17 ] && J=$((CH - 13))
  cat >$DO_EXEC<<END
#!/bin/bash
if [ $YES_NO = "DISABLE" ];then
  exit
elif [ "$DI" != -1 ];then
  [ -e $DIRD ] && . $DIRD
  [ -z "\${di[$DI_CH]}" ] && exit
  [ "\${di[$DI_CH]}" = "-1" ] && exit
  [ "\${di[$DI_CH]}" != "$DI" ] && exit
fi
if [ $CH -gt 7 -a $CH -lt 14 ];then
  $ACT_IRKIT $J $TM
elif [ $CH -gt 13 -a $CH -lt 17 ];then
  $ACT_TOCOS $J $DO $TM
elif [ $CH -gt 16 -a $CH -lt 27 ];then
  $SOUND_DO $DO $TM
elif [ $CH -lt 8 ];then
  $ACT_DO $CH $DO $TM
fi
END
  chmod +x $DO_EXEC
  if [ ! -z "${WORK[3]}" ] && [ "${WORK[3]}" != "*" ];then
    WORK[2]="${WORK[2]}/${WORK[3]}"
  fi
  if [ ! -z "${WORK[5]}" ] && [ "${WORK[5]}" != "*" ];then
    WORK[4]="${WORK[4]}/${WORK[5]}"
  fi
  if [ -e $PING_CRON ];then
    cat $PING_CRON | awk "! /($CRON_NAME)/{print}" > $tPING_CRON
    mv $tPING_CRON $PING_CRON
    echo "${WORK[2]}" "${WORK[4]}" "${WORK[6]}" "${WORK[7]}" "${WORK[8]}" "cp -f $DO_EXEC $CMD" >>"$PING_CRON"
  elif [ ! -e $PING_CRON ];then
    echo "${WORK[2]}" "${WORK[4]}" "${WORK[6]}" "${WORK[7]}" "${WORK[8]}" "cp -f $DO_EXEC $CMD">"$PING_CRON"
   fi
  crontab $PING_CRON
  if [ -e $AUTO_ACT_LIST ];then
    cat $AUTO_ACT_LIST | awk "! /($CRON_COND_NAME|$CRON_NAME)/{print}" > $tAUTO_ACT_LIST
    cat $QUERY | awk "/($CRON_COND_NAME|$CRON_NAME)/{print}" >> $tAUTO_ACT_LIST
    mv $tAUTO_ACT_LIST $AUTO_ACT_LIST
  elif [ ! -e $AUTO_ACT_LIST ];then
    cat $QUERY | awk "/($CRON_COND_NAME|$CRON_NAME)/{print}" > $AUTO_ACT_LIST
  fi
}

auto_cron_del() {
  DO_EXEC=$DIR/$CRON_NAME
  if [ -e $AUTO_ACT_LIST ];then
    cat $AUTO_ACT_LIST | awk "! /($CRON_COND_NAME|$CRON_NAME)/{print}" > $tAUTO_ACT_LIST
    mv $tAUTO_ACT_LIST $AUTO_ACT_LIST
    WLEN=`cat $AUTO_ACT_LIST | wc -l`
    [ $WLEN = 0 ] && rm -f $AUTO_ACT_LIST
  fi
  if [ -e $PING_CRON ];then
    cat $PING_CRON | awk "! /($CRON_NAME)/{print}" > $tPING_CRON
    mv $tPING_CRON $PING_CRON
    LEN=`cat $PING_CRON | wc -l`
    if [ $LEN != 0 ];then
      crontab $PING_CRON
    else
      crontab -r
      rm -f $PING_CRON
      rm -f $AUTO_ACT_LIST
    fi
    [ -e $DO_EXEC ] && rm -f $DO_EXEC
  fi
}

CRON_NAME="auto_act0"
CRON_COND_NAME="auto_act_con\\[0\\]"
if [ "${auto_act0_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act0_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[0]}"
  auto_cron_reg
elif [ "${auto_act0_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act1"
CRON_COND_NAME="auto_act_con\\[1\\]"
if [ "${auto_act1_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act1_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[1]}"
  auto_cron_reg
elif [ "${auto_act1_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act2"
CRON_COND_NAME="auto_act_con\\[2\\]"
if [ "${auto_act2_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act2_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[2]}"
  auto_cron_reg
elif [ "${auto_act2_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act3"
CRON_COND_NAME="auto_act_con\\[3\\]"
if [ "${auto_act3_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act3_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[3]}"
  auto_cron_reg
elif [ "${auto_act3_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act4"
CRON_COND_NAME="auto_act_con\\[4\\]"
if [ "${auto_act4_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act4_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[4]}"
  auto_cron_reg
elif [ "${auto_act4_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act5"
CRON_COND_NAME="auto_act_con\\[5\\]"
if [ "${auto_act5_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act5_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[5]}"
  auto_cron_reg
elif [ "${auto_act5_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act6"
CRON_COND_NAME="auto_act_con\\[6\\]"
if [ "${auto_act6_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act6_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[6]}"
  auto_cron_reg
elif [ "${auto_act6_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act7"
CRON_COND_NAME="auto_act_con\\[7\\]"
if [ "${auto_act7_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act7_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[7]}"
  auto_cron_reg
elif [ "${auto_act7_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act8"
CRON_COND_NAME="auto_act_con\\[8\\]"
if [ "${auto_act8_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act8_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[8]}"
  auto_cron_reg
elif [ "${auto_act8_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act9"
CRON_COND_NAME="auto_act_con\\[9\\]"
if [ "${auto_act9_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act9_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[9]}"
  auto_cron_reg
elif [ "${auto_act9_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act10"
CRON_COND_NAME="auto_act_con\\[10\\]"
if [ "${auto_act10_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act10_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[10]}"
  auto_cron_reg
elif [ "${auto_act10_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act11"
CRON_COND_NAME="auto_act_con\\[11\\]"
if [ "${auto_act11_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act11_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[11]}"
  auto_cron_reg
elif [ "${auto_act11_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act12"
CRON_COND_NAME="auto_act_con\\[12\\]"
if [ "${auto_act12_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act12_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[12]}"
  auto_cron_reg
elif [ "${auto_act12_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act13"
CRON_COND_NAME="auto_act_con\\[13\\]"
if [ "${auto_act13_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act13_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[13]}"
  auto_cron_reg
elif [ "${auto_act13_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act14"
CRON_COND_NAME="auto_act_con\\[14\\]"
if [ "${auto_act14_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act14_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[14]}"
  auto_cron_reg
elif [ "${auto_act14_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act15"
CRON_COND_NAME="auto_act_con\\[15\\]"
if [ "${auto_act15_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act15_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[15]}"
  auto_cron_reg
elif [ "${auto_act15_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act16"
CRON_COND_NAME="auto_act_con\\[16\\]"
if [ "${auto_act16_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act16_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[16]}"
  auto_cron_reg
elif [ "${auto_act16_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act17"
CRON_COND_NAME="auto_act_con\\[17\\]"
if [ "${auto_act17_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act17_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[17]}"
  auto_cron_reg
elif [ "${auto_act17_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act18"
CRON_COND_NAME="auto_act_con\\[18\\]"
if [ "${auto_act18_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act18_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[18]}"
  auto_cron_reg
elif [ "${auto_act18_val[9]}" = "del" ];then
  auto_cron_del
fi

CRON_NAME="auto_act19"
CRON_COND_NAME="auto_act_con\\[19\\]"
if [ "${auto_act19_val[9]}" = "reg" ];then
  for n in 0 1 2 3 4 5 6 7 8 9;do
    VAL="${auto_act19_val[$n]}"
    WORK[${n}]=$VAL
  done
  CRON_COND="${auto_act_con[19]}"
  auto_cron_reg
elif [ "${auto_act19_val[9]}" = "del" ];then
  auto_cron_del
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
