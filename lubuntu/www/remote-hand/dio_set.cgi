#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2019.12.21

PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2019.12.21">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>DIO settings</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Processing DIO settings</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2020-2022 pepolinux.com</TD><TR></TABLE>
</BODY>'

CONV=./conv_get.cgi
. $CONV
DIR=/www/remote-hand/tmp
DOWD=$DIR/.do_write_data
tDOWD=$DIR/.do_write_data.tmp
sDOWD=$DIR/.do_write_data.set
ALIAS_DI=$DIR/.alias_di
ALIAS_DO=$DIR/.alias_do
ALIAS_VDO=$DIR/.alias_vdo
tALIAS_VDO=$DIR/.alias_vdo.tmp
tALIAS_DI=$DIR/.alias_di.tmp
tALIAS_DO=$DIR/.alias_do.tmp
MODEM_DEV=$DIR/.modem
TCOSRD=$DIR/.tocos_read_data
TEMPERFILE=$DIR/temperature
DIORD=$DIR/.di_read_data
SCANTIME=300
rm -f $tALIAS_DI $tALIAS_DO $tDOWD $sDOWD
if [ -e "$ALIAS_DI" ];then
  cat "$ALIAS_DI"|grep -v "DI_TTY" >${tALIAS_DI}
  echo "DI_TTY="${DI_TTY} >>${tALIAS_DI}
  mv "$tALIAS_DI" "$ALIAS_DI"  
else
  echo "DI_TTY="${DI_TTY} >${ALIAS_DI}
fi
if [ -e "$ALIAS_DI" ];then
  cat "$ALIAS_DI"|grep -v "TOCOS_TTY" >${tALIAS_DI}
  echo "TOCOS_TTY"=${TOCOS_TTY} >>${tALIAS_DI}
  mv "$tALIAS_DI" "$ALIAS_DI"  
else
  echo "TOCOS_TTY"=$TOCOS_TTY >>${tALIAS_DI}
fi
if [ ! -z "$tocos_ip" ];then
  if [ -e "$ALIAS_DI" ];then
    cat "$ALIAS_DI"|grep -v "tocos_ip" >${tALIAS_DI}
    echo "tocos_ip"=${tocos_ip} >>${tALIAS_DI}
    mv "$tALIAS_DI" "$ALIAS_DI"
  else
    echo "tocos_ip"=${tocos_ip} >>${tALIAS_DI}
  fi
else
  if [ -e "$ALIAS_DI" ];then
    cat "$ALIAS_DI"|grep -v "tocos_ip" >${tALIAS_DI}
    echo "tocos_ip"="" >>${tALIAS_DI}
    mv "$tALIAS_DI" "$ALIAS_DI"  
  else
    echo "tocos_ip"="" >>${tALIAS_DI}
  fi
fi
if [ ! -z "$piface_ip" ];then
  if [ -e "$ALIAS_DI" ];then
    cat "$ALIAS_DI"|grep -v "piface_ip" >${tALIAS_DI}
    echo "piface_ip"=${piface_ip} >>${tALIAS_DI}
    mv "$tALIAS_DI" "$ALIAS_DI"  
  else
    echo "piface_ip"=${piface_ip} >>${tALIAS_DI}
  fi
else
  if [ -e "$ALIAS_DI" ];then
    cat "$ALIAS_DI"|grep -v "piface_ip" >${tALIAS_DI}
    echo "piface_ip"="" >>${tALIAS_DI}
    mv "$tALIAS_DI" "$ALIAS_DI"  
  else
    echo "piface_ip"="" >>${tALIAS_DI}
  fi
fi

for n in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16; do
  if [ "${do[$n]}" != "none" ];then
    if [ -e "$DOWD" ];then
      cat "$DOWD" | grep -F -v [$n] > "$tDOWD"
      mv "$tDOWD" "$DOWD"
    fi
    echo "do[$n]"="${do[$n]}" >> "$DOWD"
    echo don_time[$n]="${don_time[$n]}" >> "$DOWD"
    echo "do[$n]"="${do[$n]}" >> "$sDOWD"
    echo don_time[$n]="${don_time[$n]}" >> "$sDOWD"
  fi
  if [ "${alias_do_reg[$n]}" != "none" ];then
    if [ -e "$ALIAS_DO" ];then
      cat "$ALIAS_DO" | grep -F -v [$n] > "$tALIAS_DO"
      mv "$tALIAS_DO" "$ALIAS_DO"
    fi
    echo "alias_do[$n]=""${alias_do[$n]}" >>"$ALIAS_DO"
  fi
  if [ "${alias_do_reg[$n]}" == "del" ];then
    if [ -e "$ALIAS_DO" ];then
      cat "$ALIAS_DO" | grep -F -v [$n] > "$tALIAS_DO"
      mv "$tALIAS_DO" "$ALIAS_DO"
    fi
  fi
  unset do[$n]
done
for n in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25; do
  if [ "${alias_di_reg[$n]}" != "none" ];then
    if [ -e "$ALIAS_DI" ];then
      cat "$ALIAS_DI" | grep -F -v [$n] > "$tALIAS_DI"
      mv "$tALIAS_DI" "$ALIAS_DI"
    fi
    echo "alias_di[$n]=""${alias_di[$n]}" >> "$ALIAS_DI"
    echo "slice_ai[$n]=""${slice_ai[$n]}" >> "$ALIAS_DI"
  fi
  if [ "${alias_di_reg[$n]}" == "del" ];then
    if [ -e "$ALIAS_DI" ];then
      cat "$ALIAS_DI" | grep -F -v [$n] > "$tALIAS_DI"
      mv "$tALIAS_DI" "$ALIAS_DI"
    fi
  fi
done
mv "$tALIAS_DI" "$ALIAS_DI"
n=0
while [ $n -lt 34 ];do
  case $n in
    0 | 1) m=0;;
    2 | 3) m=1;;
    4 | 5) m=2;;
    6 | 7) m=3;;
    8 | 9) m=4;;
    10 | 11) m=5;;
    12 | 13) m=6;;
    14 | 15) m=7;;
    16 | 17) m=8;;
    18 | 19) m=9;;
    20 | 21) m=10;;
    22 | 23) m=11;;
    24 | 25) m=12;;
    26 | 27) m=13;;
    28 | 29) m=14;;
    30 | 31) m=15;;
    32 | 33) m=16;;
  esac
  if [ "${alias_do_reg[$m]}" != "none" ];then
    if [ -e "$ALIAS_VDO" ];then
      cat cat "$ALIAS_VDO" | grep -F -v [$n] > "$tALIAS_VDO"
      mv "$tALIAS_VDO" "$ALIAS_VDO"
    fi
    echo "alias_vdo[$n]=""${alias_vdo[$n]}" >> "$ALIAS_VDO"
  fi
  if [ "${alias_do_reg[$m]}" == "del" ];then
    if [ -e "$ALIAS_VDO" ];then
      cat cat "$ALIAS_VDO" | grep -F -v [$n] > "$tALIAS_VDO"
      mv "$tALIAS_VDO" "$ALIAS_VDO"
    fi
  fi
  n=$(($n + 1))
done
if [ "$TOCOS_TTY" = "none" -a ! -z "$tocos_ip" ];then
  msleep $SCANTIME
  CMD=$DIR/dio_set_tocos.pepocmd
  cat >$CMD<<END
#!/bin/bash
rm -f $TCOSRD
rm -f $TEMPERFILE
rm -f /www/remote-hand/pepotocoshelp
ln -s /usr/local/bin/pepoipdio /www/remote-hand/pepotocoshelp
END
else
  msleep $SCANTIME
  CMD=$DIR/dio_set_tocos.pepocmd
  cat >$CMD<<END
#!/bin/bash
rm -f $TCOSRD
rm -f $TEMPERFILE
rm -f /www/remote-hand/pepotocoshelp
ln -s /usr/local/bin/pepotocoshelp_local /www/remote-hand/pepotocoshelp
END
fi
if [ "$DI_TTY" = "none" -a ! -z "$piface_ip" ];then
  msleep $SCANTIME
  CMD=$DIR/dio_set_piface.pepocmd
  cat >$CMD<<END
#!/bin/bash
rm -f $DIORD
rm -f $TEMPERFILE
rm -f /www/remote-hand/pepopiface
ln -s /usr/local/bin/pepoipdio /www/remote-hand/pepopiface
END
elif [ "$DI_TTY" = "gpio" ];then
  msleep $SCANTIME
  CMD=$DIR/dio_set_piface.pepocmd
  cat >$CMD<<END
#!/bin/bash
rm -f $DIORD
rm -f $TEMPERFILE
rm -f /www/remote-hand/pepopiface
ln -s /usr/local/bin/pepogpiohelp /www/remote-hand/pepopiface
END
elif [ "$DI_TTY" = "piface" ];then
  msleep $SCANTIME
  CMD=$DIR/dio_set_piface.pepocmd
  cat >$CMD<<END
#!/bin/bash
rm -f $DIORD
rm -f $TEMPERFILE
rm -f /www/remote-hand/pepopiface
ln -s /usr/local/bin/pepopiface_local /www/remote-hand/pepopiface
END
fi
cat >$MODEM_DEV<<END
modem_dev=$modem
END
echo -en '
</HTML>'
