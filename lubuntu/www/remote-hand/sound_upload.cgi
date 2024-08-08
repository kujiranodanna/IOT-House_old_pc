#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi ,2023.11.10 update 2024.2.14

PATH=$PATH:/usr/local/bin
echo -n '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META NAME="build" content="2024.2.10">
<META http-equiv="Refresh" content="2;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>Upload Sound File settings</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Processing Upload Sound File settings</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2023-2026 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
prog=sound_upload
FILE_NAME=$DIR/.sound_file_name
tFILE_NAME=$DIR/.${prog}.$$
SOUND_FILE=$DIR/.${prog}_file.$$
tSOUND_FILE=$DIR/.${prog}_file_tmp.$$
CMD=$DIR/${prog}_$$.pepocmd
MAXFILESIZE=$((2048 * 1024))
error(){
  [ -e ${tFILE_NAME} ] && rm ${tFILE_NAME}
  [ -e ${SOUND_FILE} ] && rm ${SOUND_FILE}
  [ -e ${tSOUND_FILE} ] && rm ${tSOUND_FILE}
  exit 0
}
trap error TERM HUP KILL INT QUIT
cat >$tSOUND_FILE
if [ -e $tSOUND_FILE ];then
  dd if=$tSOUND_FILE bs=256 count=1 |mawk '/^[a-z]/{gsub("\r","",$0);print}' >$tFILE_NAME
  . $tFILE_NAME  
  if [ ! -z $size ];then
    if [ $size -gt $MAXFILESIZE ];then
      [ -e $tFILE_NAME ] && rm $tFILE_NAME
      [ -e $SOUND_FILE ] && rm $SOUND_FILE
      [ -e $tSOUND_FILE ] && rm $tSOUND_FILE
      echo -n '</HTML>'
      exit
    fi
  fi
  cat $tSOUND_FILE | sed -n 6,6p|mawk '{gsub("\r","",$0);gsub(";","",$0);printf("%s\n%s\n",$3,$4)}' >$tFILE_NAME
  . $tFILE_NAME
  SIZE=`cat $tSOUND_FILE | mawk 'NR == 4{gsub("\r","",$0);printf $1}'`
else
  exit
fi
tSIZE=$(wc -c $tSOUND_FILE | mawk '{print $1}')
while [ $tSIZE -lt $SIZE ];do
  msleep 1000
  cat >> $tSOUND_FILE
  tSIZE=$(wc -c $tSOUND_FILE | mawk '{print $1}')
done
case $name in
  "sound_file_0")
     tmp="sound_file[0]"
     n=0
  ;;
  "sound_file_1")
     tmp="sound_file[1]"
     n=1
  ;;
  "sound_file_2")
     tmp="sound_file[2]"
     n=2
  ;;
  "sound_file_3")
     tmp="sound_file[3]"
     n=3
  ;;
  "sound_file_4")
     tmp="sound_file[4]"
     n=4
  ;;
  "sound_file_5")
     tmp="sound_file[5]"
     n=5
  ;;
  "sound_file_6")
     tmp="sound_file[6]"
     n=6
  ;;
  "sound_file_7")
     tmp="sound_file[7]"
     n=7
  ;;
  "sound_file_8")
     tmp="sound_file[8]"
     n=8
  ;;
  "sound_file_9")
     tmp="sound_file[9]"
     n=9
  ;;
esac
MP3_YES_NO=$(echo $filename |mawk 'BEGIN{TMP="NO"};/mp3$/{TMP="YES"};END{printf TMP}')
WAV_YES_NO=$(echo $filename |mawk 'BEGIN{TMP="NO"};/wav$/{TMP="YES"};END{printf TMP}')
if [ $MP3_YES_NO = "NO" -a $WAV_YES_NO = "NO" ];then
  tmpFILENAME=`echo $filename |mawk -F "." '{printf("%s",$1".mp3")}'`
  CONVERT_YES_NO="YES"
else
  tmpFILENAME=$filename
  CONVERT_YES_NO="NO"
fi
if [ -e $FILE_NAME ];then
  . $FILE_NAME
  if [ $(echo -n ${sound_file[$n]} | wc -c) -ne 0 ];then
    [ -e $DIR/${sound_file[$n]} ] && rm $DIR/${sound_file[$n]}
  fi
  cat $FILE_NAME |grep -F -v $tmp >$tFILE_NAME
  echo "$tmp"="$tmpFILENAME" >> $tFILE_NAME
  mv $tFILE_NAME $FILE_NAME
else
  echo "$tmp"="$tmpFILENAME" > $FILE_NAME
fi
inputFILE=${DIR}/$filename
outputFILE=${DIR}/$tmpFILENAME
echo -n '
</HTML>'
cat>$CMD<<EOF
#!/bin/sh
cat $tSOUND_FILE | sed '1,8d' >$SOUND_FILE
dd if=$SOUND_FILE of=$inputFILE bs=$SIZE count=1
if [ $CONVERT_YES_NO = "YES" ];then
  ffmpeg -i $inputFILE -ab 64k -y $outputFILE >/dev/null 2>&1
  [ -e $inputFILE ] && rm $inputFILE
fi
[ -e $tSOUND_FILE ] && rm $tSOUND_FILE
[ -e $tFILE_NAME ] && rm $tFILE_NAME
[ -e $SOUND_FILE ] && rm $SOUND_FILE
EOF
