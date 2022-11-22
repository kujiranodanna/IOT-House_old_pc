#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2022.10.24

PATH=$PATH:/usr/local/bin
echo -en '
<HTML>
<HEAD>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.com">
<META NAME="build" content="2022.10.24">
<META http-equiv="Refresh" content="30;URL=/remote-hand/wait_for.cgi">
<META NAME="reply-to" content="izamu@pepolinux.com">
<TITLE>Upload Sound File settings</TITLE>
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
<TR ALIGN=CENTER class="blink"><TD>Processing Upload Sound File settings</TD></TR></TABLE>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2021-2025 pepolinux.com</TD><TR></TABLE>
</BODY>'

DIR=/www/remote-hand/tmp
FILE_NAME=$DIR/.sound_file_name
tFILE_NAME=$DIR/.sound_file_name_tmp
SOUND_FILE=$DIR/.sound_file
tSOUND_FILE=$DIR/.sound_file_tmp
ttSOUND_FILE=${tSOUND_FILE}.tmp
CMD=$DIR/sound_upload.pepocmd
if [ -e $tSOUND_FILE ];then
  cat >$ttSOUND_FILE
  rm $ttSOUND_FILE
  echo -en '
  </HTML>'
  exit
fi
cat >$tSOUND_FILE
cat $tSOUND_FILE | sed -n 6,6p|awk '{gsub("\r","",$0);gsub(";","",$0);printf("%s\n%s\n",$3,$4)}' >$tFILE_NAME
. $tFILE_NAME
SIZE=`cat $tSOUND_FILE | awk 'NR == 4{gsub("\r","",$0);printf $1}'`
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
M4A_YES_NO=`echo $filename |awk 'BEGIN{TMP="NO"};/m4a$/{TMP="YES"};END{printf TMP}'`
WAV_YES_NO=`echo $filename |awk 'BEGIN{TMP="NO"};/wav$/{TMP="YES"};END{printf TMP}'`
tmpFILENAME=$filename
convertFILE="NO"
if [ $M4A_YES_NO = "YES" ];then
  tmpFILENAME=`echo $filename |awk '{sub("m4a","mp3",$0);printf $0}'`
  convertFILE="YES"
fi
if [ $WAV_YES_NO = "YES" ];then
  tmpFILENAME=`echo $filename |awk '{sub("wav","mp3",$0);printf $0}'`
  convertFILE="YES"
fi
if [ -e $FILE_NAME ];then
  . $FILE_NAME
  [ -e $DIR/${sound_file[$n]} ] && rm -f $DIR/${sound_file[$n]}
  cat $FILE_NAME |grep -F -v $tmp >$tFILE_NAME
  echo "$tmp"="$tmpFILENAME" >> $tFILE_NAME
  mv $tFILE_NAME $FILE_NAME
else
  echo "$tmp"="$tmpFILENAME" > $FILE_NAME
fi
inputFILE=${DIR}/$filename
outputFILE=${DIR}/$tmpFILENAME
echo -en '
</HTML>'
cat>$CMD<<EOF
#!/bin/bash
cat $tSOUND_FILE | sed '1,8d' >$SOUND_FILE
dd if=$SOUND_FILE of=$inputFILE bs=1 count=$SIZE
if [ $convertFILE = "YES" ];then
  ffmpeg -i $inputFILE -ab 64k -y $outputFILE >/dev/null 2>&1
  rm -f $inputFILE
fi
rm -f $tSOUND_FILE $tFILE_NAME $SOUND_FILE
EOF
