#!/bin/sh
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2024.2.10

echo -n '
<HTML>
<META http-equiv="Content-Type" content="text/html; charset=UTF-8">
<HEAD>
<META NAME="auther" content="yamauchi.isamu">
<META NAME="copyright" content="pepolinux.jpn.org">
<META http-equiv="Refresh" content="60;URL=/remote-hand/wait_for.cgi">
<META NAME="build" content="2024.2.10">
<META NAME="reply-to" content="izamu@pepolinux.jpn.org">
<TITLE>command of execution</TITLE>
<script type="text/javascript">
<!--
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
<TR ALIGN=CENTER class="blink"><TD>Privileged Command running</TD></TR>
</TABLE>
<BR>
<HR>
<TABLE ALIGN=RIGHT><TR><TD>&copy;2022-2025 pepolinux.jpn.org</TD><TR></TABLE>
</BODY>
</HTML>'
CMD=/www/remote-hand/tmp/exec_cmd.pepocmd
ACT=`echo $QUERY_STRING |mawk 'BEGIN{FS="&"};/poweroff/{print "poweroff"};/reboot/{print "reboot"};/init/{print "init"}'`
if [ $ACT = "init" ];then
cat>$CMD<<EOF
#!/bin/sh
[ -e /usr/src/pepolinux/back_up.tar.gz ] && rm -f /usr/src/pepolinux/back_up.tar.gz
[ -e /usr/src/pepolinux/startup.s ] && rm -f /usr/src/pepolinux/startup.s
(cd /www/remote-hand ; tar cfz ../remote_tmp.tar.gz ./ ; cd /www ; rm -rf t ; mkdir -p t ; cd t ; tar xfz ../remote_tmp.tar.gz ; rm -rf tmp .di_read_data.json .di_read_data.json.tmp pi_int.html dio_sh.tar.gz ; tar cfz /usr/src/pepolinux/remote_pi.tar.gz ./ ; echo > /root/.bash_history ; cd /usr/bin ; rm -f dio*)
rm /etc/network/interfaces ;touch /etc/network/interfaces
rm /etc/wpa_supplicant/wpa_supplicant.conf ;touch /etc/wpa_supplicant/wpa_supplicant.conf
rm /etc/exim4/passwd.client
rm /boot/iothouse_config.txt
rm /etc/rc.pepo/linenotify
echo -n >/home/pi/.bash_history
echo -n >/root/.bash_history
hostnamectl set-hostname iot000
cat>/etc/hosts<<END
:1              localhost ip6-localhost ip6-loopback
fe00::0         ip6-localnet
ff00::0         ip6-mcastprefix
ff02::1         ip6-allnodes
ff02::2         ip6-allrouters
127.0.1.1       iot000
END
sync
/sbin/poweroff
EOF
elif [ $ACT = "reboot" ];then
cat>$CMD<<EOF
#!/bin/sh
/sbin/reboot
EOF
elif [ $ACT = "poweroff" ];then
cat>$CMD<<EOF
#!/bin/sh
/sbin/poweroff
EOF
fi
