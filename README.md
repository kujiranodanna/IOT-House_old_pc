# IOT-House_old_pc
### Revive abandoned PCs at IOT House
- News 2021.11.23 ![IOT-House_docker]() release
- docker run -itd --privileged --name iot-house_docker -p 8022:22 -p 80:80 -p 443:443 kujiranodanna/iot-house_docker:v0.01

Give your old laptop another chance to play an active role!
![IOT-House_old_pc](https://user-images.githubusercontent.com/70492305/115954863-33bc7700-a52e-11eb-9c52-d42f607de5d1.jpeg)

### It has the following features.
1. [Sunhayato's MM-CP2122A](https://www.sunhayato.co.jp/material2/ett09/item_1083) is used for gpio to revive an old PC with IOT-House.
2. IOT-House of raspberry pi is arranged according to lubuntu.
3. IOT-House is a server, so it doesn't require much power on your PC.
4. Use the environmental gas sensor BME680 connected to I2C of cp2112
5. It is assumed that the operating environment is an old notebook PC with a CPU of 1GHz and a memory of about 1GB.
6. Compared to the raspberry pi zero w, older notebook PCs consume more power, but I think they have outstanding stability, such as speed and UPS standard installation.
### INSTALL
1. Please prepare the flash memory of the following capacity

```
fdisk /dev/sdb

Welcome to fdisk (util-linux 2.29.2).
Changes will remain in memory only, until you decide to write them.
Be careful before using the write command.

Command (m for help): p
Disk /dev/mmcblk0: 7.3 GiB, 7820279808 bytes, 15273984 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x15ae16d5
```
2. Download the binaries from here
https://osdn.net/projects/pepolinux/releases/75081

3. Extract the downloaded IOT-House_old_pc_xx.7z and write IOT-House_old_pc_xx.img to the flash memory
4. Write IOT-House_old_pc_xx.img to flash memory with the dd command,For example
- dd if=IOT-House_old_pc_xx.img of=/dev/sdb bs=100M count=80
5. Attach it to the USB of the flash memory notebook PC and start it by turning on the power.
- It may take about 5 minutes to start.
6. When the login screen appears, user: remote, password: hand ,Log in with.
7. Please set the network properly,
- the initial state is wired LAN, the host name is iot000.
8. From another PC http://iot000.local etc. user:remote, password: hand ,Log in with.
Control panel below
![sshot_iot-house_old_pc](https://user-images.githubusercontent.com/70492305/117527575-08df2200-b008-11eb-990a-0898cda7eed1.png)
![sshot_iot-house_old_pc-2](https://user-images.githubusercontent.com/70492305/117527767-6758d000-b009-11eb-8510-1410da3beade.png)
