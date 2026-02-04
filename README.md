# hhbaseserver
landslide sensor base server

You need this in rpi /boot/firmware/config.txt:
[all]
#JohnM this fixes "Device or resource busy" error with SPI pins
dtoverlay=spi0-0cs

These libraries and packages are needed to build:
Crow git
RadioLib git
apt install libasio-dev liblgpiod-dev libfmt-dev 
