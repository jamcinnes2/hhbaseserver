# hhbaseserver
landslide sensor base server

You can build directly on rpi like this:
```
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -G "Unix Makefiles"
cmake --build . -- -j
```

Or cross compile like this where SYSROOT_DIR is the path to a suitable raspberrypi sysroot:
```
mkdir -p build && cd build
cmake .. \
  -DSYSROOT_DIR=/home/me/rpios_bookworm_sysroot \
  -DCMAKE_TOOLCHAIN_FILE=../aarch64-toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -G "Unix Makefiles"
cmake --build . -- -j
```

On the raspberrypi you need this in /boot/firmware/config.txt:
```
[all]
#this fixes "Device or resource busy" error with SPI pins
dtoverlay=spi0-0cs
```

These libraries and packages are needed to build:
Crow git
RadioLib git
apt install libasio-dev liblgpiod-dev libfmt-dev 
