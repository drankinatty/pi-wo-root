# pi-wo-root
**Raspberry Pi Peripheral Access Wtihout root**

The Pi is a wonderful development platform, but using the Broadcom BCM2835 library for peripheral access has one major drawback -- you must run all executables as `root` due to its mmapping of `"/dev/mem"`. Not only does that pose security and access issues if your application is to be run by normal users, the direct memory access through `/dev/mem` has long been a route for any user to directly compromise the system, either intentionally, or through a careless bit-shift, index or write. Simply search *"Why is direct access through `/dev/mem` a bad idea?"* and you will find a wealth of reasons.

If the `/dev/mem` access were not bad enough, many of the older tools used with the Pi are now deprecated and removed from the current Raspberry Pi OS (WiringPi, etc..). Given the state of things we started with a goal of not requiring root access and to use to Linux kernel peripheral interfaces to access I2C, SPI, PWM, GPIO, etc...

The result? A solid set of libraries to provide peripheral access using the kernel `ioctl()` system calls or direct devfs interface. The BCM2835 library is still provided if you want to compare/contrast, but not used in the other libraries.

## Layout of Files in this Repository

**Note:** Every Library or Example has a complete `Makefile` in its directory allowing you to simply type `make` to compile the code on your Pi (or any Linux box with the peripherals enabled in the kernel). All are written in C. (the oled interface, originally written by Gavin Lyons, is still written in C++ and was not rewritten in C when it was ported to use Linux `ioctl()` for hardware access.

There are three directories, `examples`, `lib` and `tst`. The `examples` directory holds examples using the various libraries that are in a complete enough state to simply `make` and then run (after configuring your hardware of course). All examples come from the testing directory `tst` where much of the development work is done. The `tst` directory contains many intermediate examples that show the progression of development. Many being simple examples that are a great resource to use to come up to speed on how the libraries work and what calls are needed to initialize, use and then stop, remove whatever library you are interested in.

The `lib` directory contains all the access libraries for the peripherals. The libraries install to `prefix=/usr/local` by default. The Pi OS provides the `staff` group that can own the files/directories in `/usr/local`. Configuring `/usr/local` that way and adding yourself to the `staff` group will allow you to maintain the files there without needing to be root, or use `sudo` or `su`. (you will still need root access to run `ldconfig` after installing or removing a library -- I can't help you there)

Also add your user to the `gpio` group. (e.g. `sudo gpasswd -a you gpio`, then log out/in so group changes are seen by your environment) `/dev/gpiomem`, used by `tinygpio`, is group-owned by the `gpio` group.

The primary libraries for peripheral access are:

 * `i2c` - which provide I2C access using the kernel i2c/smbus ioctl() interface.
 * `itimer` - a library that uses the kernel interval timer capabailities (via `timer_create()`) to generate repeating signals at whatever frequency you need. This is used by the `pwmsoft` library as well as the `mpu` and `ssd1306-oled-ioctl` library.
 * `pwm` - hardware PWM access using the sysfs interface.
 * `pwmsoft` - software PWM providing PWM on any GPIO pin with interval signal generated by the `itimer` library and GPIO access provided by the kernel **gpio_v2 ABI** (of course, the gpio V1 ABI is ... deprecated). Interval timer signals are handled in a single thread isolated from the main program. The usable limit of the software clock generated by interval timer signals is the number of signals that can be processed by a single core. For 64-bit Arm, that limit is ~25KHz. For 32-bit Arm you can double that to ~50HKz. The library imposes those limits.
 * `ssd1306-oled-ioctl` - rewrite of [Gavin Lyons's library](https://github.com/gavinlyonsrepo/SSD1306_OLED_RPI) removing the backend requiring root and replaceing with the Linux kernel i2c/smbus ioctl interface. (note: setting your i2c bus speed to 400K provides 24 FPS refresh. Setting the bus speed to the experiental 1.48MHz or 1.5MHz setting can give up to 50 FPS refresh.
 * `tinygio` - A partial port of the [Tiny GPIO Access library](http://abyz.me.uk/rpi/pigpio/examples.html) making use of the special carve out of `"/dev/gpiomem"` is provided to set GPIO pin-states or read/write pin values on the fly.

There are a few secondary libraries for specific board or feature access:

 * `ads1115` - a library for interfacing with the TI ADS1115 4-channel 16-bit ADC board is fully complete and quite helpful.
 * `fusion` and `fusionx` are two implementation of sensor-fusion algorithms. The are usable, but still in work. They take the raw linear acceleration, angular rate and magnetometer data and convert the positioning information into quaternions used to provide compass heading, and absoluted position by using the magnetometer to stabilze the gyro drift inherent in those sensors.
 * `mpu` - library for using the Inversense MPU6050, MPU6500, MPU9250 or MPU9255 sensors over I2C. Library is complete and uses i2c/smbus access to the I2C bus.

## Using the libraries in your projects

The libraries are safe to include with both C and C++ sources. The intended way to use the libraries are simply to make and install the libraries in `/usr/local`. The `Makefile` for each has both `install` and `uninstall` rules. Simply:

```none
 $ cd to/lib/ofinterest
 $ make
 $ sudo make install
```

Calling `ldconfig` and creating the normal libary symlinks are handled in the install.

Then just include the header for the library in you source code and link against it during compile. (e.g. `#include <pwmsoft.h>` and then link against the library by providing `-lpwmsoft` as part of your compiler string) Most distributions include `/usr/local/include` as part of the default include search path. If not just add `-I/usr/local/include` to your compile string. And if `/usr/local/lib` isn't in your default library search path, just add `-L/usr/local/lib` as well. (you can simply edit the `Makefile` and add the include search path to `CFLAGS` and the library search path to `LDFLAGS`).

*Note:*

These libraries have been tested on Raspberry Pi Zero, 2, 3, 4. While they should work with the Pi 5 as well, no hardware was available to test.

This Readme is a work in progress, but all code builds with full warnings enabled (e.g. with `-Wall -Wextra -pedantic -Wshadow -std=c11 ...` and should not have any glaring issues. (that's not to suggest the libraries or bug-free or cannot be further improved) The primary library code is heavily commented to help with the learning process. If you find a bug, open an issue, or create a pull-request and provide a proposed fix. Enjoy the libraries.