# cs466_s21
Lab information and examples for CS466 Embedded Systems, Spring 2021

## If you want to use my pre-configured makefiles there are two required peer directories
Assuming that you let this repo install with a default name cs466_s21 there are two required directories that you need to install.
1. TivaDriver 
  * This is an library containing header files describing the processor and Tiva Board
  * It also contains a series of higher level API's that we will be using in class later
2. FreeRTOS

Before you build your parent directoy will look like:
```
   ...
   cs466_s20/
   TivaDriver/
   FreeRTOSvxxxxxx/
   ...
```
You can rename the directories or move them around but my provided makefiles will require modification each week.

for Linux (and probably OSx) you can use the TivaDriver tar file that I have put in .../cs466_s21/doc.

1. change your directory so that you are in ../cs466_s21 or the parent of cs466_s21.
2. Run the command.
```
$ tar zxvf ./cs466_s21/doc/TivaDriver.tar.gz
```
It should create the peer TivaDriver directory.

## you should have the ARM gnu compiler toolchain installed as well and have the 'bin' directory in your path
Get the toolchain from https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

## FreeRTOS is not used for lab01 but you can install it

1. change your directory so that you are in ../cs466_s21 or the parent of cs466_s21.
2. Download latest FreeRTOS Version from the following URL
   * https://www.freertos.org
3. Unzip the distrubition to the peer directory
```
```

