#!/bin/bash

VBCC=/opt/vbcc
NDK32=/opt/ndk32

PATH=$PATH:$VBCC/bin

BIN=bin_pi
make -j -C Software ${BIN}/a314d-tf4060 ${BIN}/spi-a314.dtbo

BIN=bin_amiga
VBCC=$VBCC NDK32=$NDK32 make -j -C Software -f Makefile-amiga -j ${BIN}/Devs/a314-tf4060.device ${BIN}/Devs/a314eth.device
