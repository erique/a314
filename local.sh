#!/bin/bash

VBCC=/opt/vbcc
NDK32=/opt/ndk32

PATH=$PATH:$VBCC/bin

BIN=bin_pi
make -j -C Software ${BIN}/a314d-tf4060 ${BIN}/spi-a314.dtbo

VBCC=$VBCC NDK32=$NDK32 make -j -C Software -f Makefile-amiga -j
