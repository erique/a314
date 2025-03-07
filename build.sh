#!/bin/bash

VBCC=/opt/vbcc
NDK32=/opt/ndk32

PATH=$PATH:$VBCC/bin

BIN=bin_pi
make -j -C Software ${BIN}/a314d-tf ${BIN}/spi-a314-tf.dtbo

BIN=bin_amiga
VBCC=$VBCC NDK32=$NDK32 make -j -C Software -f Makefile-amiga -j bin_dir ${BIN}/Devs/a314-tf.device
