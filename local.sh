#!/bin/bash

make -C Software
VBCC=/opt/vbcc PATH=$PATH:$VBCC/bin NDK32=/opt/ndk32 make -C Software -f Makefile-amiga
