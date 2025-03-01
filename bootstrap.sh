#!/bin/bash

set -xe

git clone --depth 1 https://github.com/erique/vbcc_vasm_vlink.git /tmp/vbcc
cd /tmp/vbcc

./test.sh

wget http://aminet.net/dev/misc/NDK3.2.lha
7z x NDK3.2.lha -obuild/ndk32
chmod -R 755 build/ndk32

sudo cp -r build/vbcc  /opt/vbcc
sudo cp -r build/ndk32 /opt/ndk32

if ! [ -z ${USER+x} ]; then
	sudo chown -R $USER /opt/vbcc
	sudo chown -R $USER /opt/ndk32
fi

cd ~
rm -rf /tmp/vbcc
