#!/bin/bash

set -xe
sudo cp Software/bin_pi/a314d-tf4060 /opt/a314/a314d
cd Software/bin_amiga/Devs
~/rlaunch/t2-output/linux-gcc-release-default/rl-controller 192.168.10.23 c:copy a314-tf4060.device devs:a314.device || true
~/rlaunch/t2-output/linux-gcc-release-default/rl-controller 192.168.10.23 c:copy a314eth.device devs: || true
cd -
