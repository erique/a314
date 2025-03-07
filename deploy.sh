#!/bin/bash

set -xe
#sudo systemctl stop a314d
sudo cp Software/bin_pi/a314d-tf /opt/a314/a314d || echo "failed to deploy"
#sudo systemctl start a314d
cd Software/bin_amiga/Devs
~/rlaunch/t2-output/linux-gcc-release-default/rl-controller 192.168.2.2 c:copy a314-tf.device devs:a314.device || echo "failed to deploy"
cd -
