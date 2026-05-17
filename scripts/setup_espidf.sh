#!/bin/bash
set -e
# Run once on WSL2 Ubuntu 22.04
sudo apt-get update && sudo apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

cd ~
git clone --recursive https://github.com/espressif/esp-idf.git --branch v5.3.1
cd esp-idf
./install.sh esp32p4
echo 'alias get_idf=". $HOME/esp-idf/export.sh"' >> ~/.bashrc
source ~/.bashrc
