#!/bin/bash

mkdir -p sdk
cd ./sdk
wget https://www.nordicsemi.com/eng/nordic/download_resource/54291/46/30914717 -O sdk.zip
unzip sdk.zip
rm sdk.zip
https://www.nordicsemi.com/eng/nordic/download_resource/54285/46/2742117
cd ..
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2015-q4-major/+download/gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2
bzip2 -dc gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2 | tar xvf -
rm gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2

export PATH=$PATH:`pwd`/gcc-arm-none-eabi-5_2-2015q4/bin
export NRF51_SDK_DIR=`pwd`/sdk

echo "GNU_INSTALL_ROOT := `pwd`/gcc-arm-none-eabi-5_2-2015q4"$'\r\n'"GNU_VERSION := 5.2.1"$'\r\n'"GNU_PREFIX := arm-none-eabi"$'\r\n' > ${NRF51_SDK_DIR}/components/toolchain/gcc/Makefile.posix

mkdir ${NRF51_SDK_DIR}/components/drivers_ext/segger_rtt
git clone git@gist.github.com:25a566178766c7d0a7e04a18b341a732.git
cp ./25a566178766c7d0a7e04a18b341a732/* ${NRF51_SDK_DIR}/components/drivers_ext/segger_rtt/

cd pca10028
make