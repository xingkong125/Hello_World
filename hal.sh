#!/bin/bash
set -e

if [ "$1" == "-help"];then
	echo "./hal.sh [B32/B64]"
	exit
else
	export HAL_MODULE=gps.defaullt
fi

CURRENT_PATH=$(pwd)

android_401=/home/android/android_401
android_501=
android_601=
android_701=

echo "choose build platform:"
echo "1:harmony2.2"
echo "2:ventana2.3"
echo "3:ventana3.1"
echo "4:android4.2"
echo "5:tcc8800_2.3"
echo "6:samsung smdkv210 4.0.3"
echo "7:android 5.1"
echo "8:android 6.1"
echo "9:android 7.0"

read PLATFORM
case $PLATFORM in



########################################################Android4.2#####################################################################
4)
rm -rf ${android_401}/hardware/libhardware/modules/wl_gps
cp -r wl_gps ${android_401}/hardware/libhardware/modules/wl_gps

cd $android_401

#make clean

source Android_build.sh

rm -rf ${android_401}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so


#编译hal
export SECURE_OS_BUILD=y
source build/envsetup.sh

if [ "$1"=="B64" ];then
	lunch 2
else
	lunch 1
fi

echo "[[[[build $HAL_MODULE]]]]"
make $HAL_MODULE

cd $CURRENT_PATH

mkdir $(date +%Y-%m-%d-)$(date +%H-%M-%S)-android4.2-$1

cp ${android_401}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so $(date +%T-%m-%d-)$(date +%H-%M-%S)-android4.2-$1
;;

##############################################################android 5.1############################################################################
7)
#拷贝HAL代码到编译环境中
rm -rf ${android_501}/hardware/libhardware/modules/wl_gps
cp -r wl_gps ${android_501}/hardware/libhardware/modules/wl_gps

#切换到编译环境的根目录
cd $android-501

#删除之前生成的连接文件
if [ "$1" == "B64" ];then
	rm -rf ${android_501}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so
else
	rm -rf ${android_501}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so
fi

echo "delete old hal successs!"

#编译hal
exportSECURE_OS_BUILLD=y
source build/envsetup.sh

if [ "$1" == "B64" ];then
	lunch 2
else
	lunch 1
fi

echo "[[[[build $HAL_MODULE]]]]"
make $HAL_MODULE

#拷贝编译结果到当前目录
cd $CURRENT_PATH
mkdir $(date +%Y-%m-%d-)$(date +%H-%M-%S)-android5.1-$1

if [ "$1" == "B64" ];then
	cp ${android_501}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so $(date +%Y-%m-%d-)$(date +%H-%M-%s)-android5.1-$1
else
	cp ${android_501}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so $(date +%Y-%m-%d-)$(date +%H-%M-%S)-android5.1-$1
fi

;;

#######################################################android 7.1####################################################################
7)
#拷贝HAL代码到编译环境中
rm -rf ${android_701}/hardware/libhardware/modules/wl_gps
cp -r wl_gps ${android_701}/hardware/libhardware/modules/wl_gps

#切换到编译环境的根目录
cd $android-501

#删除之前生成的连接文件
if [ "$1" == "B64" ];then
	rm -rf ${android_701}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so
else
	rm -rf ${android_701}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so
fi

echo "delete old hal successs!"

#编译hal
exportSECURE_OS_BUILLD=y
source build/envsetup.sh

if [ "$1" == "B64" ];then
	lunch 2
else
	lunch 1
fi

echo "[[[[build $HAL_MODULE]]]]"
make $HAL_MODULE

#拷贝编译结果到当前目录
cd $CURRENT_PATH
mkdir $(date +%Y-%m-%d-)$(date +%H-%M-%S)-android5.1-$1

if [ "$1" == "B64" ];then
	cp ${android_701}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so $(date +%Y-%m-%d-)$(date +%H-%M-%s)-android5.1-$1
else
	cp ${android_701}/out/target/product/generic/system/lib/hw/${HAL_MODULE}.so $(date +%Y-%m-%d-)$(date +%H-%M-%S)-android5.1-$1
fi

;;

esac
