#!/bin/bash

#����·��
SEC_BUILD_PATH=`pwd`

#����make����
cd ${SEC_BUILD_PATH}/src
make -f Makefile

echo  "copy libsecurec.so to release finished!"
