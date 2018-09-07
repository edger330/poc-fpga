#!/bin/bash

#±àÒëÂ·¾¶
SEC_BUILD_PATH=`pwd`

#µ÷ÓÃmake±àÒë
cd ${SEC_BUILD_PATH}/src
make -f Makefile

echo  "copy libsecurec.so to release finished!"
