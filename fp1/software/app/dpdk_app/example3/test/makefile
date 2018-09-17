 #
 #   BSD LICENSE
 #
 #   Copyright(c)  2017 Huawei Technologies Co., Ltd. All rights reserved.
 #   All rights reserved.
 #
 #   Redistribution and use in source and binary forms, with or without
 #   modification, are permitted provided that the following conditions
 #   are met:
 #
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in
 #       the documentation and/or other materials provided with the
 #       distribution.
 #     * Neither the name of Huawei Technologies Co., Ltd  nor the names of its
 #       contributors may be used to endorse or promote products derived
 #       from this software without specific prior written permission.
 #
 #   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 #   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 #   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 #   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 #   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 #   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 #   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 #   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 #   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 
include $(MAKEROOT)/env.mk

SRCS := $(wildcard *.c)
OBJS := $(patsubst %.c,%.o,$(SRCS))
BINS := $(patsubst %.c,%,$(SRCS))

dpdk_path=$(/home/hust/zzt/poc-fpga/fp1/software/app/dpdk_app)

JRE_HOME?=${JAVA_HOME}/jre/
JNI_COMPILATION_FLAGS=-D_REENTRANT -fPIC -I${JRE_HOME}/../include -I${JRE_HOME}/../include/linux
COMMON_COMPILATION_FLAGS=$(JNI_COMPILATION_FLAGS) -O3 -Wall $(OMPCFLAGS) -Wno-unknown-pragmas -Wno-write-strings -Wno-unused-variable -Wno-unused-but-set-variable

.PHONY : all

all : $(BINS)
	echo "BINS: ${BINS}; OBJS: ${OBJS}"
	
$(BINS) : $(OBJS)
	gcc -shared -g -fPIC -lpthread -lrt -ldl -fstack-protector -Wl,-z,relro -Wl,-z,noexecstack -L/home/hust/zzt/poc-fpga/fp1/software/app/dpdk_app/../../userspace/dpdk_src/securec/lib -lsecurec -L/home/hust/zzt/poc-fpga/fp1/software/app/dpdk_app/../../userspace/dpdk_src/dpdk-16.04/x86_64-native-linuxapp-gcc/lib -lethdev -lrte_mbuf -lrte_mempool -lrte_ring -lrte_eal -lrte_pmd_acc -L/usr/lib64/ -lfpgamgmt -D_REENTRANT -fPIC -I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.40.x86_64/jre//../include -I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.40.x86_64/jre//../include/linux -O3 -Wall  -Wno-unknown-pragmas -Wno-write-strings -Wno-unused-variable -Wno-unused-but-set-variable -o /home/hust/zzt/poc-fpga/fp1/software/app/dpdk_app/bin/libFPGAPairHMM.so /home/hust/zzt/poc-fpga/fp1/software/app/dpdk_app/example3/test/org_broadinstitute_hellbender_utils_pairhmm_pairhmm_FPGAPairHMM.o -L/home/hust/zzt/poc-fpga/fp1/software/app/dpdk_app/lib -lfpgarw	
#gcc ${LDFLAGS} ${COMMON_COMPILATION_FLAGS} -o ${DIR_BINS}/$@ ${DIR_EXAMPLE3_TEST_SRC}/$@.o -L${DIR_LIBS} -lfpgarw

%.o : %.c
	gcc ${CFLAGS} -c ${COMMON_COMPILATION_FLAGS} $< -o ${DIR_EXAMPLE3_TEST_SRC}/$@
