#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <linux/limits.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>
#include <string.h>
#include "ringbuffer.h"
#include "queue_ctrl.h"
#include <regs_infos.h>
#include "fpga_ddr_rw_interface.h"

pthread_t thread[5];
unsigned char *write_buff,*read_buff;
unsigned char* random_array[128];
int packageNum = 1000000;//包数目
int isPassMatchTestNum = 0;
int isPassMatchTest(unsigned char* A){
    int id = (int)A[0];
    for(int i = 1;i < 128*1024 ;i++){
        if(A[i]==random_array[id][i]){
            continue;
        }else{
            printf("find mismatch at idx: %d\n",i);
            return 0;
        }
    }
    return 1;
}

void* write_func(){//向FPGA写数据线程
    int packageCount = 0;
    for(int i=0;i<packageNum;i++){
	while(1){
	//int Num = DS_PCAP - GetDsPkgCnt();
	//printf("DDR中可以写入%d包\n",Num);
     		if(isWriteable()){
        		writePkg(write_buff);
        		break;
      		}
    	}
    }
}
void* write_data_to_FPGA(){
    for (int i=0; i<packageNum; i++) {
        int id = rand()%127;//从一个随机生成的数据队列中取出一个随机数据放到本地队列（写）中
        write_ringbuffer_push_data(random_array[id]);
        //write_ringbuffer_push_data(random_array[i]);
        //printf("已经写入%d个包\n",i);
    }
}

void* read_func(){
    //printf("read_func\n");
    while(1){
     // usleep(1000);
      if(isReadable()){
        readPkg();
	updateUsHead();	
        //break;
      }
    }
}

void* read_data_from_FPGA(){
    unsigned char* buff = (unsigned char*)malloc(128*1024);
    for (int i = 0; i < packageNum; i++) {
	 read_ringbuffer_pop_data(buff);//从本地队列（写）中取出数据到buff中
	/* int isPass = isPassMatchTest(buff);//校验buff中的数据是否和本地的随机数组中的数据匹配，如果匹配，isPass为真
        if(isPass){
            isPassMatchTestNum++;
        }*/
    }
    free(buff);
    if(isPassMatchTestNum==packageNum){//判断是否所有数据全部校验成功，校验成功则打印以下信息
        printf("read data from FPGA done! all %d package pass test!\n",packageNum);
    }else{
        printf("total package num %d , pass %d package\n",packageNum,isPassMatchTestNum);
    }
}

void* updateWriteInfoToFPGA(){
    while(1){
	updateDsTail();
	printRegInfo();
	usleep(100);
    }
}

int main(int argc, char* argv[]) {
    init();
    //printRegInfo();
    for (int i = 0; i < 128 ; i++) {//生成一个随机数组用作发送数据
        random_array[i] = (unsigned char*)malloc(sizeof(unsigned char)*128*1024);
        //random_array[i][0] = i;
       // for(int j = 1;j < 128*1024;j++){
           // random_array[i][j] = (unsigned char)(rand()%255);
       //     random_array[i][j] = 0x00;
       // }
       // printf("read data from file\n");
	   prepareData(random_array[i]);
    }
    //printRegInfo();
    printf("开始数据回环测试\n");
    write_buff = (unsigned char*)malloc(sizeof(unsigned char)*128*1024);
    read_buff = (unsigned char*)malloc(sizeof(unsigned char)*128*1024);
    prepareData(write_buff);
    clock_t start,end;
    start = clock();
//    pthread_create(&thread[0],NULL,write_data_to_FPGA,NULL);
//    pthread_create(&thread[1],NULL,read_data_from_FPGA,NULL);
    pthread_create(&thread[2],NULL,read_func,NULL);
    pthread_create(&thread[3],NULL,write_func,NULL);
    pthread_create(&thread[4],NULL,updateWriteInfoToFPGA,NULL);
//    pthread_join(thread[0],NULL);
//    pthread_join(thread[1],NULL);
//    pthread_join(thread[2],NULL);
    pthread_join(thread[3],NULL);
    //printf("发包完成，主线程休眠十秒等待收回数据\n");
    //sleep(10);
    end = clock();
    double dur = (double)(end-start);
    printf("%f\n",(dur/CLOCKS_PER_SEC));
    /*for (int i = 0; i < 128 ; i++) {//清理工作
        free(random_array[i]);
    }*/
    
    clean();
    free(write_buff);
    free(read_buff);
    printf("end\n");
    return 0;
}
