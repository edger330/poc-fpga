#ifndef SRC_DUPLEX_QUEUE_DUPLEX_QUEUE_H_
#define SRC_DUPLEX_QUEUE_DUPLEX_QUEUE_H_

#define DS_BASE_ADDR    0x00000000
#define US_BASE_ADDR    0xa0000000
#define DS_PSIZE_EXP    17  // package size 128KiB
#define DS_PCAP_EXP     14  // package capacity 2048 (2047 actual available)

#define US_PSIZE_EXP    17  // package size 128KiB
#define US_PCAP_EXP     13  // package capacity 8192 (8191 actual available)

#define DS_PSIZE        (1L << DS_PSIZE_EXP)
#define DS_PCAP         ((1L << DS_PCAP_EXP) - 1)
#define DS_ADDR_LIMIT   (DS_BASE_ADDR + (1L << (DS_PSIZE_EXP + DS_PCAP_EXP)))

#define US_PSIZE        (1L << US_PSIZE_EXP)
#define US_PCAP         ((1L << US_PCAP_EXP) - 1)
#define US_ADDR_LIMIT   (US_BASE_ADDR + (1L << (US_PSIZE_EXP + US_PCAP_EXP)))

#define MAP_SIZE (32*1024UL)
#define MAP_MASK (MAP_SIZE - 1)

#define DS_TAIL_REG     0
//#define DS_HEAD_REG     1
#define DS_CNT_REG      2
//#define US_TAIL_REG     4
#define US_HEAD_REG     5
#define US_CNT_REG      6

//#define buff_size 128*1024
#define RETRY_TIME  (100000)

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <linux/limits.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>
#include <string.h>
#include <pthread.h>
#include "ringbuffer.h"
#include "regs_infos.h"
#include "fpga_ddr_rw_interface.h"

#define capacity 128
#define buff_size 128*1024
int result_flag[10] = { 0 }; 
unsigned long long fpga_ddr_wr_addr = 0;
unsigned long long fpga_ddr_rd_addr = 0;
unsigned int dsTail = DS_BASE_ADDR;
unsigned int local_dsTail = DS_BASE_ADDR;
unsigned int usHead = US_BASE_ADDR;
unsigned int slot_id = 0;
unsigned int thread_write,thread_read;
rw_ddr_data write_data,read_data;
void *read_src_addr = NULL;
void *write_src_addr = NULL;
int packageWriteCount = 0;
int packageReadCount = 0;

unsigned char* write_ringbuffer[capacity];
unsigned char* read_ringbuffer[capacity];
unsigned char result_buff[128*1024];

void write_ringbuffer_push_data(unsigned char* data){
    while(1){
        if(isPushWriteData()){
            pushWriteData(data,write_ringbuffer);
            break;
        }
    }
}
void write_ringbuffer_pop_data(unsigned char* data){
    while(1){
        if(isPopWriteData()){
            popWriteData(data,write_ringbuffer);
            break;
        }
    }
}
void read_ringbuffer_push_data(unsigned char* data){
    while(1){
        if(isPushReadData()){
            pushReadData(data,read_ringbuffer);
            break;
        }
    }
}
void read_ringbuffer_pop_data(unsigned char* data){
    while(1){
        if(isPopReadData()){
            popReadData(data,read_ringbuffer);
            break;
        }
    }
}

void printRegInfo(){
    printf("after update reg\n");
    unsigned int value;
    read_register(slot_id, down_tail, &value); 
    printf("down_tail : 0x%08x\n",value);
    read_register(slot_id, down_head, &value); 
    printf("down_head : 0x%08x\n",value);
    read_register(slot_id, down_cnt, &value);

    printf("down_cnt  : 0x%08x\n",value);
    read_register(slot_id, up_tail, &value);

    printf("up_tail   : 0x%08x\n",value);
    read_register(slot_id, up_head, &value);

    printf("up_head   : 0x%08x\n",value);
    read_register(slot_id, up_cnt, &value);

    printf("up_cnt    : 0x%08x\n",value);
   
    read_register(slot_id, data_sta, &value);
    printf("data_sta  : 0x%08x\n",value);

    read_register(slot_id, read_pkt_cnt, &value);
    printf("read_pkt_cnt : 0x%08x\n",value);

    read_register(slot_id, write_pkt_cnt, &value);
    printf("write_pkt_cnt : 0x%08x\n",value);

    read_register(slot_id, sop_cnt_0, &value);
    printf("sop_cnt_0 : 0x%08x\n",value);

    read_register(slot_id, sop_cnt_2, &value);
    printf("sop_cnt_2 : 0x%08x\n",value);

 
}
unsigned int GetUsPkgCnt(){
    unsigned int value;
    read_register(slot_id, up_cnt, &value);
    return value;
}
int isReadable(){
    unsigned int cnt = GetUsPkgCnt();
    return cnt>=1;
}
void updateUsHead(){
    usHead += US_PSIZE;
    if(usHead >= US_ADDR_LIMIT){
        usHead = US_BASE_ADDR;
    }
//    usHead = 0x00;
    write_register(slot_id, up_head, usHead);
}
unsigned int GetDsPkgCnt(){
    unsigned int value;
    read_register(slot_id, down_cnt, &value);
    return value;
}
int isWriteable(){
    unsigned int cnt = GetDsPkgCnt();
    return cnt + 1 <= (DS_PCAP - 2048);
}
void updateDsTail(){
    dsTail += DS_PSIZE;
    if(dsTail >= DS_ADDR_LIMIT){
        dsTail = DS_BASE_ADDR;
    }
    //dsTail = local_dsTail;
    //write_register(slot_id, down_tail, 0x00);
    write_register(slot_id, down_tail, dsTail);
}
void updateLocalDsTail(){
    local_dsTail += DS_PSIZE;
    if(local_dsTail >= DS_ADDR_LIMIT){
        local_dsTail = DS_BASE_ADDR;
    }
}

void callback( unsigned int thread_id, unsigned int slot_id, rw_ddr_data rw_data, int rw_flag)
{   
    if(rw_flag == 1) {
        memcpy(result_buff,(unsigned char*)rw_data.cpu_vir_dst_addr,sizeof(unsigned char)*128*1024);
        /*for(int i=0;i<128;i++){
            printf("%02x ", result_buff[i]);
        }
	*/
        //printRegInfo();
	    //printf("\n");
	//printf("读取数据成功\n");
        //writeDataToFile(result_buff);
        read_ringbuffer_push_data(result_buff);//读取数据成功之后，将数据放进本地队列（读）中       
        (void)memory_manager_free_bulk((void*)(rw_data.cpu_vir_dst_addr));
    }
    else if(rw_flag == 2)
    {   
        //if(packageWriteCount%1024==0){updateDsTail();packageWriteCount++;}
        //printf("写入数据成功，准备更新寄存器dstail\n");
        updateDsTail();
        (void)memory_manager_free_bulk((void*)(rw_data.cpu_vir_src_addr));
      // printRegInfo();
    }
    else
    {
        (void)memory_manager_free_bulk((void*)(rw_data.cpu_vir_dst_addr));
        (void)memory_manager_free_bulk((void*)(rw_data.cpu_vir_src_addr));
    }
   
    result_flag[thread_id]++;
    return;
}

/*void prepareData(unsigned char* Data){
   // printf("prepareData\n");
    FILE *fp;
    if((fp=fopen("/home/hust/poc-fpga/fp1/software/app/dpdk_app/testDataFPGA.txt", "r"))==NULL){
        printf("file connot be opened!\n");
        exit(1);
    }
    for(int i=0;i<128*1024;i++)
    {
        fscanf(fp,"%d ",&Data[i]);
       // if(i<32)
	 //   printf("%02x ",Data[i]);
    }
    fclose(fp);
    printf("===========prepare matrix data done!==========\n");
    for(int i=0;i<128*1024;i++){
                printf("%02x ",Data[i]);
        }
    printf("==============================================\n");
}*/

/*void writeDataToFile(unsigned char* Data){
    FILE *fp = fopen("/home/hust/poc-fpga/fp1/software/app/dpdk_app/test.txt", "a");
    fprintf(fp,"ready to write data to file\n");
    for(int i=0;i<128*1024;i++)
    {
        fprintf(fp,"%d\n",Data[i]);
    }
    fprintf(fp,"\n");
    printf("write data to file success\n");
    fclose(fp);
}*/

int init(){
    int ret;
    unsigned int retry_time = 0;
    for (int i = 0; i < 128; ++i) {
        write_ringbuffer[i] = (unsigned char*)malloc(sizeof(unsigned char)*128*1024);
        read_ringbuffer[i] = (unsigned char*)malloc(sizeof(unsigned char)*128*1024);
    }
    ret = fddr_access_mode_init(callback);
    if(ret) {
        printf("call fpga_ddr_rw_module_init fail .\n");
        return 0;
    }
    //printRegInfo();
    write_register(slot_id, RSV0_REG, 1);
    write_register(slot_id, down_tail, dsTail);
    write_register(slot_id, up_head, usHead);
    //printRegInfo();
    while(GetDsPkgCnt()||GetUsPkgCnt());
    //printRegInfo();
    write_register(slot_id, RSV0_REG, 0);
    //printRegInfo();
    ret = alloc_thread_id(&thread_write);
    if(ret) {
        return 0;
    }
    ret = alloc_thread_id(&thread_read);
    if(ret) {
        return 0;
    }
    /*for(retry_time = 0; retry_time < RETRY_TIME; retry_time++) {
            write_src_addr = memory_manager_alloc_bulk(buff_size/8);
            if(NULL == write_src_addr) {
                usleep(100);
                continue;
            }
            else {
                break;
            }
    }
    if(retry_time == RETRY_TIME) {
            printf("call memory_manager_alloc_bulk fail . \n");
            return NULL;
    }
    for(retry_time = 0; retry_time < RETRY_TIME; retry_time++) {
            read_src_addr = memory_manager_alloc_bulk(buff_size/8);
            if(NULL == read_src_addr) {
                usleep(100);
                continue;
            }
            else {
                break;
            }
    }*/
    if(retry_time == RETRY_TIME) {
        printf("call memory_manager_alloc_bulk fail1 . \n");
        return 0;
    }
    return 1;
}
void clean(){
    int ret;
    for (int i = 0; i < 128; ++i)
    {
        free(write_ringbuffer[i]);
        free(read_ringbuffer[i]);
    }
    ret = free_thread_id(thread_write);
    if(ret) {
        printf("call free_thread_id fail .\n");
        return;
    }
    printf("free thread_id success, %d\n", thread_write);
    ret = free_thread_id(thread_read);
    if(ret) {
        printf("call free_thread_id fail .\n");
        return;
    }
    printf("free thread_id success, %d\n", thread_read);
}
void writePkg(unsigned char* write_buff){
    //printf("调用写数据API\n");
    int ret;
    unsigned int retry_time = 0;
    for(retry_time = 0; retry_time < RETRY_TIME; retry_time++) {
            write_src_addr = memory_manager_alloc_bulk((buff_size));
            if(NULL == write_src_addr) {
                usleep(100);
                continue;
            }
            else {
                break;
            }
    }
    if(retry_time == RETRY_TIME) {
            printf("call memory_manager_alloc_bulk fail . \n");
            return;
    }
    write_data.cpu_vir_src_addr = (unsigned long long)write_src_addr;
    write_data.fpga_ddr_wr_addr = local_dsTail;//本地指针，用作表明写入位置，这个指针是为了用向FPGA DDR做异步写入操作
    //write_data.fpga_ddr_wr_addr = 0x00000000;
    write_data.length = (buff_size);
    memcpy(write_src_addr,write_buff,buff_size);
    for(retry_time = 0; retry_time < RETRY_TIME; retry_time++) {
        //printf("ready to write data to addr %08x \n",write_data.fpga_ddr_wr_addr );
        ret = write_data_to_fddr(thread_write, slot_id, write_data);//写数据API
        if(ret) {
            usleep(100);
            printf("write error.\n");
            continue;
        }
        else {
            break;
        }
    }
    updateLocalDsTail();//立即更新本地指针，表明下一次的写入应该写入在DDR中的递增位置，写入数据成功之后会在回调函数中更新对应的FPGA寄存器表明数据已经成功写入到地址，FPGA现在可以去该地址读取数据
    //printf("update local dsTail \n");
    //(void)memory_manager_free_bulk(write_src_addr);
    //printf("write func done!\n");
}
void readPkg(){
    //printf("调用读数据API\n");
    int ret;
    unsigned int retry_time = 0;
    for(retry_time = 0; retry_time < RETRY_TIME; retry_time++) {
            read_src_addr = memory_manager_alloc_bulk((buff_size));
            if(NULL == read_src_addr) {
                usleep(100);
                continue;
		printf("error_0\n");
            }
            else {
                break;
            }
    }
    if(retry_time == RETRY_TIME) {
            printf("call memory_manager_alloc_bulk fail1 . \n");
        return;
    }
    
        read_data.cpu_vir_dst_addr = (unsigned long long)read_src_addr;
        read_data.fpga_ddr_rd_addr = usHead; 
//        read_data.fpga_ddr_rd_addr = 0xa0000000;       
        read_data.length = (buff_size);
        /*while(1){
            if(isReadable()){
                break;
            }    
        }*/
        //printf("fpga is readable %d \n",GetUsPkgCnt());
        for(retry_time = 0; retry_time < RETRY_TIME; retry_time++) {
            //printf("ready to read data from addr %08x \n",read_data.fpga_ddr_rd_addr );
           // printf("ret = %d\n",ret);
	    ret = read_data_from_fddr(thread_read, slot_id, read_data);//调用读数据API
            //printf("ret = %d   retry_time:%d\n",ret,retry_time);
	    if(ret) {
                usleep(100);
                printf("read error.\n");
                continue;
            }
            else {
                break;
            }
        }
        //(void)memory_manager_free_bulk(read_src_addr);
        //memcpy(read_buff,read_src_addr,buff_size);
        /*printf("=============ready to print result===========\n");
        for(int i=0;i<128;i++){
            printf("%02x ",read_buff[i]);
        }
        printf("============print result done!=============\n");*/
}
#endif



