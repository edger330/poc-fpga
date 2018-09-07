//
// Created by howard on 18-6-23.
//

#ifndef BLOCKING_QUEUE_RINGBUFFER_H
#define BLOCKING_QUEUE_RINGBUFFER_H

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

#define capacity 128


int write_head = 0;
int write_tail = 0;
int read_head = 0;
int read_tail = 0;

void updateWriteHead(){
    write_head++;
    if(write_head >= 128){
        write_head = 0;
    }
    //printf("current head ptr is at %d \n",head);
}

void updateWriteTail(){
    write_tail++;
    if(write_tail >= 128){
        write_tail = 0;
    }
    //printf("current tail ptr is at %d \n",tail);
}

int GetWritePkgCnt(){
    int value = write_head - write_tail;
    if(value >= 0){
        return value;
    }else{
        return (value+capacity);
    }
}

void pushWriteData(unsigned char* write_data , unsigned char** write_buffer){
    memcpy(write_buffer[write_head],write_data,128*1024);
    updateWriteHead();
}

void popWriteData(unsigned char * write_data , unsigned char** write_buffer){
    memcpy(write_data,write_buffer[write_tail],128*1024);
    updateWriteTail();
}

int isPopWriteData(){
    if(GetWritePkgCnt()>0){
        return 1;
    }else{
        return 0;
    }
}

int isPushWriteData(){
    if(GetWritePkgCnt()>=(capacity-1)){
        return 0;
    }else{
        return 1;
    }
}

void updateReadHead(){
    read_head++;
    if(read_head >= 128){
        read_head = 0;
    }
    //printf("current head ptr is at %d \n",head);
}

void updateReadTail(){
    read_tail++;
    if(read_tail >= 128){
        read_tail = 0;
    }
    //printf("current tail ptr is at %d \n",tail);
}

int GetReadPkgCnt(){
    int value = read_head - read_tail;
    if(value >= 0){
        return value;
    }else{
        return (value+capacity);
    }
}

void pushReadData(unsigned char* read_data , unsigned char** read_buffer){
    memcpy(read_buffer[read_head],read_data,sizeof(unsigned char)*128*1024);
    updateReadHead();
}

void popReadData(unsigned char * read_data , unsigned char** read_buffer){
    memcpy(read_data,read_buffer[read_tail],sizeof(unsigned char)*128*1024);
    updateReadTail();
}

int isPopReadData(){
    if(GetReadPkgCnt()>0){
        return 1;
    }else{
        return 0;
    }
}

int isPushReadData(){
    if(GetReadPkgCnt()>=(capacity-1)){
        return 0;
    }else{
        return 1;
    }
}

#endif //BLOCKING_QUEUE_RINGBUFFER_H
