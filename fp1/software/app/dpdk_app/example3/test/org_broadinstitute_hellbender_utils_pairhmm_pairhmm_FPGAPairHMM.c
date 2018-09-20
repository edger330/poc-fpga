#include <jni.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include "utils.h"
#include "queue_ctrl.h"
#include "ringbuffer.h"

/* Header for class org_broadinstitute_gatk_utils_pairhmm_FPGAPairHMM */

#include "org_broadinstitute_hellbender_utils_pairhmm_pairhmm_FPGAPairHMM.h"
/*
 * Class:     org_broadinstitute_gatk_utils_pairhmm_FPGAPairHMM
 * Method:    initFPGA
 * Signature: ()V
 */
#define _write_size 128*1024
#define _read_size 128*1024
static int dev_fd=-1;
unsigned char write_buff[_write_size];
unsigned char send_buff[_write_size];
unsigned char read_buff[_read_size];
int packageNum = 0;

JNIEXPORT void JNICALL Java_org_broadinstitute_hellbender_utils_pairhmm_FPGAPairHMM_initFPGA
  (JNIEnv *env, jobject obj){
     int open_statue = init();
     if(!open_statue){
        printf("init device failed!\n");
        exit(-1);
     }
     printf("init success!\n");
}

/*
 * Class:     org_broadinstitute_gatk_utils_pairhmm_FPGAPairHMM
 * Method:    doneFPGA
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_broadinstitute_hellbender_utils_pairhmm_FPGAPairHMM_doneFPGA
        (JNIEnv *env, jobject obj){
    clean();
}




/*
 * Class:     org_broadinstitute_gatk_utils_pairhmm_FPGAPairHMM
 * Method:    pushData
 * Signature: ([B)V
 */
JNIEXPORT void JNICALL Java_org_broadinstitute_hellbender_utils_pairhmm_FPGAPairHMM_pushData
  (JNIEnv * env, jobject obj, jbyteArray write_buff){
    jbyte buffData[_write_size];
    (*env)->GetByteArrayRegion(env,write_buff,0,_write_size,buffData);
    unsigned char *a = (unsigned char*)buffData;
    /*uint8_t *tmp = (uint8_t *)a;
    data_print(tmp);*/
    //write_ringbuffer_push_data(a);
     while(1){
//	int i = isWriteable();
      if(isWriteable()){
        writePkg(a);
        break;
      }
   }    
}

/*
 * Class:     org_broadinstitute_gatk_utils_pairhmm_FPGAPairHMM
 * Method:    popData
 * Signature: ()[B
 */
JNIEXPORT jbyteArray JNICALL Java_org_broadinstitute_hellbender_utils_pairhmm_FPGAPairHMM_popData
  (JNIEnv *env, jobject obj){
    memset(read_buff,0,_read_size);
    //read(dev_fd,read_buff,_read_size);
    read_ringbuffer_pop_data(read_buff);
    jbyteArray result = (*env)->NewByteArray(env,_read_size);
    (*env)->SetByteArrayRegion(env,result,0,_read_size,(jbyte*)read_buff);
    return result;
}

JNIEXPORT void JNICALL Java_org_broadinstitute_hellbender_utils_pairhmm_FPGAPairHMM_resiveFromFPGA
  (JNIEnv *env, jobject obj){

    while(1){
	usleep(5000);
/*	int i=isReadable();
	if(1){
            readPkg();
            updateUsHead();
            printRegInfo();		 
	}*/
	if(isReadable()){
	    readPkg();
	    updateUsHead();
	    //printRegInfo();
        }
//	printRegInfo();
    }
}
