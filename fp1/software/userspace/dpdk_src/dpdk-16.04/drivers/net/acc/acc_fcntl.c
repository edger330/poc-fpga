/*-
 *   BSD LICENSE
 *
 *   Copyright(c)  2017 Huawei Technologies Co., Ltd. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Huawei Technologies Co., Ltd  nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <sys/file.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include <rte_log.h>

#include "acc_fcntl.h"
#include "acc_logs.h"
#include "securec.h"

/*****************************************************************************
 Function name      : acc_fcntl_read_lock
 Description        : open file lock to prevent multiple process
 Input parameters   : slot_id : fpga slot id
 Output parameters  : file_id : open file id
 Return value       : Success : 0
                      Fail : -EINVAL/-ENOMEM
 Function explain   : 
 calling function   : None
 called function    : None
 
 Modify history     :
    1.Date              : 2018/03/15
      Author            : AI SDK Team
      Content           : New function
*/
int
acc_fcntl_read_lock(uint16_t slot_id, int *file_id) {
    char lock_file_name[FILE_NAME_MAX_LEN + 1] = {0};
    int diag = 0;
    int open_file_id = 0;
    struct flock lock;

    if(FPGA_SLOT_MAX <= slot_id) {
        PMD_ACC_CRIT("acc_fcntl_read_lock invalid slot_id = %d!", slot_id);
        return -EINVAL;
    }

    if(NULL == file_id) {
        PMD_ACC_CRIT("acc_fcntl_read_lock file_id is NULL!");
        return -EINVAL;
    }

    diag = snprintf_s(lock_file_name, sizeof(lock_file_name), (sizeof(lock_file_name) - 1)
        , HW_MUTEX_PATH, slot_id);
    if(-1 == diag) {
        PMD_ACC_ERROR("lock_file_name: %s has some wrong, ", lock_file_name);
        return -EINVAL;
    }

    /* create the file lock */
    open_file_id = open(lock_file_name, (O_CREAT | O_RDWR), 0644);
    if(0 > open_file_id) {
        PMD_ACC_ERROR("open %s file error.", lock_file_name);
        return -EINVAL;
    }

    /* initialize the flock struct */
    lock.l_type = F_RDLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;

    /* control the file rlock */
    diag = fcntl(open_file_id, F_SETLK, &lock);
    if(0 != diag) {
        PMD_ACC_ERROR("fcntl %s file error.", lock_file_name);
        close(open_file_id);
        return -EINVAL;
    }

    *file_id = open_file_id;

    return 0;
 }

/*****************************************************************************
 Function name      : acc_fcntl_read_unlock
 Description        : file unlock
 Input parameters   : file_id : file id
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-ENOMEM
 Function explain   : 
 calling function   : None
 called function    : None
 
 Modify history     :
    1.Date              : 2018/03/15
      Author            : AI SDK Team
      Content           : New function
*/

int
acc_fcntl_read_unlock(int file_id) {
    int diag = 0;
    struct flock lock;
    
    if(0 > file_id) {
        PMD_ACC_CRIT("file_id = %d is error.!", file_id);
        return -EINVAL;
    }

    /* initialize the flock struct */    
    lock.l_type = F_UNLCK;     
    lock.l_whence = SEEK_SET;     
    lock.l_start = 0;     
    lock.l_len = 0;

    /* unlock the file */    
    diag = fcntl(file_id, F_SETLK, &lock);    
    if (0 != diag) {               
        PMD_ACC_ERROR( "file unlock failed diag = %d, %d", diag, errno);   
        close(file_id);        
        return -EINVAL;        
    }

    close(file_id);

    return 0;
}
