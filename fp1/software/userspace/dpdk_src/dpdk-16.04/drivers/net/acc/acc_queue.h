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

#ifndef __ACC_QUEUE_H__
#define __ACC_QUEUE_H__

#include <stdint.h>
#include "rte_ethdev.h"
#include "rte_mempool.h"

#define VFS_MAX_NUM_EVERY_PF        (8)
#define QUEUES_MAX_NUM_EVERY_IP     (8)
#define DESC_MAX_NUM_EVERY_QUEUE    (1024 * 8)
#define DESC_MIN_NUM_EVERY_QUEUE    (1024 * 1)

#define PMD_ACC_BD_QUEUE_ALIGN      (1024*4)
#define PMD_ACC_BD_PTR_WRITE_ALIGN  (1024*4)

#define PMD_ACC_BD_LENGTH           (32)

struct acc_adapter;

#pragma pack(1)
typedef struct _acc_bd_ {
    uint64_t    src_phy_addr;
    uint64_t    dst_phy_addr;
    uint32_t    length;

    /* AE fill ve_info_xxx */
    uint16_t    ve_info_queue_id;
    uint16_t    ve_info_pf_vf_id;
    uint16_t    ve_info_vm_id;

    uint8_t     acc_type;
    uint8_t     rsv[3];

    uint8_t     bd_code;    /* fix: 0x5a */

    uint8_t     odd_even_0_31 :     1;
    uint8_t     odd_even_32_63 :    1;
    uint8_t     odd_even_64_95 :    1;
    uint8_t     odd_even_96_127 :   1;
    uint8_t     odd_even_128_159 :  1;
    uint8_t     odd_even_160_191 :  1;
    uint8_t     odd_even_192_223 :  1;
    uint8_t     odd_even_224_247 :  1;
    
} acc_rx_bd, acc_tx_bd;
#pragma pack()


typedef struct _acc_rx_queue_ {
    struct acc_adapter    *adapter;
    unsigned int    socket_id;
    uint8_t     port_id;
    uint8_t     queue_id;
    uint16_t    head;
    uint16_t    tail;
    struct rte_mempool  *mp;
    acc_rx_bd   *bd_queue;
    uint64_t    bd_queue_phy;
    uint16_t    bd_queue_nb;
    uint16_t    rx_free_thresh;
    uint8_t     rx_queue_set_flag;
	
	/* rx_mbufs is as rtx_eth_rx_burst's output which is initialized when setup, so we need not allocate	
	 * rte_mbuf every time once a BD comes from SHELL logic, that is used to improve RX performance
	*/
    struct rte_mbuf*   rx_mbufs[DESC_MAX_NUM_EVERY_QUEUE*2];
    uint16_t    rx_mbufs_head;
    uint16_t    rx_mbufs_tail;
    uint16_t    rx_mbufs_nb;
} acc_rx_queue;

typedef struct _acc_tx_queue_ {
    struct acc_adapter    *adapter;
    unsigned int    socket_id;
    uint8_t     port_id;
    uint8_t     queue_id;
    uint16_t    head;
    uint16_t    tail;
    acc_tx_bd   *bd_queue;
    uint64_t    bd_queue_phy;
    uint16_t    bd_queue_nb;
    uint16_t    tx_free_thresh;
    uint8_t     tx_queue_set_flag;
} acc_tx_queue;


#define PMD_ACC_QUEUE_PTR_FORWARD(ptr, step, depth)     do {    \
        (ptr) = ((ptr) + (step)) & ((depth) - 1);               \
    } while(0)

#define ACC_RX_RES_BD_THRESHOLD     (2048)

#endif
