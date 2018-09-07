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

#ifndef __ACC_HW_H__
#define __ACC_HW_H__

#include <stdint.h>
#include "acc_queue.h"

typedef	enum _acc_hw_soc_ip_index_ {
    SOC_IP0_INDEX   =   (0),
    SOC_IP1_INDEX   =   (1),
    SOC_IP2_INDEX   =   (2),    
    SOC_IP3_INDEX   =   (3),
    SOC_IP_RESERVED_INDEX,
}acc_hw_soc_ip_index;

typedef enum _acc_hw_soc_ip_type_ {
    SOC_IP_NONE     =   0,
    SOC_IP0_TYPE    =   (1 << SOC_IP0_INDEX),
    SOC_IP1_TYPE    =   (1 << SOC_IP1_INDEX),
    SOC_IP2_TYPE    =   (1 << SOC_IP2_INDEX),
    SOC_IP3_TYPE    =   (1 << SOC_IP3_INDEX),
}acc_hw_soc_ip_type;

#define IP_MAX_NUM_EVERY_VF         (4)

struct acc_hw {
    uint8_t *hw_res_addr;
    uint64_t hw_res_addr_phys;
    uint64_t hw_res_len;
    /*
     * mcga_soc_ip_type and mcga_soc_ip_index may used to identify different IP, not used at present
    */
    acc_hw_soc_ip_type acc_soc_ip_type;
    acc_hw_soc_ip_index acc_soc_ip_index;

    uint16_t *ptr_tx_container_virt;     /* virtual addr of pointer container */
    uint64_t ptr_tx_container_phy;       /* physical addr of pointer container */

    uint16_t *ptr_rx_container_virt;     /* virtual addr of pointer container */
    uint64_t ptr_rx_container_phy;       /* physical addr of pointer container */

    uint8_t port_id;

    struct rte_pci_addr pci_location;
    struct rte_pci_id   pci_id;
    
    uint16_t rx_queues_max_num;
    uint16_t tx_queues_max_num;
    
    uint16_t rx_queue_desc_max_num;
    uint16_t tx_queue_desc_max_num;
    
    uint16_t rx_queue_desc_min_num;
    uint16_t tx_queue_desc_min_num;
        
    uint8_t rx_soft_loop;
    uint8_t tx_soft_loop;

    struct _acc_rx_queue_    rx_queues[QUEUES_MAX_NUM_EVERY_IP];
    struct _acc_tx_queue_    tx_queues[QUEUES_MAX_NUM_EVERY_IP];
};

#endif