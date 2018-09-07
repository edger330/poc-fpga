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

#ifndef __ACC_DEV_H__
#define __ACC_DEV_H__

#include "acc_hw.h"

/* Vendor ID */
#define ACC_VENDOR_ID             0x19e5

#define ACC_DEV_ID                0x0190
#define ACC_DEV_ID_VF             0x0191
#define ACC_SEC_DEV_ID            0x0170
#define ACC_SEC_DEV_ID_VF         0x0171
#define ACC_THIRD_DEV_ID          0xd502
#define ACC_THIRD_DEV_ID_VF       0xd503  

#define ACC_SUB_VENDOR_ID         0x19e5
#define ACC_SUB_DEVICE_ID         0x2000

#ifndef ACC_PCI_VDEVICE
#define ACC_PCI_VDEVICE(dev)        \
	ACC_VENDOR_ID, (dev),  \
	PCI_ANY_ID, PCI_ANY_ID
#endif

#define ACC_PMD_NAME "rte_acc_pmd" /* PMD name */

struct acc_adapter {
    struct acc_hw  hw;
};

#define ACC_DEV_PRIVATE_TO_HW(adapter)\
    (&(((struct acc_adapter *)adapter)->hw))

uint16_t acc_dev_tx_burst(void *txq, 
    struct rte_mbuf **tx_pkts, uint16_t nb_pkts);

uint16_t acc_dev_rx_burst(void *txq, 
    struct rte_mbuf **tx_pkts, uint16_t nb_pkts);

int  acc_dev_alloc_rx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev);
int  acc_dev_free_rx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev);
int  acc_dev_alloc_tx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev);
int  acc_dev_free_tx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev);
void dump_acc_dev_config(struct rte_eth_dev* acc_dev);
void dump_queues_config(struct rte_eth_dev *acc_dev);
int  check_queue_desc_nb_valid(uint16_t queue_desc_nb);
int  acc_dev_configure(struct rte_eth_dev *acc_dev);
int  acc_dev_rx_queue_setup(
        struct rte_eth_dev *acc_dev,
        uint16_t queue_idx,
        uint16_t nb_desc,
        unsigned int socket_id,
        const struct rte_eth_rxconf *rx_conf,
        struct rte_mempool *mp);
void acc_dev_rx_queue_release(void *rxq);
int  acc_dev_tx_queue_setup(struct rte_eth_dev *acc_dev,
             uint16_t queue_idx,
             uint16_t nb_desc,
             unsigned int socket_id,
             const struct rte_eth_txconf *tx_conf);
void acc_dev_tx_queue_release(void *txq);
int  acc_dev_config_rx_queue(acc_rx_queue *p_rx_queue);
int  acc_dev_config_tx_queue(acc_tx_queue *p_tx_queue);
int  acc_dev_start(struct rte_eth_dev *acc_dev);
void acc_dev_stop(struct rte_eth_dev *acc_dev);
int  acc_dev_link_update(struct rte_eth_dev *acc_dev, int wait_to_complete);
void acc_dev_close(struct rte_eth_dev *acc_dev);
void acc_dev_info_get(struct rte_eth_dev *acc_dev, struct rte_eth_dev_info *dev_info);
int  acc_dev_init(struct rte_eth_dev *acc_dev);
int  acc_dev_uninit(struct rte_eth_dev *acc_dev);
int  rte_pmd_acc_init(const char *name __rte_unused, const char *param __rte_unused);

#endif
