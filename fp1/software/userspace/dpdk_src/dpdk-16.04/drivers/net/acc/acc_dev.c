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

#include <sys/queue.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_cycles.h>

#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_alarm.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_dev.h>
#include <rte_errno.h>

#include "acc_dev.h"
#include "acc_logs.h"
#include "acc_reg.h"
#include "securec.h"
#include "acc_fcntl.h"
/* SW communicates with SHELL logic via BD data, at present, SHELL logic only use 32 bytes BD;
 * for future compatibility, SW define MAX BD len is 1024 bytes. Application should use the 
 * proper rte_mempool* specification to match BD length when call rte_eth_rx_queue_setup
*/
#define ACC_MIN_RX_BUFSIZE          (32)
#define ACC_MAX_RX_PKTLEN           (1024)


#define RX_BD_QUEUE_NAME_PREFIX     "RxBDQ"
#define TX_BD_QUEUE_NAME_PREFIX     "TxBDQ"

#define CHECK_INPUT_PARAM() \
    if (NULL == acc_dev) {\
        PMD_ACC_CRIT("rte_eth_dev is NULL!");\
        return -EINVAL;\
    }\
    if (NULL == acc_dev->data) {\
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");\
        return -EINVAL;\
    }\
    if (NULL == acc_dev->data->dev_private) {\
        PMD_ACC_CRIT("rte_eth_dev->data->dev_private is NULL!");\
        return -EINVAL;\
    }\
    hw = ACC_DEV_PRIVATE_TO_HW(acc_dev->data->dev_private);\
    if (SOC_IP_NONE == hw->acc_soc_ip_type) {\
        PMD_ACC_ERROR("hw->acc_soc_ip_type should not be %u", hw->acc_soc_ip_type);\
        return -EINVAL;\
    }

#define CHECK_INPUT_PARAM_VOID() \
    if (NULL == acc_dev) {\
        PMD_ACC_CRIT("rte_eth_dev is NULL!");\
        return;\
    }\
    if (NULL == acc_dev->data) {\
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");\
        return;\
    }\
    if (NULL == acc_dev->data->dev_private) {\
        PMD_ACC_CRIT("rte_eth_dev->data->dev_private is NULL!");\
        return;\
    }\
    hw = ACC_DEV_PRIVATE_TO_HW(acc_dev->data->dev_private);\
    if (SOC_IP_NONE == hw->acc_soc_ip_type) {\
        PMD_ACC_ERROR("hw->acc_soc_ip_type should not be %u", hw->acc_soc_ip_type);\
        return;\
    }

/* SHELL logic specification: every queue's depth only should be 1024/2048/4096/8192 */
static uint16_t g_s_queue_desc_nb_valid_values[] = {
    DESC_MIN_NUM_EVERY_QUEUE, 
    DESC_MIN_NUM_EVERY_QUEUE*2,
    DESC_MIN_NUM_EVERY_QUEUE*4, 
    DESC_MIN_NUM_EVERY_QUEUE*8
};

/*
 * The set of PCI devices this driver supports (for SHELL VF)
 */
static struct rte_pci_id pci_id_acc_map[] = {
	{ ACC_PCI_VDEVICE(ACC_THIRD_DEV_ID_VF) },
    /* required last entry */
    {0, }
};

static int g_file_id = 0;
static int g_file_flag = 0;
/*****************************************************************************
 Function name      : dump_acc_dev_config
 Description        : print the VF's configuration
 Input parameters   : struct rte_eth_dev *acc_dev
 Output parameters  : None
 Return value       : None
 Function explain   : 
 calling function   : 
 called function    : acc_dev_configure
 
 Modify history     :
    1.Date              : 2017/07/25
      Author            : AI SDK Team
      Content           : New function
*/
void 
dump_acc_dev_config(struct rte_eth_dev* acc_dev) {
    struct rte_eth_dev_data* acc_data = NULL;
    struct acc_hw *hw = NULL;

    if (NULL == acc_dev) {
        PMD_ACC_CRIT("input rte_eth_dev parameter is NULL!");
        return;
    }

    acc_data = acc_dev->data;
    if (NULL == acc_data) {
        PMD_ACC_CRIT("input rte_eth_dev->data parameter is NULL!");
        return;
    }

    if (NULL == acc_data->dev_private) {
        PMD_ACC_CRIT("input rte_eth_dev->data->dev_private parameter is NULL!");
        return;
    }
    
    hw = ACC_DEV_PRIVATE_TO_HW(acc_data->dev_private);
    if (NULL == hw) {
        PMD_ACC_CRIT("acc_hw is NULL!");
        return;
    }

    PMD_ACC_INFO("acc_dev->attached: %u, acc_dev->dev_type: %u (0-UNKNOWN, 1-PCI, 2-VIRTUAL)", 
        acc_dev->attached, acc_dev->dev_type);
    PMD_ACC_INFO("acc_data->names: %s\r\n"
        "acc_data->nb_rx_queues: %u, acc_data->nb_tx_queues: %u\r\n"
        "acc_data->dev_flags: %u\r\n"
        "acc_data->drv_name: %s",
        acc_data->name, acc_data->nb_rx_queues, acc_data->nb_tx_queues, acc_data->dev_flags, acc_data->drv_name);
    PMD_ACC_INFO("acc_data->kdrv: %u(0-UNKNOWN, 1-IGB_UIO, 2-VFIO, 3-UIO_GENERIC, 4-NIC_UIO, 5-NONE)", acc_data->kdrv);

    PMD_ACC_INFO("acc_hw->rx_queues_max_num: %u, acc_hw->tx_queues_max_num: %u", 
        hw->rx_queues_max_num, hw->tx_queues_max_num);
    PMD_ACC_INFO("acc_hw->rx_queue_desc_max_num: %u, acc_hw->tx_queue_desc_max_num: %u\r\n"
        "acc_hw->rx_queue_desc_min_num: %u, acc_hw->tx_queue_desc_min_num: %u", 
        hw->rx_queue_desc_max_num, hw->tx_queue_desc_max_num, hw->rx_queue_desc_min_num, hw->tx_queue_desc_min_num);
    return;
    
}

/*****************************************************************************
 Function name      : check_queue_desc_nb_valid
 Description        : check queue depth configuration whether is valid
 Input parameters   : uint16_t queue_desc_nb: the queue depth value configuring to the RX/TX queue
 Output parameters  : None
 Return value       : Success: 0
                      Fail: -EINVAL
 Function explain   : 
 calling function   : 
 called function    : acc_dev_rx_queue_setup/acc_dev_tx_queue_setup
 
 Modify history     :
  1.Date              : 2017/07/28
    Author            : AI SDK Team
    Content           : New function
*/
int
check_queue_desc_nb_valid(uint16_t queue_desc_nb) {
    uint8_t idx = 0;
    
    for (idx = 0; idx < sizeof(g_s_queue_desc_nb_valid_values)/sizeof(g_s_queue_desc_nb_valid_values[0]); ++idx) {
        if (queue_desc_nb == g_s_queue_desc_nb_valid_values[idx]) {
            return 0;
        }
    }
    return -EINVAL;
}



/*****************************************************************************
 Function name      : acc_dev_configure
 Description        : 
 Input parameters   : struct rte_eth_dev *acc_dev                        
 Output parameters  : None
 Return value       : 0
 Function explain   : NULL function
 calling function   : 
 called function    : rte_eth_dev_configure
 
 Modify history     :
  1.Date              : 2017/07/19
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_configure(struct rte_eth_dev *acc_dev) {
    PMD_ACC_INFO("Configured Virtual Function port id: %d",
             acc_dev->data->port_id);
    dump_acc_dev_config(acc_dev);
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_rx_queue_setup
 Description        : configure RX Queue's parameters, not effect the hardware
 Input parameters   : struct rte_eth_dev *acc_dev
             uint16_t queue_idx : RX queue index
             uint16_t nb_desc : RX queue depth
             unsigned int socket_id : for NUMA, it is NUMA's socket id, for non-NUMA, use SOCKET_ID_ANY
             const struct rte_eth_rxconf *rx_conf : RX Queue's configuration, this parameter is used for NIC, 
                                            it is useless for current device.
             struct rte_mempool *mp : Used as rte_eth_tx_burst's output rte_mbuf's memory pool.
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-ENOMEM
 Function explain   : 
                1. Create the RX BD Queue's space, in which, SHELL logic will fill via DMA
                2. Create the RX output rte_mbuf's space, SW will fill them from RX BD Queue's space
 calling function   : 
 called function    : rte_eth_rx_queue_setup
 
 Modify history     :
  1.Date              : 2017/07/19
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_rx_queue_setup(
        struct rte_eth_dev *acc_dev,
        uint16_t queue_idx,
        uint16_t nb_desc,
        unsigned int socket_id,
        const struct rte_eth_rxconf *rx_conf,
        struct rte_mempool *mp){
        
    char z_name[RTE_MEMZONE_NAMESIZE]={0};
    const struct rte_memzone *mz_bd_queue = NULL;;
    int diag = 0;
    uint16_t real_nb_desc = nb_desc;
    struct acc_hw *hw = NULL;
    acc_rx_queue* p_rx_queue = NULL;
    uint16_t idx = 0;
    
    if (NULL == acc_dev) {
        PMD_ACC_CRIT("rte_eth_dev is NULL!");
        return -EINVAL;
    }

    if (NULL == rx_conf) {
        PMD_ACC_CRIT("rte_eth_rxconf is NULL!");
        return -EINVAL;
    }

    if (NULL == mp) {
        PMD_ACC_CRIT("rte_mempool is NULL!");
        return -EINVAL;
    }

    if (NULL == acc_dev->data) {
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");
        return -EINVAL;
    }
    
    if (NULL == acc_dev->data->dev_private) {
        PMD_ACC_CRIT("rte_eth_dev->data->dev_private is NULL!");
        return -EINVAL;
    }
    
    hw = ACC_DEV_PRIVATE_TO_HW(acc_dev->data->dev_private);    
    if (queue_idx >= hw->rx_queues_max_num) {
        PMD_ACC_CRIT("queue_idx[%u] must less than[%u]", queue_idx, hw->rx_queues_max_num);
        return -EINVAL;
    }
    
    p_rx_queue = (acc_rx_queue*)&hw->rx_queues[queue_idx];
    if (0 != p_rx_queue->rx_queue_set_flag) {
        PMD_ACC_CRIT("queue_idx[%u] has setup ever, please check it", queue_idx);
        return -EINVAL;
    }

    if (0 != check_queue_desc_nb_valid(nb_desc)) {
        PMD_ACC_INFO("queue index %u's nb_desc is not valid, set it to max value %u", 
            queue_idx, hw->rx_queue_desc_max_num);
        real_nb_desc = hw->rx_queue_desc_max_num;
    }
    
    PMD_ACC_INFO("start to setup port: %u([vendorID=0x%x deviceID=0x%x]:[domain=0x%04x bus=0x%02x devid=0x%02x function=0x%02x]) "
            "queue_id[%u], nb_desc[%u], socket_id[%u], rx_conf->rx_free_thresh[%u], mp->names[%s]", 
            acc_dev->data->port_id,
            hw->pci_id.vendor_id, hw->pci_id.device_id,
            hw->pci_location.domain, hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function, 
            queue_idx, nb_desc, socket_id, rx_conf->rx_free_thresh, mp->name);
    
    
    diag = snprintf_s(z_name, sizeof(z_name), sizeof(z_name) - 1, "%s_%u_%u_%u_%u_%u_%u_%u",
        RX_BD_QUEUE_NAME_PREFIX, hw->port_id, 
        hw->pci_location.domain,hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function, 
        hw->acc_soc_ip_index, queue_idx);
    if(-1 == diag) {
        PMD_ACC_ERROR("z_name: %s has some wrong, ", z_name);
        return -EINVAL;
    }
    PMD_ACC_INFO("queue_id[%u] create memzone %s", queue_idx, z_name);
    mz_bd_queue = rte_memzone_reserve_aligned(z_name, sizeof(acc_rx_bd)*real_nb_desc,
        socket_id, 0, PMD_ACC_BD_QUEUE_ALIGN);
    if (NULL == mz_bd_queue) {
        PMD_ACC_ERROR("queue_id[%u] allocate memzone for rx bd queue failed ", queue_idx);
        return -ENOMEM;
    }
    p_rx_queue->bd_queue = mz_bd_queue->addr;
    p_rx_queue->bd_queue_phy = mz_bd_queue->phys_addr;
    p_rx_queue->bd_queue_nb = real_nb_desc;
    p_rx_queue->adapter = (struct acc_adapter*)acc_dev->data->dev_private;
    p_rx_queue->queue_id = queue_idx;
    p_rx_queue->socket_id = socket_id;
    p_rx_queue->port_id = acc_dev->data->port_id;
    p_rx_queue->mp = mp;
    p_rx_queue->head = p_rx_queue->tail = 0;
    p_rx_queue->rx_free_thresh = rx_conf->rx_free_thresh;    
    p_rx_queue->rx_queue_set_flag = 1;
    
    for (idx = 0; idx < sizeof(p_rx_queue->rx_mbufs)/sizeof(struct rte_mbuf*); ++idx) {
        p_rx_queue->rx_mbufs[idx] = NULL;
    }
    diag = rte_pktmbuf_alloc_bulk(p_rx_queue->mp, p_rx_queue->rx_mbufs, DESC_MAX_NUM_EVERY_QUEUE*2);
    if (0 != diag) {
        PMD_ACC_ERROR("rte_pktmbuf_alloc_bulk failed");
        return diag;
    }
    for (idx = 0; idx < sizeof(p_rx_queue->rx_mbufs)/sizeof(struct rte_mbuf*); ++idx) {
        rte_pktmbuf_data_len(p_rx_queue->rx_mbufs[idx]) = rte_pktmbuf_pkt_len(p_rx_queue->rx_mbufs[idx]) = sizeof(acc_rx_bd);
    }
    p_rx_queue->rx_mbufs_head = p_rx_queue->rx_mbufs_tail = 0;
    p_rx_queue->rx_mbufs_nb = DESC_MAX_NUM_EVERY_QUEUE*2;
    
    p_rx_queue->rx_free_thresh = p_rx_queue->bd_queue_nb/2;

    acc_dev->data->rx_queues[queue_idx] = p_rx_queue;
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_rx_queue_release
 Description        : Clear RX Queue's resource
 Input parameters   : acc_rx_queue* 
 Output parameters  : None
 Return value       : None
 Function explain   : 
                1. Clear RX Queue's BD queue resouce
                2. Clear RX Queue's output rte_mbuf resource
 calling function   : 
 called function    : rte_eth_dev_configure, acc_dev_close
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
void
acc_dev_rx_queue_release(void *rxq) {
    acc_rx_queue* p_rx_queue = (acc_rx_queue*)rxq;
    struct acc_hw *hw = NULL;
    uint16_t rx_mbuf_idx = 0;

    if (NULL == p_rx_queue) { 
        PMD_ACC_CRIT("acc_rx_queue is NULL!");
        return;
    }

    if (NULL == p_rx_queue->adapter) {
        PMD_ACC_CRIT("acc_rx_queue->adapter is NULL!");
        return;
    }
    
    hw = &p_rx_queue->adapter->hw;    
    if (NULL != p_rx_queue->bd_queue) {
        char z_name[RTE_MEMZONE_NAMESIZE]={0};
        const struct rte_memzone *mz = NULL;
        int diag = 0;
        diag = snprintf_s(z_name, sizeof(z_name), sizeof(z_name) - 1, "%s_%u_%u_%u_%u_%u_%u_%u",
            RX_BD_QUEUE_NAME_PREFIX, hw->port_id, 
            hw->pci_location.domain,hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function, 
            hw->acc_soc_ip_index, p_rx_queue->queue_id);
        if(-1 == diag) {
            PMD_ACC_ERROR("z_name: %s has some wrong, ", z_name);
            return;
        }
        PMD_ACC_INFO("to find memzone %s", z_name);
        mz = rte_memzone_lookup(z_name);
        if (NULL != mz) {
            diag = rte_memzone_free(mz);
            if (diag != 0) {
                PMD_ACC_INFO("to release port id %u, queue id %u 's memzone %s failed %d", p_rx_queue->port_id, p_rx_queue->queue_id,z_name, diag);
                return;
            }
            p_rx_queue->bd_queue = NULL;
        }
    }

    /* need release rx_mbufs based on a queue */
    for(rx_mbuf_idx=0; rx_mbuf_idx<p_rx_queue->rx_mbufs_nb; ++rx_mbuf_idx) {
        if (NULL != p_rx_queue->rx_mbufs[rx_mbuf_idx]) {
            rte_pktmbuf_free(p_rx_queue->rx_mbufs[rx_mbuf_idx]);
            p_rx_queue->rx_mbufs[rx_mbuf_idx] = NULL;
        } 
    }

    PMD_ACC_INFO("to release port id %u, queue id %u.", p_rx_queue->port_id, p_rx_queue->queue_id);
    
    if (EOK != memset_s((void*)p_rx_queue, sizeof(acc_rx_queue), 0, sizeof(acc_rx_queue))) {
        PMD_ACC_ERROR("memset_s call failed");
    }
    
    return;
}

/*****************************************************************************
 Function name      : acc_dev_tx_queue_setup
 Description        : config TX Queue's parameters, not effect the hardware
 Input parameters   : struct rte_eth_dev *acc_dev
             uint16_t queue_idx : TX queue index
             uint16_t nb_desc : TX queue depth
             unsigned int socket_id : for NUMA, it is NUMA's socket id, for non-NUMA, use SOCKET_ID_ANY
             const struct rte_eth_txconf *tx_conf : TX Queue's configuration, this parameter is used for NIC, 
                                            it is useless for current device.
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-ENOMEM
 Function explain   : 
                1. Create the TX BD Queue's space, in which, SHELL logic will read via DMA
 calling function   : 
 called function    : rte_eth_tx_queue_setup
 
 Modify history     :
  1.Date              : 2017/07/19
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_tx_queue_setup(struct rte_eth_dev *acc_dev,
             uint16_t queue_idx,
             uint16_t nb_desc,
             unsigned int socket_id,
             const struct rte_eth_txconf *tx_conf) {

    struct acc_hw *hw = NULL;
    acc_tx_queue* p_tx_queue = NULL;
    uint16_t real_nb_desc = 0;
    char z_name[RTE_MEMZONE_NAMESIZE]={0};
    const struct rte_memzone *mz_bd_queue = NULL;
    int diag = 0;
    
    if (NULL == acc_dev) {
        PMD_ACC_CRIT("rte_eth_dev is NULL!");
        return -EINVAL;
    }

    if (NULL == tx_conf) {
        PMD_ACC_CRIT("rte_eth_tx_conf is NULL!");
        return -EINVAL;
    }

    if (NULL == acc_dev->data) {
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");
        return -EINVAL;
    }
    
    if (NULL == acc_dev->data->dev_private) {
        PMD_ACC_CRIT("rte_eth_dev->data->dev_private is NULL!");
        return -EINVAL;
    }
    
    hw = ACC_DEV_PRIVATE_TO_HW(acc_dev->data->dev_private);    
    if (queue_idx >= hw->tx_queues_max_num) {
        PMD_ACC_CRIT("queue_idx[%u] must less than[%u]", queue_idx, hw->tx_queues_max_num);
        return -EINVAL;
    }
    
    p_tx_queue = (acc_tx_queue*)&hw->tx_queues[queue_idx];
    if (0 != p_tx_queue->tx_queue_set_flag) {
        PMD_ACC_CRIT("queue_idx[%u] has setup ever, please check it", queue_idx);
        return -EINVAL;
    }
    
    real_nb_desc = nb_desc;
    if (0 != check_queue_desc_nb_valid(nb_desc)) {
        PMD_ACC_INFO("queue index %u's nb_desc is not valid, set it to max value %u", 
            queue_idx, hw->tx_queue_desc_max_num);
        real_nb_desc = hw->tx_queue_desc_max_num;
    }
    
    PMD_ACC_INFO("start to setup port: %u([vendorID=0x%x deviceID=0x%x]:[domain=0x%04x bus=0x%02x devid=0x%02x function=0x%02x]) "
            "queue_id[%u], nb_desc[%u], socket_id[%u], tx_conf->tx_free_thresh[%u]", 
            acc_dev->data->port_id,
            hw->pci_id.vendor_id, hw->pci_id.device_id,
            hw->pci_location.domain, hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function, 
            queue_idx, nb_desc, socket_id, tx_conf->tx_free_thresh);
    
    diag = snprintf_s(z_name, sizeof(z_name), sizeof(z_name) - 1, "%s_%u_%u_%u_%u_%u_%u_%u",
        TX_BD_QUEUE_NAME_PREFIX, hw->port_id, 
        hw->pci_location.domain,hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function, 
        hw->acc_soc_ip_index, queue_idx);
    if(-1 == diag) {
        PMD_ACC_ERROR("z_name: %s has some wrong, ", z_name);
        return -EINVAL;
    }
    PMD_ACC_INFO("queue_id[%u] create memzone %s", queue_idx, z_name);
    mz_bd_queue = rte_memzone_reserve_aligned(z_name, sizeof(acc_tx_bd)*real_nb_desc,
        socket_id, 0, PMD_ACC_BD_QUEUE_ALIGN);
    if (NULL == mz_bd_queue) {
        PMD_ACC_ERROR("queue_id[%u] allocate memzone for tx bd queue failed, rte_errno %d:%s", queue_idx, rte_errno, rte_strerror(rte_errno));
        return -ENOMEM;
    }
    p_tx_queue->bd_queue = mz_bd_queue->addr;
    p_tx_queue->bd_queue_phy = mz_bd_queue->phys_addr;
    p_tx_queue->bd_queue_nb = real_nb_desc;
    p_tx_queue->adapter = (struct acc_adapter*)acc_dev->data->dev_private;
    p_tx_queue->queue_id = queue_idx;
    p_tx_queue->socket_id = socket_id;
    p_tx_queue->port_id = acc_dev->data->port_id;
    p_tx_queue->head = p_tx_queue->tail = 0;
    p_tx_queue->tx_free_thresh = tx_conf->tx_free_thresh;    
    p_tx_queue->tx_queue_set_flag = 1;

    p_tx_queue->tx_free_thresh = p_tx_queue->bd_queue_nb/4;

    acc_dev->data->tx_queues[queue_idx] = p_tx_queue;
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_rx_queue_release
 Description        : Clear TX Queue's resource
 Input parameters   : acc_tx_queue* 
 Output parameters  : None
 Return value       : None
 Function explain   : 
                1. Clear TX Queue's BD queue resouce
 calling function   : 
 called function    : rte_eth_dev_configure, acc_dev_close
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
void 
acc_dev_tx_queue_release(void *txq) {
    acc_tx_queue* p_tx_queue = (acc_tx_queue*)txq;
    struct acc_hw *hw = NULL;  
    
    if (NULL == p_tx_queue) { 
        PMD_ACC_CRIT("acc_tx_queue is NULL!");
        return;
    }
    
    if (NULL == p_tx_queue->adapter) {
        PMD_ACC_CRIT("acc_tx_queue->adapter is NULL!");
        return;
    }
    
    hw = &p_tx_queue->adapter->hw;             
    if (NULL != p_tx_queue->bd_queue) {
        char z_name[RTE_MEMZONE_NAMESIZE]={0};
        const struct rte_memzone *mz;
        int diag;
        diag = snprintf_s(z_name, sizeof(z_name), sizeof(z_name) - 1, "%s_%u_%u_%u_%u_%u_%u_%u",
            TX_BD_QUEUE_NAME_PREFIX, hw->port_id, 
            hw->pci_location.domain,hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function, 
            hw->acc_soc_ip_index, p_tx_queue->queue_id);
        if(-1 == diag) {
            PMD_ACC_ERROR("z_name: %s has some wrong, ", z_name);
            return;
        }
        PMD_ACC_INFO("to find memzone %s", z_name);
        mz = rte_memzone_lookup(z_name);
        if (NULL != mz) {
            diag = rte_memzone_free(mz);
            if (diag != 0) {
                PMD_ACC_INFO("to release port id %u, queue id %u 's memzone %s failed %d", p_tx_queue->port_id, p_tx_queue->queue_id,z_name, diag);
                return;
            }
            p_tx_queue->bd_queue = NULL;
        }
    }

    PMD_ACC_INFO("to release port id %u, queue id %u.", p_tx_queue->port_id, p_tx_queue->queue_id);
    
    if (EOK != memset_s((void*)p_tx_queue, sizeof(acc_tx_queue), 0, sizeof(acc_tx_queue))) {
        PMD_ACC_ERROR("memset_s call failed");
    }

    return;
}

/*****************************************************************************
 Function name      : acc_dev_config_rx_queue
 Description        : Write the configuration to SHELL logic's registers, effect the SHELL logic's TX registers
 Input parameters   : acc_rx_queue *
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-EIO
 Function explain   : 
                1. Write RX BD Queue's physical address to RXQM's BD base addreses register
                2. Write RX BD Queue's depth to the RXQM's BD depth register
                3. Write 0 to RXQM's ring register, means current RX Queue has no space to receive BD
 calling function   : 
 called function    : acc_dev_start
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_config_rx_queue(acc_rx_queue *p_rx_queue) {
    struct acc_hw *hw = NULL;
    uint16_t reset_max_time = 200;
    
    if (NULL == p_rx_queue) {
        PMD_ACC_CRIT("acc_rx_queue is NULL!");
        return -EINVAL;
    }

    if (NULL == p_rx_queue->adapter) {
        PMD_ACC_CRIT("acc_rx_queue->adapter is NULL!");
        return -EINVAL;
    }
    
    hw = &p_rx_queue->adapter->hw;
    if (SOC_IP_NONE == hw->acc_soc_ip_type) {
        PMD_ACC_ERROR("hw->acc_soc_ip_type should not be %u", hw->acc_soc_ip_type);
        return -EINVAL;
    }
    
    /* Reset RXQM */
    ACC_RXQM_CONF_QUEUE_RESET(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id);
    do {
        rte_delay_ms(10);
        if (ACC_RXQM_CHECK_QUEUE_RST_CLR(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id)) {
            break;
        }
    } while (0 != (--reset_max_time));
    if (0 == reset_max_time) {
        PMD_ACC_ERROR("HW rx queue %d reset overtime!", p_rx_queue->queue_id);
        return -EIO;
    }

    /* Config RXQM: Write RX BD Queue's physical address to RXQM's BD base addreses register */
    ACC_RXQM_CONF_RES_BDQ_BASE(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id, p_rx_queue->bd_queue_phy); //lint !e572
    rte_delay_ms(10);
    
    /* Config RXQM: Write RX BD Queue's depth to the RXQM's BD depth register */
    ACC_RXQM_CONF_BDQ_DEPTH(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id, p_rx_queue->bd_queue_nb);
    rte_delay_ms(1);
    
    /* Config RXQM: Write 0 to RX BD Queue's ring register, means current RX Queue has no space to receive BD */
    ACC_RXQM_CONF_BDQ_RING(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id, p_rx_queue->head);
    rte_delay_ms(1);
    
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_config_tx_queue
 Description        : Write the configuration to SHELL logic's registers, effect the SHELL logic's RX registers
 Input parameters   : acc_tx_queue *
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-EIO
 Function explain   :
                1. Write TX BD Queue's physical address to TXQM's BD base addreses register
                2. Write TX BD Queue's depth to the TXQM's BD depth register
                3. Write 0 to TXQM's ring register, means current TX Queue has no BD sending to SHELL logic
 calling function   : 
 called function    : acc_dev_start
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_config_tx_queue(acc_tx_queue *p_tx_queue) {
    struct acc_hw *hw = NULL;
    uint16_t reset_max_time = 200;
    
    if (NULL == p_tx_queue) {
        PMD_ACC_CRIT("acc_tx_queue is NULL!");
        return -EINVAL;
    }

    if (NULL == p_tx_queue->adapter) {
        PMD_ACC_CRIT("p_tx_queue->adapter is NULL!");
        return -EINVAL;
    }

    hw = &p_tx_queue->adapter->hw;
    if (SOC_IP_NONE == hw->acc_soc_ip_type) {
        PMD_ACC_ERROR("hw->acc_soc_ip_type should not be %u", hw->acc_soc_ip_type);
        return -EINVAL;
    }

    /* Reset TXQM */
    ACC_TXQM_CONF_QUEUE_RESET(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id);
    do {
        rte_delay_ms(10);
        if (ACC_TXQM_CHECK_QUEUE_RST_CLR(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id)) {
            break;
        }
    } while (0 != (--reset_max_time));
    if (0 == reset_max_time) {
        PMD_ACC_ERROR("HW tx queue %d reset overtime!", p_tx_queue->queue_id);
        return -EIO;
    }
    
    /* Config TXQM: Write TX BD Queue's physical address to TXQM's BD base addreses register */
    ACC_TXQM_CONF_BDQ_BASE(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id, p_tx_queue->bd_queue_phy); //lint !e572
    rte_delay_ms(1);
    
    /* Config TXQM: Write TX BD Queue's depth to the TXQM's BD depth register */
    ACC_TXQM_CONF_BDQ_DEPTH(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id, p_tx_queue->bd_queue_nb);
    rte_delay_ms(1);

    /* Config TXQM: Write 0 to TXQM's ring register, means current TX Queue has no BD sending to SHELL logic */
    ACC_TXQM_CONF_BDQ_RING(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id, p_tx_queue->head);
    rte_delay_ms(1);
    
    return 0;
}


/*****************************************************************************
 Function name      : acc_dev_alloc_rx_queues_ptr_write_addr
 Description        : allocate and config RX Queue's logic write-back space, effect the SHELL logic register
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-ENOMEM/-EFAULT
 Function explain   : Allocate SHELL logic's RX write-back pointers space, and write the space's physical address to the SHELL
                  logic's RX write-back registers. The RX write-back pointer space is belong to VF but not a special RX queue,
                  so it must only config it once. Every RX queue's write-back pointer space has 2 bytes, and iterates its 
                  write-back pointer offset according to its queue index.
                      SHELL logic will write a value in a queue's responding write-back pointer space after the SHELL logic
                  write the RX BD to the RX BD Queue, which means that the current rx BD's position information in the RX 
                  BD Queue.
                        
 calling function   : 
 called function    : acc_dev_start
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/

int
acc_dev_alloc_rx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;
    uint16_t queue_idx = 0;
	
    CHECK_INPUT_PARAM();
   
    if (NULL != hw->ptr_rx_container_virt) {
        PMD_ACC_INFO("ptr write addr space has allocated, free it first!");
        rte_free(hw->ptr_rx_container_virt);
        hw->ptr_rx_container_virt = NULL;
        hw->ptr_rx_container_phy = 0;
    }
    
    hw->ptr_rx_container_virt = rte_malloc("", sizeof(uint16_t) * hw->rx_queues_max_num * 2, 
        PMD_ACC_BD_PTR_WRITE_ALIGN);
    if (NULL == hw->ptr_rx_container_virt) {
        PMD_ACC_CRIT("allocate ptr write addr space failed!");
        hw->ptr_rx_container_phy = 0;
        return -ENOMEM;
    }

    hw->ptr_rx_container_phy = rte_mem_virt2phy(hw->ptr_rx_container_virt);
    if (unlikely(RTE_BAD_PHYS_ADDR == hw->ptr_rx_container_phy)) {
        PMD_ACC_CRIT("fail to get physical addr of mapped virtual addr!");
        rte_free(hw->ptr_rx_container_virt);
        hw->ptr_rx_container_virt = NULL;
        hw->ptr_rx_container_phy = 0;
        return -EFAULT;
    }

    if (EOK != memset_s(hw->ptr_rx_container_virt, sizeof(uint16_t) * hw->rx_queues_max_num * 2, 
        0, sizeof(uint16_t) * hw->rx_queues_max_num * 2)) {
        PMD_ACC_ERROR("memset_s call failed");
        rte_free(hw->ptr_rx_container_virt);
        hw->ptr_rx_container_virt = NULL;
        hw->ptr_rx_container_phy = 0;
        return -EFAULT;
    }
    
    ACC_RXQM_CONF_PTR_WB_BD_ADDR(hw, hw->acc_soc_ip_index, hw->ptr_rx_container_phy); //lint !e572
    rte_delay_ms(1);

    for (queue_idx = 0; queue_idx < hw->rx_queues_max_num; ++queue_idx) {
        PMD_ACC_INFO("After config RX write PTR: queue %u Write-Back pointer: %u, release_ptr_value: %u",
            queue_idx, hw->ptr_rx_container_virt[queue_idx], 
            ACC_RXQM_GET_RELEASE_PTR(hw, hw->acc_soc_ip_index, queue_idx));
    }

    return 0;
}


/*****************************************************************************
 Function name      : acc_dev_free_rx_queues_ptr_write_addr
 Description        : release the VF's RX write-back pointers space, effect the SHELL logic resigster
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL
 Function explain   : 
                    1. release the VF's RX write-back pointers space
                    2. write 0 to the VF's RX write-back pointers register
 calling function   : 
 called function    : acc_dev_start, acc_dev_stop
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_free_rx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;
    CHECK_INPUT_PARAM();

    if (NULL != hw->ptr_rx_container_virt) {
        PMD_ACC_INFO("ptr write addr space has allocated, free it");
        rte_free(hw->ptr_rx_container_virt);
        hw->ptr_rx_container_virt = NULL;
        hw->ptr_rx_container_phy = 0;
    }

    ACC_RXQM_CONF_PTR_WB_BD_ADDR(hw, hw->acc_soc_ip_index, (uint64_t)0x0); //lint !e572
    rte_delay_ms(1);

    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_alloc_tx_queues_ptr_write_addr
 Description        : allocate and config TX Queue's logic write-back space, effect the SHELL logic register
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL/-ENOMEM/-EFAULT
 Function explain   : Allocate SHELL logic's TX write-back pointers space, and write the space's physical address to the SHELL
                  logic's TX write-back registers. The TX write-back pointer space is belong to VF but not a special TX queue,
                  so it must only config it once. Every TX queue's write-back pointer space has 2 bytes, and iterates its 
                  write-back pointer offset according to its queue index.
                      SHELL logic will write a value in a queue's responding write-back pointer space after the SHELL logic
                  read the TX BD from the TX BD Queue, which means that the current tx BD's position information in the TX 
                  BD Queue.
 calling function   : 
 called function    : acc_dev_start
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_alloc_tx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;

    CHECK_INPUT_PARAM();
    
    if (NULL != hw->ptr_tx_container_virt) {
        PMD_ACC_INFO("ptr write addr space has allocated, free it first!");
        rte_free(hw->ptr_tx_container_virt);
        hw->ptr_tx_container_virt = NULL;
        hw->ptr_tx_container_phy = 0;
    }
    hw->ptr_tx_container_virt = rte_malloc("", sizeof(uint16_t) * hw->tx_queues_max_num * 2, 
        PMD_ACC_BD_PTR_WRITE_ALIGN);
    if (NULL == hw->ptr_tx_container_virt) {
        PMD_ACC_CRIT("allocate ptr write addr space failed!");
        hw->ptr_tx_container_phy = 0;
        return -ENOMEM;
    }

    hw->ptr_tx_container_phy = rte_mem_virt2phy(hw->ptr_tx_container_virt);
    if (unlikely(RTE_BAD_PHYS_ADDR == hw->ptr_tx_container_phy)) {
        PMD_ACC_CRIT("Failed to get physical addr of mapped virtual addr!");
        rte_free(hw->ptr_tx_container_virt);
        hw->ptr_tx_container_virt = NULL;
        hw->ptr_tx_container_phy = 0;
        return -EFAULT;
    }

    if (EOK != memset_s(hw->ptr_tx_container_virt, sizeof(uint16_t) * hw->tx_queues_max_num * 2, 
        0, sizeof(uint16_t) * hw->tx_queues_max_num * 2)) {
        PMD_ACC_ERROR("memset_s call failed");
        rte_free(hw->ptr_tx_container_virt);
        hw->ptr_tx_container_virt = NULL;
        hw->ptr_tx_container_phy = 0;
        return -EFAULT;
    }
    
    ACC_TXQM_CONF_PTR_WB_BD_ADDR(hw, hw->acc_soc_ip_index, hw->ptr_tx_container_phy); //lint !e572
    rte_delay_ms(1);
    
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_free_tx_queues_ptr_write_addr
 Description        : release the VF's TX write-back pointers space, effect the SHELL logic resigster
 Input parameters   : struct rte_eth_dev *acc_dev : device data struct of VF
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL
 Function explain   : 
                    1. release the VF's TX write-back pointers space
                    2. write 0 to the VF's TX write-back pointers register
 calling function   : 
 called function    : acc_dev_start, acc_dev_stop
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_free_tx_queues_ptr_write_addr(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;

    CHECK_INPUT_PARAM();

    if (NULL != hw->ptr_tx_container_virt) {
        PMD_ACC_INFO("ptr write addr space has allocated, free it");
        rte_free(hw->ptr_tx_container_virt);
        hw->ptr_tx_container_virt = NULL;
        hw->ptr_tx_container_phy = 0;
    }

    ACC_TXQM_CONF_PTR_WB_BD_ADDR(hw, hw->acc_soc_ip_index, (uint64_t)0x0); //lint !e572
    rte_delay_ms(1);

    return 0;
}

/*****************************************************************************
 Function name      : dump_queues_config
 Description        : print VF every TX/RX's queue configuration parameters
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : None
 Function explain   : 
 calling function   : 
 called function    : acc_dev_start
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
void 
dump_queues_config(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;

    CHECK_INPUT_PARAM_VOID();
    
    acc_rx_queue *p_rx_queue = NULL;
    acc_tx_queue *p_tx_queue = NULL;
    uint8_t queue_idx = 0;

    for (queue_idx = 0; queue_idx < acc_dev->data->nb_rx_queues; ++queue_idx) {
        p_rx_queue = (acc_rx_queue*)acc_dev->data->rx_queues[queue_idx];
        if ((NULL != p_rx_queue) && p_rx_queue->rx_queue_set_flag) {
            PMD_ACC_INFO("rx queue_idx %u bd_queue=%p, bd_queue_phy=0x%lx, "
                "bd_queue_nb=%u", 
                p_rx_queue->queue_id, p_rx_queue->bd_queue, p_rx_queue->bd_queue_phy,
                p_rx_queue->bd_queue_nb);
        }
    }

    PMD_ACC_INFO("ptr_rx_container_virt=%p, ptr_rx_container_phy=0x%lx", 
        hw->ptr_rx_container_virt, hw->ptr_rx_container_phy);

    for (queue_idx = 0; queue_idx < acc_dev->data->nb_tx_queues; ++queue_idx) {
        p_tx_queue = (acc_tx_queue*)acc_dev->data->tx_queues[queue_idx];
        if ((NULL != p_tx_queue) && p_tx_queue->tx_queue_set_flag) {
            PMD_ACC_INFO("tx queue_idx %u bd_queue=%p, bd_queue_phy=0x%lx, "
                "bd_queue_nb=%u", 
                p_tx_queue->queue_id, p_tx_queue->bd_queue, p_tx_queue->bd_queue_phy,
                p_tx_queue->bd_queue_nb);
        }
    }

    PMD_ACC_INFO("ptr_tx_container_virt=%p, ptr_tx_container_phy=0x%lx", 
        hw->ptr_tx_container_virt, hw->ptr_tx_container_phy);

    for (queue_idx = 0; queue_idx < hw->rx_queues_max_num; ++queue_idx) {
        PMD_ACC_INFO("After enable queue %u Write-Back pointer: %u, release_ptr_value=%u", 
            queue_idx, hw->ptr_rx_container_virt[queue_idx], 
            ACC_RXQM_GET_RELEASE_PTR(hw, hw->acc_soc_ip_index, queue_idx));
    }
}

/*****************************************************************************
 Function name      : acc_dev_start
 Description        : Write VF's every queue's configuration parameters to registers, effect the SHELL logic
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL
 Function explain   : This function enable VF's every queue, and write the queues' configuration to SHELL logic registers, step is as follow
             1. disable VF's every TX/RX queue 
             2. config VF's TX/RX write-back pointers space registers
             3. config VF's TX/RX queue's paramters (BD base address, BD queue depth, ring value and so on)
             4. enable VF's every TX/RX queue 
 calling function   : acc_dev_alloc_rx_queues_ptr_write_addr/acc_dev_alloc_tx_queues_ptr_write_addr, 
             acc_dev_config_rx_queue/acc_dev_config_tx_queue
 called function    : rte_eth_dev_start
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int 
acc_dev_start(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;
    acc_rx_queue *p_rx_queue = NULL;
    acc_tx_queue *p_tx_queue = NULL;
    uint8_t queue_idx = 0;
    int ret = 0;

    CHECK_INPUT_PARAM();
    
    /* Disable ISO area */
    /* Must disable ISO area in Host First */
    
    /* Disable All TX/RX Queue */
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_rx_queues; ++queue_idx) {
        p_rx_queue = (acc_rx_queue*)acc_dev->data->rx_queues[queue_idx];
        if ((NULL != p_rx_queue) && p_rx_queue->rx_queue_set_flag) {
            /* Stop RXQM*/
            ACC_RXQM_CONF_QUEUE_STOP(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id);
            rte_delay_ms(1);
        }
    }
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_tx_queues; ++queue_idx) {
        p_tx_queue = &hw->tx_queues[queue_idx];
        if ((NULL != p_tx_queue) && p_tx_queue->tx_queue_set_flag) {
            /* Stop TXQM*/
            ACC_TXQM_CONF_QUEUE_STOP(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id);
            rte_delay_ms(1);
        }
    }
    
    /* config RX/TX's write-back pointer space addr */
    ret = acc_dev_alloc_rx_queues_ptr_write_addr(acc_dev);
    if (0 != ret) {
        PMD_ACC_CRIT("acc_dev_alloc_rx_queues_ptr_write_addr failed");
        return ret;
    }

    ret = acc_dev_alloc_tx_queues_ptr_write_addr(acc_dev);
    if (0 != ret) {
        PMD_ACC_CRIT("acc_dev_alloc_tx_queues_ptr_write_addr failed");
        if (0 != acc_dev_free_rx_queues_ptr_write_addr(acc_dev)){
            PMD_ACC_CRIT("acc_dev_free_rx_queues_ptr_write_addr failed");
        }
        return ret;
    }
    
    /* Config All TX/RX BD registers */
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_rx_queues; ++queue_idx) {
        p_rx_queue = (acc_rx_queue*)acc_dev->data->rx_queues[queue_idx];
        if ((NULL != p_rx_queue) && p_rx_queue->rx_queue_set_flag) {
            if (0 == acc_dev_config_rx_queue(p_rx_queue)) {
                PMD_ACC_INFO("port: %u rx queue_idx: %u started success", acc_dev->data->port_id, p_rx_queue->queue_id);
            } else {
                PMD_ACC_INFO("port: %u rx queue_idx: %u started failed, release it", acc_dev->data->port_id, p_rx_queue->queue_id);
                acc_dev_rx_queue_release((void*)p_rx_queue);
            }
        }
    }
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_tx_queues; ++queue_idx) {
        p_tx_queue = (acc_tx_queue*)acc_dev->data->tx_queues[queue_idx];
        if ((NULL != p_tx_queue) && p_tx_queue->tx_queue_set_flag) {
            if (0 == acc_dev_config_tx_queue(p_tx_queue)) {
                PMD_ACC_INFO("port %u tx queue_idx: %u started success", acc_dev->data->port_id, p_tx_queue->queue_id);
            } else {
                PMD_ACC_ERROR("port %u tx queue_idx: %u started failed, release it", acc_dev->data->port_id, p_tx_queue->queue_id);
                acc_dev_tx_queue_release((void*)p_tx_queue);
            }
        }
    }

    /* Enable All TX/RX Queue */
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_rx_queues; ++queue_idx) {
        p_rx_queue = (acc_rx_queue*)acc_dev->data->rx_queues[queue_idx];
        if ((NULL != p_rx_queue) && p_rx_queue->rx_queue_set_flag) {
            /* Start RXQM*/
            ACC_RXQM_CONF_QUEUE_START(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id);
            rte_delay_ms(1);
        }
    }
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_tx_queues; ++queue_idx) {
        p_tx_queue = (acc_tx_queue*)acc_dev->data->tx_queues[queue_idx];
        if ((NULL != p_tx_queue) && p_tx_queue->tx_queue_set_flag) {
            /* Start TXQM*/
            ACC_TXQM_CONF_QUEUE_START(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id);
            rte_delay_ms(1);
        }
    }

    dump_queues_config(acc_dev);
    
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_stop
 Description        : disable VF's every TX/RX queue
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : None
 Function explain   : 
             1. disaable the VF's every TX/RX queue
             2. release the VF's logic write-back pointers space.
 calling function   : acc_dev_free_rx_queues_ptr_write_addr/acc_dev_free_tx_queues_ptr_write_addr
 called function    : rte_eth_dev_stop
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
void
acc_dev_stop(struct rte_eth_dev *acc_dev) {
    struct acc_hw *hw = NULL;
    acc_rx_queue *p_rx_queue = NULL;
    acc_tx_queue *p_tx_queue = NULL;
    uint8_t queue_idx = 0;

    CHECK_INPUT_PARAM_VOID();
        
    /* Disable All TX/RX Queue */
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_tx_queues; ++queue_idx) {
        p_tx_queue = (acc_tx_queue*)acc_dev->data->tx_queues[queue_idx];
        if ((NULL != p_tx_queue) && p_tx_queue->tx_queue_set_flag) {
            /* Stop TXQM*/
            PMD_ACC_INFO("port %u tx queue_idx: %u to disable.", acc_dev->data->port_id, p_tx_queue->queue_id);
            ACC_TXQM_CONF_QUEUE_STOP(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id);
            rte_delay_ms(1);
        }

        p_rx_queue = (acc_rx_queue*)acc_dev->data->rx_queues[queue_idx];
        if ((NULL != p_rx_queue) && p_rx_queue->rx_queue_set_flag) {
            /* Stop RXQM*/
            PMD_ACC_INFO("port %u rx queue_idx: %u to disable.", acc_dev->data->port_id, p_rx_queue->queue_id);
            ACC_RXQM_CONF_QUEUE_STOP(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id);
            rte_delay_ms(1);
        }
    }

    if (0 != acc_dev_free_rx_queues_ptr_write_addr(acc_dev)) {
        PMD_ACC_INFO("acc_dev_free_rx_queues_ptr_write_addr failed.");
    }
    if (0 != acc_dev_free_tx_queues_ptr_write_addr(acc_dev)) {
        PMD_ACC_INFO("acc_dev_free_tx_queues_ptr_write_addr failed.");
    }

    /* from dpdk's eth_em_stop, here could config hardware's register, to close
     * rx/tx interruption, and in uninit interface, to call rte_intr_disable to 
     * close uio interrption
     */
    return;
}

/* return 0 means link status changed, -1 means not changed */
int
acc_dev_link_update(struct rte_eth_dev *acc_dev, int wait_to_complete) {
    if (NULL == acc_dev) {
        PMD_ACC_CRIT("rte_eth_dev is NULL!");
        return -EINVAL;
    }

    if (NULL == acc_dev->data) {
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");
        return -EINVAL;
    }
    
    acc_dev->data->dev_link.link_duplex = ETH_LINK_FULL_DUPLEX;
    /* We don't use this paramter now, to aviod compile and pclint error
        , we do useless operations on this paramter*/
    (void)wait_to_complete;
    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_close
 Description        : release VF every TX/RX queue's resource
 Input parameters   : struct rte_eth_dev *acc_dev : device data struct of VF
 Output parameters  : None
 Return value       : None
 Function explain   : 
 calling function   : acc_dev_rx_queue_release/acc_dev_tx_queue_release
 called function    : rte_eth_dev_close
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
void
acc_dev_close(struct rte_eth_dev *acc_dev) {
    uint8_t queue_idx = 0;
    acc_rx_queue* p_rx_queue = NULL;
    acc_tx_queue* p_tx_queue = NULL;
    
    if (NULL == acc_dev) {
        PMD_ACC_CRIT("rte_eth_dev is NULL!");
        return;
    }

    if (NULL == acc_dev->data) {
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");
        return;
    }

    if (NULL == acc_dev->dev_ops) {
        PMD_ACC_CRIT("rte_eth_dev->dev_ops is NULL!");
        return;
    }
    
    PMD_ACC_INFO("start to close port %u", acc_dev->data->port_id);
    
    RTE_FUNC_PTR_OR_RET(*acc_dev->dev_ops->dev_stop);
    (*acc_dev->dev_ops->dev_stop)(acc_dev);

    RTE_FUNC_PTR_OR_RET(*acc_dev->dev_ops->tx_queue_release);
    RTE_FUNC_PTR_OR_RET(*acc_dev->dev_ops->rx_queue_release);
    
    for (queue_idx = 0; queue_idx < acc_dev->data->nb_tx_queues; ++queue_idx) {
        p_tx_queue = (acc_tx_queue*)acc_dev->data->tx_queues[queue_idx];
        if ((NULL != p_tx_queue) && (p_tx_queue->tx_queue_set_flag)) {
            (*acc_dev->dev_ops->tx_queue_release)(p_tx_queue);
            acc_dev->data->tx_queues[queue_idx] = NULL;
        }
    }
    acc_dev->data->nb_tx_queues = 0;

    for (queue_idx = 0; queue_idx < acc_dev->data->nb_rx_queues; ++queue_idx) {
        p_rx_queue = (acc_rx_queue*)acc_dev->data->rx_queues[queue_idx];
        if ((NULL != p_rx_queue) && (p_rx_queue->rx_queue_set_flag)) {
            (*acc_dev->dev_ops->rx_queue_release)(p_rx_queue);
            acc_dev->data->rx_queues[queue_idx] = NULL;
        }
    }
    acc_dev->data->nb_rx_queues = 0;
       
    return;
}

/*****************************************************************************
 Function name      : acc_dev_info_get
 Description        : Get default TX/Rx queue's depth
 Input parameters   : struct rte_eth_dev *acc_dev
 Output parameters  : struct rte_eth_dev_info *dev_info
 Return value       : None
 Function explain   : 
 calling function   : 
 called function    : rte_eth_dev_config
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
void
acc_dev_info_get(struct rte_eth_dev *acc_dev, struct rte_eth_dev_info *dev_info) {
    struct acc_hw *hw = NULL;

    if (NULL == acc_dev || NULL == dev_info) {
        PMD_ACC_CRIT("rte_eth_dev is NULL or rte_eth_dev_info is NULL!");
        return;
    }

    if (NULL == acc_dev->data) {
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");
        return;
    }

    if (NULL == acc_dev->data->dev_private) {
        PMD_ACC_CRIT("rte_eth_dev->data->dev_private is NULL!");
        return;
    }

    if (NULL == acc_dev->pci_dev) {
        PMD_ACC_CRIT("rte_eth_dev->pci_dev is NULL!");
        return;
    }

    hw = ACC_DEV_PRIVATE_TO_HW(acc_dev->data->dev_private);

    dev_info->max_rx_queues  = (uint16_t)hw->rx_queues_max_num;
    dev_info->max_tx_queues  = (uint16_t)hw->tx_queues_max_num;

    dev_info->min_rx_bufsize = ACC_MIN_RX_BUFSIZE;
    dev_info->max_rx_pktlen  = ACC_MAX_RX_PKTLEN;
    dev_info->max_vfs        = acc_dev->pci_dev->max_vfs;

    return;
}

/*****************************************************************************
 Variable Name      : acc_dev_ops
 Description        : Callback functions set of DPDK standard interface(TX/RX callback not included)
 called function    : acc_dev_init
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
struct eth_dev_ops acc_dev_ops = {
    .dev_configure        = acc_dev_configure,
    .rx_queue_setup       = acc_dev_rx_queue_setup,
    .rx_queue_release     = acc_dev_rx_queue_release,
    .tx_queue_setup       = acc_dev_tx_queue_setup,
    .tx_queue_release     = acc_dev_tx_queue_release,
    .dev_start            = acc_dev_start,
    .dev_stop             = acc_dev_stop,
    .link_update          = acc_dev_link_update,
    /*
    .stats_get            = acc_dev_stats_get,
    .stats_reset          = acc_dev_stats_reset,
    .xstats_get           = mcga_dev_xstats_get,
    */
    .dev_close            = acc_dev_close,
    .dev_infos_get        = acc_dev_info_get,
    /*
    .mtu_set              = mcga_dev_mtu_set,
    .mac_addr_set         = mcga_dev_mac_addr_set,
    */
};


/*****************************************************************************
 Function name      : acc_dev_init
 Description        : initilize the system and process's global variables
 Input parameters   : struct rte_eth_dev *
 Output parameters  : None
 Return value       : Success : 0
 Function explain   : 
             for PRIMARY process, do following:
             1. initilize system's global variables, struct acc_hw data structure, including VF's BAR space accessing interface
             2. initilize process's global variables, struct rte_eth_dev data structure, including configuration call back interface, TX/RX call back interface
 calling function   : 
 called function    : rte_eal_init
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_init(struct rte_eth_dev *acc_dev) {  
    struct rte_pci_device *pci_dev = NULL;
    struct acc_hw *hw = NULL;
    int ret = 0;

    if (NULL == acc_dev) {
        PMD_ACC_CRIT("rte_eth_dev is NULL!");
        return -EINVAL;
    }
    if (NULL == acc_dev->data) {
        PMD_ACC_CRIT("rte_eth_dev->data is NULL!");
        return -EINVAL;
    }
    if (NULL == acc_dev->data->dev_private) {
        PMD_ACC_CRIT("rte_eth_dev->data->dev_private is NULL!");
        return -EINVAL;
    }
    hw = ACC_DEV_PRIVATE_TO_HW(acc_dev->data->dev_private);
    
    acc_dev->dev_ops = &acc_dev_ops;
    acc_dev->rx_pkt_burst = (eth_rx_burst_t)acc_dev_rx_burst;
    acc_dev->tx_pkt_burst = (eth_tx_burst_t)acc_dev_tx_burst;

    /* Lock the Read file Lock */
    if(0 == g_file_flag) {
        ret = acc_fcntl_read_lock(0, &g_file_id);
        if(0 != ret) {
            PMD_ACC_ERROR("acc_fcntl_read_lock error, ret %d", ret);
            return ret;
        }
        g_file_flag = 1;
    }
    
    if (rte_eal_process_type() != RTE_PROC_PRIMARY) {
        PMD_ACC_INFO("This is Secondary process, skipping config acc_dev->data->dev_private");
        return 0;
    }

    pci_dev = acc_dev->pci_dev;

    rte_eth_copy_pci_info(acc_dev, pci_dev);  

    if (EOK != memset_s(hw->rx_queues, sizeof(acc_rx_queue) * QUEUES_MAX_NUM_EVERY_IP, 0, 
        sizeof(acc_rx_queue) * QUEUES_MAX_NUM_EVERY_IP)) {
        PMD_ACC_ERROR("memset_s call failed.");
    }   
    if (EOK != memset_s(hw->tx_queues, sizeof(acc_tx_queue) * QUEUES_MAX_NUM_EVERY_IP, 0, 
        sizeof(acc_tx_queue) * QUEUES_MAX_NUM_EVERY_IP)){
        PMD_ACC_ERROR("memset_s call failed.");
    }   
    

    PMD_ACC_DEBUG("----------------start-------------------");

    hw->acc_soc_ip_type = SOC_IP0_TYPE;
    hw->acc_soc_ip_index = SOC_IP0_INDEX;

    hw->rx_queues_max_num = QUEUES_MAX_NUM_EVERY_IP;
    hw->tx_queues_max_num = QUEUES_MAX_NUM_EVERY_IP;
    hw->rx_queue_desc_max_num = DESC_MAX_NUM_EVERY_QUEUE;
    hw->tx_queue_desc_max_num = DESC_MAX_NUM_EVERY_QUEUE;
    hw->rx_queue_desc_min_num = DESC_MIN_NUM_EVERY_QUEUE;
    hw->tx_queue_desc_min_num = DESC_MIN_NUM_EVERY_QUEUE;

    
    hw->port_id = acc_dev->data->port_id;
    hw->pci_location = pci_dev->addr;
    hw->pci_id = pci_dev->id;

    hw->rx_soft_loop = 0;
    hw->tx_soft_loop = 0;

    if (!pci_dev->mem_resource[0].addr) {
        PMD_ACC_CRIT("port %u vendorID=0x%x deviceID=0x%x Bar[0] addr is NULL!",
            acc_dev->data->port_id, pci_dev->id.vendor_id, pci_dev->id.device_id);
        return -EFAULT;
    }

    hw->hw_res_addr = (uint8_t *)pci_dev->mem_resource[0].addr;
    hw->hw_res_addr_phys = pci_dev->mem_resource[0].phys_addr;
    hw->hw_res_len = pci_dev->mem_resource[0].len;
    PMD_ACC_INFO("[hw->hw_res_addr, phy addr, len] = [ %p : 0x%lx : %ld ]",
        hw->hw_res_addr, hw->hw_res_addr_phys, hw->hw_res_len);

    PMD_ACC_INFO("hw->acc_soc_ip_type=0x%x, hw->acc_soc_ip_INDEX=0x%x, "
        "port_id=%u, pci_location=0x%04x:0x%02x:0x%02x.0x%02x.", 
        hw->acc_soc_ip_type, hw->acc_soc_ip_index, hw->port_id, 
        hw->pci_location.domain, hw->pci_location.bus, hw->pci_location.devid, hw->pci_location.function);

    return 0;
}

/*****************************************************************************
 Function name      : acc_dev_uninit
 Description        : 
 Input parameters   : struct rte_eth_dev *acc_dev VF device data structure
 Output parameters  : None
 Return value       : Success : 0
                      Fail : -EINVAL
 Function explain   : NULL function
 calling function   : 
 called function    : rte_eal_uninit
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
acc_dev_uninit(struct rte_eth_dev *acc_dev) {
    int ret = 0;
    
    if (NULL == acc_dev) {
        PMD_ACC_CRIT("rte_eth_dev is NULL!");
        return -EINVAL;
    }

    /* Unlock file lock */
    if(1 == g_file_flag) {
        ret = acc_fcntl_read_unlock(g_file_id);
        if(0 != ret) {
            PMD_ACC_INFO("acc_fcntl_read_unlock error, ret = %d", ret);
        }
        g_file_flag = 0;
    }

    return 0;
}


/*****************************************************************************
 Variable Name      : rte_pmd_acc
 Description        : VF device's driver data structure
 called function    : rte_eth_driver_register
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
static struct eth_driver rte_pmd_acc = {
    {
        .name      = ACC_PMD_NAME,
        .id_table  = pci_id_acc_map,
        .drv_flags = RTE_PCI_DRV_NEED_MAPPING | RTE_PCI_DRV_DETACHABLE,
    },
    .eth_dev_init = acc_dev_init,
    .eth_dev_uninit = acc_dev_uninit,
    .dev_private_size = sizeof(struct acc_adapter),
};


/*****************************************************************************
 Function name      : rte_pmd_acc_init
 Description        : VF device's driver register interface
 Input parameters   : struct eth_driver rte_pmd_acc
 Output parameters  : None
 Return value       : 0
 Function explain   : This function registers the SHELL logic's PMD callback interface to the DPDK
 calling function   : 
 called function    : 
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
int
rte_pmd_acc_init(const char *name __rte_unused, const char *param __rte_unused)
{
    rte_eth_driver_register(&rte_pmd_acc);
    return 0;
}

static struct rte_driver rte_acc_driver = {
    .type = PMD_PDEV,
    .init = rte_pmd_acc_init,
};


/*****************************************************************************
 Function name      : PMD_REGISTER_DRIVER
 Description        : VF device's driver register interface
 Input parameters   : struct rte_eth_dev *acc_dev VF device data structure
 Output parameters  : None
 Return value       : None
 Function explain   : This macro takes use of a compiling technique, the macro's codes run beform main entry, so it could 
                  register the PMD callback interface before DPDK runs
 calling function   : 
 called function    : 
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*/
PMD_REGISTER_DRIVER(rte_acc_driver)


