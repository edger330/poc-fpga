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
#include <sys/time.h>
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

#include "acc_dev.h"
#include "acc_queue.h"
#include "acc_reg.h"
#include "securec.h"
#include "acc_logs.h"

static inline uint16_t 
get_tx_bd_free_nb(acc_tx_queue* p_tx_queue);

static inline uint16_t
get_rx_bd_usable_nb(acc_rx_queue* p_rx_queue);

/*****************************************************************************
 Function name      : get_tx_bd_free_nb
 Description        : get free BD space number in the TX BD Queue space
 Input parameters   : acc_tx_queue* p_tx_queue                         
 Output parameters  : None
 Return value       : free BD space number in the TX BD Queue space
 Function explain   : p_tx_queue->tail : position from which SHELL logic read BD last time
				    p_tx_queue->head : position to which SW write BD last time
				  p_tx_queue->head should always be ahead of p_tx_queue->tail, so if (p_tx_queue->head+1)%depth==p_tx_queue->tail,
				  it means that the BD queue is full, SW can not write new BD in it, need wait SHELL logic read them; if 
				  p_tx_queue->tail==p_tx_queue->head, it means the BD queue is empty, SHELL logic has no BD to read in BD Queue, need
				  wait SW write BD in the BD Queue
 calling function   : 
 called function    : 
 
 Modify history     :
  1.Date              : 2017/07/07
    Author            : AI SDK Team
    Content           : New function
*/
static inline uint16_t 
get_tx_bd_free_nb(acc_tx_queue* p_tx_queue) {
    if (unlikely(NULL == p_tx_queue)) {
       rte_exit(EXIT_FAILURE, "acc_tx_queue is NULL!"); 
    }
    
    return (uint16_t)((p_tx_queue->tail > p_tx_queue->head) ? 
        (uint16_t)(p_tx_queue->tail - p_tx_queue->head - 1) : 
        (uint16_t)(p_tx_queue->tail + p_tx_queue->bd_queue_nb - p_tx_queue->head - 1));
}


/*****************************************************************************
 Function name      : get_rx_bd_free_nb
 Description        : get the free BD space number in the RX BD Queue from
 Input parameters   : acc_rx_queue* p_rx_queue                         
 Output parameters  : None
 Return value       : free BD space number in the RX BD Queue from
 Function explain   : p_rx_queue->tail : position to which SHELL logic write BD last time
                      p_rx_queue->head : position to which SHELL logic could write BD to
                    p_rx_queue->head should be ahead of p_rx_queue->tail
 calling function   : 
 called function    : 
 
 Modify history     :
  1.Date              : 2017/07/07
    Author            : AI SDK Team
    Content           : New function
*/
static inline uint16_t
get_rx_bd_usable_nb(acc_rx_queue* p_rx_queue) {
    if (unlikely(NULL == p_rx_queue)) {
       rte_exit(EXIT_FAILURE, "acc_rx_queue is NULL!"); 
    }

    return (uint16_t)((p_rx_queue->head >= p_rx_queue->tail) ? 
        (uint16_t)(p_rx_queue->head - p_rx_queue->tail) :
        (uint16_t)(p_rx_queue->head + p_rx_queue->bd_queue_nb - p_rx_queue->tail));
}

/*****************************************************************************
 Function name      : acc_dev_tx_burst
 Description        : transmit BD in the BD queue from which SHELL logic will read via DMA
 Input parameters   : void txq : acc_tx_queue*             
                      struct rte_mbuf **tx_pkts : BD pointer array need transmit to BD queue
                      uint16_t nb_pkts number of BD pointer need transmit to BD queue
 Output parameters  : None
 Return value       : actual number of BD that has been transmitted to BD queue (should be equal or less than nb_pkts)
 Function explain   : this function only transmit the BDs to the BD queue, but not to the SHELL logic directly, the SHELL
                   logic read BD from BD queue via DMA is asynchronous with the SW transmitting action.
 calling function   : 
 called function    : rte_eth_tx_burst
 
 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function

*****************************************************************************/
/*#define TX_CALCULATE_TIME*/
#ifdef TX_CALCULATE_TIME
static struct timeval  g_s_tx_time_start[QUEUES_MAX_NUM_EVERY_IP];
static struct timeval  g_s_tx_time_end[QUEUES_MAX_NUM_EVERY_IP];
uint64_t     g_s_tx_time[QUEUES_MAX_NUM_EVERY_IP] = {0};
#endif
static uint64_t     g_s_tx_total[QUEUES_MAX_NUM_EVERY_IP] = {0};
#define   MAX_STATISTICS_NUM    (10000000)
uint16_t
acc_dev_tx_burst(void *txq,
				   struct rte_mbuf **tx_pkts,
				   uint16_t nb_pkts) {
	acc_tx_queue* p_tx_queue = NULL;
    struct acc_hw *hw = NULL;
    uint16_t i = 0;
    uint16_t tx_nb = 0;
    uint16_t free_bd_num = 0;
    uint16_t real_nb_need_tx = 0;
    
    p_tx_queue = (acc_tx_queue*)txq;
    if (unlikely(NULL == p_tx_queue)) {
       rte_exit(EXIT_FAILURE, "acc_tx_queue is NULL!");
    }

    if (unlikely(NULL == p_tx_queue->adapter)) {
       rte_exit(EXIT_FAILURE, "acc_tx_queue->adapter is NULL!");
    }

    hw = &p_tx_queue->adapter->hw;
    
    if (unlikely(NULL == tx_pkts)) {
        rte_exit(EXIT_FAILURE, "tx_pkts is NULL!");
    }
           
    /* obtain SHELL logic write-back pointer value, the value means SHELL logic will read BD from the position next time */
    if (unlikely(NULL == hw->ptr_tx_container_virt)) {
        rte_exit(EXIT_FAILURE, "hw->ptr_tx_container_virt is NULL!");
    }
    p_tx_queue->tail = *(hw->ptr_tx_container_virt + p_tx_queue->queue_id);

    free_bd_num = get_tx_bd_free_nb(p_tx_queue);
    real_nb_need_tx = free_bd_num > nb_pkts ? nb_pkts : free_bd_num;
    real_nb_need_tx = real_nb_need_tx > p_tx_queue->tx_free_thresh ? p_tx_queue->tx_free_thresh : real_nb_need_tx;

    if (0 == real_nb_need_tx) {
        return 0;
    }
#ifdef TX_CALCULATE_TIME
    if (unlikely(0 == g_s_tx_total[p_tx_queue->queue_id])) {
        (void)gettimeofday(&g_s_tx_time_start[p_tx_queue->queue_id], NULL);
    }
#endif
    for (i = 0; i < real_nb_need_tx; ++i) {
        if (unlikely(NULL == tx_pkts[i])) {
            rte_exit(EXIT_FAILURE, "input %u rte_mbuf is NULL!", i);
        }
        if (PMD_ACC_BD_LENGTH != rte_pktmbuf_data_len(tx_pkts[i])) {
            PMD_ACC_ERROR("the tx mbuf's pkt len is not %u, but %u", PMD_ACC_BD_LENGTH, rte_pktmbuf_data_len(tx_pkts[i]));
            rte_exit(EXIT_FAILURE, "BD len is %u\r\n", rte_pktmbuf_data_len(tx_pkts[i])); 
        }
        /* copy pkt to bd queue */    
        if (EOK != memcpy_s(&p_tx_queue->bd_queue[p_tx_queue->head], 
            rte_pktmbuf_data_len(tx_pkts[i]), rte_pktmbuf_mtod(tx_pkts[i], uint8_t*), rte_pktmbuf_data_len(tx_pkts[i]))) {
            rte_exit(EXIT_FAILURE, "call memcpy_s failed");
        }

        if (p_tx_queue->bd_queue[p_tx_queue->head].src_phy_addr == 0 || p_tx_queue->bd_queue[p_tx_queue->head].dst_phy_addr == 0 || 
            p_tx_queue->bd_queue[p_tx_queue->head].bd_code != 0x5a) {
            rte_exit(EXIT_FAILURE, "port id %u, queue id %u, BD content src %lu, dst %lu, bd_code %u, g_s_tx_total %lu\r\n", 
                p_tx_queue->port_id, p_tx_queue->queue_id, 
                p_tx_queue->bd_queue[p_tx_queue->head].src_phy_addr, 
                p_tx_queue->bd_queue[p_tx_queue->head].dst_phy_addr, 
                p_tx_queue->bd_queue[p_tx_queue->head].bd_code, g_s_tx_total[p_tx_queue->queue_id]); 
        }

        PMD_ACC_QUEUE_PTR_FORWARD(p_tx_queue->head, 1, p_tx_queue->bd_queue_nb);
        ++tx_nb;
    }

    ACC_TXQM_CONF_BDQ_RING(hw, hw->acc_soc_ip_index, p_tx_queue->queue_id, p_tx_queue->head);
    g_s_tx_total[p_tx_queue->queue_id] += i;
    
#ifdef TX_CALCULATE_TIME
    if (unlikely(g_s_tx_total[p_tx_queue->queue_id] >= MAX_STATISTICS_NUM)) {
        (void)gettimeofday(&g_s_tx_time_end[p_tx_queue->queue_id], NULL);
        
        g_s_tx_time[p_tx_queue->queue_id] = 1000000*(g_s_tx_time_end[p_tx_queue->queue_id].tv_sec-g_s_tx_time_start[p_tx_queue->queue_id].tv_sec) + 
            g_s_tx_time_end[p_tx_queue->queue_id].tv_usec-g_s_tx_time_start[p_tx_queue->queue_id].tv_usec;
        
        PMD_ACC_INFO("port id: %u, queue id: %u: g_s_tx_total=%lu, g_s_tx_time=%lu(us)", \
            p_tx_queue->port_id, p_tx_queue->queue_id, g_s_tx_total[p_tx_queue->queue_id], g_s_tx_time[p_tx_queue->queue_id]);
        g_s_tx_total[p_tx_queue->queue_id] = 0;
    }
#endif
    return tx_nb;
}

/*****************************************************************************
 Function name      : acc_rx_reserve_prefetch_bd_space
 Description        : supply the pre-fetch BD space in the BD queue for SHELL logic and tell SHELL logic the pre-fetch position via 
					writing ring register. It is best to configure more pre-fetch BD space for RX queue that SHELL logic could write
					RX BD to the BD queue smoothly. 
 Input parameters   : acc_rx_queue *p_rx_queue  
 Output parameters  : None
 Return value       : None

 Modify history     :
  1.Date              : 2017/07/20
    Author            : AI SDK Team
    Content           : New function
*****************************************************************************/
static void acc_rx_reserve_prefetch_bd_space(acc_rx_queue *p_rx_queue)
{
    struct acc_hw *hw = NULL;
    uint16_t pre_alloc_space = 0;
    uint16_t max_pre_alloc_space = 0;
    
    if (unlikely(NULL == p_rx_queue)) {
       rte_exit(EXIT_FAILURE, "acc_rx_queue is NULL!"); 
    }

    if (unlikely(NULL == p_rx_queue->adapter)) {
       rte_exit(EXIT_FAILURE, "acc_rx_queue->adapter is NULL!"); 
    }
    
    hw = &p_rx_queue->adapter->hw;
    pre_alloc_space = get_rx_bd_usable_nb(p_rx_queue);
    max_pre_alloc_space = p_rx_queue->bd_queue_nb/2;
    if (pre_alloc_space < max_pre_alloc_space) {
        PMD_ACC_QUEUE_PTR_FORWARD(p_rx_queue->head, max_pre_alloc_space, p_rx_queue->bd_queue_nb);
        ACC_RXQM_CONF_BDQ_RING(hw, hw->acc_soc_ip_index, p_rx_queue->queue_id, p_rx_queue->head);
    }

    
    return;
}

/*****************************************************************************
 Function name      : acc_dev_rx_burst
 Description        : receive BD in the RX BD queue to which SHELL logic will write via DMA
 Input parameters   : void *rxq : acc_rx_queue* p_rx_queue
					  struct rte_mbuf **rx_pkts: BD pointer array need receive from the RX BD queue
					  uint16_t nb_pkts:  maximum size could receive
 Output parameters  : None
 Return value       : actual number of BD that has been transmitted to BD queue (should be equal or less than nb_pkts)
 calling function   : 
 called function    : rte_eth_rx_burst
 
 Modify history     :
  1.Date              : 2017/06/23
    Author            : AI SDK Team
    Content           : New function

*****************************************************************************/
#define RX_CALCULATE_TIME
#ifdef RX_CALCULATE_TIME
static struct timeval  g_s_rx_time_start[VFS_MAX_NUM_EVERY_PF][QUEUES_MAX_NUM_EVERY_IP];
static struct timeval  g_s_rx_time_end[VFS_MAX_NUM_EVERY_PF][QUEUES_MAX_NUM_EVERY_IP];
uint64_t     g_s_rx_time[VFS_MAX_NUM_EVERY_PF][QUEUES_MAX_NUM_EVERY_IP] = {{0}};
#endif
static uint64_t     g_s_rx_total[VFS_MAX_NUM_EVERY_PF][QUEUES_MAX_NUM_EVERY_IP] = {{0}};
uint16_t
acc_dev_rx_burst(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts) {
    uint16_t i = 0;
    uint16_t rx_nb = 0;
    struct rte_mbuf *rx_pkt = NULL;
    acc_rx_bd* p_rx_res_bd = NULL;
    
    acc_rx_queue* p_rx_queue = (acc_rx_queue*)rx_queue;
    struct acc_hw *hw = NULL;
    uint16_t logic_product_bd_pos = 0;
    uint16_t real_nb_need_rx = 0;

    if (unlikely(NULL == p_rx_queue)) {
        rte_exit(EXIT_FAILURE, "acc_rx_queue is NULL!"); 
    }

    if (unlikely(NULL == p_rx_queue->adapter)) {
        rte_exit(EXIT_FAILURE, "acc_rx_queue->adapter is NULL!"); 
    }

    hw = &p_rx_queue->adapter->hw;

    if (unlikely(NULL == rx_pkts)) {
        rte_exit(EXIT_FAILURE, "tx_pkts is NULL!");
    }

    acc_rx_reserve_prefetch_bd_space(p_rx_queue);
	
    /* logic_product_bd_pos is SHELL logic write-back pointer value, PMD uses the position to check
	 * where SHELL logic writes RX BD to the RX BD queue via DMA. p_rx_queue->tail means position 
	 * CPU read BD in the RX BD queue from, so we could look p_rx_queue->tail as customer(CPU)'s position,
	 * logic_product_bd_pos as producer(SHELL logic)'s position, if the two position is not equal, CPU could 
	 * read BD in the BD queue.
	*/
	if (unlikely(NULL == hw->ptr_rx_container_virt)) {
        rte_exit(EXIT_FAILURE, "hw->ptr_rx_container_virt is NULL!");
	}
    
    logic_product_bd_pos = *(hw->ptr_rx_container_virt + p_rx_queue->queue_id);
    real_nb_need_rx = p_rx_queue->rx_free_thresh < nb_pkts ?  p_rx_queue->rx_free_thresh : nb_pkts;

    /* check if has any BD ready to deal with */
    if (logic_product_bd_pos == p_rx_queue->tail) {
        return 0;
    }
    
    /* Shell logic write-back pointer check */
    if (p_rx_queue->head > p_rx_queue->tail) {
        if ((logic_product_bd_pos < p_rx_queue->tail) || (logic_product_bd_pos > p_rx_queue->head)) {
            PMD_ACC_CRIT("error 01: queue(%u), logic_product_bd_pos=%u, head=%u, tail=%u", 
                p_rx_queue->queue_id, logic_product_bd_pos, p_rx_queue->head, p_rx_queue->tail);
            rte_exit(EXIT_FAILURE, "exit because logic error\r\n"); 
        }
    } else {
        if ((logic_product_bd_pos < p_rx_queue->tail) && (logic_product_bd_pos > p_rx_queue->head)) {
            PMD_ACC_CRIT("error 02: queue(%u), logic_product_bd_pos=%u, head=%u, tail=%u", 
                p_rx_queue->queue_id, logic_product_bd_pos, p_rx_queue->head, p_rx_queue->tail);
            rte_exit(EXIT_FAILURE, "exit because logic error\r\n"); 
        }
    }

#ifdef RX_CALCULATE_TIME
    if (unlikely(0 == g_s_rx_total[p_rx_queue->port_id][p_rx_queue->queue_id])) {
        (void)gettimeofday(&g_s_rx_time_start[p_rx_queue->port_id][p_rx_queue->queue_id], NULL);
    }
#endif
    for (i = 0; i < real_nb_need_rx; i++) {
        if (logic_product_bd_pos == p_rx_queue->tail) { /* customer(CPU) has not any BD could be received */
            break;
        }
		/* obtain a BD in the RX BD queue, that is to say customer(CPU) has consumed a BD */
        p_rx_res_bd = (acc_rx_bd *)&p_rx_queue->bd_queue[p_rx_queue->tail];
        if (unlikely(NULL == p_rx_res_bd)) {
            rte_exit(EXIT_FAILURE, "rx a rte_mbuf is NULL!");
        }

        if (unlikely((RTE_BAD_PHYS_ADDR==p_rx_res_bd->dst_phy_addr) || (0 == p_rx_res_bd->dst_phy_addr))) {
            rte_exit(EXIT_FAILURE, "p_rx_res_bd->dst_phy_addr %lu, len %u, g_s_rx_total %lu\r\n", 
                p_rx_res_bd->dst_phy_addr, p_rx_res_bd->length, g_s_rx_total[p_rx_queue->port_id][p_rx_queue->queue_id]); 
        }

        rx_pkt = p_rx_queue->rx_mbufs[p_rx_queue->rx_mbufs_head];
        if (rte_pktmbuf_mtod(rx_pkt, void*)!=rte_memcpy(rte_pktmbuf_mtod(rx_pkt, void*), (void*)p_rx_res_bd, sizeof(acc_rx_bd))) {
            rte_exit(EXIT_FAILURE, "call rte_memcpy failed");
        }
        
        rx_pkts[i] = p_rx_queue->rx_mbufs[p_rx_queue->rx_mbufs_head];
        PMD_ACC_QUEUE_PTR_FORWARD(p_rx_queue->rx_mbufs_head, 1, p_rx_queue->rx_mbufs_nb);

		/* Update the consumption position */
        PMD_ACC_QUEUE_PTR_FORWARD(p_rx_queue->tail, 1, p_rx_queue->bd_queue_nb);
        
        rx_nb++;
    }
    g_s_rx_total[p_rx_queue->port_id][p_rx_queue->queue_id] += i;
    
#ifdef RX_CALCULATE_TIME
    if (unlikely(g_s_rx_total[p_rx_queue->port_id][p_rx_queue->queue_id] >= MAX_STATISTICS_NUM)) {
        (void)gettimeofday(&g_s_rx_time_end[p_rx_queue->port_id][p_rx_queue->queue_id], NULL);
        
        g_s_rx_time[p_rx_queue->port_id][p_rx_queue->queue_id] = 1000000*(g_s_rx_time_end[p_rx_queue->port_id][p_rx_queue->queue_id].tv_sec-g_s_rx_time_start[p_rx_queue->port_id][p_rx_queue->queue_id].tv_sec) + \
            g_s_rx_time_end[p_rx_queue->port_id][p_rx_queue->queue_id].tv_usec - g_s_rx_time_start[p_rx_queue->port_id][p_rx_queue->queue_id].tv_usec;
        
        PMD_ACC_INFO("port id: %u, queue id: %u: g_s_rx_total=%lu, g_s_rx_time=%lu(us)", \
            p_rx_queue->port_id, p_rx_queue->queue_id, 
            g_s_rx_total[p_rx_queue->port_id][p_rx_queue->queue_id], g_s_rx_time[p_rx_queue->port_id][p_rx_queue->queue_id]);
        g_s_rx_total[p_rx_queue->port_id][p_rx_queue->queue_id] = 0;
    }
#endif

    return rx_nb;
}
