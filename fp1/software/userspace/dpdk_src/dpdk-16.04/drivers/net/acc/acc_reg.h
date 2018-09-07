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

#ifndef __ACC_REG_H__
#define __ACC_REG_H__

#include <stdint.h>
#include "rte_byteorder.h"
#include "acc_hw.h"

/*
 * This file describe VF's device's registers in BAR space
*/

/* VF's BAR space layout is as follow */
/**
0x000000	0x003FFF	VF mailbox
0x004000	0x007FFF	VF reg_base reserved space
0x008000	0x00BFFF	VF reg_base reserved space
0x00C000	0x00FFFF	VF reg_base reserved space
0x010000	0x013FFF	VF reg_base reserved space
0x014000	0x017FFF	VF IP0_DMAE
0x018000	0x01BFFF	VF IP0_TXQM
0x01C000	0x01FFFF	VF IP0_TXM
0x020000	0x023FFF	VF IP0_RXQM
0x024000	0x027FFF	VF IP0_RXM
0x028000	0x0FFFFF	rsv.
*/

#define QUEUE_SPACE_SIZE_EVERY_IP   (5 * 16 * 1024)     /* IP0_DMAE+IP0_TXQM+IP0_TXM+IP0_RXQM+IP0_RXM */


/* MailBox area */
#define     VF_MAILBOX_REG_BASE_OFFSET              (0x00000000)

/* TXQM_VF area */
#define     VF_TXQM_IP0_REG_BASE_OFFSET                 (0X00006000 << 2)
#define     VF_TXQM_IP0_REG_Q0_RING_OFFSET              (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0000 << 2))
#define     VF_TXQM_IP0_REG_Q0_BD_ADDR_HIG_OFFSET       (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0008 << 2))
#define     VF_TXQM_IP0_REG_Q0_BD_ADDR_LOW_OFFSET       (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0009 << 2))
#define     VF_TXQM_IP0_REG_Q0_BD_DEEP_OFFSET           (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0018 << 2))
#define     VF_TXQM_IP0_REG_Q0_WORK_EN_OFFSET           (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0020 << 2))
#define     VF_TXQM_IP0_REG_Q0_RST_EN_OFFSET            (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0028 << 2))
#define     VF_TXQM_IP0_REG_PTR_WB_ADDR_HIG_OFFSET      (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0030 << 2))
#define     VF_TXQM_IP0_REG_PTR_WB_ADDR_LOW_OFFSET      (VF_TXQM_IP0_REG_BASE_OFFSET + (0x0031 << 2))

/* TXM_VF area */
#define     VF_TXM_IP0_REG_BASE_OFFSET                  (0x00007000 << 2)

/* RXQM_VF area */
#define     VF_RXQM_IP0_REG_BASE_OFFSET                 (0x00008000 << 2)
#define     VF_RXQM_IP0_REG_Q0_RING_OFFSET              (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0000 << 2))
#define     VF_RXQM_IP0_REG_Q0_BD_ADDR_HIG_OFFSET       (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0008 << 2))
#define     VF_RXQM_IP0_REG_Q0_BD_ADDR_LOW_OFFSET       (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0009 << 2))
#define     VF_RXQM_IP0_REG_Q0_BD_DEEP_OFFSET           (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0018 << 2))
#define     VF_RXQM_IP0_REG_Q0_WORK_EN_OFFSET           (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0020 << 2))
#define     VF_RXQM_IP0_REG_Q0_RST_EN_OFFSET            (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0028 << 2))
#define     VF_RXQM_IP0_REG_PTR_WB_ADDR_HIG_OFFSET      (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0030 << 2))
#define     VF_RXQM_IP0_REG_PTR_WB_ADDR_LOW_OFFSET      (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0031 << 2))

#define     VF_RXQM_IP0_REG_Q0_RELEASE_PTR_OFFSET       (VF_RXQM_IP0_REG_BASE_OFFSET + (0x0108 << 2))

/* RXM_VF area */
#define     VF_RXM_IP0_REG_BASE_OFFSET                  (0x00009000 << 2)





/* ISO area */
#define     VF_ISO_REG_BASE_OFFSET                      (0x0000C000 << 2)
#define     VF_ISO_REG_ISO_EN_OFFSET                    (VF_ISO_REG_BASE_OFFSET + (0x0000 << 2))

/* DEMO1 area */
#define     VF_DEMO1_REG_BASE_OFFSET                    (0x00092000 << 2)
#define     VF_DEMO1_REG_VERSION_OFFSET                 (VF_DEMO1_REG_BASE_OFFSET + (0x0000 << 2))
#define     VF_DEMO1_REG_ADDER_CFG_WDATA0               (VF_DEMO1_REG_BASE_OFFSET + (0x0001 << 2))
#define     VF_DEMO1_REG_ADDER_CFG_WDATA1               (VF_DEMO1_REG_BASE_OFFSET + (0x0002 << 2))
#define     VF_DEMO1_REG_ADDER_SUM_RDATA                (VF_DEMO1_REG_BASE_OFFSET + (0x0003 << 2))
#define     VF_DEMO1_REG_OPPOS_DATA                     (VF_DEMO1_REG_BASE_OFFSET + (0x0004 << 2))



#define PCI_REG(reg) (*((volatile uint32_t *)(reg)))

#define	PCI_REG_READ(reg) (rte_le_to_cpu_32(PCI_REG(reg)))

#define PCI_REG_WRITE(reg, value) do { \
    PCI_REG((reg)) = (rte_cpu_to_le_32(value)); \
} while(0)

#define ACC_PCI_REG_READ32(hw, reg) \
    PCI_REG_READ(hw->hw_res_addr + reg)

#define ACC_PCI_REG_WRITE32(hw, reg, value) \
    PCI_REG_WRITE(hw->hw_res_addr + reg, value)

/* value must be uint64_t type */
/* uint64_t value: High 32bits write to low register, low 32bits write to high register */
#define ACC_PCI_REG_WRITE64(hw, reg, value) do {                                            \
    uint32_t _high_value_ = (uint32_t)((((uint64_t)value) >> 32) & 0x00000000FFFFFFFF );      \
    ACC_PCI_REG_WRITE32(hw, reg, _high_value_);                                             \
    uint32_t  _low_value_ = (uint32_t)(value & 0x00000000FFFFFFFF);               \
    ACC_PCI_REG_WRITE32(hw, (reg + 4), _low_value_);                                        \
} while (0)


/* TXQM_VF register operation */
#define ACC_TXQM_CONF_QUEUE_STOP(hw, ip_idx, queue_idx)                     \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_WORK_EN_OFFSET), 0x00000000)

#define ACC_TXQM_CONF_QUEUE_START(hw, ip_idx, queue_idx)                     \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_WORK_EN_OFFSET), 0x00000001)

#define ACC_TXQM_CONF_QUEUE_RESET(hw, ip_idx, queue_idx)                    \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_RST_EN_OFFSET), 0x00000001);  \
    rte_delay_ms(10);

#define ACC_TXQM_CHECK_QUEUE_RST_CLR(hw, ip_idx, queue_idx)                 \
    (0 == (ACC_PCI_REG_READ32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_RST_EN_OFFSET)) & 0x00000001))

#define ACC_TXQM_CONF_BDQ_BASE(hw, ip_idx, queue_idx, addr)                 \
    ACC_PCI_REG_WRITE64((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint64_t) + VF_TXQM_IP0_REG_Q0_BD_ADDR_HIG_OFFSET), addr)

#define ACC_TXQM_CONF_BDQ_DEPTH(hw, ip_idx, queue_idx, nb_desc)             \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_BD_DEEP_OFFSET), (nb_desc))

#define ACC_TXQM_CONF_BDQ_RING(hw, ip_idx, queue_idx, ring_value)            \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_RING_OFFSET), (ring_value))

#define ACC_TXQM_GET_BDQ_RING(hw, ip_idx, queue_idx)            \
    ACC_PCI_REG_READ32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_TXQM_IP0_REG_Q0_RING_OFFSET))

#define ACC_TXQM_CONF_PTR_WB_BD_ADDR(hw, ip_idx, addr)                      \
    ACC_PCI_REG_WRITE64((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + VF_TXQM_IP0_REG_PTR_WB_ADDR_HIG_OFFSET), addr)

/* RXQM_VF register operation */
#define ACC_RXQM_CONF_QUEUE_STOP(hw, ip_idx, queue_idx)                     \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_WORK_EN_OFFSET), 0x00000000)

#define ACC_RXQM_CONF_QUEUE_START(hw, ip_idx, queue_idx)                     \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_WORK_EN_OFFSET), 0x00000001)


#define ACC_RXQM_CONF_QUEUE_RESET(hw, ip_idx, queue_idx)                    \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_RST_EN_OFFSET), 0x00000001);  \
    rte_delay_ms(10);

#define ACC_RXQM_CHECK_QUEUE_RST_CLR(hw, ip_idx, queue_idx)                 \
    (0 == (ACC_PCI_REG_READ32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_RST_EN_OFFSET)) & 0x00000001))

#define ACC_RXQM_CONF_RES_BDQ_BASE(hw, ip_idx, queue_idx, addr)                 \
    ACC_PCI_REG_WRITE64((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint64_t) + VF_RXQM_IP0_REG_Q0_BD_ADDR_HIG_OFFSET), addr)


#define ACC_RXQM_CONF_BDQ_DEPTH(hw, ip_idx, queue_idx, nb_desc)             \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_BD_DEEP_OFFSET), (nb_desc))

#define ACC_RXQM_CONF_BDQ_RING(hw, ip_idx, queue_idx, ring_value)            \
    ACC_PCI_REG_WRITE32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_RING_OFFSET), (ring_value))

#define ACC_RXQM_GET_BDQ_RING(hw, ip_idx, queue_idx)            \
    ACC_PCI_REG_READ32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_RING_OFFSET))


#define ACC_RXQM_CONF_PTR_WB_BD_ADDR(hw, ip_idx, addr)                      \
    ACC_PCI_REG_WRITE64((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + VF_RXQM_IP0_REG_PTR_WB_ADDR_HIG_OFFSET), addr)


#define ACC_RXQM_GET_RELEASE_PTR(hw, ip_idx, queue_idx)       \
    ACC_PCI_REG_READ32((hw), (ip_idx*QUEUE_SPACE_SIZE_EVERY_IP + queue_idx*sizeof(uint32_t) + VF_RXQM_IP0_REG_Q0_RELEASE_PTR_OFFSET))


#endif
