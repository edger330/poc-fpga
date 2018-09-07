#ifndef _UTILS_H
#define _UTILS_H


#define BUILD_UINT8(loByte, hiByte) \
          ((uint8_t)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT64(Byte0, Byte1, Byte2, Byte3, Byte4, Byte5, Byte6, Byte7) \
          ((uint64)((uint64)((Byte0) & 0x00FF) \
          + ((uint64_t)((Byte1) & 0x00FF) << 8) \
          + ((uint64_t)((Byte2) & 0x00FF) << 16) \
          + ((uint64_t)((Byte3) & 0x00FF) << 24) \
          + ((uint64_t)((Byte4) & 0x00FF) << 32) \
          + ((uint64_t)((Byte5) & 0x00FF) << 40) \
          + ((uint64_t)((Byte6) & 0x00FF) << 48) \
          + ((uint64_t)((Byte7) & 0x00FF) << 56)))


#define XBUF_TO_UINT08(p)       ((p)[0])
#define XBUF_TO_UINT16(p)       BUILD_UINT16((p)[1], (p)[0])
#define XBUF_TO_UINT24(p)       BUILD_UINT32((p)[2], (p)[1], (p)[0], 0)
#define XBUF_TO_UINT32(p)       BUILD_UINT32((p)[3], (p)[2], (p)[1], (p)[0])
#define XBUF_TO_UINT48(p)       BUILD_UINT64((p)[5], (p)[4], (p)[3], (p)[2], (p)[1], (p)[0], 0, 0)
#define XBUF_TO_UINT64(p)       BUILD_UINT64((p)[7], (p)[6], (p)[5], (p)[4], (p)[3], (p)[2], (p)[1], (p)[0])

#endif