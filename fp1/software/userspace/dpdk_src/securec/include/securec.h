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

#ifndef __SECUREC_H__
#define __SECUREC_H__

#include "securectype.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* success */
#define EOK (0)

/*define error code*/
#ifndef errno_t
typedef int errno_t;
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
    * @Description: The  strtok_s  function parses a string into a sequence of tokens,On the first call to strtok_s the string to be parsed should be specified in strToken.  
    *                       In each subsequent call that should parse the same string, strToken should be NULL
    * @param strToken - the string to be delimited
    * @param strDelimit -specifies a set of characters that delimit the tokens in the parsed string
    * @param context -is a pointer to a char * variable that is used internally by strtok_s function
    * @return:returns a pointer to the first character of a token, or a null pointer if there is no token or there is a runtime-constraint violation.
    */
    char* strtok_s(char* strToken, const char* strDelimit, char** context);


    /**
    * @Description: The snprintf_s function is equivalent to the snprintf function except for the parameter destMax/count and the explicit runtime-constraints violation
    * @param strDest -  produce output according to a format ,write to the character string strDest
    * @param destMax - The maximum length of destination buffer(including the terminating null  byte ('\0'))
    * @param count - do not write more than count bytes to strDest(not including the terminating null  byte ('\0'))
    * @param format - fromat string
    * @param argptr - instead of  a variable  number of arguments
    * @return:return the number of characters printed(not including the terminating null byte ('\0')), If an error occurred return -1.
    */
    int snprintf_s(char* strDest, size_t destMax, size_t count, const char* format, ...);

     /**
    * @Description:The memset_s function copies the value of c (converted to an unsigned char) into each of the first count characters of the object pointed to by dest.
    * @param dest - destination  address
    * @param destMax -The maximum length of destination buffer
    * @param c - the value to be copied
    * @param count -copies fisrt count characters of  dest
    * @return  EOK if there was no runtime-constraint violation
    */
    int memset_s(void* dest, size_t destMax, int c, size_t count);

    /**
    * @Description:The memcpy_s function copies n characters from the object pointed to by src into the object pointed to by dest.
    * @param dest - destination  address
    * @param destMax -The maximum length of destination buffer
    * @param src -source  address
    * @param count -copies count  characters from the  src
    * @return  EOK if there was no runtime-constraint violation
    */
    int memcpy_s(void* dest, size_t destMax, const void* src, size_t count);

    /**
    * @Description:The strcpy_s function copies the string pointed to by strSrc (including the terminating null character) into the array pointed to by strDest
    * @param strDest - destination  address
    * @param destMax -The maximum length of destination buffer(including the terminating null character) 
    * @param strSrc -source  address
    * @return  EOK if there was no runtime-constraint violation
    */
    int strcpy_s(char* strDest, size_t destMax, const char* strSrc);
    


#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif/* __SECUREC_H__ */


