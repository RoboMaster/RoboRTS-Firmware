//******************************************************************************************
//!
//! \file   FIFO.h
//! \brief  Genernal FIFO Model Interface.
//!         You can use uniform FIFO Model to manager Any type of data element.
//! \author cedar
//! \changed damom.li
//! \changed sky.huang
//! \date   2016-11-03
//! \email  xuesong5825718@gmail.com
//!
//! \version: 1.1 add fifo_s_gets_noprotect function
//!
//! Copyright (c) 2013 Cedar MIT License
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights to
//! use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
//! the Software, and to permit persons to whom the Software is furnished to do so,
//! subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in all
//! copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//! IN THE SOFTWARE.
///
//******************************************************************************************
#ifndef __FIFO_H__
#define __FIFO_H__
#ifdef __cplusplus
"C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

    //******************************************************************************************
    //!                           CONFIGURE MACRO
    //******************************************************************************************

#define FIFO_NDEBUG
#define USE_DYNAMIC_MEMORY //!< Use system malloc/free function

#include "sys.h"

    //******************************************************************************************
    //!                     Macro Function
    //******************************************************************************************

    //******************************************************************************************
    //!                           PUBLIC TYPE
    //******************************************************************************************

    //! FIFO Memory Model (Single Byte Mode)
    typedef struct
    {
        char *p_start_addr; //!< FIFO Memory Pool Start Address
        char *p_end_addr;   //!< FIFO Memory Pool End Address
        int free_num;       //!< The remain capacity of FIFO
        int used_num;       //!< The number of elements in FIFO
        char *p_read_addr;  //!< FIFO Data Read Index Pointer
        char *p_write_addr; //!< FIFO Data Write Index Pointer
        MUTEX_DECLARE(mutex);
    } fifo_s_t;

    //! FIFO Memory Model
    typedef struct
    {
        char *p_start_addr; //!< FIFO Memory Pool Start Address
        char *p_end_addr;   //!< FIFO Memory Pool End Address
        int free_num;       //!< The remain capacity of FIFO
        int used_num;       //!< The number of elements in FIFO
        int unit_size;      //!< FIFO Element Size(Unit: Byte)
        char *p_read_addr;  //!< FIFO Data Read Index Pointer
        char *p_write_addr; //!< FIFO Data Write Index Pointer
        MUTEX_DECLARE(mutex);
    } fifo_t;

    //******************************************************************************************
    //!                           PUBLIC API
    //******************************************************************************************

#ifdef USE_DYNAMIC_MEMORY

    //******************************************************************************************
    //
    //! \brief  Create An New FIFO Instance(in Single Mode).
    //! This function allocate enought room for N blocks fifo elements, then return the pointer
    //! of FIFO.
    //!
    //! \param  [in] uint_cnt is count of fifo elements.
    //! \retval The Pointer of FIFO instance, return NULL is failure to allocate memory.
    //!
    //! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
    //!            Header file before use this function.
    //! \note   -# Functions FIFO_Create and FIFO_Destory must be used in pairs.
    //!
    //******************************************************************************************
    fifo_s_t *fifo_s_create(int uint_cnt);

    //******************************************************************************************
    //
    //! \brief  Destory FIFO Instance(in Single Mode).
    //!  This function release memory, then reinit the FIFO struct.
    //!
    //! \param  [in] p_fifo is the pointer of FIFO instance
    //! \retval None.
    //!
    //! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
    //!            Header file before use this function.
    //
    //******************************************************************************************
    void fifo_s_destroy(fifo_s_t *p_fifo);

#endif // USE_DYNAMIC_MEMORY

    //******************************************************************************************
    //
    //! \brief  Initialize an static FIFO struct(in single mode).
    //!
    //! \param  [in] p_fifo is the pointer of valid FIFO instance.
    //! \param  [in] p_base_addr is the base address of pre-allocate memory, such as array.
    //! \param  [in] uint_cnt is count of fifo elements.
    //! \retval 0 if initialize successfully, otherwise return -1.
    //
    //******************************************************************************************
    int fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt);

    //******************************************************************************************
    //
    //! \brief  Put an element into FIFO(in single mode).
    //!
    //! \param  [in]  p_fifo is the pointer of valid FIFO.
    //! \param  [in]  element is the data element you want to put
    //!
    //! \retval 0 if operate successfully, otherwise return -1.
    //
    //******************************************************************************************
    int fifo_s_put(fifo_s_t *p_fifo, char element);

    int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len);
    int fifo_s_puts_noprotect(fifo_s_t *p_fifo, char *p_source, int len);

    //******************************************************************************************
    //
    //! \brief  Get an element from FIFO(in single mode).
    //!
    //! \param  [in]  p_fifo is the pointer of valid FIFO.
    //!
    //! \retval the data element of FIFO.
    //
    //******************************************************************************************
    char fifo_s_get(fifo_s_t *p_fifo);
    int fifo_s_gets(fifo_s_t *p_fifo, char *p_dest, int len);
    int fifo_s_gets_noprotect(fifo_s_t *p_fifo, char *p_dest, int len);

    //******************************************************************************************
    //
    //! \brief  Pre-Read an element from FIFO(in single mode).
    //!
    //! \param  [in]  p_fifo is the pointer of valid FIFO.
    //! \param  [in]  Offset is the offset from current pointer.
    //!
    //! \retval the data element of FIFO.
    //
    //******************************************************************************************
    char fifo_s_preread(fifo_s_t *p_fifo, int offset);
    int fifo_s_prereads(fifo_s_t *p_fifo, char *p_dest, int offset, int len);

    //******************************************************************************************
    //
    //! \brief  FIFO is empty (in single mode)?
    //!
    //! \param  [in] p_fifo is the pointer of valid FIFO.
    //!
    //! \retval - None-zero(true) if empty.
    //!         - Zero(false) if not empty.
    //
    //******************************************************************************************
    char fifo_s_isempty(fifo_s_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  FIFO is full (in single mode)?
    //!
    //! \param  [in] p_fifo is the pointer of valid FIFO.
    //!
    //! \retval - None-zero(true) if full.
    //!         - Zero(false) if not full.
    //
    //******************************************************************************************
    char fifo_s_isfull(fifo_s_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Get FIFO the number of elements(in single mode)?
    //!
    //! \param  [in] p_fifo is the pointer of valid FIFO.
    //!
    //! \retval The number of elements in FIFO.
    //
    //******************************************************************************************
    int fifo_s_used(fifo_s_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Get FIFO the number of elements(in single mode)?
    //!
    //! \param  [in] p_fifo is the pointer of valid FIFO.
    //!
    //! \retval The number of elements in FIFO.
    //
    //******************************************************************************************
    int fifo_s_free(fifo_s_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Flush the content of FIFO.
    //!
    //! \param  [in] p_fifo is the pointer of valid FIFO.
    //!
    //! \retval 0 if success, -1 if failure.
    //
    //******************************************************************************************
    void fifo_s_flush(fifo_s_t *p_fifo);
    int fifo_s_discard(fifo_s_t *p_fifo, int len);

    //******************************************************************************************
    //
    //! \brief  Create An New FIFO Instance.
    //! This function allocate enought room for N blocks fifo elements, then return the pointer
    //! of FIFO.
    //!
    //! \param  [in] UnitSize is fifo element size.
    //! \param  [in] UnitCnt is count of fifo elements.
    //! \retval The Pointer of FIFO instance, return NULL is failure to allocate memory.
    //!
    //! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
    //!            Header file before use this function.
    //! \note   -# Functions FIFO_Create and FIFO_Destory must be used in pairs.
    //!
    //******************************************************************************************
    fifo_t *fifo_create(char unit_size, int unit_cnt);

    //******************************************************************************************
    //
    //! \brief  Destory FIFO Instance.
    //!  This function release memory, then reinit the FIFO struct.
    //!
    //! \param  [in] pFIFO is the pointer of FIFO instance
    //! \retval None.
    //!
    //! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
    //!            Header file before use this function.
    //
    //******************************************************************************************
    void fifo_destory(fifo_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Initialize an static FIFO struct.
    //!
    //! \param  [in] pFIFO is the pointer of valid FIFO instance.
    //! \param  [in] pBaseAddr is the base address of pre-allocate memory, such as array.
    //! \param  [in] UnitSize is fifo element size.
    //! \param  [in] UnitCnt is count of fifo elements.
    //! \retval 0 if initialize successfully, otherwise return -1.
    //
    //******************************************************************************************
    int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt);

    //******************************************************************************************
    //
    //! \brief  Put an element into FIFO.
    //!
    //! \param  [in]  pFIFO is the pointer of valid FIFO.
    //! \param  [in]  pElement is the address of element you want to put
    //!
    //! \retval 0 if operate successfully, otherwise return -1.
    //
    //******************************************************************************************
    int fifo_put(fifo_t *p_fifo, void *p_element);
    int fifo_put_noprotect(fifo_t *p_fifo, void *p_element);
    //******************************************************************************************
    //
    //! \brief  Get an element from FIFO.
    //!
    //! \param  [in]  pFIFO is the pointer of valid FIFO.
    //! \param  [out] pElement is the address of element you want to get
    //!
    //! \retval 0 if operate successfully, otherwise return -1.
    //
    //******************************************************************************************
    int fifo_get(fifo_t *p_fifo, void *p_element);
    int fifo_get_noprotect(fifo_t *p_fifo, void *p_element);

    //******************************************************************************************
    //
    //! \brief  Pre-Read an element from FIFO.
    //!
    //! \param  [in]  pFIFO is the pointer of valid FIFO.
    //! \param  [in]  Offset is the offset from current pointer.
    //! \param  [out] pElement is the address of element you want to get
    //!
    //! \retval 0 if operate successfully, otherwise return -1.
    //
    //******************************************************************************************
    int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element);

    //******************************************************************************************
    //
    //! \brief  FIFO is empty ?
    //!
    //! \param  [in] pFIFO is the pointer of valid FIFO.
    //!
    //! \retval - None-zero(true) if empty.
    //!         - Zero(false) if not empty.
    //
    //******************************************************************************************
    int fifo_is_empty(fifo_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  FIFO is full ?
    //!
    //! \param  [in] pFIFO is the pointer of valid FIFO.
    //!
    //! \retval - None-zero(true) if full.
    //!         - Zero(false) if not full.
    //
    //******************************************************************************************
    int fifo_is_full(fifo_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Get FIFO the number of elements?
    //!
    //! \param  [in] pFIFO is the pointer of valid FIFO.
    //!
    //! \retval The number of elements in FIFO.
    //
    //******************************************************************************************
    int fifo_used(fifo_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Get FIFO the number of elements?
    //!
    //! \param  [in] pFIFO is the pointer of valid FIFO.
    //!
    //! \retval The number of elements in FIFO.
    //
    //******************************************************************************************
    int fifo_free(fifo_t *p_fifo);

    //******************************************************************************************
    //
    //! \brief  Flush the content of FIFO.
    //!
    //! \param  [in] pFIFO is the pointer of valid FIFO.
    //!
    //! \retval 0 if success, -1 if failure.
    //
    //******************************************************************************************
    int fifo_flush(fifo_t *p_fifo);

#ifdef __cplusplus
}
#endif

#endif // __FIFO_H__
