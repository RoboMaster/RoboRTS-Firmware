/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file data_fifo.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief  genernal FIFO model interface for any data element type.
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "data_fifo.h"
#include "stdlib.h"
#include "stdio.h"

fifo_s_t* fifo_s_create(uint32_t unit_cnt, osMutexId mutex)
{
  fifo_s_t *pfifo     = NULL;
  uint8_t  *base_addr = NULL;
  
  //! Check input parameters.
  ASSERT(0 != unit_cnt);

  //! Allocate Memory for pointer of new FIFO Control Block.
  pfifo = (fifo_s_t*) malloc(sizeof(fifo_s_t));
  if(NULL == pfifo)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  //! Allocate memory for FIFO.
  base_addr = malloc(unit_cnt);
  if(NULL == base_addr)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  fifo_s_init(pfifo, base_addr, unit_cnt, mutex);

  return (pfifo);
}

void fifo_s_destory(fifo_s_t* pfifo)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  ASSERT(NULL != pfifo->start_addr);

  //! free FIFO memory
  free(pfifo->start_addr);
  
  //! delete mutex
  osMutexDelete(pfifo->mutex);
  
  //! free FIFO Control Block memory.
  free(pfifo);

  return;
}



int32_t fifo_s_init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt, osMutexId mutex)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  ASSERT(NULL != base_addr);
  ASSERT(0    != unit_cnt);

  pfifo->mutex = mutex;
  
  if (mutex != NULL)
  {
    //! Initialize FIFO Control Block.
    pfifo->start_addr  = (uint8_t*) base_addr;
    pfifo->end_addr    = (uint8_t*) base_addr + unit_cnt - 1;
    pfifo->buf_size    = unit_cnt;
    pfifo->free        = unit_cnt;
    pfifo->used        = 0;
    pfifo->read_index  = 0;
    pfifo->write_index = 0;
    
    return 0;
  }
  else
  {
    return -1;
  }
}



/******************************************************************************************
//
//! \brief  Put an element into FIFO
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  element is the data element you want to put
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
*******************************************************************************************/
int32_t fifo_s_put(fifo_s_t* pfifo, uint8_t element)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);

  if(0 >= pfifo->free)
  {
    //! Error, FIFO is full!
    return -1;
  }
  
  MUTEX_WAIT();
  pfifo->start_addr[pfifo->write_index++] = element;
  pfifo->write_index %= pfifo->buf_size;
  pfifo->free--;
  pfifo->used++;
  MUTEX_RELEASE();
  
  return 0;
}

/******************************************************************************************
//
//! \brief  Put some elements into FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  element is the data element you want to put
//! \param  [in]  the number of elements
//! \retval 0 if operate successfully, otherwise return -1.
//
******************************************************************************************/
int32_t fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint32_t number)
{
  int puts_num = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  if(psource == NULL)
      return -1;
  
  MUTEX_WAIT();
  for(uint32_t i = 0; (i < number) && (pfifo->free > 0); i++)
  {
    pfifo->start_addr[pfifo->write_index++] = psource[i];
    pfifo->write_index %= pfifo->buf_size;
    pfifo->free--;
    pfifo->used++;
    puts_num++;
  }
  MUTEX_RELEASE();
  return puts_num;
}

int32_t fifo_s_puts_no_mutex(fifo_s_t *pfifo, uint8_t *psource, uint32_t number)
{
  int puts_num = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  if(psource == NULL)
      return -1;
  
  for(int i = 0; (i < number) && (pfifo->free > 0); i++)
  {
    pfifo->start_addr[pfifo->write_index++] = psource[i];
    pfifo->write_index %= pfifo->buf_size;
    pfifo->free--;
    pfifo->used++;
    puts_num++;
  }
  return puts_num;
}

/******************************************************************************************
//
//! \brief  Get an element from FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//!
//! \retval the data element of FIFO.
//
******************************************************************************************/
uint8_t fifo_s_get(fifo_s_t* pfifo)
{
  uint8_t   retval = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  taskENTER_CRITICAL();
  retval = pfifo->start_addr[pfifo->read_index++];
  pfifo->read_index %= pfifo->buf_size;
  pfifo->free++;
  pfifo->used--;
  taskEXIT_CRITICAL();

  return retval;
}

uint8_t fifo_s_get_no_mutex(fifo_s_t* pfifo)
{
  uint8_t   retval = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  retval = pfifo->start_addr[pfifo->read_index++];
  pfifo->read_index %= pfifo->buf_size;
  pfifo->free++;
  pfifo->used--;

  return retval;
}




uint16_t fifo_s_gets(fifo_s_t* pfifo, uint8_t* source, uint8_t len)
{
  uint8_t   retval = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  //Enter critical region and close interrupt
  taskENTER_CRITICAL();
  for (int i = 0; (i < len) && (pfifo->used > 0); i++)
  {
    source[i] = pfifo->start_addr[pfifo->read_index++];
    pfifo->read_index %= pfifo->buf_size;
    pfifo->free++;
    pfifo->used--;
    retval++;
  }
  taskEXIT_CRITICAL();

  return retval;
}

uint16_t fifo_s_gets_no_mutex(fifo_s_t* pfifo, uint8_t* source, uint8_t len)
{
  uint8_t   retval = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  for (int i = 0; (i < len) && (pfifo->used > 0); i++)
  {
    source[i] = pfifo->start_addr[pfifo->read_index++];
    pfifo->read_index %= pfifo->buf_size;
    pfifo->free++;
    pfifo->used--;
    retval++;
  }

  return retval;
}


/******************************************************************************************
//
//! \brief  Pre-Read an element from FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  offset is the offset from current pointer.
//!
//! \retval the data element of FIFO.
//
******************************************************************************************/
uint8_t fifo_s_pre_read(fifo_s_t* pfifo, uint8_t offset)
{
  uint32_t index;

  //! Check input parameters.
  ASSERT(NULL != pfifo);

  if(offset > pfifo->used)
  {        
    return 0x00;
  }
  else
  {
    index = ((pfifo->read_index + offset) % pfifo->buf_size);
    // Move Read Pointer to right position   
    return pfifo->start_addr[index];
  }
}


/******************************************************************************************
//!
//! \retval - None-zero(true) if empty.
//!         - Zero(false) if not empty.
//
******************************************************************************************/
uint8_t fifo_is_empty(fifo_s_t* pfifo)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (0 == pfifo->used);
}

/*****************************************************************************************
//!
//! \retval - None-zero(true) if full.
//!         - Zero(false) if not full.
//
*****************************************************************************************/
uint8_t fifo_is_full(fifo_s_t* pfifo)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (0 == pfifo->free);
}

/******************************************************************************************
//!
//! \retval The number of elements in FIFO.
//
******************************************************************************************/
uint32_t fifo_used_count(fifo_s_t* pfifo)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (pfifo->used);
}

/******************************************************************************************
//!
//! \retval The number of elements in FIFO.
//
******************************************************************************************/
uint32_t fifo_free_count(fifo_s_t* pfifo)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (pfifo->free);
}


/******************************************************************************************
//
//! \brief  Flush the content of FIFO.
//!
//! \param  [in] pfifo is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
******************************************************************************************/
uint8_t fifo_flush(fifo_s_t* pfifo)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);

  //! Initialize FIFO Control Block.
  MUTEX_WAIT();
  pfifo->free        = pfifo->buf_size;
  pfifo->used        = 0;
  pfifo->read_index  = 0;
  pfifo->write_index = 0;
  MUTEX_RELEASE();

  return 0;
}

