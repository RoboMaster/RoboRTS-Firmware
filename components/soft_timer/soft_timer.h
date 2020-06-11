//******************************************************************************************
//!
//! \file   SoftTimer.h
//! \brief  SoftTimer Interface File
//!         This module is used to replace 'delay' function in state machine
//! \author cedar
//! \date   2014-9-17
//! \email  xuesong5825718@gmail.com
//! \qq     819280802
//!
//! \license
//!
//! Copyright (c) 2014 Cedar MIT License
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

#ifndef __SOFT_TIMER_H__
#define __SOFT_TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

//******************************************************************************************
//!                           Portable Configure
//******************************************************************************************

//! If your compiler support stdint.h file, then try to use it, otherwise using folling section
#if 0
typedef unsigned char      uint8_t ;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
#else
#include <stdint.h>
#endif

//******************************************************************************************
//!                           Function Parameters
//******************************************************************************************

#ifndef TIMER_ELEMENT_NUM_MAX
#define TIMER_ELEMENT_NUM_MAX      20                      //!< Maxium SoftTimer
#endif

#define SOFT_TIMER_SUCCESS         0                       //!< Operate success
#define SOFT_TIMER_ING             1                       //!< Timer Counting
#define SOFT_TIMER_TIMEOUT         2                       //!< Timer Timeout
#define SOFT_TIMER_ERR             3                       //!< Operate failure, invaild handle


//******************************************************************************************
//!                     Typedef
//******************************************************************************************

typedef struct
{
    uint16_t handle;                                       // Timer Resource Handle
    uint32_t delay;                                        // Delay Value
} TimerElem_t;

//******************************************************************************************
//!                           PUBLIC API
//******************************************************************************************
void TimerISR_Hook(void);

extern uint16_t soft_timer_init(void);
extern uint16_t soft_timer_destory(void);
extern uint16_t soft_timer_req(uint32_t Delay);
extern uint16_t soft_timer_update(uint32_t Id, uint32_t Tick);
extern uint16_t soft_timer_check(uint16_t Handle);

#ifdef __cplusplus
}
#endif

#endif // __SOFT_TIMER_H__


