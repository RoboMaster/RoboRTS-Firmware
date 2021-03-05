//******************************************************************************************
//!
//! \file   SoftTimer.c
//! \brief  SoftTimer Implement File
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
#include "soft_timer.h"
#include "sys.h"

//******************************************************************************************
//!                     Private Variable
//******************************************************************************************

TimerElem_t SoftTimer[TIMER_ELEMENT_NUM_MAX + 1];

//******************************************************************************************
//!                     Macro Function
//******************************************************************************************

//******************************************************************************************
//!                     Function Implement
//******************************************************************************************

//******************************************************************************************
//
//! \brief  SoftTimer Hook Function
//!         This callback function must be called interval
//!
//! \note   Typical 1ms interval
//
//******************************************************************************************
void TimerISR_Hook(void)
{
    for (uint8_t i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        if (SoftTimer[i].handle != 0)
        {
            if (SoftTimer[i].delay)
            {
                SoftTimer[i].delay--;
            }
        }
    }
}

//******************************************************************************************
//
//! \brief  Initialize Timer resource
//!
//! \param  None.
//! \retval
//!         - SOFT_TIMER_SUCCESS
//!
//! \note
//!         - This function must be called first !.
//!
//
//******************************************************************************************
uint16_t soft_timer_init(void)
{
    uint8_t i = 0;

    // Clear All Elements
    for (i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        SoftTimer[i].handle = 0;
        SoftTimer[i].delay = 0;
    }

    return (SOFT_TIMER_SUCCESS);
}

uint16_t soft_timer_destory(void)
{
    uint8_t i = 0;

    // Clear All Elements
    for (i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        SoftTimer[i].handle = 0;
        SoftTimer[i].delay = 0;
    }

    return (SOFT_TIMER_SUCCESS);
}

//******************************************************************************************
//
//! \brief  Request Timer resource
//!
//! \param  [in] Tick is time eclipse value.
//! \retval
//!         - Zero                     Operate Failure, No Timer Available
//!         - Non-Zero                 Valid Timer Handle
//!
//! \note
//!         - Timer handle only can be used once.
//!
//
//******************************************************************************************
uint16_t soft_timer_req(uint32_t Tick)
{
    uint8_t i = 0;

    for (i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        if (SoftTimer[i].handle == 0)
        {
            CRITICAL_SETCION_ENTER();

            SoftTimer[i].handle = i;
            SoftTimer[i].delay = Tick;

            CRITICAL_SETCION_EXIT();

            return (i);
        }
    }

    return (0);
}

//******************************************************************************************
//
//! \brief  Update Timer
//!
//! \param  [in] id is timer handler
//! \retval
//!         - Zero                     success
//!         - Non-Zero                 failure
//!
//
//******************************************************************************************
uint16_t soft_timer_update(uint32_t Id, uint32_t Tick)
{
    for (uint8_t i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        if (SoftTimer[i].handle == Id)
        {
            CRITICAL_SETCION_ENTER();

            SoftTimer[i].delay = Tick;

            CRITICAL_SETCION_EXIT();

            // Success
            return (0);
        }
    }

    // Failure
    return (1);
}

//******************************************************************************************
//
//! \brief  Check Timer status
//!         You can check register timer status at any time.
//!
//! \param  [in] Handle is Timer Handle, which you can get it from \ref soft_timer_req.
//! \retval
//!         - \ref SOFT_TIMER_ING      Timer Counting
//!         - \ref SOFT_TIMER_TIMEOUT  Timer TimeOut
//!         - \ref SOFT_TIMER_ERR      Invalid Timer Handle
//!
//! \note
//!         - You must call \ref soft_timer_req to request an valid timer handle.
//!         - Timer handle only can be used once.
//!
//
//******************************************************************************************
uint16_t soft_timer_check(uint16_t Handle)
{
    uint16_t retval = SOFT_TIMER_ERR;

    CRITICAL_SETCION_ENTER();
    if (SoftTimer[Handle].handle == Handle)
    {
        if (SoftTimer[Handle].delay)
        {
            retval = SOFT_TIMER_ING;
        }
        else
        {
            retval = SOFT_TIMER_TIMEOUT;
        }
    }
    CRITICAL_SETCION_EXIT();

    return (retval);
}
