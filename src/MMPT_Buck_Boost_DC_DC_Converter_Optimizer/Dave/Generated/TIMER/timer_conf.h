/*********************************************************************************************************************
* DAVE APP Name : TIMER       APP Version: 4.1.12
*
* NOTE:
* This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
*********************************************************************************************************************/

/**
 * @cond
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-02-16:
 *     - Initial version<br>
 *
 * 2015-05-08:
 *     - Updated the file guards.<br>
 *     - Added macro generation for sahdow tarnsfer mask and instance handle<br>
 *
 * @endcond
 *
 */

#ifndef TIMER_CONF_H 
#define TIMER_CONF_H

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define TIMER_MAJOR_VERSION (4U)
#define TIMER_MINOR_VERSION (1U)
#define TIMER_PATCH_VERSION (12U) 
 

/* Moudule and Kernel Pointers */
#define TIMER_0_KERNEL_PTR (XMC_CCU4_MODULE_t*)(void *)CCU40_BASE
#define TIMER_0_SLICE_PTR  (XMC_CCU4_SLICE_t*)(void *)CCU40_CC40
/* Shadow transfer masks */
#define TIMER_0_SLICE_SH_MSK      XMC_CCU4_SHADOW_TRANSFER_SLICE_0
#define TIMER_0_PRESCALER_SH_MSK  XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_0

/* Moudule and Kernel Pointers */
#define TIMER_1_KERNEL_PTR (XMC_CCU4_MODULE_t*)(void *)CCU40_BASE
#define TIMER_1_SLICE_PTR  (XMC_CCU4_SLICE_t*)(void *)CCU40_CC41
/* Shadow transfer masks */
#define TIMER_1_SLICE_SH_MSK      XMC_CCU4_SHADOW_TRANSFER_SLICE_1
#define TIMER_1_PRESCALER_SH_MSK  XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1

/** This is used to calculate the time in GetTime API */
#define TIMER_CLK_CONST_SCALED (6710886400U)

/** This enables the functionality for the CCU4 timer */
#define TIMER_CCU4_USED

#endif /* TIMER_CONF_H */

