/**
 * @file xmc_3p3z_filter_fixed.h
 * @date 2015-11-18
 *
 * @cond
 *********************************************************************************************************************
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes with
 * Infineon Technologies AG dave@infineon.com).
 *********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-11-18:
 *     - Initial <br>
 *
 * <b>Details of use for node configuration related APIs</b><br>
 * This file provides APIs for initializing the filter structure,XMC_3P3Z_InitFixed(), and
 * performing the 3 poles 3 zeros filtering to the input values using fix point values,XMC_3P3Z_FilterFixed()
 *
 *
 * @endcond
 *
 */
#ifndef XMC_3P3Z_FILTER_FIXED_H
#define XMC_3P3Z_FILTER_FIXED_H

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
/**< Minimum value  calculation macro */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/**< Maximum value  calculation macro */
#define MAX(a,b) ((a) > (b) ? (a) : (b))
/**< Fix point to float calculation macro */
#define FIX_TO_FLOAT( i, q ) ((float)(i) / ((unsigned)1<<(q)) )
/**< Fix point from float calculation macro */
#define FIX_FROM_FLOAT( f, q ) (int)((f) * ((unsigned)1<<(q)) )

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

/**
 * Structure defining the Filter calculation input parameters
 */
typedef struct XMC_3P3Z_DATA_FIXED
{
  volatile uint32_t*  m_pFeedBack;  /**< pointer to ADC register which is used for feedback */
  uint32_t            m_pOut;
  int32_t             m_Acc;
  int32_t             m_KpwmMin;
  int32_t             m_KpwmMax;
  int32_t             m_KpwmMaxNeg;
  int32_t*            m_Ref;        /**< ADC reference */
  int32_t             m_B[4];
  int32_t             m_A[4];
  int32_t             m_E[4];
  int32_t             m_U[3];
  int                 m_AShift;
  int                 m_BShift;
  int                 m_OShift;
} XMC_3P3Z_DATA_FIXED_t;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/

/**
 *
 * @param     [out] ptr Pointer to the filter structure
 * @param     [in]  cB0 B0 filter coefficient
 * @param     [in]  cB1 B1 filter coefficient
 * @param     [in]  cB2 B2 filter coefficient
 * @param     [in]  cB3 B3 filter coefficient
 * @param     [in]  cA1 A1 filter coefficient
 * @param     [in]  cA2 A2 filter coefficient
 * @param     [in]  cA3 A3 filter coefficient
 * @param     [in]  cK k factor of the filter
 * @param     [in]  ref Reference value for the VADC
 * @param     [in]  pwmMin 24 bit min PWM value.
 * @param     [in]  pwmMax 24 bit max PWM value.
 * @param     [out] pFeedBack pointer to ADC register.
 * @return None
 *
 * \par<b>Description:</b><br>
 *  This API uses the raw coefficients for the filter and fills the filter structure.
 *
 */
__STATIC_INLINE void XMC_3P3Z_InitFixed(XMC_3P3Z_DATA_FIXED_t* ptr, float cB0, float cB1, float cB2,
                                        float cB3, float cA1, float cA2, float cA3, float cK,
                                        uint16_t ref, uint16_t pwmMin, uint16_t pwmMax,
                                        volatile uint32_t* pFeedBack)
{
  int A_iq, B_iq, U_iq;
  int AU_iq, BE_iq;

  /*Resetting the filter structure values */
  memset( ptr, 0, sizeof(*ptr));

  /* Initializing Feedback, reference, and OUT values Out  */
  ptr->m_pFeedBack  = pFeedBack;
  *(ptr->m_Ref)    = ref;
  ptr->m_pOut   = 0;

  //          IQ int      iQ fract    Bit size
  //   B        -1          19          19
  //   E         12          0          13
  //   ------------------------
  //   sum BnEn  12         19          32

  B_iq = 19;
  //E_iq = 0;
  BE_iq = 19;

  /* Initializing coefficients */
  ptr->m_B[3] = FIX_FROM_FLOAT(cB3*cK,B_iq);
  ptr->m_B[2] = FIX_FROM_FLOAT(cB2*cK,B_iq);
  ptr->m_B[1] = FIX_FROM_FLOAT(cB1*cK,B_iq);
  ptr->m_B[0] = FIX_FROM_FLOAT(cB0*cK,B_iq);

  //          IQ int      iQ fract    Bit size
  //   A         1          14          16
  //   U         8          8           17
  //   ------------------------
  //   sum AnUn  9          22          32
  A_iq = 14;
  U_iq = 8;
  AU_iq = 22;
  ptr->m_A[3] = FIX_FROM_FLOAT(cA3,A_iq);
  ptr->m_A[2] = FIX_FROM_FLOAT(cA2,A_iq);
  ptr->m_A[1] = FIX_FROM_FLOAT(cA1,A_iq);

  /* Initializing maximum and minimum PWM value */
  ptr->m_KpwmMin        = pwmMin;
  ptr->m_KpwmMax        = FIX_FROM_FLOAT((pwmMax-1),U_iq);
  ptr->m_KpwmMaxNeg     = -ptr->m_KpwmMax;

  /* Initializing shifting values */
  ptr->m_AShift = AU_iq - BE_iq;
  ptr->m_BShift = BE_iq - U_iq;
  ptr->m_OShift = U_iq;
}

/**
 * @param     [out] Ptr Pointer to the filter structure
 *
 * \par<b>Description:</b><br>
 *  This API performs the 3p3z filtering by using fix point coefficients
 */
__STATIC_INLINE void XMC_3P3Z_FilterFixed( XMC_3P3Z_DATA_FIXED_t* ptr )
{
    int32_t acc;

    // acc (iq9.22) = An (iq-1.19) * Un (iq(8.8)
    acc  = ptr->m_A[3]*ptr->m_U[2];
    ptr->m_U[2] = ptr->m_U[1];

    acc += ptr->m_A[2]*ptr->m_U[1];
    ptr->m_U[1] = ptr->m_U[0];

    acc += ptr->m_A[1]*ptr->m_U[0];
    acc = acc >> ptr->m_AShift;  //iq is now iq9.19

    // acc (iq12.19) = Bn (iq1.14) * En (iq(12.0)
    acc += ptr->m_B[3]*ptr->m_E[2];
    ptr->m_E[2] = ptr->m_E[1];

    acc += ptr->m_B[2]*ptr->m_E[1];
    ptr->m_E[1] = ptr->m_E[0];

    acc += ptr->m_B[1]*ptr->m_E[0];
    ptr->m_E[0] = *(ptr->m_Ref)-((uint16_t)*ptr->m_pFeedBack);
    acc += ptr->m_B[0]*ptr->m_E[0];

    //our number is now a iq12.19, but we need to store U as a iq8.8
    acc = acc >> ptr->m_BShift; //now its a iq12.8

    acc = MIN( acc , ptr->m_KpwmMax );
    acc = MAX( acc , ptr->m_KpwmMaxNeg ); //now its a iq8.8
    ptr->m_U[0] = acc;

    acc = acc >> ptr->m_OShift; //now its a iq8.0
    if ( acc < ptr->m_KpwmMin) acc = ptr->m_KpwmMin;
    ptr->m_pOut = acc;
}


#endif /* #ifndef XMC_3P3Z_FILTER_FIXED_H */
