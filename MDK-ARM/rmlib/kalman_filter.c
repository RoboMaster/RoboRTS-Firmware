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
/** @file kalman_filter.c
 *  @version 1.0
 *  @date Apr 2018
 *
 *  @brief kalman filter realization
 *
 *  @copyright 2018 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "kalman_filter.h"

//float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
//float P_data[4] = {2, 0, 0, 2};
//float AT_data[4], HT_data[4];
//float A_data[4] = {1, 0.001, 0, 1};
//float H_data[4] = {1, 0, 0, 1};
//float Q_data[4] = {1, 0, 0, 1};
//float R_data[4] = {2000, 0, 0, 5000};


float matrix_value1;
float matrix_value2;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);
  mat_init(&F->A,2,2,(float *)I->A_data);
  mat_init(&F->H,2,2,(float *)I->H_data);
  mat_init(&F->Q,2,2,(float *)I->Q_data);
  mat_init(&F->R,2,2,(float *)I->R_data);
  mat_init(&F->P,2,2,(float *)I->P_data);
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
  mat_init(&F->K,2,2,(float *)I->K_data);
  mat_init(&F->AT,2,2,(float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}



float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

  F->z.pData[0] = signal1;
  F->z.pData[1] = signal2;

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}


#ifdef IIR_FILTER
double NUM[7] = 
{
  0.000938328292536,
 -0.005375225952906,
  0.01306656496967,
 -0.01725922344534,
  0.01306656496967,
 -0.005375225952906,
  0.000938328292536
};
double DEN[7] = 
{
   1,
  -5.733703351811,
  13.70376013927,
 -17.47505866491,
  12.53981348666,
  -4.800968865471,
   0.7661573674342
};

typedef struct
{
  double raw_value;
  double xbuf[7];
  double ybuf[7];
  double filtered_value;
} iir_filter_t;

iir_filter_t yaw_msg_t, pit_msg_t;


double iir_filter(iir_filter_t *F)
{
  int i;
  for(i = 6; i > 0; i--)
  {
    F->xbuf[i] = F->xbuf[i-1];
    F->ybuf[i] = F->ybuf[i-1];
  }
  
  F->xbuf[0] = F->raw_value;
  F->ybuf[0] = NUM[0] * F->xbuf[0];
  
  for(i = 1; i < 7; i++)
  {
    F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
  }
  
  F->filtered_value = F->ybuf[0];
  
  return F->filtered_value;
}
#endif


