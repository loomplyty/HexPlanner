/* Produced by CVXGEN, 2017-04-12 23:11:51 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void Solver::multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A_1[0])-rhs[1]*(params.A_1[6])-rhs[2]*(params.A_1[12])-rhs[3]*(params.A_2[0])-rhs[4]*(params.A_2[6])-rhs[5]*(params.A_2[12])-rhs[6]*(params.A_3[0])-rhs[7]*(params.A_3[6])-rhs[8]*(params.A_3[12]);
  lhs[1] = -rhs[0]*(params.A_1[1])-rhs[1]*(params.A_1[7])-rhs[2]*(params.A_1[13])-rhs[3]*(params.A_2[1])-rhs[4]*(params.A_2[7])-rhs[5]*(params.A_2[13])-rhs[6]*(params.A_3[1])-rhs[7]*(params.A_3[7])-rhs[8]*(params.A_3[13]);
  lhs[2] = -rhs[0]*(params.A_1[2])-rhs[1]*(params.A_1[8])-rhs[2]*(params.A_1[14])-rhs[3]*(params.A_2[2])-rhs[4]*(params.A_2[8])-rhs[5]*(params.A_2[14])-rhs[6]*(params.A_3[2])-rhs[7]*(params.A_3[8])-rhs[8]*(params.A_3[14]);
  lhs[3] = -rhs[0]*(params.A_1[3])-rhs[1]*(params.A_1[9])-rhs[2]*(params.A_1[15])-rhs[3]*(params.A_2[3])-rhs[4]*(params.A_2[9])-rhs[5]*(params.A_2[15])-rhs[6]*(params.A_3[3])-rhs[7]*(params.A_3[9])-rhs[8]*(params.A_3[15]);
  lhs[4] = -rhs[0]*(params.A_1[4])-rhs[1]*(params.A_1[10])-rhs[2]*(params.A_1[16])-rhs[3]*(params.A_2[4])-rhs[4]*(params.A_2[10])-rhs[5]*(params.A_2[16])-rhs[6]*(params.A_3[4])-rhs[7]*(params.A_3[10])-rhs[8]*(params.A_3[16]);
  lhs[5] = -rhs[0]*(params.A_1[5])-rhs[1]*(params.A_1[11])-rhs[2]*(params.A_1[17])-rhs[3]*(params.A_2[5])-rhs[4]*(params.A_2[11])-rhs[5]*(params.A_2[17])-rhs[6]*(params.A_3[5])-rhs[7]*(params.A_3[11])-rhs[8]*(params.A_3[17]);
}
void Solver::multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A_1[0])-rhs[1]*(params.A_1[1])-rhs[2]*(params.A_1[2])-rhs[3]*(params.A_1[3])-rhs[4]*(params.A_1[4])-rhs[5]*(params.A_1[5]);
  lhs[1] = -rhs[0]*(params.A_1[6])-rhs[1]*(params.A_1[7])-rhs[2]*(params.A_1[8])-rhs[3]*(params.A_1[9])-rhs[4]*(params.A_1[10])-rhs[5]*(params.A_1[11]);
  lhs[2] = -rhs[0]*(params.A_1[12])-rhs[1]*(params.A_1[13])-rhs[2]*(params.A_1[14])-rhs[3]*(params.A_1[15])-rhs[4]*(params.A_1[16])-rhs[5]*(params.A_1[17]);
  lhs[3] = -rhs[0]*(params.A_2[0])-rhs[1]*(params.A_2[1])-rhs[2]*(params.A_2[2])-rhs[3]*(params.A_2[3])-rhs[4]*(params.A_2[4])-rhs[5]*(params.A_2[5]);
  lhs[4] = -rhs[0]*(params.A_2[6])-rhs[1]*(params.A_2[7])-rhs[2]*(params.A_2[8])-rhs[3]*(params.A_2[9])-rhs[4]*(params.A_2[10])-rhs[5]*(params.A_2[11]);
  lhs[5] = -rhs[0]*(params.A_2[12])-rhs[1]*(params.A_2[13])-rhs[2]*(params.A_2[14])-rhs[3]*(params.A_2[15])-rhs[4]*(params.A_2[16])-rhs[5]*(params.A_2[17]);
  lhs[6] = -rhs[0]*(params.A_3[0])-rhs[1]*(params.A_3[1])-rhs[2]*(params.A_3[2])-rhs[3]*(params.A_3[3])-rhs[4]*(params.A_3[4])-rhs[5]*(params.A_3[5]);
  lhs[7] = -rhs[0]*(params.A_3[6])-rhs[1]*(params.A_3[7])-rhs[2]*(params.A_3[8])-rhs[3]*(params.A_3[9])-rhs[4]*(params.A_3[10])-rhs[5]*(params.A_3[11]);
  lhs[8] = -rhs[0]*(params.A_3[12])-rhs[1]*(params.A_3[13])-rhs[2]*(params.A_3[14])-rhs[3]*(params.A_3[15])-rhs[4]*(params.A_3[16])-rhs[5]*(params.A_3[17]);
  lhs[9] = 0;
}
void Solver::multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.Aconstraint_1[0])-rhs[1]*(-params.Aconstraint_1[11])-rhs[2]*(-params.Aconstraint_1[22])-rhs[9]*(1);
  lhs[1] = -rhs[0]*(-params.Aconstraint_1[1])-rhs[1]*(-params.Aconstraint_1[12])-rhs[2]*(-params.Aconstraint_1[23])-rhs[9]*(1);
  lhs[2] = -rhs[0]*(-params.Aconstraint_1[2])-rhs[1]*(-params.Aconstraint_1[13])-rhs[2]*(-params.Aconstraint_1[24])-rhs[9]*(1);
  lhs[3] = -rhs[0]*(-params.Aconstraint_1[3])-rhs[1]*(-params.Aconstraint_1[14])-rhs[2]*(-params.Aconstraint_1[25])-rhs[9]*(1);
  lhs[4] = -rhs[0]*(-params.Aconstraint_1[4])-rhs[1]*(-params.Aconstraint_1[15])-rhs[2]*(-params.Aconstraint_1[26])-rhs[9]*(1);
  lhs[5] = -rhs[0]*(-params.Aconstraint_1[5])-rhs[1]*(-params.Aconstraint_1[16])-rhs[2]*(-params.Aconstraint_1[27])-rhs[9]*(1);
  lhs[6] = -rhs[0]*(-params.Aconstraint_1[6])-rhs[1]*(-params.Aconstraint_1[17])-rhs[2]*(-params.Aconstraint_1[28])-rhs[9]*(1);
  lhs[7] = -rhs[0]*(-params.Aconstraint_1[7])-rhs[1]*(-params.Aconstraint_1[18])-rhs[2]*(-params.Aconstraint_1[29])-rhs[9]*(1);
  lhs[8] = -rhs[0]*(-params.Aconstraint_1[8])-rhs[1]*(-params.Aconstraint_1[19])-rhs[2]*(-params.Aconstraint_1[30])-rhs[9]*(1);
  lhs[9] = -rhs[0]*(-params.Aconstraint_1[9])-rhs[1]*(-params.Aconstraint_1[20])-rhs[2]*(-params.Aconstraint_1[31])-rhs[9]*(1);
  lhs[10] = -rhs[0]*(-params.Aconstraint_1[10])-rhs[1]*(-params.Aconstraint_1[21])-rhs[2]*(-params.Aconstraint_1[32])-rhs[9]*(1);
  lhs[11] = -rhs[3]*(-params.Aconstraint_2[0])-rhs[4]*(-params.Aconstraint_2[11])-rhs[5]*(-params.Aconstraint_2[22])-rhs[9]*(1);
  lhs[12] = -rhs[3]*(-params.Aconstraint_2[1])-rhs[4]*(-params.Aconstraint_2[12])-rhs[5]*(-params.Aconstraint_2[23])-rhs[9]*(1);
  lhs[13] = -rhs[3]*(-params.Aconstraint_2[2])-rhs[4]*(-params.Aconstraint_2[13])-rhs[5]*(-params.Aconstraint_2[24])-rhs[9]*(1);
  lhs[14] = -rhs[3]*(-params.Aconstraint_2[3])-rhs[4]*(-params.Aconstraint_2[14])-rhs[5]*(-params.Aconstraint_2[25])-rhs[9]*(1);
  lhs[15] = -rhs[3]*(-params.Aconstraint_2[4])-rhs[4]*(-params.Aconstraint_2[15])-rhs[5]*(-params.Aconstraint_2[26])-rhs[9]*(1);
  lhs[16] = -rhs[3]*(-params.Aconstraint_2[5])-rhs[4]*(-params.Aconstraint_2[16])-rhs[5]*(-params.Aconstraint_2[27])-rhs[9]*(1);
  lhs[17] = -rhs[3]*(-params.Aconstraint_2[6])-rhs[4]*(-params.Aconstraint_2[17])-rhs[5]*(-params.Aconstraint_2[28])-rhs[9]*(1);
  lhs[18] = -rhs[3]*(-params.Aconstraint_2[7])-rhs[4]*(-params.Aconstraint_2[18])-rhs[5]*(-params.Aconstraint_2[29])-rhs[9]*(1);
  lhs[19] = -rhs[3]*(-params.Aconstraint_2[8])-rhs[4]*(-params.Aconstraint_2[19])-rhs[5]*(-params.Aconstraint_2[30])-rhs[9]*(1);
  lhs[20] = -rhs[3]*(-params.Aconstraint_2[9])-rhs[4]*(-params.Aconstraint_2[20])-rhs[5]*(-params.Aconstraint_2[31])-rhs[9]*(1);
  lhs[21] = -rhs[3]*(-params.Aconstraint_2[10])-rhs[4]*(-params.Aconstraint_2[21])-rhs[5]*(-params.Aconstraint_2[32])-rhs[9]*(1);
  lhs[22] = -rhs[6]*(-params.Aconstraint_3[0])-rhs[7]*(-params.Aconstraint_3[11])-rhs[8]*(-params.Aconstraint_3[22])-rhs[9]*(1);
  lhs[23] = -rhs[6]*(-params.Aconstraint_3[1])-rhs[7]*(-params.Aconstraint_3[12])-rhs[8]*(-params.Aconstraint_3[23])-rhs[9]*(1);
  lhs[24] = -rhs[6]*(-params.Aconstraint_3[2])-rhs[7]*(-params.Aconstraint_3[13])-rhs[8]*(-params.Aconstraint_3[24])-rhs[9]*(1);
  lhs[25] = -rhs[6]*(-params.Aconstraint_3[3])-rhs[7]*(-params.Aconstraint_3[14])-rhs[8]*(-params.Aconstraint_3[25])-rhs[9]*(1);
  lhs[26] = -rhs[6]*(-params.Aconstraint_3[4])-rhs[7]*(-params.Aconstraint_3[15])-rhs[8]*(-params.Aconstraint_3[26])-rhs[9]*(1);
  lhs[27] = -rhs[6]*(-params.Aconstraint_3[5])-rhs[7]*(-params.Aconstraint_3[16])-rhs[8]*(-params.Aconstraint_3[27])-rhs[9]*(1);
  lhs[28] = -rhs[6]*(-params.Aconstraint_3[6])-rhs[7]*(-params.Aconstraint_3[17])-rhs[8]*(-params.Aconstraint_3[28])-rhs[9]*(1);
  lhs[29] = -rhs[6]*(-params.Aconstraint_3[7])-rhs[7]*(-params.Aconstraint_3[18])-rhs[8]*(-params.Aconstraint_3[29])-rhs[9]*(1);
  lhs[30] = -rhs[6]*(-params.Aconstraint_3[8])-rhs[7]*(-params.Aconstraint_3[19])-rhs[8]*(-params.Aconstraint_3[30])-rhs[9]*(1);
  lhs[31] = -rhs[6]*(-params.Aconstraint_3[9])-rhs[7]*(-params.Aconstraint_3[20])-rhs[8]*(-params.Aconstraint_3[31])-rhs[9]*(1);
  lhs[32] = -rhs[6]*(-params.Aconstraint_3[10])-rhs[7]*(-params.Aconstraint_3[21])-rhs[8]*(-params.Aconstraint_3[32])-rhs[9]*(1);
}
void Solver::multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.Aconstraint_1[0])-rhs[1]*(-params.Aconstraint_1[1])-rhs[2]*(-params.Aconstraint_1[2])-rhs[3]*(-params.Aconstraint_1[3])-rhs[4]*(-params.Aconstraint_1[4])-rhs[5]*(-params.Aconstraint_1[5])-rhs[6]*(-params.Aconstraint_1[6])-rhs[7]*(-params.Aconstraint_1[7])-rhs[8]*(-params.Aconstraint_1[8])-rhs[9]*(-params.Aconstraint_1[9])-rhs[10]*(-params.Aconstraint_1[10]);
  lhs[1] = -rhs[0]*(-params.Aconstraint_1[11])-rhs[1]*(-params.Aconstraint_1[12])-rhs[2]*(-params.Aconstraint_1[13])-rhs[3]*(-params.Aconstraint_1[14])-rhs[4]*(-params.Aconstraint_1[15])-rhs[5]*(-params.Aconstraint_1[16])-rhs[6]*(-params.Aconstraint_1[17])-rhs[7]*(-params.Aconstraint_1[18])-rhs[8]*(-params.Aconstraint_1[19])-rhs[9]*(-params.Aconstraint_1[20])-rhs[10]*(-params.Aconstraint_1[21]);
  lhs[2] = -rhs[0]*(-params.Aconstraint_1[22])-rhs[1]*(-params.Aconstraint_1[23])-rhs[2]*(-params.Aconstraint_1[24])-rhs[3]*(-params.Aconstraint_1[25])-rhs[4]*(-params.Aconstraint_1[26])-rhs[5]*(-params.Aconstraint_1[27])-rhs[6]*(-params.Aconstraint_1[28])-rhs[7]*(-params.Aconstraint_1[29])-rhs[8]*(-params.Aconstraint_1[30])-rhs[9]*(-params.Aconstraint_1[31])-rhs[10]*(-params.Aconstraint_1[32]);
  lhs[3] = -rhs[11]*(-params.Aconstraint_2[0])-rhs[12]*(-params.Aconstraint_2[1])-rhs[13]*(-params.Aconstraint_2[2])-rhs[14]*(-params.Aconstraint_2[3])-rhs[15]*(-params.Aconstraint_2[4])-rhs[16]*(-params.Aconstraint_2[5])-rhs[17]*(-params.Aconstraint_2[6])-rhs[18]*(-params.Aconstraint_2[7])-rhs[19]*(-params.Aconstraint_2[8])-rhs[20]*(-params.Aconstraint_2[9])-rhs[21]*(-params.Aconstraint_2[10]);
  lhs[4] = -rhs[11]*(-params.Aconstraint_2[11])-rhs[12]*(-params.Aconstraint_2[12])-rhs[13]*(-params.Aconstraint_2[13])-rhs[14]*(-params.Aconstraint_2[14])-rhs[15]*(-params.Aconstraint_2[15])-rhs[16]*(-params.Aconstraint_2[16])-rhs[17]*(-params.Aconstraint_2[17])-rhs[18]*(-params.Aconstraint_2[18])-rhs[19]*(-params.Aconstraint_2[19])-rhs[20]*(-params.Aconstraint_2[20])-rhs[21]*(-params.Aconstraint_2[21]);
  lhs[5] = -rhs[11]*(-params.Aconstraint_2[22])-rhs[12]*(-params.Aconstraint_2[23])-rhs[13]*(-params.Aconstraint_2[24])-rhs[14]*(-params.Aconstraint_2[25])-rhs[15]*(-params.Aconstraint_2[26])-rhs[16]*(-params.Aconstraint_2[27])-rhs[17]*(-params.Aconstraint_2[28])-rhs[18]*(-params.Aconstraint_2[29])-rhs[19]*(-params.Aconstraint_2[30])-rhs[20]*(-params.Aconstraint_2[31])-rhs[21]*(-params.Aconstraint_2[32]);
  lhs[6] = -rhs[22]*(-params.Aconstraint_3[0])-rhs[23]*(-params.Aconstraint_3[1])-rhs[24]*(-params.Aconstraint_3[2])-rhs[25]*(-params.Aconstraint_3[3])-rhs[26]*(-params.Aconstraint_3[4])-rhs[27]*(-params.Aconstraint_3[5])-rhs[28]*(-params.Aconstraint_3[6])-rhs[29]*(-params.Aconstraint_3[7])-rhs[30]*(-params.Aconstraint_3[8])-rhs[31]*(-params.Aconstraint_3[9])-rhs[32]*(-params.Aconstraint_3[10]);
  lhs[7] = -rhs[22]*(-params.Aconstraint_3[11])-rhs[23]*(-params.Aconstraint_3[12])-rhs[24]*(-params.Aconstraint_3[13])-rhs[25]*(-params.Aconstraint_3[14])-rhs[26]*(-params.Aconstraint_3[15])-rhs[27]*(-params.Aconstraint_3[16])-rhs[28]*(-params.Aconstraint_3[17])-rhs[29]*(-params.Aconstraint_3[18])-rhs[30]*(-params.Aconstraint_3[19])-rhs[31]*(-params.Aconstraint_3[20])-rhs[32]*(-params.Aconstraint_3[21]);
  lhs[8] = -rhs[22]*(-params.Aconstraint_3[22])-rhs[23]*(-params.Aconstraint_3[23])-rhs[24]*(-params.Aconstraint_3[24])-rhs[25]*(-params.Aconstraint_3[25])-rhs[26]*(-params.Aconstraint_3[26])-rhs[27]*(-params.Aconstraint_3[27])-rhs[28]*(-params.Aconstraint_3[28])-rhs[29]*(-params.Aconstraint_3[29])-rhs[30]*(-params.Aconstraint_3[30])-rhs[31]*(-params.Aconstraint_3[31])-rhs[32]*(-params.Aconstraint_3[32]);
  lhs[9] = -rhs[0]*(1)-rhs[1]*(1)-rhs[2]*(1)-rhs[3]*(1)-rhs[4]*(1)-rhs[5]*(1)-rhs[6]*(1)-rhs[7]*(1)-rhs[8]*(1)-rhs[9]*(1)-rhs[10]*(1)-rhs[11]*(1)-rhs[12]*(1)-rhs[13]*(1)-rhs[14]*(1)-rhs[15]*(1)-rhs[16]*(1)-rhs[17]*(1)-rhs[18]*(1)-rhs[19]*(1)-rhs[20]*(1)-rhs[21]*(1)-rhs[22]*(1)-rhs[23]*(1)-rhs[24]*(1)-rhs[25]*(1)-rhs[26]*(1)-rhs[27]*(1)-rhs[28]*(1)-rhs[29]*(1)-rhs[30]*(1)-rhs[31]*(1)-rhs[32]*(1);
}
void Solver::multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
}
void Solver::fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = -1;
}
void Solver::fillh(void) {
  work.h[0] = -params.bconstraint_1[0];
  work.h[1] = -params.bconstraint_1[1];
  work.h[2] = -params.bconstraint_1[2];
  work.h[3] = -params.bconstraint_1[3];
  work.h[4] = -params.bconstraint_1[4];
  work.h[5] = -params.bconstraint_1[5];
  work.h[6] = -params.bconstraint_1[6];
  work.h[7] = -params.bconstraint_1[7];
  work.h[8] = -params.bconstraint_1[8];
  work.h[9] = -params.bconstraint_1[9];
  work.h[10] = -params.bconstraint_1[10];
  work.h[11] = -params.bconstraint_2[0];
  work.h[12] = -params.bconstraint_2[1];
  work.h[13] = -params.bconstraint_2[2];
  work.h[14] = -params.bconstraint_2[3];
  work.h[15] = -params.bconstraint_2[4];
  work.h[16] = -params.bconstraint_2[5];
  work.h[17] = -params.bconstraint_2[6];
  work.h[18] = -params.bconstraint_2[7];
  work.h[19] = -params.bconstraint_2[8];
  work.h[20] = -params.bconstraint_2[9];
  work.h[21] = -params.bconstraint_2[10];
  work.h[22] = -params.bconstraint_3[0];
  work.h[23] = -params.bconstraint_3[1];
  work.h[24] = -params.bconstraint_3[2];
  work.h[25] = -params.bconstraint_3[3];
  work.h[26] = -params.bconstraint_3[4];
  work.h[27] = -params.bconstraint_3[5];
  work.h[28] = -params.bconstraint_3[6];
  work.h[29] = -params.bconstraint_3[7];
  work.h[30] = -params.bconstraint_3[8];
  work.h[31] = -params.bconstraint_3[9];
  work.h[32] = -params.bconstraint_3[10];
}
void Solver::fillb(void) {
  work.b[0] = params.b[0];
  work.b[1] = params.b[1];
  work.b[2] = params.b[2];
  work.b[3] = params.b[3];
  work.b[4] = params.b[4];
  work.b[5] = params.b[5];
}
void Solver::pre_ops(void) {
}
