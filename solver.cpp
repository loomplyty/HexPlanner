/* Produced by CVXGEN, 2017-04-12 23:11:51 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double Solver::eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 33; i++)
    gap += work.z[i]*work.s[i];
  return gap;
}
void Solver::set_defaults(void) {
  settings.resid_tol = 1e-6;
  settings.eps = 1e-4;
  settings.max_iters = 25;
  settings.refine_steps = 1;
  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;
  settings.better_start = 1;
  settings.kkt_reg = 1e-7;
}
void Solver::setup_pointers(void) {
  work.y = work.x + 10;
  work.s = work.x + 16;
  work.z = work.x + 49;
  vars.f_1 = work.x + 0;
  vars.f_2 = work.x + 3;
  vars.f_3 = work.x + 6;
  vars.t = work.x + 9;
}
void Solver::setup_indexed_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  params.A[1] = params.A_1;
  params.A[2] = params.A_2;
  params.A[3] = params.A_3;
  params.Aconstraint[1] = params.Aconstraint_1;
  params.bconstraint[1] = params.bconstraint_1;
  params.Aconstraint[2] = params.Aconstraint_2;
  params.bconstraint[2] = params.bconstraint_2;
  params.Aconstraint[3] = params.Aconstraint_3;
  params.bconstraint[3] = params.bconstraint_3;
}
void Solver::setup_indexed_optvars(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  vars.f[1] = vars.f_1;
  vars.f[2] = vars.f_2;
  vars.f[3] = vars.f_3;
}
void Solver::setup_indexing(void) {
  setup_pointers();
  setup_indexed_params();
  setup_indexed_optvars();
}
void Solver::set_start(void) {
  int i;
  for (i = 0; i < 10; i++)
    work.x[i] = 0;
  for (i = 0; i < 6; i++)
    work.y[i] = 0;
  for (i = 0; i < 33; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
  for (i = 0; i < 33; i++)
    work.z[i] = settings.z_init;
}
double Solver::eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);
  objv = 0;
  for (i = 0; i < 10; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 10; i++)
    objv += work.q[i]*work.x[i];
  objv += 0;
  objv = -objv;
  return objv;
}
void Solver::fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 10;
  r3 = work.rhs + 43;
  r4 = work.rhs + 76;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 10; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 10; i++)
    r1[i] -= work.buffer[i] + work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 33; i++)
    r2[i] = -work.z[i];
  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 33; i++)
    r3[i] += -work.s[i] + work.h[i];
  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 6; i++)
    r4[i] += work.b[i];
}
void Solver::fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = work.rhs + 10;
  ds_aff = work.lhs_aff + 10;
  dz_aff = work.lhs_aff + 43;
  mu = 0;
  for (i = 0; i < 33; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 33; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 33; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 33; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.030303030303030304;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 10; i++)
    work.rhs[i] = 0;
  for (i = 43; i < 82; i++)
    work.rhs[i] = 0;
  for (i = 0; i < 33; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void Solver::refine(double *target, double *var) {
  int i, j;
  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 82; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 82; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 82; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double Solver::calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG(work.buffer, work.x);
  /* Add -s + h. */
  for (i = 0; i < 33; i++)
    work.buffer[i] += -work.s[i] + work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 33; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
double Solver::calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA(work.buffer, work.x);
  /* Add +b. */
  for (i = 0; i < 6; i++)
    work.buffer[i] += work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 6; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
void Solver::better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 33; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();
  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = work.lhs_aff;
  s = work.lhs_aff + 10;
  z = work.lhs_aff + 43;
  y = work.lhs_aff + 76;
  /* Just set x and y as is. */
  for (i = 0; i < 10; i++)
    work.x[i] = x[i];
  for (i = 0; i < 6; i++)
    work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 33; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 33; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 33; i++)
      work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 33; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 33; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 33; i++)
      work.z[i] = z[i] + alpha;
  }
}
void Solver::fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 10;
  r3 = work.rhs + 43;
  r4 = work.rhs + 76;
  for (i = 0; i < 10; i++)
    r1[i] = -work.q[i];
  for (i = 0; i < 33; i++)
    r2[i] = 0;
  for (i = 0; i < 33; i++)
    r3[i] = work.h[i];
  for (i = 0; i < 6; i++)
    r4[i] = work.b[i];
}
long Solver::solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  work.converged = 0;
  setup_pointers();
  pre_ops();
#ifndef ZERO_LIBRARY_MODE
  //if (settings.verbose)
  //  printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq();
  fillh();
  fillb();
  if (settings.better_start)
    better_start();
  else
    set_start();
  for (iter = 0; iter < settings.max_iters; iter++) {
    for (i = 0; i < 33; i++) {
      work.s_inv[i] = 1.0 / work.s[i];
      work.s_inv_z[i] = work.s_inv[i]*work.z[i];
    }
    work.block_33[0] = 0;
    fill_KKT();
    ldl_factor();
    /* Affine scaling directions. */
    fillrhs_aff();
    ldl_solve(work.rhs, work.lhs_aff);
    refine(work.rhs, work.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc();
    ldl_solve(work.rhs, work.lhs_cc);
    refine(work.rhs, work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 82; i++)
      work.lhs_aff[i] += work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 10;
    dz = work.lhs_aff + 43;
    dy = work.lhs_aff + 76;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 33; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 33; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 10; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 33; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 33; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 6; i++)
      work.y[i] += alpha*dy[i];
    work.gap = eval_gap();
    work.eq_resid_squared = calc_eq_resid_squared();
    work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose) {
      work.optval = eval_objv();
      //printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
      //    iter+1, work.optval, work.gap, sqrt(work.eq_resid_squared),
      //    sqrt(work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (work.gap < settings.eps)
        && (work.eq_resid_squared <= settings.resid_tol*settings.resid_tol)
        && (work.ineq_resid_squared <= settings.resid_tol*settings.resid_tol)
       ) {
      work.converged = 1;
      work.optval = eval_objv();
      return iter+1;
    }
  }
  return iter;
}

void Solver::load_default_data(void) {
	params.A_1[0] = 0.20319161029830202;
	params.A_1[1] = 0.8325912904724193;
	params.A_1[2] = -0.8363810443482227;
	params.A_1[3] = 0.04331042079065206;
	params.A_1[4] = 1.5717878173906188;
	params.A_1[5] = 1.5851723557337523;
	params.A_1[6] = -1.497658758144655;
	params.A_1[7] = -1.171028487447253;
	params.A_1[8] = -1.7941311867966805;
	params.A_1[9] = -0.23676062539745413;
	params.A_1[10] = -1.8804951564857322;
	params.A_1[11] = -0.17266710242115568;
	params.A_1[12] = 0.596576190459043;
	params.A_1[13] = -0.8860508694080989;
	params.A_1[14] = 0.7050196079205251;
	params.A_1[15] = 0.3634512696654033;
	params.A_1[16] = -1.9040724704913385;
	params.A_1[17] = 0.23541635196352795;
	params.A_2[0] = -0.9629902123701384;
	params.A_2[1] = -0.3395952119597214;
	params.A_2[2] = -0.865899672914725;
	params.A_2[3] = 0.7725516732519853;
	params.A_2[4] = -0.23818512931704205;
	params.A_2[5] = -1.372529046100147;
	params.A_2[6] = 0.17859607212737894;
	params.A_2[7] = 1.1212590580454682;
	params.A_2[8] = -0.774545870495281;
	params.A_2[9] = -1.1121684642712744;
	params.A_2[10] = -0.44811496977740495;
	params.A_2[11] = 1.7455345994417217;
	params.A_2[12] = 1.9039816898917352;
	params.A_2[13] = 0.6895347036512547;
	params.A_2[14] = 1.6113364341535923;
	params.A_2[15] = 1.383003485172717;
	params.A_2[16] = -0.48802383468444344;
	params.A_2[17] = -1.631131964513103;
	params.A_3[0] = 0.6136436100941447;
	params.A_3[1] = 0.2313630495538037;
	params.A_3[2] = -0.5537409477496875;
	params.A_3[3] = -1.0997819806406723;
	params.A_3[4] = -0.3739203344950055;
	params.A_3[5] = -0.12423900520332376;
	params.A_3[6] = -0.923057686995755;
	params.A_3[7] = -0.8328289030982696;
	params.A_3[8] = -0.16925440270808823;
	params.A_3[9] = 1.442135651787706;
	params.A_3[10] = 0.34501161787128565;
	params.A_3[11] = -0.8660485502711608;
	params.A_3[12] = -0.8880899735055947;
	params.A_3[13] = -0.1815116979122129;
	params.A_3[14] = -1.17835862158005;
	params.A_3[15] = -1.1944851558277074;
	params.A_3[16] = 0.05614023926976763;
	params.A_3[17] = -1.6510825248767813;
	params.b[0] = -0.06565787059365391;
	params.b[1] = -0.5512951504486665;
	params.b[2] = 0.8307464872626844;
	params.b[3] = 0.9869848924080182;
	params.b[4] = 0.7643716874230573;
	params.b[5] = 0.7567216550196565;
	params.Aconstraint_1[0] = -0.5055995034042868;
	params.Aconstraint_1[1] = 0.6725392189410702;
	params.Aconstraint_1[2] = -0.6406053441727284;
	params.Aconstraint_1[3] = 0.29117547947550015;
	params.Aconstraint_1[4] = -0.6967713677405021;
	params.Aconstraint_1[5] = -0.21941980294587182;
	params.Aconstraint_1[6] = -1.753884276680243;
	params.Aconstraint_1[7] = -1.0292983112626475;
	params.Aconstraint_1[8] = 1.8864104246942706;
	params.Aconstraint_1[9] = -1.077663182579704;
	params.Aconstraint_1[10] = 0.7659100437893209;
	params.Aconstraint_1[11] = 0.6019074328549583;
	params.Aconstraint_1[12] = 0.8957565577499285;
	params.Aconstraint_1[13] = -0.09964555746227477;
	params.Aconstraint_1[14] = 0.38665509840745127;
	params.Aconstraint_1[15] = -1.7321223042686946;
	params.Aconstraint_1[16] = -1.7097514487110663;
	params.Aconstraint_1[17] = -1.2040958948116867;
	params.Aconstraint_1[18] = -1.3925560119658358;
	params.Aconstraint_1[19] = -1.5995826216742213;
	params.Aconstraint_1[20] = -1.4828245415645833;
	params.Aconstraint_1[21] = 0.21311092723061398;
	params.Aconstraint_1[22] = -1.248740700304487;
	params.Aconstraint_1[23] = 1.808404972124833;
	params.Aconstraint_1[24] = 0.7264471152297065;
	params.Aconstraint_1[25] = 0.16407869343908477;
	params.Aconstraint_1[26] = 0.8287224032315907;
	params.Aconstraint_1[27] = -0.9444533161899464;
	params.Aconstraint_1[28] = 1.7069027370149112;
	params.Aconstraint_1[29] = 1.3567722311998827;
	params.Aconstraint_1[30] = 0.9052779937121489;
	params.Aconstraint_1[31] = -0.07904017565835986;
	params.Aconstraint_1[32] = 1.3684127435065871;
	params.bconstraint_1[0] = 0.979009293697437;
	params.bconstraint_1[1] = 0.6413036255984501;
	params.bconstraint_1[2] = 1.6559010680237511;
	params.bconstraint_1[3] = 0.5346622551502991;
	params.bconstraint_1[4] = -0.5362376605895625;
	params.bconstraint_1[5] = 0.2113782926017822;
	params.bconstraint_1[6] = -1.2144776931994525;
	params.bconstraint_1[7] = -1.2317108144255875;
	params.bconstraint_1[8] = 0.9026784957312834;
	params.bconstraint_1[9] = 1.1397468137245244;
	params.bconstraint_1[10] = 1.8883934547350631;
	params.Aconstraint_2[0] = 1.4038856681660068;
	params.Aconstraint_2[1] = 0.17437730638329096;
	params.Aconstraint_2[2] = -1.6408365219077408;
	params.Aconstraint_2[3] = -0.04450702153554875;
	params.Aconstraint_2[4] = 1.7117453902485025;
	params.Aconstraint_2[5] = 1.1504727980139053;
	params.Aconstraint_2[6] = -0.05962309578364744;
	params.Aconstraint_2[7] = -0.1788825540764547;
	params.Aconstraint_2[8] = -1.1280569263625857;
	params.Aconstraint_2[9] = -1.2911464767927057;
	params.Aconstraint_2[10] = -1.7055053231225696;
	params.Aconstraint_2[11] = 1.56957275034837;
	params.Aconstraint_2[12] = 0.5607064675962357;
	params.Aconstraint_2[13] = -1.4266707301147146;
	params.Aconstraint_2[14] = -0.3434923211351708;
	params.Aconstraint_2[15] = -1.8035643024085055;
	params.Aconstraint_2[16] = -1.1625066019105454;
	params.Aconstraint_2[17] = 0.9228324965161532;
	params.Aconstraint_2[18] = 0.6044910817663975;
	params.Aconstraint_2[19] = -0.0840868104920891;
	params.Aconstraint_2[20] = -0.900877978017443;
	params.Aconstraint_2[21] = 0.608892500264739;
	params.Aconstraint_2[22] = 1.8257980452695217;
	params.Aconstraint_2[23] = -0.25791777529922877;
	params.Aconstraint_2[24] = -1.7194699796493191;
	params.Aconstraint_2[25] = -1.7690740487081298;
	params.Aconstraint_2[26] = -1.6685159248097703;
	params.Aconstraint_2[27] = 1.8388287490128845;
	params.Aconstraint_2[28] = 0.16304334474597537;
	params.Aconstraint_2[29] = 1.3498497306788897;
	params.Aconstraint_2[30] = -1.3198658230514613;
	params.Aconstraint_2[31] = -0.9586197090843394;
	params.Aconstraint_2[32] = 0.7679100474913709;
	params.bconstraint_2[0] = 1.5822813125679343;
	params.bconstraint_2[1] = -0.6372460621593619;
	params.bconstraint_2[2] = -1.741307208038867;
	params.bconstraint_2[3] = 1.456478677642575;
	params.bconstraint_2[4] = -0.8365102166820959;
	params.bconstraint_2[5] = 0.9643296255982503;
	params.bconstraint_2[6] = -1.367865381194024;
	params.bconstraint_2[7] = 0.7798537405635035;
	params.bconstraint_2[8] = 1.3656784761245926;
	params.bconstraint_2[9] = 0.9086083149868371;
	params.bconstraint_2[10] = -0.5635699005460344;
	params.Aconstraint_3[0] = 0.9067590059607915;
	params.Aconstraint_3[1] = -1.4421315032701587;
	params.Aconstraint_3[2] = -0.7447235390671119;
	params.Aconstraint_3[3] = -0.32166897326822186;
	params.Aconstraint_3[4] = 1.5088481557772684;
	params.Aconstraint_3[5] = -1.385039165715428;
	params.Aconstraint_3[6] = 1.5204991609972622;
	params.Aconstraint_3[7] = 1.1958572768832156;
	params.Aconstraint_3[8] = 1.8864971883119228;
	params.Aconstraint_3[9] = -0.5291880667861584;
	params.Aconstraint_3[10] = -1.1802409243688836;
	params.Aconstraint_3[11] = -1.037718718661604;
	params.Aconstraint_3[12] = 1.3114512056856835;
	params.Aconstraint_3[13] = 1.8609125943756615;
	params.Aconstraint_3[14] = 0.7952399935216938;
	params.Aconstraint_3[15] = -0.07001183290468038;
	params.Aconstraint_3[16] = -0.8518009412754686;
	params.Aconstraint_3[17] = 1.3347515373726386;
	params.Aconstraint_3[18] = 1.4887180335977037;
	params.Aconstraint_3[19] = -1.6314736327976336;
	params.Aconstraint_3[20] = -1.1362021159208933;
	params.Aconstraint_3[21] = 1.327044361831466;
	params.Aconstraint_3[22] = 1.3932155883179842;
	params.Aconstraint_3[23] = -0.7413880049440107;
	params.Aconstraint_3[24] = -0.8828216126125747;
	params.Aconstraint_3[25] = -0.27673991192616;
	params.Aconstraint_3[26] = 0.15778600105866714;
	params.Aconstraint_3[27] = -1.6177327399735457;
	params.Aconstraint_3[28] = 1.3476485548544606;
	params.Aconstraint_3[29] = 0.13893948140528378;
	params.Aconstraint_3[30] = 1.0998712601636944;
	params.Aconstraint_3[31] = -1.0766549376946926;
	params.Aconstraint_3[32] = 1.8611734044254629;
	params.bconstraint_3[0] = 1.0041092292735172;
	params.bconstraint_3[1] = -0.6276245424321543;
	params.bconstraint_3[2] = 1.794110587839819;
	params.bconstraint_3[3] = 0.8020471158650913;
	params.bconstraint_3[4] = 1.362244341944948;
	params.bconstraint_3[5] = -1.8180107765765245;
	params.bconstraint_3[6] = -1.7774338357932473;
	params.bconstraint_3[7] = 0.9709490941985153;
	params.bconstraint_3[8] = -0.7812542682064318;
	params.bconstraint_3[9] = 0.0671374633729811;
	params.bconstraint_3[10] = -1.374950305314906;
}

 