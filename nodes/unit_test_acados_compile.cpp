/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_astrobee.h"


int main()
{

    nlp_solver_capsule *acados_ocp_capsule = astrobee_acados_create_capsule();
    int status = astrobee_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("astrobee_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = astrobee_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = astrobee_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = astrobee_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = astrobee_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = astrobee_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = astrobee_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int horizon = nlp_dims->N;
    int idxbx0[13];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;

    double lbx0[13];
    double ubx0[13];
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;
    lbx0[6] = 0;
    ubx0[6] = 0;
    lbx0[7] = 0;
    ubx0[7] = 0;
    lbx0[8] = 0;
    ubx0[8] = 0;
    lbx0[9] = 1;
    ubx0[9] = 1;
    lbx0[10] = 0;
    ubx0[10] = 0;
    lbx0[11] = 0;
    ubx0[11] = 0;
    lbx0[12] = 0;
    ubx0[12] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[13];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 0.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 1.0;
    x_init[10] = 0.0;
    x_init[11] = 0.0;
    x_init[12] = 0.0;

    // initial value for control input
    double u0[6];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    // set parameters
    double p[13];
    
    p[0] = 0;
    
    p[1] = 0;
    
    p[2] = 0;
    
    p[3] = 0;
    
    p[4] = 0;
    
    p[5] = 0;
    
    p[6] = 0;
    
    p[7] = 0;
    
    p[8] = 0.216;
    
    p[9] = 0.976;
    
    p[10] = 0;
    
    p[11] = 0;
    
    p[12] = 0;
    

    for (int ii = 0; ii <= horizon; ii++)
    {
        astrobee_acados_update_params(acados_ocp_capsule, ii, p, 13);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[13 * (horizon+1)];
    double utraj[6 * (horizon)];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i <= nlp_dims->N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = astrobee_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*13]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*6]);

    printf("\n--- xtraj ---\n");
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        printf("X %d: \n P:%f %f %f\n V: %f %f %f\n Q: %f %f %f %f\n W: %f %f %f\n", ii, 
                    xtraj[13*ii], xtraj[13*ii+1], xtraj[13*ii+2],
                    xtraj[13*ii+3], xtraj[13*ii+4], xtraj[13*ii+5],
                    xtraj[13*ii+6], xtraj[13*ii+7], xtraj[13*ii+8], xtraj[13*ii+9],
                    xtraj[13*ii+10], xtraj[13*ii+11], xtraj[13*ii+12]);
    //d_print_exp_tran_mat( 13, 5+1, xtraj, 13 );
    printf("\n--- utraj ---\n");
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        printf("U %d: \n F:%f %f %f\n T: %f %f %f\n", ii, 
                    utraj[6*ii], utraj[6*ii+1], utraj[6*ii+2],
                    utraj[6*ii+3], utraj[6*ii+4], utraj[6*ii+5]);
    //d_print_exp_tran_mat( 6, 5, utraj, 6 );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("astrobee_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("astrobee_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    astrobee_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = astrobee_acados_free(acados_ocp_capsule);
    if (status) {
        printf("astrobee_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = astrobee_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("astrobee_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
