/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

mxArray* ModelFcn_1_f = NULL;
mxArray* ModelFcn_1_jac = NULL;
mxArray* ModelFcn_1T  = NULL;
mxArray* ModelFcn_1X  = NULL;
mxArray* ModelFcn_1XA = NULL;
mxArray* ModelFcn_1U  = NULL;
mxArray* ModelFcn_1P  = NULL;
mxArray* ModelFcn_1W  = NULL;
mxArray* ModelFcn_1DX = NULL;
unsigned int ModelFcn_1NT  = 0;
unsigned int ModelFcn_1NX  = 0;
unsigned int ModelFcn_1NXA = 0;
unsigned int ModelFcn_1NU  = 0;
unsigned int ModelFcn_1NP  = 0;
unsigned int ModelFcn_1NW  = 0;
unsigned int ModelFcn_1NDX = 0;
unsigned int jacobianNumber_1 = -1;
double* f_store_1             = NULL;
double* J_store_1             = NULL;

void clearAllGlobals1( ){ 
    if ( f_store_1 != NULL ){
        f_store_1 = NULL;
    }

    if ( J_store_1 != NULL ){
        J_store_1 = NULL;
    }

    if ( ModelFcn_1_f != NULL ){
        mxDestroyArray( ModelFcn_1_f );
        ModelFcn_1_f = NULL;
    }

    if ( ModelFcn_1T != NULL ){
        mxDestroyArray( ModelFcn_1T );
        ModelFcn_1T = NULL;
    }

    if ( ModelFcn_1X != NULL ){
        mxDestroyArray( ModelFcn_1X );
        ModelFcn_1X = NULL;
    }

    if ( ModelFcn_1XA != NULL ){
        mxDestroyArray( ModelFcn_1XA );
        ModelFcn_1XA = NULL;
    }

    if ( ModelFcn_1U != NULL ){
        mxDestroyArray( ModelFcn_1U );
        ModelFcn_1U = NULL;
    }

    if ( ModelFcn_1P != NULL ){
        mxDestroyArray( ModelFcn_1P );
        ModelFcn_1P = NULL;
    }

    if ( ModelFcn_1W != NULL ){
        mxDestroyArray( ModelFcn_1W );
        ModelFcn_1W = NULL;
    }

    if ( ModelFcn_1DX != NULL ){
        mxDestroyArray( ModelFcn_1DX );
        ModelFcn_1DX = NULL;
    }

    if ( ModelFcn_1_jac != NULL ){
        mxDestroyArray( ModelFcn_1_jac );
        ModelFcn_1_jac = NULL;
    }

    ModelFcn_1NT  = 0;
    ModelFcn_1NX  = 0;
    ModelFcn_1NXA = 0;
    ModelFcn_1NU  = 0;
    ModelFcn_1NP  = 0;
    ModelFcn_1NW  = 0;
    ModelFcn_1NDX = 0;
    jacobianNumber_1 = -1;
}

void genericODE1( double* x, double* f, void *userData ){
    unsigned int i;
    double* tt = mxGetPr( ModelFcn_1T );
    tt[0] = x[0];
    double* xx = mxGetPr( ModelFcn_1X );
    for( i=0; i<ModelFcn_1NX; ++i )
        xx[i] = x[i+1];
    double* uu = mxGetPr( ModelFcn_1U );
    for( i=0; i<ModelFcn_1NU; ++i )
        uu[i] = x[i+1+ModelFcn_1NX];
    double* pp = mxGetPr( ModelFcn_1P );
    for( i=0; i<ModelFcn_1NP; ++i )
        pp[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU];
    double* ww = mxGetPr( ModelFcn_1W );
    for( i=0; i<ModelFcn_1NW; ++i )
        ww[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP];
    mxArray* FF = NULL;
    mxArray* argIn[]  = { ModelFcn_1_f,ModelFcn_1T,ModelFcn_1X,ModelFcn_1U,ModelFcn_1P,ModelFcn_1W };
    mxArray* argOut[] = { FF };

    mexCallMATLAB( 1,argOut, 6,argIn,"generic_ode" );
    double* ff = mxGetPr( *argOut );
    for( i=0; i<ModelFcn_1NX; ++i ){
        f[i] = ff[i];
    }
    mxDestroyArray( *argOut );
}

void genericJacobian1( int number, double* x, double* seed, double* f, double* df, void *userData  ){
    unsigned int i, j;
    double* ff;
    double* J;
    if (J_store_1 == NULL){
        J_store_1 = (double*) calloc ((ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW)*(ModelFcn_1NX),sizeof(double));
        f_store_1 = (double*) calloc (ModelFcn_1NX,sizeof(double));
    }
    if ( (int) jacobianNumber_1 == number){
        J = J_store_1;
        ff = f_store_1;
        for( i=0; i<ModelFcn_1NX; ++i ) {
            df[i] = 0;
            f[i] = 0;
            for (j=0; j < ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW; ++j){
                df[i] += J[(j*(ModelFcn_1NX))+i]*seed[j+1]; 
            }
        }
        for( i=0; i<ModelFcn_1NX; ++i ){
            f[i] = ff[i];
        }
    }else{
        jacobianNumber_1 = number; 
        double* tt = mxGetPr( ModelFcn_1T );
        tt[0] = x[0];
        double* xx = mxGetPr( ModelFcn_1X );
        for( i=0; i<ModelFcn_1NX; ++i )
            xx[i] = x[i+1];
        double* uu = mxGetPr( ModelFcn_1U );
        for( i=0; i<ModelFcn_1NU; ++i )
            uu[i] = x[i+1+ModelFcn_1NX];
        double* pp = mxGetPr( ModelFcn_1P );
        for( i=0; i<ModelFcn_1NP; ++i )
            pp[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU];
        double* ww = mxGetPr( ModelFcn_1W );
            for( i=0; i<ModelFcn_1NW; ++i )
        ww[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP];
        mxArray* FF = NULL;
        mxArray* argIn[]  = { ModelFcn_1_jac,ModelFcn_1T,ModelFcn_1X,ModelFcn_1U,ModelFcn_1P,ModelFcn_1W };
        mxArray* argOut[] = { FF };
        mexCallMATLAB( 1,argOut, 6,argIn,"generic_jacobian" );
        unsigned int rowLen = mxGetM(*argOut);
        unsigned int colLen = mxGetN(*argOut);
        if (rowLen != ModelFcn_1NX){
            mexErrMsgTxt( "ERROR: Jacobian matrix rows do not match (should be ModelFcn_1NX). " );
        }
        if (colLen != ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW){
            mexErrMsgTxt( "ERROR: Jacobian matrix columns do not match (should be ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW). " );
        }
        J = mxGetPr( *argOut );
        memcpy(J_store_1, J, (ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW)*(ModelFcn_1NX) * sizeof ( double ));
        for( i=0; i<ModelFcn_1NX; ++i ) {
            df[i] = 0;
            f[i] = 0;
            for (j=0; j < ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW; ++j){
                df[i] += J[(j*(ModelFcn_1NX))+i]*seed[j+1];
            }
        }
        mxArray* FF2 = NULL;
        mxArray* argIn2[]  = { ModelFcn_1_f,ModelFcn_1T,ModelFcn_1X,ModelFcn_1U,ModelFcn_1P,ModelFcn_1W };
        mxArray* argOut2[] = { FF2 };
        mexCallMATLAB( 1,argOut2, 6,argIn2,"generic_ode" );
        ff = mxGetPr( *argOut2 );
        memcpy(f_store_1, ff, (ModelFcn_1NX) * sizeof ( double ));
        for( i=0; i<ModelFcn_1NX; ++i ){
            f[i] = ff[i];
        }
        mxDestroyArray( *argOut );
        mxDestroyArray( *argOut2 );
    }
}
#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState z;
    DifferentialState zd;
    Control T;
    Disturbance W;
    Function acadodata_f2;
    acadodata_f2 << z;
    acadodata_f2 << T;
    DMatrix acadodata_M1;
    acadodata_M1.read( "hovering_data_acadodata_M1.txt" );
    DVector acadodata_v1(2);
    acadodata_v1(0) = -1;
    acadodata_v1(1) = 0;
    DVector acadodata_v2(2);
    acadodata_v2(0) = -1;
    acadodata_v2(1) = 0;
    DMatrix acadodata_M2;
    acadodata_M2.read( "hovering_data_acadodata_M2.txt" );
    DMatrix acadodata_M3;
    acadodata_M3.read( "hovering_data_acadodata_M3.txt" );
    DVector acadodata_v3(2);
    acadodata_v3(0) = -3.000000E-01;
    acadodata_v3(1) = 0;
    ModelFcn_1T  = mxCreateDoubleMatrix( 1, 1,mxREAL );
    ModelFcn_1X  = mxCreateDoubleMatrix( 2, 1,mxREAL );
    ModelFcn_1XA = mxCreateDoubleMatrix( 0, 1,mxREAL );
    ModelFcn_1DX = mxCreateDoubleMatrix( 2, 1,mxREAL );
    ModelFcn_1U  = mxCreateDoubleMatrix( 1, 1,mxREAL );
    ModelFcn_1P  = mxCreateDoubleMatrix( 0, 1,mxREAL );
    ModelFcn_1W  = mxCreateDoubleMatrix( 1, 1,mxREAL );
    ModelFcn_1NT  = 1;
    ModelFcn_1NX  = 2;
    ModelFcn_1NXA = 0;
    ModelFcn_1NDX = 2;
    ModelFcn_1NP  = 0;
    ModelFcn_1NU  = 1;
    ModelFcn_1NW  = 1;
    DifferentialEquation acadodata_f1;
    ModelFcn_1_f = mxCreateString("ode");
    IntermediateState setc_is_1(5);
    setc_is_1(0) = autotime;
    setc_is_1(1) = z;
    setc_is_1(2) = zd;
    setc_is_1(3) = T;
    setc_is_1(4) = W;
    ModelFcn_1_jac = NULL;
    CFunction cLinkModel_1( ModelFcn_1NX, genericODE1 ); 
    acadodata_f1 << cLinkModel_1(setc_is_1); 

    OCP ocp1(0, 1, 50);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2, acadodata_v2);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.00000000000000000000e+00 <= T <= 1.05947999999999997733e+00);
    ocp1.subjectTo(W == 0.00000000000000000000e+00);


    OutputFcn acadodata_f3;

    DynamicSystem dynamicsystem1( acadodata_f1,acadodata_f3 );
    Process process2( dynamicsystem1,INT_RK45 );
    process2.setProcessDisturbance( acadodata_M2 );

    RealTimeAlgorithm algo1(ocp1, 0.02);
    algo1.set( MAX_NUM_ITERATIONS, 2 );
    algo1.set( INTEGRATOR_TYPE, INT_RK45 );
    algo1.set( INTEGRATOR_TOLERANCE, 1.000000E-05 );
    algo1.set( ABSOLUTE_TOLERANCE, 1.000000E-04 );

    PeriodicReferenceTrajectory referencetrajectory(acadodata_M3);

    Controller controller3( algo1,referencetrajectory );

    SimulationEnvironment algo2(0, 6, process2, controller3);
     algo2.init(acadodata_v3);
    returnValue returnvalue = algo2.run();


    VariablesGrid out_processout; 
    VariablesGrid out_feedbackcontrol; 
    VariablesGrid out_feedbackparameter; 
    VariablesGrid out_states; 
    VariablesGrid out_algstates; 
    algo2.getSampledProcessOutput(out_processout);
    algo2.getProcessDifferentialStates(out_states);
    algo2.getFeedbackControl(out_feedbackcontrol);
    const char* outputFieldNames[] = {"STATES_SAMPLED", "CONTROLS", "PARAMETERS", "STATES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutSS = NULL;
    double  *outSS = NULL;
    OutSS = mxCreateDoubleMatrix( out_processout.getNumPoints(),1+out_processout.getNumValues(),mxREAL ); 
    outSS = mxGetPr( OutSS );
    for( int i=0; i<out_processout.getNumPoints(); ++i ){ 
      outSS[0*out_processout.getNumPoints() + i] = out_processout.getTime(i); 
      for( int j=0; j<out_processout.getNumValues(); ++j ){ 
        outSS[(1+j)*out_processout.getNumPoints() + i] = out_processout(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES_SAMPLED",OutSS );
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_feedbackcontrol.getNumPoints(),1+out_feedbackcontrol.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_feedbackcontrol.getNumPoints(); ++i ){ 
      outC[0*out_feedbackcontrol.getNumPoints() + i] = out_feedbackcontrol.getTime(i); 
      for( int j=0; j<out_feedbackcontrol.getNumValues(); ++j ){ 
        outC[(1+j)*out_feedbackcontrol.getNumPoints() + i] = out_feedbackcontrol(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_feedbackparameter.getNumPoints(),1+out_feedbackparameter.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_feedbackparameter.getNumPoints(); ++i ){ 
      outP[0*out_feedbackparameter.getNumPoints() + i] = out_feedbackparameter.getTime(i); 
      for( int j=0; j<out_feedbackparameter.getNumValues(); ++j ){ 
        outP[(1+j)*out_feedbackparameter.getNumPoints() + i] = out_feedbackparameter(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );

    clearAllGlobals1( ); 

    clearAllStaticCounters( ); 
 
} 

