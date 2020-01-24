#define S_FUNCTION_NAME RCACSimulink
#define S_FUNCTION_LEVEL 2

#include <iostream>
#include <string>
#include "simstruc.h"
#include "RCACCreator.hpp"
#include "Eigen/Core"

//create a mex function if needed
#if defined(MATLAB_MEX_FILE)
    #include "mex.h"
#else
#endif

//Name: Nima Mohseni
//Date: 1/24/2020
//Purpose: This file contains the implementation of the C mex functions to allow
//simulink to use RCAC

//Function mdlInitializeSizes used by simulink to determine the
//S-Function block's characteristics (# inputs, outputs, etc)

static void mdlInitializeSizes(SimStruct *S)
{
    //Parameters: RCACtype, FLAG, FILT
    ssSetNumSFcnParams(S, 3);

    //Set the parameters when using a mex file
    #if defined(MATLAB_MEX_FILE)
        if (ssGetNumSFcn(S) == ssGetSFcnParamsCount(S))
        {
            mdlCheckParameters(S);
            if (ssGetErrorStatus(S) != NULL)
            {
                return;
            }
        }
        else
        {
            return;
        }
    #endif

    //get RCAC type
    std::string rcacType;
    rcacType(mxArrayToString(ssGetSFcnParam(S, 1)));

    //Get a pointer to the flag values
    double* FLAGSmx =  (double*)mxGetPr(ssGetSFcnParam(S, 2));

    //Get a pointer to the filter coefficients
    double* FILTmx =  (double*)mxGetPr(ssGetSFcnParam(S, 3));

    //Create RCAC, using simulink method
    RCAC *myRCAC;
    myRCAC = myRCAC->initSimulink(FLAGSmx, FILTmx, rcacType);

    

}