#define S_FUNCTION_NAME RCACSimulink
#define S_FUNCTION_LEVEL 2

#ifdef  S_FUNCTION_LEVEL    //Is this header used in Simulink or Mex?
    #define M_ERR_MSG(S, txt) ssSetErrorStatus(S, txt)
#else
    typedef int SimStruct; //Define dummy SimStruct
    #define M_ERR_MSG(S, txt) mexErrMsgTxt(txt)
#endif

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

//Input to the resulting mex file is: Sample Time, RCACtype, FLAGarray, FILTarray

//Function mdlInitializeSizes used by simulink to determine the
//S-Function block's characteristics (# inputs, outputs, etc)
static void mdlInitializeSizes(SimStruct *S)
{
    //Parameters: Sample Time, RCACtype, FLAG, FILT
    ssSetNumSFcnParams(S, 4);

    //Set the parameters when using a mex file
    #if defined(MATLAB_MEX_FILE)
        if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
        {
            //mdlCheckParameters(S);
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
    //char* preRcacType = (char*)ssGetSFcnParam(S, 1);
    //mexPrintf("okay1.5\n");
    //printf(preRcacType);
    //std::string rcacType(preRcacType);
    
    int buflen = mxGetN((ssGetSFcnParam(S, 1)))*sizeof(mxChar)+1;
    char* String = (char*)mxMalloc(buflen); 
    int status = mxGetString((ssGetSFcnParam(S, 1)),String,buflen);
    //printf("the output is %s\n",String);
    std::string rcacType(String);
    //std::string rcacType = "RLS";
    
    //Get a pointer to the flag values
    double* FLAGSmx =  (double*)mxGetPr(ssGetSFcnParam(S, 2));
     
    //Get a pointer to the filter coefficients
    double* FILTmx =  (double*)mxGetPr(ssGetSFcnParam(S, 3));
    
    //Create RCAC, using simulink method
    RCAC *myRCAC;
    myRCAC = initSimulink(FLAGSmx, FILTmx, rcacType);

    //set the number of states for simulink
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    //there must be 3 input ports (z(k), y(k), u(k))
    if (!ssSetNumInputPorts(S, 3)) return;

    //set the input port sizes
    ssSetInputPortWidth(S, 0, myRCAC->getlz());
    ssSetInputPortWidth(S, 1, myRCAC->getly());
    ssSetInputPortWidth(S, 2, myRCAC->getlu());

    //require the input ports to be contiguous
    ssSetInputPortRequiredContiguous(S, 0, true);
    ssSetInputPortRequiredContiguous(S, 1, true);
    ssSetInputPortRequiredContiguous(S, 2, true);

    //Allow direct feedthrough of the input ports
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2 ,1);

    //set the number of sample times being used
    ssSetNumSampleTimes(S,1);

    //set the number of work vector elements
    ssSetNumRWork(S, 0); //Number of real work vectors
    ssSetNumIWork(S, 1); //Number of integer work vectors (store timestep here)
    ssSetNumPWork(S, 4); //Number of pointer work vectors (store socket?)
    ssSetNumModes(S, 0); //Number of mode work vectors
    ssSetNumNonsampledZCs(S, 0); //number of nonsampled zero crossings

    //require the number of output ports to be 2
    if (!ssSetNumOutputPorts(S,2)) return;

    ssSetOutputPortWidth(S, 0, myRCAC->getlu()); //control out
    int ltheta = myRCAC->getNc()*myRCAC->getlu()*(myRCAC->getlu()+ myRCAC->getly());
    ssSetOutputPortWidth(S, 1, ltheta); //coefficient out

    //Set simstate compliance to be the same as a built-in block
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    
    //Call the terminate function on exit to free memory
    ssSetOptions(S, SS_OPTION_CALL_TERMINATE_ON_EXIT);

    //clear the RCAC memory
    delete myRCAC;
    #if defined(MATLAB_MEX_FILE)
        mexPrintf("mdlInitSizes ok \n");
    #else
    #endif
}

//Function mdlInitializeSampleTimes: This function tells simulink what the sample
//time is for the block
static void mdlInitializeSampleTimes(SimStruct *S)
{
    //check if sample time in parameter list is a scalar
    //and set it as the sample time for the block
    if (mxGetScalar(ssGetSFcnParam(S,0))==-1)
    {
        ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    }
    else
    {
        ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
        ssSetModelReferenceSampleTimeDefaultInheritance(S);
    }   
    //add offset to sample time if needed
    ssSetOffsetTime(S, 0, 0.0);
    #if defined(MATLAB_MEX_FILE)
        mexPrintf("Sample time ok \n");
    #else
    #endif   
}

#define MDL_START
#if defined(MDL_START)

    //Function mldStart: called once at start of model execution to initialize variables
    static void mdlStart(SimStruct *S)
    {
        //get RCAC type
        //std::string rcacType(mxArrayToString(ssGetSFcnParam(S, 1)));
        //char* preRcacType = (char*)ssGetSFcnParam(S, 1);
        //std::string rcacType(preRcacType);
        int buflen = mxGetN((ssGetSFcnParam(S, 1)))*sizeof(mxChar)+1; 
        char* String = (char*)mxMalloc(buflen); 
        int status = mxGetString((ssGetSFcnParam(S, 1)),String,buflen);
        std::string rcacType(String);

        //Get a pointer to the flag values
        double* FLAGSmx =  (double*)mxGetPr(ssGetSFcnParam(S, 2));

        //Get a pointer to the filter coefficients
        double* FILTmx =  (double*)mxGetPr(ssGetSFcnParam(S, 3));

        //Create RCAC, using simulink method
        RCAC *myRCAC;
        //myRCAC = initSimulink(FLAGSmx, FILTmx, rcacType);
        
        //Store the RCAC pointer in a work vector
        //ssSetPWorkValue(S, 0, myRCAC);
        //ssSetPWorkValue(S, 0, (void *)myRCAC);
        //ssSetPWorkValue(S, 0, (void *)myRCAC);
        ssSetPWorkValue(S, 0, (void *)initSimulink(FLAGSmx, FILTmx, rcacType));
        
        
        //set the first timestep to 1
        ssSetIWorkValue(S, 0, 1);
    }

#endif

//Function mdlOutputs: compute the output of RCAC
static void mdlOutputs(SimStruct *S, int tid)
{ //.data() to get C array
    RCAC *myRCAC;
    double *zin;
    double *yin;
    double *uin;
    double *uout;
    double *thetaout;

    //read variables from work vector
    //int kk = ssGetIWorkValue(S, 0);
    myRCAC = (RCAC *)ssGetPWorkValue(S, 0);
    //myRCAC = (RCAC *)ssGetPWorkValue(S, 1);
    
    

    //prepare input signals, and convert them to eigen vectors
    zin = (double *) ssGetInputPortSignal(S, 0);
    Eigen::VectorXd zIn_eigen = Eigen::Map<Eigen::VectorXd>(zin, myRCAC->getlz(), 1);
    //Eigen::Map<Eigen::VectorXd> zIn_eigen(zin, myRCAC->getlz, 1);
    yin = (double *) ssGetInputPortSignal(S, 1);
    Eigen::VectorXd yIn_eigen = Eigen::Map<Eigen::VectorXd>(yin, myRCAC->getly(), 1);
    //Eigen::Map<Eigen::VectorXd> yIn_eigen(yin, myRCAC->getly, 1);
    uin = (double *) ssGetInputPortSignal(S, 2);
    Eigen::VectorXd uIn_eigen = Eigen::Map<Eigen::VectorXd>(uin, myRCAC->getlu(), 1);
    //Eigen::Map<Eigen::VectorXd> uIn_eigen(uin, myRCAC->getlu, 1);
 
    //prepare output signals
    uout = (double *) ssGetOutputPortRealSignal(S, 0);
    thetaout = (double *) ssGetOutputPortRealSignal(S, 1);
    
    //Call RCAC and return the outputs (then convert Eigen array to C array)
    myRCAC->oneStep(uIn_eigen, zIn_eigen, yIn_eigen);

    //uout = myRCAC->getControl().data();
    //thetaout = myRCAC->getCoeff().data();
    
    Eigen::Map<Eigen::VectorXd>(uout, myRCAC->getlu(), 1) = myRCAC->getControl();
    Eigen::Map<Eigen::VectorXd>(thetaout, myRCAC->getNc()*myRCAC->getlu()*(myRCAC->getlu()+myRCAC->getly()), 1) = myRCAC->getCoeff();
    
    //mexPrintf("%f\n",zin[0]);
    //mexPrintf("%f\n",uout[0]);
    //for (int test = 0; test < 6; test++)
    //    mexPrintf("%f\n",zIn_eigen[test]);
    //mexPrintf("%i\n",myRCAC->getkk());
}

//Function mdlTerminate: Performs actions that are necessary at the termination
//of a simulation. Ex: freeing memory
static void mdlTerminate(SimStruct *S)
{
    
    RCAC *myRCAC;
    myRCAC = (RCAC *)ssGetPWorkValue(S, 0);
    //delete myRCAC;
}

//Required at end of S-Function
#ifdef  MATLAB_MEX_FILE        // Is this file being compiled as a MEX-file?
    #include "simulink.c"      // MEX-file interface mechanism
#else
    #include "cg_sfun.h"       // Code generation registration function
#endif