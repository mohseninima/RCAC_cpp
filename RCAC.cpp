#include "RCAC.hpp"
#include <iostream>
#include <Eigen/Dense>

//Name: Nima Mohseni
//Date: 12/29/2019
//Purpose: This file contains the implementation of RCAC for both RLS and gradient
//implementations in the form of a portable library

void RCAC::initRLS(
    rcacRLSFlags &FLAGS,
    rcacFilt &FILT
)
{
    //Assign FLAGS and FILT variables to the specific RCAC object
    this->rlsFLAGS = FLAGS;
    this->FILT = FILT;

    //Tell that RLS RCAC is being used
    rcacRLS = true;
}

void RCAC::initGrad(
    rcacGradFlags &FLAGS,
    rcacFilt &FILT
)
{
    //Assign FLAGS and FILT variables to the specific RCAC object
    this->gradFLAGS = FLAGS;
    this->FILT = FILT;

    //Tell that RLS RCAC is being used
    rcacGrad = true;
}

void RCAC::oneStep(
    Eigen::VectorXd &uIn,
    Eigen::VectorXd &zIn,
    Eigen::VectorXd &yIn
)
{

}