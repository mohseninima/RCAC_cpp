#include "RCAC.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>

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
    //this->FLAGS = FLAGS;
    lz = FLAGS.lz;
    ly = FLAGS.ly;
    lu = FLAGS.lu;
    Nc = FLAGS.Nc;
    k_0 = FLAGS.k_0;
    lambda = FLAGS.lambda;
    theta_0 = FLAGS.theta_0;
    Rtheta = FLAGS.Rtheta;
    Ru = FLAGS.Ru;
    Rz = FLAGS.Rz;
    this->FILT = FILT;

    //Initialize the regressor variables
    initRegressor();

    //Initialized Filtered Variables
    initFiltered();

    //Tell that RLS RCAC is being used
    rcacRLS = true;
}

/*
void RCAC::initGrad(
    rcacGradFlags &FLAGS,
    rcacFilt &FILT
)
{
    //Assign FLAGS and FILT variables to the specific RCAC object
    this->gradFLAGS = FLAGS;
    this->FILT = FILT;

    //Initialize the regressor variables
    initRegressor<rcacGradFlags>(gradFLAGS);

    //Initialized Filtered Variables
    initFiltered<rcacGradFlags>(gradFLAGS, FILT);

    //Tell that RLS RCAC is being used
    rcacGrad = true;
}
*/

void RCAC::oneStep(
    Eigen::VectorXd &uIn,
    Eigen::VectorXd &zIn,
    Eigen::VectorXd &yIn
)
{
    //Compute Phi and phi before filtering only on the first step
    Eigen::VectorXd phi(uphi.rows()+yphi.rows(),1);
    Eigen::MatrixXd eye_lu =  Eigen::MatrixXd::Identity(lu,lu);
    if (kk == 0)
    {
        //Create phi with Nc past u and y measurements        
        phi << uphi, yphi;

        //Create Phi with kron(phi',I_lu)        
        Phi = Eigen::kroneckerProduct(phi.transpose(), eye_lu);
    }

    //Add Phi and u to the list of past Phis and us for filtering
    uBar.push_front(uIn);
    PhiBar.push_front(Phi);
    //remove the oldest Phi
    uBar.pop_back();
    PhiBar.pop_back();

    //Filter the u, z, and Phi variables
    computeFiltered();

    //Compute Controller Update
    coeffUpdate(zIn);

    //add uIn and yIn to the phi stack
    Eigen::MatrixXd uphitemp;
    Eigen::MatrixXd yphitemp;
    uphitemp << uIn, uphi.head((Nc-1)*lu);
    uphi = uphitemp;
    yphitemp << yIn, yphi.head((Nc-1)*ly);
    yphi= yphitemp;

    //Create phi with Nc past u and y measurements. Now containing current values  
    phi << uphi, yphi;

    //Create Phi(k+1) with kron(phi',I_lu)        
    Phi = Eigen::kroneckerProduct(phi.transpose(), eye_lu);

    //compute control input
    uOut = Phi*theta;

    //Increment kk
    kk++;
}

void RCAC::coeffUpdate(
    Eigen::VectorXd &zIn
)
{
    Eigen::MatrixXd Rsum = Rz + Ru;
    Eigen::MatrixXd Gamma;

    Gamma = lambda*Rsum.inverse() + PhifBar[0]*P*PhifBar[0].transpose();
    P = (1/lambda)*P-(1/lambda)*P*PhifBar[0].transpose()*Gamma.inverse()
        *PhifBar[0]*P;
    theta = theta - P*PhifBar[0].transpose()*Gamma.inverse()
            *(PhifBar[0].transpose()*theta + Rsum.inverse()*Rz*(zIn - ufBar[0]));
}

void RCAC::computeFiltered()
{
    //Loop through and filter the coefficients
    Eigen::MatrixXd ufSum = Eigen::MatrixXd::Zero(lz, 1);
    Eigen::MatrixXd PhifSum = Eigen::MatrixXd::Zero(lz, Nc*lu*(lu+ly));

    //Filter the numerator coefficients
    for (int i = 0; i < FILT.filtNu.cols(); i++)
    {
        ufSum = FILT.filtNu[i]*uBar[i] + ufSum;
        PhifSum = FILT.filtNu[i]*PhiBar[i] + PhifSum;
    }

    //Filter the Denominator coefficients coefficients
    for (int i = 0; i < FILT.filtDu.cols(); i++)
    {
        ufSum = FILT.filtDu[i]*ufBar[i] + ufSum;
        PhifSum = FILT.filtDu[i]*PhifBar[i] + PhifSum;
    }

    //Add the sum to the list of past filtered values for future filtering
    ufBar.push_front(ufSum);
    PhifBar.push_front(PhifSum);
    //remove the oldest filtered value
    ufBar.pop_back();
    PhifBar.pop_back();

}

void RCAC::initFiltered()
{
    //Initialize the filtering variables
    int ltheta = Nc*(lu*lu + lu*ly);

    Eigen::VectorXd uBarTemp = Eigen::VectorXd::Zero(lu);
    Eigen::VectorXd ufBarTemp = Eigen::VectorXd::Zero(lz);

    //zBar and zfBar elements are the same size
    Eigen::VectorXd zBarTemp = Eigen::VectorXd::Zero(lz);

    Eigen::MatrixXd PhiBarTemp = Eigen::MatrixXd::Zero(lu, ltheta);
    Eigen::MatrixXd PhifBarTemp = Eigen::MatrixXd::Zero(lz, ltheta);

    //Set elements of Ubar and PhiBar to 0
    for (int i = 1 ; FILT.filtNu.cols()/lu; i++)
    {
        uBar.push_front(uBarTemp);
        PhiBar.push_front(PhiBarTemp);
    }

    //Set elements of Ufbar and PhifBar to 0
    for (int i = 1 ; FILT.filtDu.cols()/lu; i++)
    {
        ufBar.push_front(ufBarTemp);
        PhifBar.push_front(PhifBarTemp);
    }

    //Set elements of zbar to 0
    for (int i = 1 ; FILT.filtNz.cols()/lz; i++)
    {
        zBar.push_front(zBarTemp);
    }

    //Set elements of zfbar to 0
    for (int i = 1 ; FILT.filtDz.cols()/lz; i++)
    {
        zfBar.push_front(zBarTemp);
    }
}

void RCAC::initRegressor()
{
    uphi = Eigen::VectorXd::Zero(Nc*lu);
    yphi = Eigen::VectorXd::Zero(Nc*ly);
}