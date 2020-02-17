#include "RCAC.hpp"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"

//Name: Nima Mohseni
//Date: 12/29/2019
//Purpose: This file contains the general implementation of RCAC to support
//derived RCAC classes for RLS, gradient and other RCAC types

/*
RCAC* RCAC::init(
    rcacRlsFlags &FLAGS, 
    rcacFilt &FILT, 
    int &whichRCAC
)
{
    if (whichRCAC == 1)
    {
        return new RCACRLS(FLAGS, FILT);
    }
    else
    {
        return NULL;
    }
    
}
*/

/*
void RCAC::init(
    rcacFlags &FLAGS,
    rcacFilt &FILT
)
{
    std::cout << "Empty Initialization!" << "\n";
    exit(EXIT_FAILURE);
}
*/

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
    Eigen::VectorXd &uIn_,
    Eigen::VectorXd &zIn,
    Eigen::VectorXd &yIn
)
{
    uIn = uIn_;

    //Compute Phi and phi before filtering only on the first step
    Eigen::VectorXd phi(uphi.rows()+yphi.rows(),1);
    Eigen::MatrixXd eye_lu =  Eigen::MatrixXd::Identity(lu,lu);
    if (kk == 1)
    {
        //Create phi with Nc past u and y measurements        
        phi << uphi, yphi;

        //Create Phi with kron(phi',I_lu)        
        Phi = Eigen::kroneckerProduct(phi.transpose(), eye_lu);

        //Initialize theta
        theta = theta_0;

    }

    //Add Phi and u to the list of past Phi's and u's for filtering
    uBar.push_front(uIn);
    PhiBar.push_front(Phi);
    //remove the oldest Phi
    uBar.pop_back();
    PhiBar.pop_back();
    
    //Filter the u, z, and Phi variables if there is enough data to filter
    if (kk > FILT.filtNu.cols()/lu)
    {
        computeFiltered();
    }

    //Compute Controller Update if kk >= k_0
    if (kk >= k_0)
    {
        coeffUpdate(zIn);
    }

    //add uIn and yIn to the phi stack
    Eigen::MatrixXd uphitemp(Nc*lu, 1);
    Eigen::MatrixXd yphitemp(Nc*ly, 1);
    uphitemp << uIn, uphi.head((Nc-1)*lu);
    //uphitemp << uIn, uphi.block(0,0,(Nc-1)*lu,1);
    uphi = uphitemp;
    yphitemp << yIn, yphi.head((Nc-1)*ly);
    //yphitemp << yIn, yphi.block(0,0,(Nc-1)*ly,1);
    yphi= yphitemp;

    //update Phi only if there is enough data
    //Create phi with Nc past u and y measurements. Now containing current values 
    if (kk > Nc)
    { 
        phi << uphi, yphi;
    
        //Create Phi(k+1) with kron(phi',I_lu)        
        Phi = Eigen::kroneckerProduct(phi.transpose(), eye_lu);
    }

    //compute control input
    if (kk >= k_0)
    {
        uOut = Phi*theta;
    }
    else
    {
        uOut = Eigen::VectorXd::Zero(lu);
    }

    //Increment kk
    kk++;
}

void RCAC::computeFiltered()
{
    //Loop through and filter the coefficients
    Eigen::MatrixXd ufSum = Eigen::MatrixXd::Zero(lz, 1);
    Eigen::MatrixXd PhifSum = Eigen::MatrixXd::Zero(lz, Nc*lu*(lu+ly));

    //Filter the numerator coefficients
    //for (int i = 0; i < FILT.filtNu.cols()/lu; i++)
    for (int i = 0; i < filtorder; i++)
    {
        ufSum = FILT.filtNu.block(0, lu*i, lz, lu)*uBar[i] + ufSum;
        PhifSum = FILT.filtNu.block(0, lu*i, lz, lu)*PhiBar[i] + PhifSum;
    }

    //Filter the Denominator coefficients coefficients
    for (int i = 0; i < (filtorder-1); i++)
    {
        ufSum = ufSum - FILT.filtDu.block(0, lz*i, lz, lz)*ufBar[i];
        PhifSum = PhifSum - FILT.filtDu.block(0, lz*i, lz, lz)*PhifBar[i];
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
    for (int i = 1 ; i <= FILT.filtNu.cols()/lu; i++)
    {
        uBar.push_front(uBarTemp);
        PhiBar.push_front(PhiBarTemp);
    }

    //Set elements of Ufbar and PhifBar to 0
    for (int i = 1 ; i <= FILT.filtDu.cols()/lz; i++)
    {
        ufBar.push_front(ufBarTemp);
        PhifBar.push_front(PhifBarTemp);
    }

    //Set elements of zbar to 0
    for (int i = 1 ; i <= FILT.filtNz.cols()/lz; i++)
    {
        zBar.push_front(zBarTemp);
    }

    //Set elements of zfbar to 0
    for (int i = 1 ; i <= FILT.filtDz.cols()/lz; i++)
    {
        zfBar.push_front(zBarTemp);
    }
}

void RCAC::initRegressor()
{
    uphi = Eigen::VectorXd::Zero(Nc*lu);
    yphi = Eigen::VectorXd::Zero(Nc*ly);
}