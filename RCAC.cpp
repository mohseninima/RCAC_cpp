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
    Eigen::VectorXd phi(uphi.rows()+yphi.rows(),1);
    phi << uphi, yphi;

    computeFiltered();


    //add uIn and yIn to the phi stack
    uphi << uIn, uphi.head((Nc-1)*lu);
    yphi << yIn, yphi.head((Nc-1)*ly);

    //Increment kk
    kk++;
}

void RCAC::computeFiltered()
{

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