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

    //Initialized Filtered Variables
    initFiltered(rlsFLAGS, FILT);

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

    //Initialized Filtered Variables
    initFiltered(gradFLAGS, FILT);

    //Tell that RLS RCAC is being used
    rcacGrad = true;
}

void RCAC::oneStep(
    Eigen::VectorXd &uIn,
    Eigen::VectorXd &zIn,
    Eigen::VectorXd &yIn
)
{
    computeFiltered();

    //Increment kk
    kk++;
}

void RCAC::computeFiltered()
{

}

template < typename T >
void RCAC::initFiltered< T >(
    T &FLAGS,
    rcacFilt &FILT
)
{
    //Initialize the filtering variables
    int ltheta = FLAGS.Nc*(FLAGS.lu*FLAGS.lu + FLAGS.lu*FLAGS.ly);

    Eigen::VectorXd uBarTemp = Eigen::VectorXd::Zero(FLAGS.lu);
    Eigen::VectorXd ufBarTemp = Eigen::VectorXd::Zero(FLAGS.lz);

    //zBar and zfBar elements are the same size
    Eigen::VectorXd zBarTemp = Eigen::VectorXd::Zero(FLAGS.lz);

    Eigen::MatrixXd PhiBarTemp = Eigen::MatrixXd::Zero(FLAGS.lu, ltheta);
    Eigen::MatrixXd PhifBarTemp = Eigen::MatrixXd::Zero(FLAGS.lz, ltheta);

    //Set elements of Ubar and PhiBar to 0
    for (int i = 1 ; FILT.filtNu.cols()/FLAGS.lu; i++)
    {
        uBar.push_front(uBarTemp);
        PhiBar.push_front(PhiBarTemp);
    }

    //Set elements of Ufbar and PhifBar to 0
    for (int i = 1 ; FILT.filtDu.cols()/FLAGS.lu; i++)
    {
        ufBar.push_front(ufBarTemp);
        PhifBar.push_front(PhifBarTemp);
    }

    //Set elements of zbar to 0
    for (int i = 1 ; FILT.filtNz.cols()/FLAGS.lz; i++)
    {
        zBar.push_front(zBarTemp);
    }

    //Set elements of zfbar to 0
    for (int i = 1 ; FILT.filtDz.cols()/FLAGS.lz; i++)
    {
        zfBar.push_front(zBarTemp);
    }
}