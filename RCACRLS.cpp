#include "RCACRLS.hpp"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"

//Name: Nima Mohseni
//Date: 1/1/2020
//Purpose: This file contains the implementation of RLS RCAC as a derived class
//of the base RCAC class

RCACRLS::RCACRLS(
    rcacRlsFlags &FLAGS,
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
    theta_0 = FLAGS.theta_0;
    filtorder = FLAGS.filtorder;
    lambda = FLAGS.lambda;
    P0 = FLAGS.P0;
    Ru = FLAGS.Ru;
    Rz = FLAGS.Rz;
    this->FILT = FILT;

    //Initialize the regressor variables
    initRegressor();

    //Initialized Filtered Variables
    initFiltered();

    //Initialize P
    P = P0;

    kk = 1;

    //Tell that RLS RCAC is being used
    rcacRLS = true;
}

void RCACRLS::coeffUpdate(
    Eigen::VectorXd &zIn
)
{
    Eigen::MatrixXd Rsum = Rz;
    Eigen::MatrixXd Gamma;
    Eigen::MatrixXd GammaInv;

    Gamma = lambda*Rsum.inverse() + PhifBar[0]*P*PhifBar[0].transpose();
    GammaInv = Gamma.inverse();
    theta = theta - P*(PhifBar[0].transpose()*(GammaInv
            *(PhifBar[0]*theta + Rsum.inverse()*Rz*(zIn - ufBar[0]))));
    P = (1/lambda)*P-(1/lambda)*(P*PhifBar[0].transpose())*GammaInv
        *(PhifBar[0]*P);
    //std::cout << "kk: " << kk << ", " << theta.transpose() << "\n";
}