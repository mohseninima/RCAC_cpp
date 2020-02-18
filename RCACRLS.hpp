#ifndef _RCACRLS_HPP_
#define _RCACRLS_HPP_

#include "RCAC.hpp"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"

//Name: Nima Mohseni
//Date: 1/1/2020
//Purpose: This file contains the implementation of RLS RCAC as a derived class
//of the base RCAC class

/**
 * This struct contains the basic parameters needed for RLS RCAC to work.
 * 
 * @param lz Number of performance measurements
 * @param ly Number of sensor measurements
 * @param lu Number of control inputs
 * @param Nc Controller order
 * @param theta_0 Eigen vector with initial controller coefficients. Usually a vector of zeros
 * @param filtorder Order of the filter (\f$G_f\f$)
 * @param k_0 Minimum number of steps to run before starting the controller (usually k_0 = Nc)
 * @param lambda Forgetting factor for RLS
 * @param P0 Matrix containing the initial value for the covariance matrix of RLS (P0 = Rtheta^-1)
 * Size is ltheta by ltheta where ltheta = Nc*lu(lu+ly)
 * @param Ru Matrix containing a weighting on the control effort. Size is lu by lu.
 * <b> Currently broken, set as a zero matrix </b>
 * @param Rz Matrix containing a weghting on the performance measurement. Size is lz by lz.
 */
//Struct flags: contains the necessary flags and data for the RLS RCAC
struct rcacRlsFlags
{
    //Default RCAC Flags. Required in all RCAC implementations
    int lz;
    int ly;
    int lu;
    int Nc;
    Eigen::VectorXd theta_0;
    int filtorder;
    int k_0;
    double lambda;

    //RLS RCAC Flags
    Eigen::MatrixXd P0;
    Eigen::MatrixXd Ru;
    Eigen::MatrixXd Rz;  
};

/**
 * This class contains the RLS implementation of RCAC. The class contains a constructor
 * and an implementation of the coeffUpdate method.
 */
class RCACRLS: public RCAC
{
    public:
        /**
         * Value constructor for the RCACRLS class. Required.
         * 
         * @param FLAGS rcacRlsFlags struct containing the flags for the RLS RCAC
         * @param FILT base rcacFilt struct containing the filter coefficients
         */
        //Value ctor: Initializes RCAC using RLS with the given flags and
        //filter values
        RCACRLS(
            rcacRlsFlags &FLAGS,
            rcacFilt &FILT
        );

    private:
        /**
         * This method contains the implementation of the RLS update for RCAC.
         * The method is called by the public oneStep method to update the coefficients
         * 
         * @param zIn Most recent performance measurment
         */
        //Function coeffUpdate: Compute the RCAC coefficient update for RLS
        void coeffUpdate(
            Eigen::VectorXd &zIn
        );

        //RLS FLAGS
        double lambda;
        Eigen::MatrixXd P0;
        Eigen::MatrixXd Ru;
        Eigen::MatrixXd Rz;
};

#endif