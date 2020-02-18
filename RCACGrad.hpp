#ifndef _RCACGRAD_HPP_
#define _RCACGRAD_HPP_

#include "RCAC.hpp"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"

//Name: Nima Mohseni
//Date: 2/18/2020
//Purpose: This file contains the implementation of Gradient Descent RCAC as
//a derived class of the base RCAC class

/**
 * This struct contains the basic parameters needed for gradient RCAC to work.
 * 
 * @param lz Number of performance measurements
 * @param ly Number of sensor measurements
 * @param lu Number of control inputs
 * @param Nc Controller order
 * @param theta_0 Eigen vector with initial controller coefficients. Usually a vector of zeros
 * @param filtorder Order of the filter (\f$G_f\f$)
 * @param k_0 Minimum number of steps to run before starting the controller (usually k_0 = Nc)
 * @param alpha Step size scaling for gradient descent
 */
//Struct flags: contains the necessary flags and data for the gradient RCAC
struct rcacGradFlags
{
    //Default RCAC Flags. Required in all RCAC implementations
    int lz;
    int ly;
    int lu;
    int Nc;
    Eigen::VectorXd theta_0;
    int filtorder;
    int k_0;

    //RCAC Gradient Coefficients
    double alpha;
 
};

/**
 * This class contains the Gradient implementation of RCAC. The class contains a constructor
 * and an implementation of the coeffUpdate method.
 */
class RCACGrad: public RCAC
{
    public:
        /**
         * Value constructor for the RCACGrad class. Required.
         * 
         * @param FLAGS rcacGradFlags struct containing the flags for the gradient RCAC
         * @param FILT base rcacFilt struct containing the filter coefficients
         */
        //Value ctor: Initializes RCAC using gradient descent with the given flags and
        //filter values
        RCACGrad(
            rcacGradFlags &FLAGS,
            rcacFilt &FILT
        );

    private:
        /**
         * This method contains the implementation of the gradient update for RCAC.
         * The method is called by the public oneStep method to update the coefficients
         * 
         * @param zIn Most recent performance measurment
         */
        //Function coeffUpdate: Compute the RCAC coefficient update for gradient RCAC
        void coeffUpdate(
            Eigen::VectorXd &zIn
        );

        //Gradient FLAGS
        double alpha; //stepsize scaling

        //Other Variables
        Eigen::VectorXd gradient;
};

#endif