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

//Struct flags: contains the necessary flags and data for the RLS RCAC
struct rcacRlsFlags
{
    //Default RCAC Flags. Required in all RCAC implementations
    int lz;
    int ly;
    int lu;
    int Nc;
    int lambda;
    Eigen::VectorXd theta_0; 
    int k_0;

    //RLS RCAC Flags
    Eigen::MatrixXd P0;
    Eigen::MatrixXd Ru;
    Eigen::MatrixXd Rz;  
    };

class RCACRLS: public RCAC
{
    public:
        //Function initRLS: Initializes RCAC using RLS with the given flags and
        //filter values
        RCACRLS(
            rcacRlsFlags &FLAGS,
            rcacFilt &FILT
        );

    private:
        //Function coeffUpdate: Compute the RCAC coefficient update for RLS
        void coeffUpdate(
            Eigen::VectorXd &zIn
        );

        //RLS FLAGS
        int lambda;
        Eigen::MatrixXd P0;
        Eigen::MatrixXd Ru;
        Eigen::MatrixXd Rz;
};

#endif