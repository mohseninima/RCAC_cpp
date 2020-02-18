#ifndef _RCACCREATOR_HPP_
#define _RCACCREATOR_HPP_

#include "RCAC.hpp"
#include "RCACRLS.hpp"
#include "RCACGrad.hpp"
//#include <any>


//Name: Nima Mohseni
//Date: 1/20/2020
//Purpose: This file contains the implementation of the factory function to
//generate new RCAC versions

/**
 * @file
 * Define the methods for initializing different RCAC types here
 * 
 * @param useRLS The flag for the RLS based RCAC
 * @param useGrad The flag for the Gradient based RCAC
 */
//RCAC Types: define your RCAC typenames here
std::string useRLS = "RLS";
std::string useGrad = "Grad";

/**
 * A factory method that creates a pointer to the specific type of RCAC you want to use.
 * 
 * @param FLAGS template for a struct containing the required flags for 
 * the type of RCAC to be used.
 * @param FILT struct containing the filter coefficients for (\f$G_f\f$).
 * @param whichRCAC string containing the type of RCAC to be used.
 */
//Factory Method (TODO: move to a separate class, fix memory leak)
template <typename T>
RCAC* RCAC::init(
    T &FLAGS, 
    rcacFilt &FILT, 
    std::string &rcacType
)
{
    //Hack to get things working with templates
    //std::any FLAGStemp(FLAGS);
    void *FLAGStemp = (void*)&FLAGS;

    if (rcacType == useRLS)
    {
        //rcacRlsFlags FLAGSnew = std::any_cast<rcacRlsFlags>(FLAGStemp);
        rcacRlsFlags *FLAGSnew = (rcacRlsFlags*)FLAGStemp;
        return new RCACRLS(*FLAGSnew, FILT);
    }
    else if (rcacType == useGrad)
    {
        //rcacGradFlags FLAGSnew = std::any_cast<rcacGradFlags>(FLAGStemp);
        rcacGradFlags *FLAGSnew = (rcacGradFlags*)FLAGStemp;
        return new RCACGrad(*FLAGSnew, FILT);
    }
    else
    {
        std::cout << "Bad RCAC Type!" << "\n";
        exit(EXIT_FAILURE);
    }  
}

/*
template <>
RCAC* RCAC::init(
    rcacRlsFlags &FLAGS, 
    rcacFilt &FILT, 
    std::string &rcacType
)
{
    if (rcacType == useRLS)
    {
        return new RCACRLS(FLAGS, FILT);
    }
    {
        std::cout << "Bad RCAC Type!" << "\n";
        exit(EXIT_FAILURE);
    } 
}

template <>
RCAC* RCAC::init(
    rcacGradFlags &FLAGS,
    rcacFilt &FILT, 
    std::string &rcacType
)
{
    if (rcacType == useGrad)
    {
        return new RCACGrad(FLAGS, FILT);
    }
    {
        std::cout << "Bad RCAC Type!" << "\n";
        exit(EXIT_FAILURE);
    }   
}
*/

//load FILTmx into FILT struct, should not be used outside initSimulink
//mxArray must be in format
//[FILTNu'(:); FILTDu'(:); FILTNz'(:); FILTDz'(:)]
//Matrices must be transposed and then vectorized
rcacFilt initFiltSimulink(
    int lz,
    int ly,
    int lu,
    int Nc,
    int filtorder,
    double* &FILTmx
)
{
    rcacFilt FILT;

    //load Gf numerator
    int filtIndex = 0;
    FILT.filtNu.resize(lz, lu*filtorder); //Tell eigen the matrix size
    FILT.filtNu = Eigen::Map<Eigen::MatrixXd>(&FILTmx[filtIndex], (double)lz, (double)lu*filtorder);
    for (int i = 0; i < (lz*lu*filtorder); i++)
    {
        //FILT.filtNu << FILTmx[filtIndex]; 
        filtIndex++;
    }
      
    //load Gf Denominator
    FILT.filtDu.resize(lz, lz*(filtorder-1)); //Tell eigen the matrix size
    FILT.filtDu = Eigen::Map<Eigen::MatrixXd>(&FILTmx[filtIndex], (double)lz, (double)lz*(filtorder-1));
    for (int i = 0; i < (lz*lz*(filtorder-1)); i++)
    {
        //FILT.filtDu << FILTmx[filtIndex]; 
        filtIndex++;
    }
       
    //load Gf_z Numerator (never used?)
    FILT.filtNz.resize(lz, lz); //Tell eigen the matrix size
    FILT.filtNz = Eigen::Map<Eigen::MatrixXd>(&FILTmx[filtIndex], (double)lz, (double)lz);
    for (int i = 0; i < (lz*lz); i++)
    {
        //FILT.filtNz << FILTmx[filtIndex]; 
        filtIndex++;
    }

    //load Gf_z Denominator (never used?)
    FILT.filtDz.resize(lz, lz); //Tell eigen the matrix size
    FILT.filtDz = Eigen::Map<Eigen::MatrixXd>(&FILTmx[filtIndex], (double)lz, (double)lz);
    for (int i = 0; i < (lz*lz); i++)
    {
        //FILT.filtDz << FILTmx[filtIndex]; 
        filtIndex++;
    }

    return(FILT);
}

/**
 * This function takes an array format of the Flags and Filt from MATLAB and converts
 * them to a Eigen compatible matrix/vector.
 * 
 * This must be defined for each type of RCAC that is created.
 * 
 * <b> This function is NOT meant to be called outside of RCACSimulink.cpp </b>
 * 
 * @param FLAGSmx Array from MATLAB in a special order for the flags (check the source file for the order)
 * @param FILTmx Array from MATLAB in a special order for the filter variables
 * @param FILT RCAC type taken from a matlab char string (C-style) and converted to a string (C++ string)
 */
//Function initSimulink, takes the array format of the RCAC variables from 
//simulink and converts it to the struct format and returns a pointer to an RCAC
//object
//
//ITEGRATE ME:
//std::string msg = std::string("Aw snap: ");       
//mexErrMsgTxt(msg.c_str());
//TODO: move filt outside of the if statement
RCAC* initSimulink(
    double* &FLAGSmx,
    double* &FILTmx,
    std::string &rcacType
)
{
    if (rcacType == useRLS)
    {
        //load FLAGSmx into flags struct
        //mxArray must be in format
        //[lz; ly; lu; Nc; filtorder; k_0; P0'(:); Ru'(:); Rz'(:); lambda; theta_0]
        //Matrices must be transposed and then vectorized

        rcacRlsFlags FLAGS;
        //load the first 6 variables
        int flagsIndex = 0;
        FLAGS.lz = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.ly = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.lu = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.Nc = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.filtorder = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.k_0 = (int)FLAGSmx[flagsIndex]; flagsIndex++;
        
        //load P0
        int ltheta = FLAGS.Nc*FLAGS.lu*(FLAGS.lu+FLAGS.ly);
        FLAGS.P0.resize(ltheta, ltheta); //Tell eigen the matrix size
        //TODO: fix this lazy indexing
        
        //assigning the array to a Eigen compatible matrix
        FLAGS.P0 = Eigen::Map<Eigen::MatrixXd>(&FLAGSmx[flagsIndex], (double)ltheta, (double)ltheta);
        for (int i = 0; i < pow(ltheta,2); i++)
        {           
            //FLAGS.P0 << FLAGSmx[flagsIndex]; 
            flagsIndex++;
        } 

        //load Ru
        FLAGS.Ru.resize(FLAGS.lu, FLAGS.lu); //Tell eigen the matrix size
        FLAGS.Ru = Eigen::Map<Eigen::MatrixXd>(&FLAGSmx[flagsIndex], (double)FLAGS.lu, (double)FLAGS.lu);
        for (int i = 0; i < pow(FLAGS.lu,2); i++)
        {
            //FLAGS.Ru << FLAGSmx[flagsIndex]; 
            flagsIndex++;
        }

        //load Rz
        FLAGS.Rz.resize(FLAGS.lz, FLAGS.lz); //Tell eigen the matrix size
        FLAGS.Rz = Eigen::Map<Eigen::MatrixXd>(&FLAGSmx[flagsIndex], (double)FLAGS.lz, (double)FLAGS.lz);
        for (int i = 0; i < pow(FLAGS.lz,2); i++)
        {
            //FLAGS.Rz << FLAGSmx[flagsIndex]; 
            flagsIndex++;
        }

        //load lambda
        FLAGS.lambda = FLAGSmx[flagsIndex]; flagsIndex++;
      
        //load theta_0
        FLAGS.theta_0.resize(ltheta); //Tell eigen the matrix size
        FLAGS.theta_0 = Eigen::Map<Eigen::VectorXd>(&FLAGSmx[flagsIndex], (double)ltheta, 1);
        for (int i = 0; i < ltheta; i++)
        {
            //FLAGS.theta_0 << FLAGSmx[flagsIndex]; 
            flagsIndex++;
        }

        rcacFilt FILT = initFiltSimulink(FLAGS.lz, FLAGS.ly, FLAGS.lu,
                                         FLAGS.Nc, FLAGS.filtorder, FILTmx);
  
        //create RCACRLS
        return new RCACRLS(FLAGS, FILT);
    }
    else if (rcacType == useGrad)
    {
        //load FLAGSmx into flags struct
        //mxArray must be in format
        //[lz; ly; lu; Nc; filtorder; k_0; alpha; theta_0]
        //Matrices must be transposed and then vectorized

        rcacGradFlags FLAGS;
        //load the first 6 variables
        int flagsIndex = 0;
        FLAGS.lz = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.ly = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.lu = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.Nc = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.filtorder = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.k_0 = (int)FLAGSmx[flagsIndex]; flagsIndex++;

        //load the step size scaling
        FLAGS.alpha = FLAGSmx[flagsIndex]; flagsIndex++;

        int ltheta = FLAGS.Nc*FLAGS.lu*(FLAGS.lu+FLAGS.ly);

        //load theta_0
        FLAGS.theta_0.resize(ltheta); //Tell eigen the matrix size
        FLAGS.theta_0 = Eigen::Map<Eigen::VectorXd>(&FLAGSmx[flagsIndex], (double)ltheta, 1);
        for (int i = 0; i < ltheta; i++)
        {
            //FLAGS.theta_0 << FLAGSmx[flagsIndex]; 
            flagsIndex++;
        }

        rcacFilt FILT = initFiltSimulink(FLAGS.lz, FLAGS.ly, FLAGS.lu,
                                         FLAGS.Nc, FLAGS.filtorder, FILTmx);

        //create RCACGrad
        return new RCACGrad(FLAGS, FILT);

    }
    else
    {
        std::cout << "Bad RCAC Type (Simulink)!" << "\n";
        //throw(EXIT_FAILURE);
        throw "Bad RCAC Type (Simulink)!";
    } 
}

#endif