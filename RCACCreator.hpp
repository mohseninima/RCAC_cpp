#ifndef _RCACCREATOR_HPP_
#define _RCACCREATOR_HPP_

#include "RCAC.hpp"
#include "RCACRLS.hpp"

//Name: Nima Mohseni
//Date: 1/20/2020
//Purpose: This file contains the implementation of the factory function to
//generate new RCAC versions

//RCAC Types
std::string useRLS = "RLS";

//Factory Method (TODO: move to a separate class, fix memory leak)
template <typename T>
RCAC* RCAC::init(
    T &FLAGS, 
    rcacFilt &FILT, 
    std::string &rcacType
)
{
    if (rcacType == useRLS)
    {
        return new RCACRLS(FLAGS, FILT);
    }
    else
    {
        std::cout << "Bad RCAC Type!" << "\n";
        exit(EXIT_FAILURE);
    }  
}






//Function initSimulink, takes the array format of the RCAC variables from 
//simulink and converts it to the struct format and returns a pointer to an RCAC
//object
//
//ITEGRATE ME:
//std::string msg = std::string("Aw snap: ");       
//mexErrMsgTxt(msg.c_str());
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
        //[lu; lz; ly; Nc; filtorder; k_0; P0'(:); Ru'(:); Rz'(:); lambda; theta_0]
        //Matrices must be transposed and then vectorized

        rcacRlsFlags FLAGS;
        //load the first 6 variables
        int flagsIndex = 0;
        FLAGS.lu = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.lz = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.ly = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.Nc = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.filtorder = FLAGSmx[flagsIndex]; flagsIndex++;
        FLAGS.k_0 = FLAGSmx[flagsIndex]; flagsIndex++;

        //load P0
        int ltheta = FLAGS.Nc*FLAGS.lu*(FLAGS.lu+FLAGS.ly);
        FLAGS.P0.resize(ltheta, ltheta); //Tell eigen the matrix size
        for (int i = 0; i < pow(ltheta,2); i++)
        {           
            FLAGS.P0 << FLAGSmx[flagsIndex]; flagsIndex++;
        } 

        //load Ru
        FLAGS.Ru.resize(FLAGS.lu, FLAGS.lu); //Tell eigen the matrix size
        for (int i = 0; i < pow(FLAGS.lu,2); i++)
        {
            FLAGS.Ru << FLAGSmx[flagsIndex]; flagsIndex++;
        }

        //load Rz
        FLAGS.Rz.resize(FLAGS.lz, FLAGS.lz); //Tell eigen the matrix size
        for (int i = 0; i < pow(FLAGS.lz,2); i++)
        {
            FLAGS.Rz << FLAGSmx[flagsIndex]; flagsIndex++;
        }

        //load lambda
        FLAGS.lambda = FLAGSmx[flagsIndex]; flagsIndex++;
      
        //load theta_0
        FLAGS.theta_0.resize(ltheta); //Tell eigen the matrix size
        for (int i = 0; i < ltheta; i++)
        {
            FLAGS.theta_0 << FLAGSmx[flagsIndex]; flagsIndex++;
        }

        //load FILTmx into FILT struct
        //mxArray must be in format
        //[FILTNu'(:); FILTDu'(:); FILTNz'(:); FILTDz'(:)]
        //Matrices must be transposed and then vectorized

        rcacFilt FILT;

        //load Gf numerator
        int filtIndex = 0;
        FILT.filtNu.resize(FLAGS.lz, FLAGS.lu*FLAGS.filtorder); //Tell eigen the matrix size
        for (int i = 0; i < (FLAGS.lz*FLAGS.lu*FLAGS.filtorder); i++)
        {
            FILT.filtNu << FILTmx[filtIndex]; filtIndex++;
        }
      
        //load Gf Denominator
        FILT.filtDu.resize(FLAGS.lz, FLAGS.lz*(FLAGS.filtorder-1)); //Tell eigen the matrix size
        for (int i = 0; i < (FLAGS.lz*FLAGS.lz*(FLAGS.filtorder-1)); i++)
        {
            FILT.filtDu << FILTmx[filtIndex]; filtIndex++;
        }
       
        //load Gf_z Numerator (never used?)
        FILT.filtNz.resize(FLAGS.lz, FLAGS.lz); //Tell eigen the matrix size
        for (int i = 0; i < (FLAGS.lz*FLAGS.lz); i++)
        {
            FILT.filtNz << FILTmx[filtIndex]; filtIndex++;
        }

        //load Gf_z Denominator (never used?)
        FILT.filtDz.resize(FLAGS.lz, FLAGS.lz); //Tell eigen the matrix size
        for (int i = 0; i < (FLAGS.lz*FLAGS.lz); i++)
        {
            FILT.filtDz << FILTmx[filtIndex]; filtIndex++;
        }       
  
        //create RCACRLS
        return new RCACRLS(FLAGS, FILT);
    }
    else
    {
        std::cout << "Bad RCAC Type (Simulink)!" << "\n";
        //throw(EXIT_FAILURE);
        throw "Bad RCAC Type (Simulink)!";
    } 
}

#endif