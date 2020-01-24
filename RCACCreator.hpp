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

#endif