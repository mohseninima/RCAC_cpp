#ifndef _RCACCREATOR_HPP_
#define _RCACCREATOR_HPP_

#include "RCAC.hpp"
#include "RCACRLS.hpp"

//Factory Method (TODO: move to a separate class, fix memory leak)
template <typename T>
RCAC* RCAC::init(
    T &FLAGS, 
    rcacFilt &FILT, 
    std::string &rcacType
)
{
    if (rcacType == "RLS")
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