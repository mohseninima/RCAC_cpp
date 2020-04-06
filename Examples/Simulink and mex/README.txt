Compile the mex file using

mex COMPFLAGS='$COMPFLAGS -O3 -std=c++11' -largeArrayDims RRCACSimulink.cpp RCAC.cpp RCACRLS.cpp RCACGrad.cpp RCACCumgrad.cpp