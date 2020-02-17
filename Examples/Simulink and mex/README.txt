Compile the mex file using

mex COMPFLAGS='$COMPFLAGS -O3 -std=c++11' -largeArrayDims RCACSimulink.cpp RCAC.cpp RCACRLS.cpp
