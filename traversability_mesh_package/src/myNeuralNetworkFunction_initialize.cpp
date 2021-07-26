//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  myNeuralNetworkFunction_initialize.cpp
//
//  Code generation for function 'myNeuralNetworkFunction_initialize'
//


// Include files
#include "myNeuralNetworkFunction_initialize.h"
#include "myNeuralNetworkFunction.h"
#include "myNeuralNetworkFunction_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void myNeuralNetworkFunction_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_Classifier = true;
}

// End of code generation (myNeuralNetworkFunction_initialize.cpp)
