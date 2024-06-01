#ifndef SO2CPG_H
#define SO2CPG_H

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

class so2CPG{
    private:
    // The activation in neuron 1 and 2
    double a1; 
    double a2;
 
    // The weights between and within neuron 1 and 2
    double w11 = 1.4;    
    double w22 = 1.4;       
    double w12;             
    double w21;             

    // The bias for neuron 1 and 2 
    double b1;
    double b2; 

    // Gamma values
    double g1;
    double g2;

    // The reflex/control parameter (To adjust the frequency)
    double reflex;      
 

    // Initialize the output to some values to activate 
    std::vector<double> output{0.01,0.01};

    // Phase adaption enable bool
    bool phaseAdaptationActive = true;


    public:

    so2CPG();
    so2CPG(double MI);
    so2CPG(double MI, double alpha);
    std::vector<double> calculate(); 
    std::vector<double> calculatePhaseAdaptation(double fs);
    std::vector<double> getCPGValues();
    std::vector<double> getActivation();
    double Tanh(double input); 
    void setReflex(double input);

    void enableForceFeedback(); 
    void disableForceFeedback();
    ~so2CPG();
};

#endif 