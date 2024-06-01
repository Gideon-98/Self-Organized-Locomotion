#include "so2CPG.h"


so2CPG::so2CPG(){
    b1 = b2 = 0.01;     // Setting bias term
    g1 = g2 = 0.00;     // Setting default feedback strength
    reflex = 0.18;      // Setting default reflex
}

so2CPG::so2CPG(double MI, double alpha)
{
    // Setting up default values
    b1 = b2 = 0.01;     // Setting bias term
    g1 = g2 = alpha;    // Setting custom feedback strength (alpha)
    reflex = MI;        // Setting custom modulation input / speed
}

so2CPG::so2CPG(double MI)
{
    // Setting up default values
    b1 = b2 = 0.01;    // Setting bias term
    g1 = g2 = 0.1;     // Setting default feedback strength 
    reflex = MI;       // Setting custom modulation input / speed
}

double so2CPG::Tanh(double input){
    return ((2.0/(1.0 + exp((-2.0) * input))) - 1.0);
}

void so2CPG::setReflex(double input){
    reflex = input;
}


std::vector<double> so2CPG::getCPGValues(){
    return output;
}

std::vector<double> so2CPG::getActivation(){
    std::vector<double> activation; 
    activation.push_back(a1); 
    activation.push_back(a2);
    return activation; 
}

void so2CPG::enableForceFeedback(){
    phaseAdaptationActive = true;
}

void so2CPG::disableForceFeedback(){
    phaseAdaptationActive = false; 
}


/* CALCULATING THE NEURON OUTPUT WITHOUT FORCE FEEDBACK */

std::vector<double> so2CPG::calculate()
{
    w12 =  0.18 + reflex; 
    w21 = -0.18 - reflex; 

    a1 = w11 * output[0] + w12 * output[1] + b1; 
    a2 = w22 * output[1] + w21 * output[0] + b2;

    output[0] = Tanh(a1);
    output[1] = Tanh(a2);

    return output;  
}

/* CALCULATING THE NEURON OUTPUT WITH FORCE FEEDBACK */

std::vector<double> so2CPG::calculatePhaseAdaptation(double fs)
{

    w12 =  0.18 + reflex; 
    w21 = -0.18 - reflex; 
    
    double force = 0;
    if(phaseAdaptationActive) 
        force = fs; //std::min(fs, 1.0); 

    
    a1 = w11 * output[0] + w12 * output[1] + b1 - g1 * force * cos(a1); 
    a2 = w22 * output[1] + w21 * output[0] + b2 - g2 * force * sin(a2);

    output[0] = Tanh(a1);
    output[1] = Tanh(a2);

    return output;  
}

so2CPG::~so2CPG()
{
}
