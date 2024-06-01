#ifndef DUALLEARNER_H
#define DUALLEARNER_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>


#include <chrono> 
using namespace std::chrono; 

#include "rbfn.h"
#include "rbfn_semicircle.h"

//*********************************//
//       Gamma manual control      //

#define GAMMA_NORMAL_MODE 11.0
#define GAMMA_MANUAL_MODE 12.0
#define GAMMA_NO_UPDATING 13.0
#define GAMMA_UPDATING    14.0
#define CPG_WITHOUT_GAMMA 15.0

/// Class including the intra limb control for a millipede robot.
/// The control system uses local sensory feedback to modulate CPG outputs and
/// a learning system able to adjust automatically the sensory feedback strenght

class DualLearner
{
public:

    DualLearner(std::vector<double> w_, bool closeLoop);
    DualLearner(std::vector<double> w_, std::vector<double> learnParams_, bool closeLoop, int index, int rand);
    ~DualLearner(){};



    //Control steps and CPG reset
    std::vector<double> control_step(double forceSensorData, double time, bool is_set_gamma, double gamma);

    void reset();

    int legIndex = -1; 

    float stime;
    double liftamplitude, wideamplitude;
    double liftamplitudeObs, wideamplitudeObs;
    double psn1 = 0, psn2 = 0;
    std::vector<double> w;
    double B1 = 0.01, B2 = 0.01;
    double a1 = 0, a2 = 0;
    double C1 = 0, C2 = 0;
    double prevC1 = 0, prevC2 = 0, prevPos = 0;
    double motorPosPrev = 0;

    //***************************// test with normal CPG
    double B1_ = 0.01, B2_ = 0.01;
    double a1_ = 0, a2_ = 0;
    double C1_ = 0, C2_ = 0;
    double prevC1_ = 0, prevC2_ = 0;



    //Active learning
    bool learning_rbfn = false; 

    bool learning_SF = true;
    bool local_SFweight = true;
    double SFweight = 0;

    //****************//
    double prev_SFweight = 0;

    double efferenceCopy_error = 0;
    double fwdMod;
    double S = 0;
    int count;
    
    //**************//
    double prev_error = 0.0;

    double signal1= 0;
    double signal2= 0;
    double s;
    double devC1;
    double devC2;
    double preCPG1;
    double Err = 0.05;
    double ErrMod = 0;
    double prevL_OF;
    double Meu = 0.01;
    double diff = 0;
    double prevdiff = 0;
    double devDiff = 0;
    double prevDev = 0;
    double L_OF_sqr;
    double k = 10;
    double fwdModelPredct = 0;
    double error = 0;
    double sensorVal;
    double Gvalue = 0;
    double NoContactThresh = 0;
    double K_s = 0;
    double K_f = 0;


    bool BendBodyJointDown = false;
    bool BendBodyJointUp = false;
    bool groundInContact = false;

    int countInterval = 0;
    int prevcountInterval = 0;

    bool start;


    double derivSense = 0;
    int senseSig = 0;
    double prevsense = 0;

    double C2_clamp = 0;
    double C2_clampOut = 0;
    double CTr_motorSignal;
    float coeff = 0.9; //0.7

    std::vector<double> motorOutput = {0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}; //**** add twelve last index
    std::vector<double> cpgOutput = {0, 0};

    //**************************//
    std::vector<double> joeData;
    double gamma_mode = GAMMA_MANUAL_MODE; 
    double prev_gamma = 0.0;

    //*********************//
    double imu[3] = {};
    //*********************// for select which size is the leg
    double leg_direction = 0.0 ; // -1 <-- left |0| right --> 1

    //************** to identify which leg 
    int leg_index = 0;

    // ************* If using semicircle RBF << -------------------------------------- SEMI CIRCLE RBF
    bool use_semicircle_RBF = false;


    std::vector<double> getCpgOutput();
    bool DetectObstacle(std::vector<float> torqueData);
    void CPG_step();
    double Derivative(double sense);
    double rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter);
    double RBF(double x, double min, double max);

    // ************************** //
    double stepFunction(double value, double threshold);
    std::vector<double> SimplePreMotor_handcraft(double C1, double C2);
    std::vector<double> SimplePreMotor(double C1, double C2);
    double max(double num1, double num2);
    double min(double num1, double num2);
    double sigmoid(double x);
    double thresholdF(double input, double min, double max);


private:
    // CPG activity
    std::vector <double> localLegOut;
    void CPG_step(double input1, double input2);


    //Sensory feedback (SF) computation
    double sFeedFilter(double dataInput);
    double SignalThreshld(double dataInput);
    double LowPassFilter(double dataInput);


    //Learning algorithm for SF adjustment
    std::vector<double> LearningModel_step(double forceSensorData, bool delay);

    double Ks = 0.0, Kf = 0;
    double W = 0, Af = 0, As = 0, Bf = 0, Bs = 0;

    // ************* MI adaptation test ************** //
    bool MI_adapt = false;
    double MI = 0.00;   // 0.08
    double Ms = 0.0, Mf = 0.0;
    double ds = 0.0, ls = 0.0; 
    double df = 0.0, lf = 0.0;

    // ************* Algorithm implementation ************* //
    // IMU 
    double imu_rad[3] = {0,0,0};
    double imu_rad_init[3] = {0,0,0};
    double imu_deg[3] = {0,0,0};

    double frtLeg_IMUbias = 0;
    double midLeg_IMUbias = 0;
    double RerLeg_IMUbias = 0;

    double ML_obset = 0;
    double MR_obset = 0;
    
    double roll_degree = 0;
    double pitch_degree = 0;
    double yaw_degree = 0;


    // adaptive offset variable
    bool adapted = false;
    double E_angle = 0.0;
    double target_angle = 0.0;
    double current_angle = 0.0;

    double offset_TCjoint = 0.0;
    double prev_offset_TCjoint = 0.0; // for low pass filter : memory for do reset offset TCjoint

    double force_filtered_signal = 0.0;
    double roll_filtered_signal = 0.0;
    double raw_offset = 0.0;

    double joe_max = 0;
    double raw_filteredData = 0;


    // *************** END ************** //


    std::vector<double> lowpass;
    std::vector<double>filtererror;
    std::vector<double> ModelOutput;
    double lowpass_n, filtererror_n;

    double LowPassError =0;
    double Error=0;

    // RBF network 
    rbfn* rbfNet;
    rbfn_semicircle* rbfNetSemicircle;

    double sigma[6]      = {0.2, 0.12, 0.15, 0.2, 0.12, 0.15};

};

#endif // DUALLEARNER_H
