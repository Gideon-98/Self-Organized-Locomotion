#ifndef ADAPTIVE_SF_H
#define ADAPTIVE_SF_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"

#include "duallearner.h"
#include <random>
#include <chrono>

using namespace std;


class adaptiveSF {
public:

    double gl_dW = 0, gl_dslow = 0, gl_dfast = 0, gl_Af = 0, gl_As = 0, gl_Bf = 0, gl_Bs = 0, gl_SFweight = 0, total_error = 0;
    double posx = 0, posy = 0, posz = 0;
    double velx = 0, vely = 0, velz = 0;
    double accx = 0, accy = 0, accz = 0;
    std::vector<double> w;
    std::vector<double> ww;
    int cpg_index;
    int count = 0;
  
    // Random generator 
    std::random_device r;    
    std::default_random_engine generator{r()};

    // Amputation related variables 
    double amputationTime = 30;
    std::vector<int> amputate;


    //*****************************//
    //Set gamma
    bool is_set_gamma = false;
    double gamma = 0.0;

    // Get IMU value
    double imu = 0.0;

    bool OpenLoopContrl;
    bool CloseLoopContrl;
    bool HorizontalMotion;
    bool BodyBendDownBehav;
    bool BodyLiftUpBehav;

    bool activ_search = false;
    bool activ_elevat = false;

    bool obstacleDetected = false;
    bool noGroundDetected = false;

    double liftamplitude = 0.2, wideamplitude = 0.2;
    double liftamplitudeObs = 0.3, wideamplitudeObs = 0.3;
    double liftamplitudeBend = 0.1, wideamplitudeBend = 0.1;

    int delayIntrvl = 150; // Delay Interval (counting decrement of 1) for keeping the body Bend Up


    double M1 = 0;
    double M2 = 0;

    int nOfSegments;
    int legsPerSegment;
    int nOfVertBodyJoints;
//    double sfeed;
    double S = 0.08;  // 0.08
    double K = 10;

//    double followed;
    int simN = 0;
    bool saveData;
    bool control_isactive = false;

    bool startDelay = false;

    bool ObsOnLeg6 = false;
    bool ObsOnLeg7 = false;
    bool ObsOnLeg0 = false;
    bool ObsOnLeg1 = false;

//    MillipedeConf mconf;
    std::vector<DualLearner> cpgs;

    std::vector<double> sensors, motors, odom;
    std::vector <double> Out;

    double LocalLegControl = 0;
    double LocalLegErr = 0;
    double LocalLegDiff = 0;
    double LocalLegWI = 0;
    double LocalLegWR = 0;
    double LocalLegB = 0;

    double torqueDiff = 0;

    double followed = 0;

    std::vector<double>SensorFB;
    int BodySegCPG_index;
    int simCount = 0;

    double sensor_data = 0;
    double sensor_dataAdaptive = 0;
    double sensor_dataObstacle = 0;

    int NoGroundAtLegNo;

    bool nolearning = false; // false


    vector<vector<double>> forcesensor;
    std::vector<float> motorId;
    std::vector<float> segId;
    std::vector<float> HsegId;
    std::vector<float> motorPosition;
    std::vector<float> positions;

    std::vector<float> simInput;
    std::vector<float> data;

    // float fwdModelOut[6][8] = {0, 0};
    //***************************//
    float fwdModelOut[6][20] = {0, 0}; // 8 + 12 = 20

    
    float sfWeightArray[6]; 

    double forceLimit = 1000;

    float * getFwdModel(int index);
    float * getAlphaValues();


    vector<double> getCpgOutput(int index);


    void initialize(double MI);
    void initializeCPGs(double * _SFweight); // To be called right after selecting a mconf
    adaptiveSF();
    
    double rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter);



    virtual ~adaptiveSF(){};


    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    //virtual void stepNoLearning(std::vector<float> SensorData, std::vector<float> JointPosData, std::vector<float> TorqueData, double simtime, std::vector<float> direction);
    virtual void stepNoLearning(vector<double> SensorData, double *target_pos, double simtime, double gamma_, double *imu_);

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const {
      return true;
    };
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f){
      return true;
    };


  protected:
    unsigned short number_channels;

    int t = 0;

};

#endif

