#ifndef ADAPTIVE_CONTROLLER_H
#define ADAPTIVE_CONTROLLER_H

// #include "modularNeuralController.h"
// #include "so2CPG.h"
#include "rbfn.h"
// #include "rbfn_semicircle.h"
#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <deque> 
#include <iterator>
#include <random>

#include "adaptiveSF.h"
#include "duallearner.h"


class AdaptiveController
{

    private:
    // ROS INIT
    ros::NodeHandle nh_; 
    ros::Publisher set_joint_target; 
    ros::Publisher pub_cpg_values; 
    ros::Publisher pub_fwd_model; 
    ros::Publisher pub_sf_weight;

    //*********************//
    ros::Publisher pubme;

    ros::Subscriber get_joint_pos; 
    ros::Subscriber get_force_sensor_x; 
    ros::Subscriber get_force_sensor_y; 
    ros::Subscriber get_force_sensor_z; 
    ros::Subscriber get_force_sensor;
    ros::Subscriber get_sim_time;
    ros::Subscriber get_sim_state;

    //*********************//
    ros::Subscriber get_gamma;
    ros::Subscriber get_imu;


    int sim_state = 99;
    bool isLearning = false;

    // Delayed controller
    std::deque<vector<double>> cpgValQueue; 
    int timeStepSize = 16; 
    int queueSize = timeStepSize * 6; 



    // Decoupled controller
    // modularController* MNC[6]; // --
    rbfn* rbfNet[6];
    // rbfn_semicircle* rbfNetSemicircle[6];
    double sigma[6]      = {0.2, 0.12, 0.15, 0.2, 0.12, 0.15}; // The sigma value for rbfn network

    double alpha[6] =  {0.15, 0.15, 0.15, 0.15, 0.15, 0.15}; //{0.14, 0.09, 0.07, 0.14, 0.09, 0.07};        // The alpha values for the feedback strength 

    // This is real MI from joe alert!!
    double control_input    = 0.99; // Delayed = 0.17, Symmetric = 0.15-->tripod
    double force_scale      = 30; 
    int rightShift          = 15; // Slow (CI  = 0.05) = 30, Fast (CI = 0.15) = 15

    // Combined

    int tick_count;
    double global_time;

    // Gausssian noise 

    const double mean = 0;
    const double stddev = 0.1*1.0;

    std::random_device r;    
    std::default_random_engine generator{r()};


    // SENSOR AND MOTOR ARRAYS
    double m_pos[18]        = {0}; 
    double target_pos[18]   = {0}; 
    double fs_x[6]          = {0};
    double fs_y[6]          = {0};
    double fs_z[6]          = {0};
    double fs[6]            = {0};
    
    // RUNNING AVARAGE
    std::deque<double> fs_queue[6]; 
    double fs_sum[6]        = {0};
    double fs_avg[6]        = {0};
    int avgCount            = 0;

    
    // Setting FR-joints
    double fr_value; 

    // Movement scale 
    double globalScale = 1.0;

    //---------------------------//
    // Set external gamma from Outside
    // Set to 11 to start as Normal Mode
    // Set to 12 to start as Manual Mode
    // Set to a number which less than 1.5 is starting at that number 
    double get_gamma_value = 0.0;
    double get_imu_value[3] = {0};


    public:

    double meanFilter(double value); 

    AdaptiveController(ros::NodeHandle &nh);
    ~AdaptiveController();

    void vrepScale(double *y);
    void getJointPositions(const std_msgs::Float64MultiArray::ConstPtr& array); 
    void getForceSensorsX(const std_msgs::Float64MultiArray::ConstPtr& array);
    void getForceSensorsY(const std_msgs::Float64MultiArray::ConstPtr& array);
    void getForceSensorsZ(const std_msgs::Float64MultiArray::ConstPtr& array);
    void getForceSensors(const std_msgs::Float64MultiArray::ConstPtr& array);
    void getSimTime(const std_msgs::Float64::ConstPtr& data);
    void getSimState(const std_msgs::Float32::ConstPtr& data);

    //******************//
    // get gamma from outside
    void getGamma(const std_msgs::Float32::ConstPtr& data);
    void getImuSenser(const std_msgs::Float64MultiArray::ConstPtr& array);


    void pubJointTargetPos(double *y, int size);
    void pubCpgValues(vector<double> y); 
    void pubFwdModel(); 
    void pubSFWeights(); 

    // Control functions 
    void decreaseFrequency();
    void increaseFrequency();

    
    // adaptive controller 
    adaptiveSF * controller;


    void stepAdaptiveSF(); 

    void step();
    void stepLearning();
    int getTickCount();
    double getGlobalTime(); 
    int getSimState();
    double degToRad(double deg);


};

#endif
