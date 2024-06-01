#include "adaptive_controller.h"


AdaptiveController::AdaptiveController(ros::NodeHandle &nh): nh_ (nh)
{
    // Subscriber
    get_joint_pos = nh_.subscribe("sim_ros_interface/sensor/joint_pos", 1, &AdaptiveController::getJointPositions, this); 
    get_force_sensor_x = nh_.subscribe("sim_ros_interface/sensor/force_sensor_x", 1, &AdaptiveController::getForceSensorsX, this); 
    get_force_sensor_y = nh_.subscribe("sim_ros_interface/sensor/force_sensor_y", 1, &AdaptiveController::getForceSensorsY, this); 
    get_force_sensor_z = nh_.subscribe("sim_ros_interface/sensor/force_sensor_z", 1, &AdaptiveController::getForceSensorsZ, this); 
    get_force_sensor = nh_.subscribe("sim_ros_interface/sensor/force_sensor", 1, &AdaptiveController::getForceSensors, this);
    
    get_sim_time = nh_.subscribe("sim_ros_interface/simulation_time", 1, &AdaptiveController::getSimTime, this);
    get_sim_state = nh_.subscribe("sim_ros_interface/simulation_state", 1, &AdaptiveController::getSimState, this);

    // *************************//
    get_gamma = nh_.subscribe("exam/gamma", 1, &AdaptiveController::getGamma, this);
    get_imu = nh.subscribe("sim_ros_interface/sensor/imu",1, &AdaptiveController::getImuSenser, this);


    // Publisher
    set_joint_target = nh_.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/target/joint_pos", 1);
    pub_cpg_values = nh_.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/target/cpg", 1);

    pub_fwd_model = nh_.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/topic/forwardModel",1); 
    pub_sf_weight = nh_.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/topic/sfweights",1); 

    //set_joint_pos = nh_.publish("sim_ros_interface/target/joint_pos")

    tick_count = 0; 
    global_time = 0; 
    double cr_scale = 0.5;


    // Intializing the controller

    controller = new adaptiveSF();
    controller->initialize(control_input);
    controller->initializeCPGs(alpha);
 

}

int AdaptiveController::getSimState(){
    return sim_state;
}

// **********************************//

void AdaptiveController::getGamma(const std_msgs::Float32::ConstPtr& data){
    get_gamma_value = data->data;
}

void AdaptiveController::getImuSenser(const std_msgs::Float64MultiArray::ConstPtr& array){
    int size =  array->data.end()  - array->data.begin(); 

    if(size == 3){
        for (size_t i = 0; i < size; i++)
        {
            get_imu_value[i] = array->data[i]; 
        }
    }else{
        ROS_INFO("ERROR IN FORCE SENSOR X DATA SIZE!");
    }
}




void AdaptiveController::getSimTime(const std_msgs::Float64::ConstPtr& data){
    if(data->data != -1)
        global_time = data->data; 
    else 
        ROS_ERROR("Error in sim time ..");
}

void AdaptiveController::getSimState(const std_msgs::Float32::ConstPtr& data){
    if(data->data != -1)
        sim_state = data->data; 
    else 
        ROS_ERROR("Error in sim state ..");
}


void AdaptiveController::getForceSensorsX(const std_msgs::Float64MultiArray::ConstPtr& array){
    int size =  array->data.end()  - array->data.begin(); 

    if(size == 6){
        for (size_t i = 0; i < size; i++)
        {
            fs_x[i] = array->data[i] * force_scale; 
        }
    }else{
        ROS_INFO("ERROR IN FORCE SENSOR X DATA SIZE!");
    }
    
}


void AdaptiveController::getForceSensors(const std_msgs::Float64MultiArray::ConstPtr& array){
    int size =  array->data.end()  - array->data.begin(); 
    if(size == 6){
        for (size_t i = 0; i < size; i++)
        {
            fs[i] = array->data[i]; 
        }
    }else{
        ROS_INFO("ERROR IN FORCE SENSOR X DATA SIZE!");
    }
    
}

void AdaptiveController::getForceSensorsY(const std_msgs::Float64MultiArray::ConstPtr& array){
    int size =  array->data.end()  - array->data.begin(); 

    if(size == 6){
        for (size_t i = 0; i < size; i++)
        {
            fs_y[i] = array->data[i] * force_scale; 
        }
    }else{
        ROS_INFO("ERROR IN FORCE SENSOR Y DATA SIZE!");
    }
    
}

void AdaptiveController::getForceSensorsZ(const std_msgs::Float64MultiArray::ConstPtr& array){
    int size =  array->data.end() - array->data.begin(); 

    if(size == 6){
        for (size_t i = 0; i < size; i++)
        {
            fs_z[i] = array->data[i] * force_scale; 
        }
    }else{
        ROS_INFO("ERROR IN FORCE SENSOR Z DATA SIZE!");
    }
}

void AdaptiveController::getJointPositions(const std_msgs::Float64MultiArray::ConstPtr& array){ 
    int size =  array->data.end()  - array->data.begin(); 

    if(size == 18){
        for (size_t i = 0; i < size; i++)
        {
            m_pos[i] = array->data[i]; 
        }
    }else{
        ROS_INFO("ERROR IN JOINT POSITION DATA SIZE!");
    }
}



void AdaptiveController::decreaseFrequency()
{

    if(controller->S > 0.005)
        controller->S = controller->S - 0.005; 
    std::cout << "Decreasing frequency: " << controller->S << std::endl;
    control_input = controller->S; 

}

void AdaptiveController::increaseFrequency()
{

    controller->S = controller->S + 0.005; 
    std::cout << "Increasing frequency: " << controller->S  << std::endl;
    control_input = controller->S; 

}




void AdaptiveController::step(){
    stepAdaptiveSF(); 
}


// go to this function first
void AdaptiveController::stepAdaptiveSF(){
    
    vector<double> force_vec; 

    for (int i = 0; i < 6; i++) // 6
    {
        std::normal_distribution<double> dist(mean, stddev);
        force_vec.push_back(fs[i] + dist(generator));

    }
    
    
    // Get the data from controller //********************//
    controller->stepNoLearning(force_vec, target_pos, global_time, get_gamma_value, get_imu_value);
    

    // Sending cpg data 
    std::vector<double> cpg;

    for (int i = 0; i < 6; i++)
    {
        cpg.push_back(controller->getCpgOutput(i)[0]);  
        cpg.push_back(controller->getCpgOutput(i)[1]);  
    }
    

    pubCpgValues(cpg); 


    //******************// for check " Is ID motor is correct ? "
    // target_pos[0] = 0.0;
    // target_pos[6] = 0.0;
    // target_pos[12] = 0.0;
    // Send target positions
    pubJointTargetPos(target_pos, 18);

    // Send forward model 
    pubFwdModel(); 

    // Send all feedback strength weights 
    pubSFWeights();

    tick_count++;
}



double AdaptiveController::getGlobalTime(){
    return global_time; 
}


int AdaptiveController::getTickCount(){
    return tick_count; 
}


void AdaptiveController::pubCpgValues(vector<double> y){
    std_msgs::Float64MultiArray msg; 

    for (int i = 0; i < y.size(); i++)
    {
        msg.data.push_back(y[i]);
    }

    pub_cpg_values.publish(msg);
}


void AdaptiveController::pubFwdModel(){
    std_msgs::Float64MultiArray msg;

    for (int j = 0; j < 6 ; j++)
    {
        float *fwdResults = controller->getFwdModel(j); 

        // for (int i = 0; i < 8; i++)
        //***************************//
        for (int i = 0; i < 20; i++) // 8 + 12 = 20
        {
            msg.data.push_back(fwdResults[i]);
        }
    }

    msg.data.push_back(control_input);

    pub_fwd_model.publish(msg);

}

void AdaptiveController::pubSFWeights(){
    std_msgs::Float64MultiArray msg; 

    float *fwdResults = controller->getAlphaValues(); 


    for (int i = 0; i < 6; i++)
    {
        msg.data.push_back(fwdResults[i]);
    }

    pub_sf_weight.publish(msg);
}


void AdaptiveController::pubJointTargetPos(double *y, int size){
    std_msgs::Float64MultiArray msg; 

    for (int i = 0; i < 18; i++)
    {
        if(i < size)
            msg.data.push_back(y[i]);
        else
            msg.data.push_back(fr_value);
    }

    if(global_time > 1.0)
        set_joint_target.publish(msg);
}

double AdaptiveController::degToRad(double deg){
    return deg * (M_PI/180);
}




AdaptiveController::~AdaptiveController(){

}
