

#include "adaptiveSF.h"


adaptiveSF::adaptiveSF()
    {

    
    }


void adaptiveSF::initialize(double MI){
    t = 0;

    ////////////////////////////////////////////////////////////////
    ////Activate Behaviors
    OpenLoopContrl = false; // Set to TRUE to activate Open Loop Walking Control
    CloseLoopContrl = true; // Set to TRUE to activate Close Loop Walking Control
    HorizontalMotion = false; // Set to TRUE to activate horizontal body movement behavior
    BodyBendDownBehav = false; // Set to TRUE to activate Body Bending Down Behavior
    BodyLiftUpBehav = false; // Set to TRUE to activate Body Lift Behvaior

    ////////////////////////////////////////////////////////////////s
    ////initializations for Leg Joints Control CPGs

    // ***** Joe Allert ****** //
    // this is not the MI that you can change the control system
    // the real main "MI" be at "adaptive_controller.h --> double control_input <-- at line 75"
    S=MI;

    // ****** !!!! take over and overwrite MI !! ****** //
    S = 0.1;
    
    //******************//
    // S = 0.1; // test frequency 0.1

    w = std::vector<double>(4); 
    w[0] = 1.4; 
    w[1] = 1.4; 
    w[2] =  0.18 + S; 
    w[3] = -0.18 - S; 

    //distributed sfw
    gl_Af =  0.57;
    gl_As =  0.998; ///As > Af

    gl_Bf =  0.006; //0.006; 
    gl_Bs =  0.0004; //0.0004;  ///Bs < Bf 
    gl_SFweight = 0;
    total_error = 0;

    nOfSegments = 1;
    legsPerSegment = 6;
    nOfVertBodyJoints = nOfSegments-1;
    NoGroundAtLegNo = 5;

    //amputate.resize(1); 
    //amputate[0] = 1; 

}


vector<double> adaptiveSF::getCpgOutput(int index){
    return cpgs[index].getCpgOutput(); 
}


float * adaptiveSF::getAlphaValues(){
    for (int i = 0; i < 6; i++)
    {
        sfWeightArray[i] = fwdModelOut[i][3]; 
    }
    return sfWeightArray; 
}


float * adaptiveSF::getFwdModel(int index){
    return fwdModelOut[index]; 
}


void adaptiveSF::initializeCPGs(double * _SFweight){
    srand (time(NULL));
    std::uniform_int_distribution<int> distribution(0, 40);

    int randomN1 = distribution(generator);
    int randomN2 = distribution(generator);

    for (int i = 0; i < legsPerSegment * nOfSegments; i++)
    {
        std::vector<double> learnParams;
        if(nolearning==true ){
            learnParams.push_back(_SFweight[i]);
        }

        else{
            learnParams.push_back(gl_Af);
            learnParams.push_back(gl_As);
            learnParams.push_back(gl_Bf);
            learnParams.push_back(gl_Bs);
        }

        if(i < 3)
            cpgs.push_back(DualLearner( w, learnParams, CloseLoopContrl, i, randomN1));
        else 
            cpgs.push_back(DualLearner( w, learnParams, CloseLoopContrl, i, randomN2));
    }
}


//void adaptiveSF::stepNoLearning(std::vector<float> SensorData, std::vector<float> JointPosData, std::vector<float> TorqueData, double simtime, std::vector<float> direction){
void adaptiveSF::stepNoLearning(vector<double> SensorData, double *target_pos, double simtime, double gamma_, double *imu_){

    ////////////////////////////////////////////////////////////////
    //// EXECUTE CONTROLLER FOR MILLIPEDE WALKING

    for(int i = 0; i < nOfSegments; i++) // --> for stick insect : nOfSegments = 1
    {
        for(int j = 0; j < legsPerSegment; j++)  // --> for stick insect : legsPerSegment = 6
        {
            NoGroundAtLegNo = j;
            cpg_index = i*legsPerSegment+j;

            cpgs[cpg_index].w[2] = 0.18+S;
            cpgs[cpg_index].w[3] = -0.18-S;
            cpgs[cpg_index].S = S;
            cpgs[cpg_index].k = K;

            sensor_dataAdaptive = SensorData[j]; //Sense Data Of All Legs for adaptive locomotion
            gamma = gamma_;

            
            // cpgs[cpg_index].MI = S;
            //************* Push data to dual learner *************//
            for(int i = 0; i < 3; i++){
                cpgs[cpg_index].imu[i] = imu_[i];
            }




            //                        __     
            //   \                   (  )                   /
            //    \                  |  |                  /
            //     \                 |  |                 /
            //      \[16]__[10](4)  3|  |0  (1)[7]___[13]/ 
            //                       |  |   
            //                       |  |   
            //  ____[17]___[11](5)  4|  |1  (2)[8]___[14]_____
            //                       |  |   
            //                       |  |   
            //      [18]___[12](6)  5|  |2  (3)[9]___[15]
            //     /                 |  |                \
            //    /                  |  |                 \
            //   /                   |  |                  \    
            //  /                    |  |                   \
            //                       |  |   
            //                       |  |   
            //                       |  |   
            //                       |  |   
            //                       |__|        


            // If the leg is right
            if (cpg_index >= 3){
                cpgs[cpg_index].leg_direction = -1.0;
            }

            // If the leg is left
            else if(cpg_index < 3){
                cpgs[cpg_index].leg_direction = 1.0;
            }
            // else{
            //     cpgs[cpg_index].leg_direction = 0.0;
            // }


            // if (cpg_index == 4){
            //     cpgs[cpg_index].leg_direction = 1.0;
            // }
            // else if(cpg_index == 1){
            //     cpgs[cpg_index].leg_direction = -1.0;
            // }
            // else{
            //     cpgs[cpg_index].leg_direction = 0.0;
            // }
            

            // ********************* push the leg index to the dual learner 
            cpgs[cpg_index].leg_index = cpg_index;


            
            

            ////////////////////////////////////////////////////////////////
            //// Amptutate legs after some time 

            bool lift = false;
            if(simtime >= amputationTime)
            {
                for(unsigned int k = 0; k < amputate.size(); k++){
                    if(amputate[k]==cpg_index)
                        lift=true;
                }
            }
            else
            {
                lift=false;
            }


            // gamma set from outside
            // gamma = 0.2;



            // This will bring all parameter in this function to duallearner
            std::vector<double> motorCommands = cpgs[cpg_index].control_step(sensor_dataAdaptive, simtime, is_set_gamma, gamma_);
            
            // -------------  = moterCommands[0];  // moter TC Output
            // -------------  = moterCommands[1];  // moter CF Output
            // -------------  = moterCommands[2];  // moter FT Output
            fwdModelOut[j][0] = motorCommands[3]; // Output of fwd Model
            fwdModelOut[j][1] = motorCommands[4]; // Error btw fwd Model and Sensor
            fwdModelOut[j][2] = motorCommands[5]; // actual sensor value
            fwdModelOut[j][3] = motorCommands[6]; // weights for sensor feedback
            fwdModelOut[j][4] = motorCommands[7]; // G value
            fwdModelOut[j][5] = motorCommands[8]; // Raw force data
            fwdModelOut[j][6] = motorCommands[9]; // Z position from RBFN
            fwdModelOut[j][7] = motorCommands[10]; // Z position  form RBFN with lowpass

            //*****************//
            fwdModelOut[j][8] = motorCommands[11];   // ks
            fwdModelOut[j][9] = motorCommands[12];   // kf
            fwdModelOut[j][10] = motorCommands[13];  // roll_degree
            fwdModelOut[j][11] = motorCommands[14];  // pitch_degree
            fwdModelOut[j][12] = motorCommands[15];  // raw_force
            fwdModelOut[j][13] = motorCommands[16];  // fiter_force
            fwdModelOut[j][14] = motorCommands[17];  // beta_fiter_force
            fwdModelOut[j][15] = motorCommands[18];  // beta_fiter_roll
            fwdModelOut[j][16] = motorCommands[19];  // E_pitch_angle
            fwdModelOut[j][17] = motorCommands[20];  // raw_cal_TC_offset
            fwdModelOut[j][18] = motorCommands[21];  // prev_TC_offset
            fwdModelOut[j][19] = motorCommands[22];  // TC_offset_degree
            





            ////////////////////////////////////////////////////////////////
            ////Set motor commands for lifting or closed loop control

            if(lift){
                target_pos[j] = 0;
                target_pos[j+6] = 2.0;
                target_pos[j+12] = -1.5;
            }

            else{
                if (CloseLoopContrl == true && OpenLoopContrl == false){
                    //Outputs For Adaptive Forward Model
                    target_pos[j] =   motorCommands[0]; 
                    target_pos[j+6] = motorCommands[1]; 
                    target_pos[j+12] = motorCommands[2]; 
                }
            }
            Out.clear();
        } // for loop 6 times
    }
    t++;
}
