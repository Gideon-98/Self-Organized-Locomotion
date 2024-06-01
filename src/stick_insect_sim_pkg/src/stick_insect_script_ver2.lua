function sysCall_init()

    require "socket"

    --*******************************************
    --*                                         *
    --*         create global variable          *
    --*                                         *
    --*******************************************

    -- For performance measurement 
    orientation = {0, 0, 0}
    orientationPrev = {0, 0, 0}
    orientation_init = {0, 0, 0}
    orientation_deg = {0, 0, 0}


    -- For logging 
    cpgValues = {}
    target = {}
    fwdModelData = {}


    -- Settings
    logging = false                    -- For logging data into files
    customMass = true                   -- To use custom mass given by the mass parameters 
    forceDataType = "z"                 -- Indicate which force to publish ("z" = z-axis or "avg" = average of three axis)
    forceMul = 1.5                      -- Scaling value for the force
    forceMulMatrix = {1,1,1,1,1,1}      -- Scaling value for each indiviual leg
    maxForce = 20.0    -- 20.0                 -- Maxmium allowed force (Newton)
    maxTorque = 30.0    -- 6.0                -- The maximum torque of the motors 
    massScale = 1 -- 126                  -- Scale the masses by some factor (126 = scaled by a factor of 5)
    -- ****************** --
    is_visualize = false
    normal_pose = false

    -- Other globals 
    forceData = {0,0,0,0,0,0}           -- force data for graph (eg. z, avg or abs)
    forceList = {{}, {}, {}, {}, {}, {}}
    maxVelocity = 0                     -- The maximum velocity observed for a given run

    velocity = 0
    vel_raw = {0,0,0}


    -- psudoBody_name = 'psudo_body'
    -- psudoBody = sim.getObjectHandle('psudo_body')

    jointName = {'TR0','TR1','TR2','TL0','TL1','TL2','CR0','CR1','CR2','CL0','CL1','CL2','FR0','FR1','FR2','FL0','FL1','FL2'}
    legName = {'R0','R1','R2','L0','L1','L2'}
    legpartName = {'COXA','FEMUR','TIBIA'}
    bodypartName = {'abdomen', 'thorax_hind','thorax_front','caput'}
    -- bodypartName = {'abdomen'}

    footName = {}
    footLeft_name = {'FOOT_L0', 'FOOT_L1', 'FOOT_L2'}
    footRight_name = {'FOOT_R0', 'FOOT_R1', 'FOOT_R2'}

    legpartMass = {0.1, 0.2926, 0.3539, 0.1, 0.1906, 0.2703, 0.1, 0.2338, 0.3709} -- FL, ML, HL (Mass of the legs)

    bodypartMass = {2, 2, 2, 2} -- = 8


    balljointMass = 0.1  -- Mass of the ball joints - Needs to be minimum 0.1 [gram]

    -- Handles 
    legpartHandles = {}
    jointHandles = {}
    jointPosition = {}
    bodypartHandles = {}
    footHandles = {}
    balljointHandles = {}
    jointTorque = {}
    leftFoot = {}
    rightFoot = {}

    -- imu
    imu_rad = {0,0,0}
    imu_rad_init = {0,0,0}
    imu_deg = {0,0,0}

    frtLeg_IMUbias = 0
    midLeg_IMUbias = 0
    RerLeg_IMUbias = 0

    ML_obset = 0
    MR_obset = 0





    -- time of os 
    -- t = os.date ("*t")
    init_time_sec = sim.getSystemTime()
    init_time_ms = simGetSystemTimeInMs(-1)


    lastSimState = sim.getSimulationState()


    -- lowPass filer
    prev_value_imu = 0

    robot_distance = 0
    test_distance = 0


    angle_bending = 0

    --*******************************************
    --*                                         *
    --*          create object handle           *
    --*                                         *
    --*******************************************


    -- ball joint part handles
    -- for i = 1, 18, 1 do 
    --     balljointHandles[i] = sim.getObjectHandle(jointName[i] .. '_BALL')
    -- end

    -- Mass
    -- body part handles
    for i = 1, 4, 1 do -- 4
        bodypartHandles[i] = sim.getObjectHandle(bodypartName[i])
    end


    -- leg part handles
    for i = 1, 6,1 do 
        for j = 1, 3, 1 do
            legpartHandles[j + 3*(i-1)] = sim.getObjectHandle(legpartName[j] .. '_' .. legName[i])
        end

        footHandles[i] = sim.getObjectHandle('FOOT_' .. legName[i])
    end

    
    -- joint handles
    for i = 1, 18, 1 do 
        jointHandles[i] = sim.getObjectHandle(jointName[i])
    end

    -- force sensor handles 
    R0=sim.getObjectHandle('R0_fs')
    R1=sim.getObjectHandle('R1_fs')
    R2=sim.getObjectHandle('R2_fs')
    L0=sim.getObjectHandle('L0_fs')
    L1=sim.getObjectHandle('L1_fs')
    L2=sim.getObjectHandle('L2_fs')

    -- Foot end effector handle
    for i = 1, 3, 1 do
        -- FL_joint[i] = sim.getObjectHandle(jointLeft_name[i])
        -- ML_joint[i] = sim.getObjectHandle(jointLeft_name[i+3])
        -- HL_joint[i] = sim.getObjectHandle(jointLeft_name[i+6])
        leftFoot[i] = sim.getObjectHandle(footLeft_name[i])

        -- FR_joint[i] = sim.getObjectHandle(jointRight_name[i])
        -- MR_joint[i] = sim.getObjectHandle(jointRight_name[i+3])
        -- HR_joint[i] = sim.getObjectHandle(jointRight_name[i+6])
        rightFoot[i] = sim.getObjectHandle(footRight_name[i])
    end


    -- IMU handle and other part handles
    -- cubtest = sim.getObjectHandle('Cuboid')
    IMU = sim.getObjectHandle('IMU')

    -- New IMU
    cube = sim.getObjectHandle('imu_1')
    yaw = 0
    pitch = 0
    roll = 0
    
    alpha = 0
    beta = 0
    gamma = 0



    -- Camera tracking
    first = 0
    camera_bodyTrack_3D = sim.getObjectHandle('Camera_3D')
    camera_bodyTrack_top = sim.getObjectHandle('Camera_top')

    cam3D_init_pos = {0,0,0}
    camTop_init_pos = {0,0,0}



    -- FlyingBox
    flyingBox = sim.getObjectHandle('flying_box')


    -- Gyroscope
    gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    inclineX = 0
    inclineY = 0
    inclineZ = 0

    --  Accelerometer
    accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    accelX = 0
    accelY = 0
    accelZ = 0

    angularVariations = {0,0,0}
    dt = sim.getSimulationTimeStep()






    --*******************************************
    --*                                         *
    --*          setting mass of limb           *
    --*                                         *
    --*******************************************

    if (customMass)  then
        AllMass = 0
        massScale_leg = 1
        -- ball jont mass
        -- for i = 1, table.getn(balljointHandles), 1 do 
        --     local mass = balljointMass/1000
        --     setMass(balljointHandles[i], mass)
        -- end

        -- leg part mass
        for i = 1, table.getn(legpartHandles)/2, 1 do
            -- local mass = (legpartMass[i]/1000)*massScale_leg
            local mass = legpartMass[i] * massScale_leg
            setMass(legpartHandles[i], mass)

            -- local mass = (legpartMass[i]/1000)*massScale_leg
            local mass = legpartMass[i] * massScale_leg
            setMass(legpartHandles[i+9], mass)

            AllMass = AllMass + (mass*2)
        end



        -- body mass 
        for i = 1, table.getn(bodypartHandles), 1 do  -- 1
            -- local mass = (bodypartMass[i]/1000)*massScale*500
            local mass = bodypartMass[i] * massScale_leg
            setMass(bodypartHandles[i], mass)
            AllMass = AllMass + mass
        end
        
        print('JoeMass -- ' .. AllMass)
        print('gian_print')
        --sim.resetDynamicObject()
   end


    -- setup joint limits
    for i = 1, 18, 1 do
        sim.setJointForce(jointHandles[i], maxTorque)
    end


    -- get graph handle
    graph_cr = sim.getObjectHandle('graph_cr')
    graph_tr = sim.getObjectHandle('graph_tr')

    graph_fs = sim.getObjectHandle('graph_fs')
    graph_fs_avg = sim.getObjectHandle('graph_fs_avg')
    graph_cpg = sim.getObjectHandle('graph_cpg')
    graph_fwdModel = sim.getObjectHandle('fwdModel')
    graph_sf = sim.getObjectHandle('graph_sf')
    graph_err = sim.getObjectHandle('graph_err')
    graph_gait = sim.getObjectHandle('graph_gait')
    graph_acc = sim.getObjectHandle('graph_acc')
    graph_ang = sim.getObjectHandle('graph_ang')
    graph_speed = sim.getObjectHandle('graph_speed')
    graph_dif = sim.getObjectHandle('Graph_phaseDif')
    graph_force = sim.getObjectHandle('graph_force')

    dist_ = sim.addGraphStream(graph_dif,'distance', 'unit', 0, {1,0,0})
    


    -- ************************-- 


    
    -- options setting
    -- 0 - show all
    -- 1 - stream is not show
    -- 2 - label is not show
    -- 4 - scatter plot
    options = 2
    dim = 3
    lineWidth = 2
    counting = 0.2

    
    -- end effector plot
    graphTraj_foot0 = sim.getObjectHandle('trajFoot0')
    graphTraj_foot1 = sim.getObjectHandle('trajFoot1')
    graphTraj_foot2 = sim.getObjectHandle('trajFoot2')

    foot0_x = sim.addGraphStream( graphTraj_foot0, 'x','m',options, {1,0,0} )
    foot0_y = sim.addGraphStream( graphTraj_foot0, 'y','m',options, {1,0,0} )
    foot0_z = sim.addGraphStream( graphTraj_foot0, 'z','m',1 , {1,0,0} )

    foot1_x = sim.addGraphStream( graphTraj_foot1, 'x','m',options, {1,0,0} )
    foot1_y = sim.addGraphStream( graphTraj_foot1, 'y','m',options, {1,0,0} )
    foot1_z = sim.addGraphStream( graphTraj_foot1, 'z','m',1 , {1,0,0} )

    foot2_x = sim.addGraphStream( graphTraj_foot2, 'x','m',options, {1,0,0} )
    foot2_y = sim.addGraphStream( graphTraj_foot2, 'y','m',options, {1,0,0} )
    foot2_z = sim.addGraphStream( graphTraj_foot2, 'z','m',1 , {1,0,0} )

    sim.addGraphCurve( graphTraj_foot0, 'graph name', dim, {foot0_x, foot0_y, foot0_z}, {0,0,0}, 'm', options, {0.2,0.2,0.2}, lineWidth )
    sim.addGraphCurve( graphTraj_foot1, 'graph name', dim, {foot1_x, foot1_y, foot1_z}, {0,0,0}, 'm', options, {0,1,0}, lineWidth )
    sim.addGraphCurve( graphTraj_foot2, 'graph name', dim, {foot2_x, foot2_y, foot2_z}, {0,0,0}, 'm', options, {1,0,0}, lineWidth )


    -- graph force
    forceSensor1 = sim.addGraphStream( graph_force, 'leg 1', 'N', options, {1,0,0} )
    forceSensor2 = sim.addGraphStream( graph_force, 'leg 2', 'N', options, {0,1,0} )
    forceSensor3 = sim.addGraphStream( graph_force, 'leg 3', 'N', options, {0,0,1} )
    -- forceSensor4 = sim.addGraphStream( graph_force, 'leg 4', 'N', options, {1,0,0} )
    -- forceSensor5 = sim.addGraphStream( graph_force, 'leg 5', 'N', options, {1,0,0} )
    -- forceSensor6 = sim.addGraphStream( graph_force, 'leg 6', 'N', options, {1,0,0} )


    


    -- setup data logger 
    dataBaseKeys = {}

    dataBaseKeys[1] = 'time'
    dataBaseKeys[2] = 'd_time'
    dataBaseKeys[3] = 'world_time'
    dataBaseKeys[4] = 'velocity'
    dataBaseKeys[5] = 'MI'
    dataBaseKeys[6] = 'oriDiff'
    dataBaseKeys[7] = 'body_x'
    dataBaseKeys[8] = 'body_y'
    dataBaseKeys[9] = 'IMU_roll'
    dataBaseKeys[10] = 'IMU_pitch'
    dataBaseKeys[11] = 'IMU_yaw'
    -- dataBaseKeys[12] = 'main_data'


    
    s = 11

    nParameters = 34
    for i=1, 6, 1 do 

        dataBaseKeys[s+1 + nParameters*(i-1)] =  "predict_" .. i    
        dataBaseKeys[s+2 + nParameters*(i-1)] =  "error_" .. i      
        dataBaseKeys[s+3 + nParameters*(i-1)] =  "sensor_" .. i     
        dataBaseKeys[s+4 + nParameters*(i-1)] =  "weight_" .. i     
        dataBaseKeys[s+5 + nParameters*(i-1)] =  "g_" .. i          
        dataBaseKeys[s+6 + nParameters*(i-1)] =  "sensor_raw_" .. i 
        dataBaseKeys[s+7 + nParameters*(i-1)] =  "rbfn_z_" .. i     
        dataBaseKeys[s+8 + nParameters*(i-1)] =  "rbfn_z_lp_" .. i 

        dataBaseKeys[s+9  + nParameters*(i-1)] = "cpg0_" .. i
        dataBaseKeys[s+10 + nParameters*(i-1)] = "cpg1_" .. i
        dataBaseKeys[s+11 + nParameters*(i-1)] = "tr_target_" .. i
        dataBaseKeys[s+12 + nParameters*(i-1)] = "cr_target_" .. i
        dataBaseKeys[s+13 + nParameters*(i-1)] = "fr_target_" .. i
        dataBaseKeys[s+14 + nParameters*(i-1)] = "tr_" .. i
        dataBaseKeys[s+15 + nParameters*(i-1)] = "cr_" .. i
        dataBaseKeys[s+16 + nParameters*(i-1)] = "fr_" .. i

        dataBaseKeys[s+17 + nParameters*(i-1)] = "sub_data_" .. i                     
        dataBaseKeys[s+18 + nParameters*(i-1)] = "raw_force_" .. i              
        dataBaseKeys[s+19 + nParameters*(i-1)] = "fitered_force_" .. i
        dataBaseKeys[s+20 + nParameters*(i-1)] = "Beta_filter_force_" .. i
        dataBaseKeys[s+21 + nParameters*(i-1)] = "Beta_filter_roll_" .. i
        dataBaseKeys[s+22 + nParameters*(i-1)] = "E_angle_" .. i
        dataBaseKeys[s+23 + nParameters*(i-1)] = "Beta_raw_offset_" .. i 
        dataBaseKeys[s+24 + nParameters*(i-1)] = "Beta_prev_offset_TCjoint_" .. i
        dataBaseKeys[s+25 + nParameters*(i-1)] = "Beta_offset_" .. i 
        dataBaseKeys[s+26 + nParameters*(i-1)] = "x_toe_" .. i
        dataBaseKeys[s+27 + nParameters*(i-1)] = "y_toe_" .. i
        dataBaseKeys[s+28 + nParameters*(i-1)] = "z_toe_" .. i


        -- ****** --
        dataBaseKeys[s+29 + nParameters*(i-1)] = "torq_tr_" .. i
        dataBaseKeys[s+30 + nParameters*(i-1)] = "torq_cr_" .. i
        dataBaseKeys[s+31 + nParameters*(i-1)] = "torq_fr_" .. i
        dataBaseKeys[s+32 + nParameters*(i-1)] = "vel_tr_" .. i
        dataBaseKeys[s+33   + nParameters*(i-1)] = "vel_cr_" .. i
        dataBaseKeys[s+34   + nParameters*(i-1)] = "vel_fr_" .. i


    end

    count_dict = 0
    dict = {}
    for _,key in ipairs(dataBaseKeys) do
        dict[key] = {}
        count_dict = count_dict + 1
    end
    
 

    initDatabase(dataBaseKeys)

    -- CREATE PUBLISH/SUBSCRIBER
    jointTargetPos = simROS.subscribe('target/joint_pos', 'std_msgs/Float64MultiArray', 'setJointTargetPositions')
    cpgValuesSub = simROS.subscribe('target/cpg', 'std_msgs/Float64MultiArray', 'setCpgValues')
    fwdModelSub = simROS.subscribe('topic/forwardModel', 'std_msgs/Float64MultiArray', 'plotFwdModel')
    sfWeightSub = simROS.subscribe('topic/sfweights', 'std_msgs/Float64MultiArray', 'plotSfWeights')

    
    jointCurrentPos = simROS.advertise('sensor/joint_pos', 'std_msgs/Float64MultiArray')
    forceSensor_x = simROS.advertise('sensor/force_sensor_x', 'std_msgs/Float64MultiArray')
    forceSensor_y = simROS.advertise('sensor/force_sensor_y', 'std_msgs/Float64MultiArray')
    forceSensor_z = simROS.advertise('sensor/force_sensor_z', 'std_msgs/Float64MultiArray')
    forceSensor = simROS.advertise('sensor/force_sensor', 'std_msgs/Float64MultiArray')
    simState_pub = simROS.advertise('simulation_state','std_msgs/Float32')

    -- ************* Add IMU publisher ************ --
    imuSensor_pub = simROS.advertise('sensor/imu','std_msgs/Float64MultiArray')


    



    sim_time = simROS.advertise('simulation_time', 'std_msgs/Float64')

    local rosnode = {'stick_insect_sim_controller'}

    for i = 1,table.getn(rosnode),1 do
        --result=sim.launchExecutable(sim.getStringParameter(sim.stringparam_scene_path) .. '/../../../projects/medaextra/catkin_ws/src/'..rosnode[i]..'/bin/'..rosnode[i],'/cpg_topic',0)
    	result=sim.launchExecutable( sim.getStringParameter(sim.stringparam_scene_path) .. '/../../../devel/lib/stick_insect_sim_pkg/'..rosnode[i], '/cpg_topic', 0)
    end

    if (result==false) then
        sim.displayDialog('Error','External ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    -- getTotalMass()

    -- Stability measurement global param
    prevFVel = {0, 0, 0}
    prevPos = {0, 0, 0}

    --/////////////////////////////////////////--

    

end -- end of function sysCall_init()

--**********************************************************************--
--                                                                      --
--**********************************************************************--

--*******************************************
--*                                         *
--*          setting mass of object         *
--*                                         *
--*******************************************
function setMass(h, m)
    sim.setObjectFloatParameter(h,sim.shapefloatparam_mass,m)
end


--*******************************************
--*                                         *
--*          Returns mass of object         *
--*                                         *
--*******************************************
function getMass(handle)
    local mass, inertia, com = simGetShapeMassAndInertia(handle)
    return mass 
end


--*******************************************
--*                                         *
--*     Prints total mass of robot          *
--*                                         *
--*******************************************
-- function getTotalMass()
--     local mj = 0.0
--     local ml = 0.0
--     local mb = 0.0

--     for i = 1, table.getn(balljointHandles),1 do
--         mj = mj + getMass(balljointHandles[i])
--     end 

--     for i = 1, table.getn(legpartHandles),1 do
--         ml = ml + getMass(legpartHandles[i])
--     end 

--     for i = 1, table.getn(bodypartHandles),1 do
--         mb = mb + getMass(bodypartHandles[i])
--     end 

--     print('Joint mass: ' .. mj*1000 ..  ' [g]')
--     print('Leg mass: ' .. ml*1000 .. ' [g]')
--     print('Body mass: ' .. mb*1000 .. ' [g]')

-- end 



--*******************************************
--*                                         *
--*       Plots cpg values on graph         *
--*                                         *
--*******************************************
function setCpgValues(msg)
    local data = msg.data
    -- set graph for cpp

    cpgValues = data

    sim.setGraphUserData(graph_cpg, "CPG0", data[1])
    sim.setGraphUserData(graph_cpg, "CPG1", data[2])

    -- print(cpgValues[12])

end


--*******************************************
--*                                         *
--*     Plots forward model to graph        *
--*                                         *
--*******************************************
function plotFwdModel(msg)
    local data = msg.data
    fwdModelData = data 
    -- set graph for cpg
    local index = 6; 
    local dataPoints = 8; 
    sim.setGraphUserData(graph_fwdModel, "predict", data[1 + dataPoints*(index - 1)])
    sim.setGraphUserData(graph_fwdModel, "error",   data[2 + dataPoints*(index - 1)])
    sim.setGraphUserData(graph_fwdModel, "sensor",  data[3 + dataPoints*(index - 1)])
    sim.setGraphUserData(graph_fwdModel, "weight",  data[4 + dataPoints*(index - 1)])
    sim.setGraphUserData(graph_fwdModel, "g",       data[5 + dataPoints*(index - 1)])
    sim.setGraphUserData(graph_fwdModel, "rbfnz",   data[7 + dataPoints*(index - 1)])
     
    sim.setGraphUserData(graph_fwdModel, "cpg0",    cpgValues[1 + 2*(index - 1)])
    sim.setGraphUserData(graph_fwdModel, "cpg1",    cpgValues[2 + 2*(index - 1)])
    --sim.setGraphUserData(graph_fwdModel, "fs",      forceData[index])

    -- Setting the values for the error plot (Forward moodel - acutual force) 
    for i = 1, 6, 1 do 
        sim.setGraphUserData(graph_err, "err" .. i, data[2 + (6*(i-1))])
    end


    for i = 1, 6, 1 do 
        sim.setGraphUserData(graph_gait, "fs" .. i,  data[5 + (6*(i-1))])
    end    

    for i = 1, 3, 1 do 
        sim.setGraphUserData(graph_tr, "g" .. i , data[5 +  (6*(i-1))])
    end



end


--*******************************************
--*                                         *
--*     Plots the feedback strength         *
--*                                         *
--*******************************************

function plotSfWeights(msg)
    local data = msg.data
    -- set graph for cpg
    
    sim.setGraphUserData(graph_sf, "sf1", data[1])
    sim.setGraphUserData(graph_sf, "sf2", data[2])
    sim.setGraphUserData(graph_sf, "sf3", data[3])
    sim.setGraphUserData(graph_sf, "sf4", data[4])
    sim.setGraphUserData(graph_sf, "sf5", data[5])
    sim.setGraphUserData(graph_sf, "sf6", data[6])

end


--*******************************************
--*                                         *+0.0000e+00
--*    Plots the gait using force sensors   *
--*                                         *
--*******************************************

function plotGait()
    for i = 1, 6, 1 do
        local force 
        if(forceData[i] > 0.1) then 
            force = 1 
        else 
            force = 0
        end 

        --sim.setGraphUserData(graph_gait, "fs" .. i, force)
    end 
end 


--*******************************************
--*                                         *
--*   Sets the joint angle of the motors    *
--*                                         *
--*******************************************

function setJointTargetPositions(msg)
    target = msg.data
    if table.getn(target) == 18 then
        for i = 1, 18, 1 do
            sim.setJointTargetPosition(jointHandles[i], target[i])
            -- print('target[' .. i .. '] ' .. target[i])
        end

        --*********** TEST ************************************************************** << Testtttt--
        leg_RF = {1,7,13}
        leg_RM = {2,8,14}
        leg_RH = {3,9,15}

        leg_LF = {4,10,16}
        leg_LM = {5,11,17}
        leg_LH = {6,12,18}

        lift_up = {0,1.5,-0.5} -- {0,1.3,-0.5}

        normal_front = {0.5 , 0.15, -1.0}
        normal_middle = {0, 0, -1.0}
        normal_rear = {-0.65, 0.15, -1.0}

        offset = {-0.5, 0.0, 0.0}
        zero = 0
        

        -- Lifting up
        for j = 1, 3, 1 do
            -- sim.setJointTargetPosition(jointHandles[ leg_RF[j] ], lift_up[j])
            -- sim.setJointTargetPosition(jointHandles[ leg_RM[j] ], lift_up[j])
            -- sim.setJointTargetPosition(jointHandles[ leg_RH[j] ], lift_up[j])

            -- sim.setJointTargetPosition(jointHandles[ leg_LF[j] ], lift_up[j])
            -- sim.setJointTargetPosition(jointHandles[ leg_LM[j] ], lift_up[j])
            -- sim.setJointTargetPosition(jointHandles[ leg_LH[j] ], lift_up[j])
        end

        -- Normal position
        if normal_pose then
            for j = 1, 3, 1 do
                -- Front Leg
                sim.setJointTargetPosition(jointHandles[ leg_RF[j] ], normal_front[j])
                sim.setJointTargetPosition(jointHandles[ leg_LF[j] ], normal_front[j])

                -- Middle Leg
                sim.setJointTargetPosition(jointHandles[ leg_RM[j] ], normal_middle[j])
                sim.setJointTargetPosition(jointHandles[ leg_LM[j] ], normal_middle[j])

                -- Rear Leg
                sim.setJointTargetPosition(jointHandles[ leg_RH[j] ], normal_rear[j])
                sim.setJointTargetPosition(jointHandles[ leg_LH[j] ], normal_rear[j])
            end
        end


        --  (4)                   __                 (1) 
        --     \                (  )                /
        --      \               |  |               /
        --       \              |  |              /
        --        \[16]__[10](4)|  |(1)[7]___[13]/ 
        --                      |  |
        --                      |  |
        -- (5)____[17]___[11](5)|  |(2)[8]___[14]_____(2)
        --                      |  |
        --                      |  |
        --        [18]___[12](6)|  |(3)[9]___[15]
        --       /              |  |             \
        --      /               |  |              \
        --     /                |  |               \    
        --    /                 |  |                \
        -- (6)                  |  |                 (3)
        --                      |  |
        --                      |  |
        --                      |  |
        --                      |__|      
        

        frtLeg_IMUbias = 0
        midLeg_IMUbias = 0
        RerLeg_IMUbias = 0
        -- IMU compensate 
        

        
        theta = 0 -- degree
        theta_rad = theta/180*math.pi
        target_angle = 0.0
        current_angle = pitch
        E = current_angle - target_angle

        -- *************************** Algo Delta rule **************************** --
        

        -- if is_deltaRule == true then

        --     delta_mu = 0.02
        --     delta_x = current_angle
        --     delta_dw = delta_mu * delta_x * E
        --     delta_w = delta_w + delta_dw

        --     delta_y = delta_w * delta_x
            
        --     if (math.abs(E) > theta_rad and adapted == false) then

        --         adapted = true
                
        --         if roll < 4*math.pi/180 then
        --             ML_obset = ML_obset + delta_y
        --         end

        --         if roll > -4*math.pi/180 then
        --             MR_obset = MR_obset + delta_y
        --         end

        --     else
        --         adapted = false
        --     end

        -- end -- If Delta rule





        -- ******************************************************************** --

        -- set limitation obset
        limit_angle = 90/180*math.pi -- 75
        if math.abs(ML_obset) > limit_angle then
            ML_obset = math.abs(ML_obset)/ML_obset * limit_angle
        end

        if math.abs(MR_obset) > limit_angle then
            MR_obset = math.abs(MR_obset)/MR_obset * limit_angle
        end

        -- print("pitch = " .. pitch*180/math.pi .. " roll = " .. roll*180/math.pi .. "|| obset =  " .. ML_obset .. " < -- > " .. MR_obset)
        -- robot_distance = robot_distance + velocity*0.01
        -- print("distance = " .. robot_distance)
        
        -- ************************************************************************ --
        -- Body-coxa joint
        -- Front leg
        -- sim.setJointTargetPosition(jointHandles[1], target[1]+frtLeg_IMUbias)  -- Right Front leg
        -- sim.setJointTargetPosition(jointHandles[4], target[4]+frtLeg_IMUbias)  -- Left Front leg
        -- sim.setJointTargetPosition(jointHandles[1], normal_front[1]+frtLeg_IMUbias)  -- Right Front leg
        -- sim.setJointTargetPosition(jointHandles[4], normal_front[1]+frtLeg_IMUbias)  -- Left Front leg

        -- Middle leg
        -- angle_bending = 0
        -- MR_obset = angle_bending/180*math.pi
        -- ML_obset = angle_bending/180*math.pi
        -- sim.setJointTargetPosition(jointHandles[2], target[2]+MR_obset)  -- Right mid leg
        -- sim.setJointTargetPosition(jointHandles[5], target[5]+ML_obset)  -- Left mid leg
        -- sim.setJointTargetPosition(jointHandles[2], normal_middle[1] + midLeg_IMUbias)  -- Right mid leg
        -- sim.setJointTargetPosition(jointHandles[5], normal_middle[1] + midLeg_IMUbias)  -- Left mid leg

        -- Rear leg
        -- sim.setJointTargetPosition(jointHandles[3], target[3]+RerLeg_IMUbias)  -- Right Rear leg
        -- sim.setJointTargetPosition(jointHandles[6], target[6]+RerLeg_IMUbias)  -- Left Rear leg
        -- sim.setJointTargetPosition(jointHandles[3], normal_rear[1]+RerLeg_IMUbias)  -- Right Rear leg
        -- sim.setJointTargetPosition(jointHandles[6], normal_rear[1]+RerLeg_IMUbias)  -- Left Rear leg

        
    -- ********************** END TEST ************************ --









    else 
        print("ERROR IN DATA SIZE!")
    end
    
end --function setJointTargetPositions



--*******************************************
--*                                         *
--* Published the state of the simulation   *
--*                                         *
--*******************************************



function publishSimState()
    local simulationState = sim.getSimulationState()
    if simulationState ~= lastSimState then                     -- only publish simulation state if it has changed
        simROS.publish(simState_pub,{data=simulationState})
        lastSimState = simulationState
    end
end


function sysCall_sensing()

    -- Set camera_track to tracking robot body
    if first == 0 then
        first = 1
        cam3D_init_pos = sim.getObjectPosition(camera_bodyTrack_3D, -1)
        camTop_init_pos = sim.getObjectPosition(camera_bodyTrack_top, -1)
        bodyPos_init = sim.getObjectPosition(bodypartHandles[2], -1)
    end

    



    -- publish sim state 
    publishSimState()

    -- get joint possision for all joints (angle sensor in radian) 
    for i = 1, 18, 1 do
        jointPosition[i] = sim.getJointPosition(jointHandles[i])
        jointTorque[i] = sim.getJointForce(jointHandles[i])
    end

    -- get 3d force feedback for all legs (Newton) < ------------------------**
    R0_bool, R0_fs_vec, R0_torq_vec = sim.readForceSensor(R0)
    R1_bool, R1_fs_vec, R1_torq_vec = sim.readForceSensor(R1)
    R2_bool, R2_fs_vec, R2_torq_vec = sim.readForceSensor(R2)
    L0_bool, L0_fs_vec, L0_torq_vec = sim.readForceSensor(L0)
    L1_bool, L1_fs_vec, L1_torq_vec = sim.readForceSensor(L1)
    L2_bool, L2_fs_vec, L2_torq_vec = sim.readForceSensor(L2)

    -- print(R0_fs_vec)

    -- get simulation time 
    t = sim.getSimulationTime() 
   
    
    -- X-Y-Z FORCE SENSOR ARRAYS
    x_fs_array={R0_fs_vec[1], R1_fs_vec[1], R2_fs_vec[1], L0_fs_vec[1], L1_fs_vec[1], L2_fs_vec[1]}
    y_fs_array={R0_fs_vec[2], R1_fs_vec[2], R2_fs_vec[2], L0_fs_vec[2], L1_fs_vec[2], L2_fs_vec[2]}
    z_fs_array={R0_fs_vec[3], R1_fs_vec[3], R2_fs_vec[3], L0_fs_vec[3], L1_fs_vec[3], L2_fs_vec[3]}

    -- calculate the force used for continious feedback
    if (forceDataType == "avg") then
        for i=1,6,1 do 
            forceData[i] = math.min(((math.abs(x_fs_array[i]) +  math.abs(y_fs_array[i]) +  math.abs(z_fs_array[i]))/3)*forceMul*forceMulMatrix[i], maxForce)
        end
    elseif (forceDataType == "z") then
        for i=1,6,1 do 
            forceData[i] = math.min(math.abs(z_fs_array[i])*forceMul*forceMulMatrix[i],maxForce)
            --forceData[i] = 1.0
        end 
    end

    


    -- Plotting total force 
    

    for i=1,6,1 do 
        if(z_fs_array[i] < 0) then 
            forceData[i] = math.sqrt(x_fs_array[i]*x_fs_array[i] +  y_fs_array[i]*y_fs_array[i]  +  z_fs_array[i]*z_fs_array[i])
        else 
            forceData[i] = 0
        end
    end


    -- set grapf for all 6 force values
    for i=1,6,1 do
        sim.setGraphUserData(graph_fs, legName[i], (forceData[i]))
        sim.setGraphUserData(graph_fs_avg, legName[i], (forceData[i]))
    end

    --plotGait()


    -- ROS pub

    for j = 1,6,1 do
        -- forceData[j] = maxForce
        -- forceData[j] = 10
    end 

    -- Publishers are publishing
    simROS.publish(jointCurrentPos,{data=jointPosition})
    simROS.publish(forceSensor_x, {data=x_fs_array})
    simROS.publish(forceSensor_y, {data=y_fs_array})
    simROS.publish(forceSensor_z, {data=z_fs_array})
    simROS.publish(forceSensor, {data=forceData})
    simROS.publish(sim_time, {data=t}) 
    -- simROS.publish(simState_pub,{data=})

    -- imu
    simROS.publish(imuSensor_pub,{data=imu_deg})
    
    


    -- ********************* New IMU ************************ --
    euler = sim.getObjectOrientation(cube,-1)
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(euler[1], euler[2], euler[3])
    alpha = euler[1]*180/math.pi
    beta = euler[2]*180/math.pi
    gamma = euler[3]*180/math.pi

    imu_deg = {roll, pitch, yaw}
    -- print("alpha = " .. alpha)
    -- ****************************************************** --

    -- Plot phase space of CPG
    counting = counting + 0.0002
    counting = 0.2

    -- print(cpgValues)


    -- ********************************************************************** --
    --                                Ploting                                 --
    -- ********************************************************************** --

    -- |
    -- |                         *  *               
    -- |                     *               
    -- |                 *                  
    -- |              *                         
    -- |           *                      
    -- |        *                           
    -- |     *                             
    -- |   *                                  
    -- |  *                                
    -- |_________________________________________    




    if counting > 0.5 then
        counting = 0.2
    end

    if #cpgValues == 12 then
        distance = math.sqrt( (cpgValues[1]-cpgValues[7])^2  + (cpgValues[2]-cpgValues[8])^2 )
        -- print("phase = " .. distance )
        sim.setGraphStreamValue(graph_dif, dist_, distance)
    end


    -- SET CAMERA tracking
    body_pos = sim.getObjectPosition(bodypartHandles[2], -1)
    sim.setObjectPosition(camera_bodyTrack_3D, -1, {body_pos[1]-0.6556, body_pos[2]-0.4406  , body_pos[3] + 0.4175})
    -- sim.setObjectPosition(camera_bodyTrack_top, -1, {camTop_init_pos[1] + body_pos[1], camTop_init_pos[2] + body_pos[2], camTop_init_pos[3]})
    sim.setObjectPosition(camera_bodyTrack_top, -1, {body_pos[1], body_pos[2], camTop_init_pos[3]})


    -- SET FlyingBox

    sim.setObjectPosition(flyingBox,-1, {0,0,5})


    -- Visualize part
    if is_visualize == true then
        pos_foot0 = sim.getObjectPosition(leftFoot[1], -1)
        pos_foot1 = sim.getObjectPosition(leftFoot[2], -1)
        -- pos_foot2 = sim.getObjectPosition(leftFoot[3], -1)

        -- pos_foot0 = sim.getObjectPosition(rightFoot[1], -1)
        -- pos_foot1 = sim.getObjectPosition(rightFoot[2], -1)
        pos_foot2 = sim.getObjectPosition(rightFoot[2], -1)

        
        -- if strLeg == 'FL' then
        sim.setGraphStreamValue(graphTraj_foot0, foot0_x, pos_foot0[1])
        sim.setGraphStreamValue(graphTraj_foot0, foot0_y, pos_foot0[2])
        sim.setGraphStreamValue(graphTraj_foot0, foot0_z, pos_foot0[3])

        -- elseif strLeg == 'ML' then
        sim.setGraphStreamValue(graphTraj_foot1, foot1_x, pos_foot1[1])
        sim.setGraphStreamValue(graphTraj_foot1, foot1_y, pos_foot1[2])
        sim.setGraphStreamValue(graphTraj_foot1, foot1_z, pos_foot1[3])

        -- elseif strLeg == 'HL' then
        sim.setGraphStreamValue(graphTraj_foot2, foot2_x, pos_foot2[1])
        sim.setGraphStreamValue(graphTraj_foot2, foot2_y, pos_foot2[2])
        sim.setGraphStreamValue(graphTraj_foot2, foot2_z, pos_foot2[3])

        -- end
    end

    
    -- force data plot
    sim.setGraphStreamValue(graph_force, forceSensor1, forceData[4])
    i_leg = 4
    gamma_test = fwdModelData[ 4 + (20*(i_leg - 1)) ]
    error_test = fwdModelData[ 2 + (20*(i_leg - 1)) ]
    predict_test = fwdModelData[ 1 + (20*(i_leg - 1)) ]
    senserRaw_test = fwdModelData[ 6 + (20*(i_leg - 1)) ]
    g_test = fwdModelData[ 5 + (20*(i_leg - 1)) ]

    -- print(gamma_test)

    if gamma_test == nil then
        sim.setGraphStreamValue(graph_force, forceSensor2, 0)
        -- sim.setGraphStreamValue(graph_force, forceSensor3, 0)
    else
        sim.setGraphStreamValue(graph_force, forceSensor2, gamma_test*10)
        sim.setGraphStreamValue(graph_force, forceSensor3, math.abs(g_test - senserRaw_test) )
    end
    -- sim.setGraphStreamValue(graph_force, forceSensor3, forceData[6])




    -- print(string.format("elapsed time: %.2f\n", (os.clock() - init_clock)))
    -- print(string.format("second time: %.2f\n", t.set))
    -- print(os.date("today is %S, in %B"))
    -- print("Milliseconds: " .. socket.gettime()*1000)
    -- print(string.format("time second : %.2f\n", sim.getSystemTime() - init_time))
    --  joetime = simGetSystemTimeInMs(-1)
    --  print(string.format("time second : %.4f\n", (joetime - init_time_ms)/1000))




















    -- ****************************************************** --
    linearVel, angularVel = sim.getObjectVelocity(bodypartHandles[1])
    orientation = sim.getObjectOrientation(bodypartHandles[1], -1)

    for i=1,3,1 do
        orientation[i] = orientation[i] - orientation_init[i]
        orientation_deg[i] = orientation[i]*180/math.pi
    end


    local pos =  sim.getObjectPosition(bodypartHandles[1], -1)
    local speed = math.sqrt(math.pow(pos[1]- prevPos[1], 2) + math.pow(pos[2] - prevPos[2], 2) + math.pow(pos[3] - prevPos[3], 2)) / sim.getSimulationTimeStep()
    local fVel = filterSpeed(linearVel)
    local oriDiff = orientation[2] --  - orientationPrev[2]

    orientationPrev = orientation
    
    -- Calculation angular velocity and acceleration 


    for i=1,3,1 do
        sim.setGraphUserData(graph_ang, 'ang' .. i, (angularVel[i]))
        sim.setGraphUserData(graph_acc, 'acc' .. i, ((fVel[i] - prevFVel[i])/sim.getSimulationTimeStep()))
    end
    
    

    prevFVel = fVel
    prevPos = pos


    -- ***************************** Printing part ********************************* --


    -- print("roll = " .. roll .. "|| pitch = " .. pitch .. " ML obset = " .. ML_obset .. " MR_set = " .. MR_obset)
    -- print( "a = " .. alpha .. "b = " .. beta .. "g = " .. gamma .. " bias = " .. midLeg_IMUbias)
    -- print("x1: " .. inclineX .. " , y1: " .. inclineY .. ", z1: " .. inclineZ)
    -- print("x1: " .. imu_deg[1] .. " , y1: " .. imu_deg[2] .. ", z1: " .. imu_deg[3])
    -- print("x: " .. cubAngle[1]*180/math.pi .. " , y: " .. cubAngle[2]*180/math.pi)
    -- print("x: " .. orientation[1] .. ", y: " .. orientation[2] .. ", z = " .. orientation[3])
    -- print("x: " .. orientation_deg[1] .. ", y: " .. orientation_deg[2] .. ", z = " .. orientation_deg[3])


    -- local velocity = 0 -- *********************
    if orientation[2] > 0 then 
        if(linearVel[1] > 0) then 
            velocity = math.sqrt(linearVel[1]*linearVel[1] + linearVel[2]*linearVel[2])
        else  
            velocity = - math.sqrt(linearVel[1]*linearVel[1] + linearVel[2]*linearVel[2])
        end
    else 
        if(linearVel[1] < 0) then 
            velocity = math.sqrt(linearVel[1]*linearVel[1] + linearVel[2]*linearVel[2])
        else  
            velocity = - math.sqrt(linearVel[1]*linearVel[1] + linearVel[2]*linearVel[2])
        end
    end 

    if(t > 0.1) then
        sim.setGraphUserData(graph_speed, 'speed', velocity)
    end

    -- Calculating performance 

    displacement(30, velocity, oriDiff)



    -- ***************************************************************************** --

    -- .d88888b                                  dP               .8888b oo dP             
    -- 88.                                       88               88        88             
    -- `Y88888b. .d8888b. dP   .dP .d8888b.    d8888P .d8888b.    88aaa  dP 88 .d8888b.    
    --       `8b 88'  `88 88   d8' 88ooood8      88   88'  `88    88     88 88 88ooood8    
    -- d8'   .8P 88.  .88 88 .88'  88.  ...      88   88.  .88    88     88 88 88.  ...    
    --  Y88888P  `88888P8 8888P'   `88888P'      dP   `88888P'    dP     dP dP `88888P'    

    -- ***************************************************************************** --                                          

    

    -- SAVE TO DATABASE 
    

    if(logging and t > 1.0 ) then
        table.insert(dict['time'], t)
        table.insert(dict['d_time'], dt)
        table.insert(dict['world_time'], (simGetSystemTimeInMs(-1) - init_time_ms)/1000)
        table.insert(dict['velocity'], velocity)
        table.insert(dict['MI'], fwdModelData[table.getn(fwdModelData)])
        -- table.insert(dict['MI'], fwdModelData[13 + N_data*(i - 1)])
        table.insert(dict['oriDiff'], oriDiff)
        table.insert(dict['body_x'], pos[1])
        table.insert(dict['body_y'], pos[2])
        table.insert(dict['IMU_roll'], roll)
        table.insert(dict['IMU_pitch'], pitch)
        table.insert(dict['IMU_yaw'], yaw)
        -- table.insert(dict['main_data'], 99)

        -- mN = 8


        -- table.insert(dict['vel_x'], vel_raw[1])
        -- table.insert(dict['vel_y'], vel_raw[2])
        -- table.insert(dict['vel_z'], vel_raw[3])

        -- number of data per leg in fwdModelData

        N_data = 20
        for i=1, 6, 1 do
            
            table.insert(dict["predict_" .. i],     fwdModelData[1 + N_data*(i - 1)])
            table.insert(dict["error_" .. i],       fwdModelData[2 + N_data*(i - 1)])
            table.insert(dict["sensor_" .. i],      fwdModelData[3 + N_data*(i - 1)])
            table.insert(dict["weight_" .. i],      fwdModelData[4 + N_data*(i - 1)])
            table.insert(dict["g_" .. i],           fwdModelData[5 + N_data*(i - 1)])
            table.insert(dict["sensor_raw_" .. i],  fwdModelData[6 + N_data*(i - 1)])
            table.insert(dict["rbfn_z_" .. i],      fwdModelData[7 + N_data*(i - 1)])
            table.insert(dict["rbfn_z_lp_" .. i],   fwdModelData[8 + N_data*(i - 1)])


            table.insert(dict["cpg0_" .. i],    cpgValues[1 + 2*(i - 1)])
            table.insert(dict["cpg1_" .. i],    cpgValues[2 + 2*(i - 1)])
            table.insert(dict["tr_target_" .. i],    target[1 +  (i-1)])
            table.insert(dict["cr_target_" .. i],    target[7 +  (i-1)])
            table.insert(dict["fr_target_" .. i],    target[13 + (i-1)])
            table.insert(dict["tr_" .. i],    sim.getJointPosition(jointHandles[1 +  (i-1)]))
            table.insert(dict["cr_" .. i],    sim.getJointPosition(jointHandles[7 +  (i-1)]))
            table.insert(dict["fr_" .. i],    sim.getJointPosition(jointHandles[13 + (i-1)]))


            table.insert(dict["sub_data_" .. i],                 99)
            table.insert(dict["raw_force_" .. i],                fwdModelData[13 + N_data*(i - 1)])
            table.insert(dict["fitered_force_" .. i],            fwdModelData[14 + N_data*(i - 1)])
            table.insert(dict["Beta_filter_force_" .. i],        fwdModelData[15 + N_data*(i - 1)])
            table.insert(dict["Beta_filter_roll_" .. i],         fwdModelData[16 + N_data*(i - 1)])
            table.insert(dict["E_angle_" .. i],                  fwdModelData[17 + N_data*(i - 1)])
            table.insert(dict["Beta_raw_offset_" .. i ],         fwdModelData[18 + N_data*(i - 1)])
            table.insert(dict["Beta_prev_offset_TCjoint_" .. i], fwdModelData[19 + N_data*(i - 1)])
            table.insert(dict["Beta_offset_" .. i ],             fwdModelData[20 + N_data*(i - 1)])


            table.insert(dict["x_toe_" .. i],  sim.getObjectPosition(footHandles[i], bodypartHandles[1]) [1])
            table.insert(dict["y_toe_" .. i],  sim.getObjectPosition(footHandles[i], bodypartHandles[1]) [2])
            table.insert(dict["z_toe_" .. i],  sim.getObjectPosition(footHandles[i], bodypartHandles[1]) [3])

            -- table.insert(dict["z_pos_" .. i],  sim.getObjectPosition(footHandles[i], -1)[3])

            -- if i <= 3 then
            --     toe_pos = sim.getObjectPosition(rightFoot[i], -1)
            -- else
            --     toe_pos = sim.getObjectPosition(leftFoot[i-3], -1)
            -- end

            -- table.insert(dict["x_toeBody_" .. i],  sim.getObjectPosition(footHandles[i], bodypartHandles[1])[1])
            -- table.insert(dict["y_toeBody_" .. i],  sim.getObjectPosition(footHandles[i], bodypartHandles[1])[2])
            -- table.insert(dict["z_toeBody_" .. i],  sim.getObjectPosition(footHandles[i], bodypartHandles[1])[3])


            if (sim.getJointForce(jointHandles[1 +  (i-1)]) ~= nil) then
                table.insert(dict["torq_tr_" .. i] ,    sim.getJointForce(jointHandles[1 +  (i-1)]))
                table.insert(dict["torq_cr_" .. i] ,    sim.getJointForce(jointHandles[7 +  (i-1)]))
                table.insert(dict["torq_fr_" .. i] ,    sim.getJointForce(jointHandles[13 + (i-1)]))
            else
                table.insert(dict["torq_tr_" .. i] ,    0)
                table.insert(dict["torq_cr_" .. i] ,    0)
                table.insert(dict["torq_fr_" .. i] ,    0)
            end


            table.insert(dict["vel_tr_" .. i] ,     sim.getJointVelocity(jointHandles[1 +  (i-1)]))
            table.insert(dict["vel_cr_" .. i] ,     sim.getJointVelocity(jointHandles[7 +  (i-1)]))
            table.insert(dict["vel_fr_" .. i] ,     sim.getJointVelocity(jointHandles[13 +  (i-1)]))

        end

        
    end -- if 

-- ******************** Time set *********************************************************************************-- 5000
    print("time " .. t)
    if(t > 1000) then 
        sim.stopSimulation()
    end



end


--*******************************************
--*                                         *
--*      Running avg filter of speed        *
--*                                         *
--*******************************************

meanxFilter = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
meanyFilter = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
meanzFilter = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

function filterSpeed(vel)
    local size = 20

    table.insert(meanxFilter, vel[1])
    table.remove(meanxFilter, 1)

    table.insert(meanyFilter, vel[2])
    table.remove(meanyFilter, 1)

    table.insert(meanzFilter, vel[3])
    table.remove(meanzFilter, 1)

    local sumx = 0 
    local sumy = 0 
    local sumz = 0 

    for i=1, size do
        sumx = sumx + meanxFilter[i]
        sumy = sumy + meanyFilter[i]
        sumz = sumz + meanzFilter[i]

    end

    return {sumx/size, sumy/size, sumz/size}

    

end

--*******************************************
--*                                         *
--* Global paramerters for performance test *
--*                                         *
--*******************************************

-- Used to calculate one of the three performance components (Displacement) to evalute the performance

startPos = {}
startOri =  0
distTraveled = 0 

preformanceStarted = false 

newPos = {}
oldPos = {}

maxvel = 0

function displacement(startTime, vel, ori)

    if (startTime < t) then
        if(preformanceStarted == false) then 
            startPos = sim.getObjectPosition(bodypartHandles[1], -1)
            startOri = ori
            oldPos = startPos
            preformanceStarted = true 
        end 

        if(vel > maxvel) then 
            maxvel = vel 
        end 

        --print("Vel: " .. vel, "Max vel: " .. maxvel)

        newPos = sim.getObjectPosition(bodypartHandles[1], -1) 

        --print("Error: " .. math.cos(startOri-ori))
        
        local deltax = math.sqrt(math.pow(newPos[1] - startPos[1], 2) + math.pow(newPos[2] - startPos[2], 2))*math.cos(startOri-ori)--(newPos[1] - startPos[1])      -- Moving forward 
        local deltay =  math.sqrt(math.pow(newPos[1] - startPos[1], 2) + math.pow(newPos[2] - startPos[2], 2))*math.sin(startOri-ori)--(newPos[2] - startPos[2])      -- Moving sideways
        local deltar = math.sqrt(math.pow(deltax, 2) + math.pow(deltay, 2))
        local deltas = math.sqrt(math.pow(newPos[1] - oldPos[1], 2) + math.pow(newPos[2] - oldPos[2], 2))
        distTraveled = deltas + distTraveled

        local x1 = math.abs(deltax)/distTraveled
        local x2 = math.abs(deltax)/deltar
        local x3 = math.abs(deltax)/((t-startTime)*maxvel) 
        disp = x1 * x2 * x3
        --print(x1, x2, x3)
    --    print(disp)


        oldPos = newPos


    end

end




function writeDataBaseToCsvFile(path)
    local t = os.date ("*t")
    local fileName = t.day .. '-' .. t.month .. '-' .. t.hour .. '-' .. t.min .. '-' .. t.sec .. '-' .. 'data.csv'
    local filePath = simGetStringParameter(sim_stringparam_scene_path) .. path

    local dataBaseMaxDepth = table.getn(dict[dataBaseKeys[1]])
    
    -- for i=1, table.getn(dict[dataBaseKeys[1]]),1 do              -- finding element with most elements
    --     local currentDepth = table.getn(dict[i])
    --     if currentDepth > dataBaseMaxDepth then
    --             dataBaseMaxDepth = currentDepth
    --     end
    -- end

    print("Max Data = " )

    -- for i=1, table.getn(dataBase),1 do              -- finding element with most elements
    --         local currentDepth = table.getn(dataBase[i])
    --         if currentDepth > dataBaseMaxDepth then
    --                 dataBaseMaxDepth = currentDepth
    --         end
    -- end

    
    local file = io.open(filePath .. fileName, "w+")      -- open/create  file
    io.output(file)                                 -- set default ouptut file to this file

    for i=1, table.getn(dataBaseKeys),1 do          -- write first line 
            io.write(dataBaseKeys[i] .. ',')
    end

    io.write('\n')                                   -- newline

    for i=1, dataBaseMaxDepth, 1 do
        for j=1, count_dict, 1 do              -- write all data
            local currentDepth = table.getn(dict[dataBaseKeys[j]])
            if currentDepth >= i then                -- make shure to not acces a element outside the array
                io.write(dict[dataBaseKeys[j]][i] .. ',')
            end
        end
        io.write('\n')
    end

    io.write('\n')                                  -- newline

    io.close(file)                                  -- close file
    print('New file saved - ' .. fileName)
end

function initDatabase(keys)
    dataBase = {}
    for i=1, table.getn(keys),1 do
            table.insert(dataBase, {})
    end
end

function saveDataToDatabase(key, data)
    local dataBaseIndex = -1;
    for i=1, table.getn(dataBaseKeys),1 do
            if dataBaseKeys[i] == key then
                    dataBaseIndex = i
            end
    end
    if dataBaseIndex == -1 then
            print('dataCollector::saveDataToDatabase : ERROR! key (' .. key ..') does not exist in database..')
    else
            table.insert(dataBase[dataBaseIndex],data)
    end
end

function sysCall_cleanup()
    simROS.shutdownSubscriber(jointTargetPos)
    simROS.shutdownSubscriber(cpgValuesSub)
    
    if(logging) then
        writeDataBaseToCsvFile('/data/fwdmodel/Beta/FL/selfOrg-Beta/')
    end
    print("$$  Robot distance = " .. robot_distance) 
end


function RBF(x, min, max)
    x_mean = (max+min)/2
    std = (max-min)
    -- print(math.exp(x))
    K = math.exp(  -math.pow((x-x_mean),2) / std  )
    return K
end


-- function digit4(x)
--     n_digit=4
--     retrun math.floor((x*n_digit*10)/(n_digit*10))
-- end

-- function digit3(x)
--     n_digit=3
--     retrun math.floor((x*n_digit*10)/(n_digit*10))
-- end

-- function digit2(x)
--     n_digit=2
--     retrun math.floor((x*n_digit*10)/(n_digit*10))
-- end