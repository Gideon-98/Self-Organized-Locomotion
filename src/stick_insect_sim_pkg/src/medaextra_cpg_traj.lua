function sysCall_init()
    -- System
    logging = true
    is_visualize = true

    -- do some initialization here
    o1 = 0
    o2 = 0
    a1 = 0
    a2 = 0
    B1 = 0.01
    B2 = 0.01
    o1_cmd = 0
    o2_cmd = 0
    MI = 0.015
    w = {1.4, 1.4, 0.18+MI, -0.18-MI}
    

    FL_offset = {0.52 , 0.174 , -1.047}
    ML_offset = {0.0, 0.3, -2.5}
    HL_offset = {-0.7, 0.37, -2.15}


    jointLeft_name = {'TL0', 'CL0', 'FL0', 'TL1', 'CL1', 'FL1', 'TL2', 'CL2', 'FL2'}
    jointRight_name = {'TR0', 'CR0', 'FR0', 'TR1', 'CR1', 'FR1', 'TR2', 'CR2', 'FR2'}
    
    footLeft_name = {'FOOT_L0', 'FOOT_L1', 'FOOT_L2'}
    footRight_name = {'FOOT_R0', 'FOOT_R1', 'FOOT_R2'}
    FL_joint = {}
    ML_joint = {}
    HL_joint = {}

    FR_joint = {}
    MR_joint = {}
    HR_joint = {}

    leftFoot = {}
    rightFoot = {}


    for i = 1, 3, 1 do
        FL_joint[i] = sim.getObjectHandle(jointLeft_name[i])
        ML_joint[i] = sim.getObjectHandle(jointLeft_name[i+3])
        HL_joint[i] = sim.getObjectHandle(jointLeft_name[i+6])
        leftFoot[i] = sim.getObjectHandle(footLeft_name[i])

        FR_joint[i] = sim.getObjectHandle(jointRight_name[i])
        MR_joint[i] = sim.getObjectHandle(jointRight_name[i+3])
        HR_joint[i] = sim.getObjectHandle(jointRight_name[i+6])
        rightFoot[i] = sim.getObjectHandle(footRight_name[i])
    end
    


    -- options setting
    -- 0 - show all
    -- 1 - stream is not show
    -- 2 - label is not show
    -- 4 - scatter plot
    options = 2
    dim = 3
    lineWidth = 2

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

    sim.addGraphCurve( graphTraj_foot0, 'graph name', dim, {foot0_x, foot0_y, foot0_z}, {0,0,0}, 'm', options, {1,0,0}, lineWidth )
    sim.addGraphCurve( graphTraj_foot1, 'graph name', dim, {foot1_x, foot1_y, foot1_z}, {0,0,0}, 'm', options, {0,1,0}, lineWidth )
    sim.addGraphCurve( graphTraj_foot2, 'graph name', dim, {foot2_x, foot2_y, foot2_z}, {0,0,0}, 'm', options, {0,0,1}, lineWidth )





    graphFL = sim.getObjectHandle('graphFL')
    jointL0 = sim.addGraphStream( graphFL, 'J0', 'rad', options, {1,0,0} )
    jointL1 = sim.addGraphStream( graphFL, 'J1', 'rad', options, {0,1,0} )    joint1 = sim.addGraphStream( graphFL, 'J1', 'rad', options, {0,1,0} )
    jointL2 = sim.addGraphStream( graphFL, 'J2', 'rad', options, {0,0,1} )

    graphFR = sim.getObjectHandle('graphFR')
    jointR0 = sim.addGraphStream( graphFR, 'J0', 'rad', options, {1,0,0} )
    jointR1 = sim.addGraphStream( graphFR, 'J1', 'rad', options, {0,1,0} )    joint1 = sim.addGraphStream( graphFL, 'J1', 'rad', options, {0,1,0} )
    jointR2 = sim.addGraphStream( graphFR, 'J2', 'rad', options, {0,0,1} )


    -- Set database
    i = 0
    dataBaseKeys = {}
     
    dataBaseKeys[i+1] = 'jointFR1'
    dataBaseKeys[i+2] = 'jointFR2'
    dataBaseKeys[i+3] = 'jointFR3'
    dataBaseKeys[i+4] = 'footFRx'
    dataBaseKeys[i+5] = 'footFRz'

    dataBaseKeys[i+6] = 'jointMR1'
    dataBaseKeys[i+7] = 'jointMR2'
    dataBaseKeys[i+8] = 'jointMR3'
    dataBaseKeys[i+9] = 'footMRx'
    dataBaseKeys[i+10] = 'footMRz'

    dataBaseKeys[i+11] = 'jointHR1'
    dataBaseKeys[i+12] = 'jointHR2'
    dataBaseKeys[i+13] = 'jointHR3'
    dataBaseKeys[i+14] = 'footHRx'
    dataBaseKeys[i+15] = 'footHRz'


    count_dict = 0
    dict = {}
    for _,key in ipairs(dataBaseKeys) do
        dict[key] = {}
        count_dict = count_dict + 1
    end
    
    initDatabase(dataBaseKeys)

end


-- ************************************************************************************************************************************* --


function sysCall_actuation()
    o1 = math.tanh(w[1]*o1 + w[3]*o2 + B1) --| - sfweight*force_feedback_all.at(index_force_sensor)*cos(o1));
    o2 = math.tanh(w[2]*o2 + w[4]*o1 + B2) --| - sfweight*force_feedback_all.at(index_force_sensor)*sin(o2));
    -- print(o1 .. o2)

    fact = 0.7
    -- FL0 = (o1+1)*fact
    -- FL1 = (o2+1.25)*fact*0.8
    -- FL2 = o1*fact*0.5 - 1.5

    cmd_FL = {0,0,0}
    -- cmd_FL[1] = o1
    -- cmd_FL[2] = math.max(o2,0)
    -- cmd_FL[3] = math.max( , 0)
    -- cmd_FL = {0,0,0}
    -- print('o1 = ' .. cmd_FL[1] .. ' o2 = ' .. cmd_FL[2])
    for i = 1, 3, 1 do
        sim.getJointPosition(FL_joint[1])
        print()
    end

    -- motorCmd('FL', cmd_FL)

    -- motorCmd('ML', {0,0,0})

    -- motorCmd('HL', {0,0,0})



end



function sysCall_sensing()
    -- put your sensing code here

    FR1 = sim.getJointPosition(FR_joint[1])
    FR2 = sim.getJointPosition(FR_joint[2])
    FR3 = sim.getJointPosition(FR_joint[3])

    MR1 = sim.getJointPosition(MR_joint[1])
    MR2 = sim.getJointPosition(MR_joint[2])
    MR3 = sim.getJointPosition(MR_joint[3])

    HR1 = sim.getJointPosition(HR_joint[1])
    HR2 = sim.getJointPosition(HR_joint[2])
    HR3 = sim.getJointPosition(HR_joint[3])

    footFRx= sim.getObjectPosition(rightFoot[1], -1)[1]
    footFRz= sim.getObjectPosition(rightFoot[1], -1)[3]
    footMRx= sim.getObjectPosition(rightFoot[2], -1)[1]
    footMRz= sim.getObjectPosition(rightFoot[2], -1)[3]
    footHRx= sim.getObjectPosition(rightFoot[3], -1)[1]
    footHRz= sim.getObjectPosition(rightFoot[3], -1)[3]


    
    -- Export data
    table.insert(dict['jointFR1'], FR1)
    table.insert(dict['jointFR2'], FR2)
    table.insert(dict['jointFR3'], FR3)
    table.insert(dict['footFRx'], footFRx)
    table.insert(dict['footFRz'], footFRz)

    table.insert(dict['jointMR1'], MR1)
    table.insert(dict['jointMR2'], MR2)
    table.insert(dict['jointMR3'], MR3)
    table.insert(dict['footMRx'], footMRx)
    table.insert(dict['footMRz'], footMRz)

    table.insert(dict['jointHR1'], HR1)
    table.insert(dict['jointHR2'], HR2)
    table.insert(dict['jointHR3'], HR3)
    table.insert(dict['footHRx'], footHRx)
    table.insert(dict['footHRz'], footHRz)

    

end

function sysCall_cleanup()
    if(logging) then
        writeDataBaseToCsvFile('/data/fwdmodel/ajKoh_data/')
    end
    -- print("$$  Robot distance = " .. robot_distance) 
end


-- **************************************************************************** --
--                              EXTERNAL FUNCTION
-- **************************************************************************** --



function motorCmd(strLeg, listCmd)
    -- FL_offset = {0, 0, 0}
    -- ML_offset = {0, 0, 0}
    -- HL_offset = {0, 0, 0}

    if strLeg == 'FL' then
        sim.setJointTargetPosition(FL_joint[1], listCmd[1] + FL_offset[1])
        sim.setJointTargetPosition(FL_joint[2], listCmd[2] + FL_offset[2])
        sim.setJointTargetPosition(FL_joint[3], listCmd[3] + FL_offset[3])
    elseif strLeg == 'ML' then
        sim.setJointTargetPosition(ML_joint[1], listCmd[1] + ML_offset[1])
        sim.setJointTargetPosition(ML_joint[2], listCmd[2] + ML_offset[2])
        sim.setJointTargetPosition(ML_joint[3], listCmd[3] + ML_offset[3])
    elseif strLeg == 'HL' then 
        sim.setJointTargetPosition(HL_joint[1], listCmd[1] + HL_offset[1])
        sim.setJointTargetPosition(HL_joint[2], listCmd[2] + HL_offset[2])
        sim.setJointTargetPosition(HL_joint[3], listCmd[3] + HL_offset[3])
    end

    -- Visualize part
    if is_visualize == true then
        pos_foot0 = sim.getObjectPosition(leftFoot[1], -1)
        pos_foot1 = sim.getObjectPosition(leftFoot[2], -1)
        pos_foot2 = sim.getObjectPosition(leftFoot[3], -1)
        if strLeg == 'FL' then
            sim.setGraphStreamValue(graphTraj_foot0, foot0_x, pos_foot0[1])
            sim.setGraphStreamValue(graphTraj_foot0, foot0_y, pos_foot0[2])
            sim.setGraphStreamValue(graphTraj_foot0, foot0_z, pos_foot0[3])

        elseif strLeg == 'ML' then
            sim.setGraphStreamValue(graphTraj_foot1, foot1_x, pos_foot1[1])
            sim.setGraphStreamValue(graphTraj_foot1, foot1_y, pos_foot1[2])
            sim.setGraphStreamValue(graphTraj_foot1, foot1_z, pos_foot1[3])

        elseif strLeg == 'HL' then
            sim.setGraphStreamValue(graphTraj_foot2, foot2_x, pos_foot2[1])
            sim.setGraphStreamValue(graphTraj_foot2, foot2_y, pos_foot2[2])
            sim.setGraphStreamValue(graphTraj_foot2, foot2_z, pos_foot2[3])

        end
    end

end -- Motor cmd function




function initDatabase(keys)
    dataBase = {}
    for i=1, table.getn(keys),1 do
            table.insert(dataBase, {})
    end
end



function writeDataBaseToCsvFile(path)
    local t = os.date ("*t")
    local fileName = t.day .. '-' .. t.month .. '-' .. t.hour .. '-' .. t.min .. '-' .. t.sec .. '-' .. 'data.csv'
    local filePath = simGetStringParameter(sim_stringparam_scene_path) .. path

    local dataBaseMaxDepth = table.getn(dict[dataBaseKeys[1]])

    
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
        -- io.write('\n')
    end

    io.write('\n')                                  -- newline

    -- io.close(file)                                  -- close file
    print('New file saved - ' .. fileName)
end

