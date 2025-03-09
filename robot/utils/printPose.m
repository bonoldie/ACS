function delta = printPose(Transform_RT,Transform_FK, printPosition, printOrientation, labels) 
    arguments 
        Transform_RT (4,4) double 
        Transform_FK  (4,4) double 
        printPosition (1,1) = true
        printOrientation (1,1) = true
        labels (1,4) string = ["/", "Robotics toolbox", "Forward kinematics", "Delta"]
    end

    % EE orientation, euler angles in the ZYX axis order
    eeOrientation_RT = rotm2eul(Transform_RT(1:3,1:3));
    eePosition_RT = Transform_RT(1:3, 4);
    
    % EE orientation, euler angles in the ZYX axis order
    eeOrientation_FK = rotm2eul(Transform_FK(1:3,1:3));
    eePosition_FK = Transform_FK(1:3, 4);
    
    delta = {eePosition_RT' - eePosition_FK', eeOrientation_RT - eeOrientation_FK};
    
    outCellArray = {};
    outCellArrayRows = 1;
    
    if printPosition
        outCellArray{outCellArrayRows, 1} = {"EE position"};
        outCellArray{outCellArrayRows, 2} = {eePosition_RT'};
        outCellArray{outCellArrayRows, 3} = {eePosition_FK'}; 
        outCellArray{outCellArrayRows, 4} = delta{1};
    
        outCellArrayRows = outCellArrayRows + 1; 
    end
    
    if printOrientation
        outCellArray{outCellArrayRows, 1} = {"EE orientation (euler angles ZYX)"};
        outCellArray{outCellArrayRows, 2} = {eeOrientation_RT};
        outCellArray{outCellArrayRows, 3} = {eeOrientation_FK}; 
        outCellArray{outCellArrayRows, 4} = delta{2};
        
        outCellArrayRows = outCellArrayRows + 1; 
    end
    
    disp(cell2table(...
        outCellArray,    ...
        "VariableNames", ...
        labels) ...
    );
end

