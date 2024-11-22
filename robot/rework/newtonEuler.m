function [torques] = newtonEuler(roboticStructure, ee, w0, dw0, ddp0)
    %NEWTONEULER Summary of this function goes here
    %   Detailed explanation goes here

    jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));
    
    z0 = [0;0;1];
    
    w = cell(1,3);
    dw = cell(1,3);
    dw_m = cell(1,3);
    ddp = cell(1,3);
    ddp_c = cell(1,3);

    ddp0 = ddp0 - roboticStructure.g;
    
    % forward
    for i=1:roboticStructure.DOF

        R = roboticStructure.T{jointsIndex(i)}(1:3, 1:3);
        
        if i == 1
            w_i_minus_one = w0;
            dw_i_minus_one = dw0;
            ddp_i_minus_one = ddp0;
        else 
            w_i_minus_one = w(i-1);
            dw_i_minus_one = dw(i-1);
            ddp_i_minus_one = ddp(i-1);
        end

        w{i} = R'*w_i_minus_one;

        if strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Revolute')
            w{i} = w{i} + R' * roboticStructure.jointsSymbol(i, 2) * z0;
        end

        dw{i} = R'*dw_i_minus_one;

        if strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Revolute')
            w{i} = w{i} + R' * roboticStructure.jointsSymbol(i, 2) * z0;
        end
        
        r = roboticStructure.T_b_i{jointsIndex(i)}(1:3,4) - roboticStructure.T_b_i{jointsIndex(i)-1}(1:3,4);
        r = roboticStructure.T_b_i{jointsIndex(i)}(1:3, 1:3) * r;

       
        ddp{i} = R'*ddp_i_minus_one + cross(dw{i},r) + cross(w{i}, cross(w{i}, r));

        if strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Prismatic')
            ddp{i} = ddp{i} + R' * roboticStructure.jointsSymbol(i, 3) * z0 + cross(2 * roboticStructure.jointsSymbol(i, 3) * w{i}, R*z0);
        end

        r_c = roboticStructure.CoM{jointsIndex(i)} - roboticStructure.T_b_i{jointsIndex(i)}(1:3,4);
        r_c = roboticStructure.T_b_i{jointsIndex(i)}(1:3, 1:3) * r_c;
            
        ddp_c{i} = ddp{i}  + cross(dw{i}, r_c) + cross(w{i}, cross(w{i}, r_c));
        
        dw_m{i} = dw_i_minus_one + ...
            roboticStructure.motors{i}.k *  roboticStructure.jointsSymbol(i, 3) * roboticStructure.motors{i}.rotationAxis + ...
            cross(roboticStructure.motors{i}.k *  roboticStructure.jointsSymbol(i, 2) * w_i_minus_one, roboticStructure.motors{i}.rotationAxis);
    end

    torques = sym(zeros(3,1));

    % backword
    for i=roboticStructure.DOF:-1:1
        
    end
    
end