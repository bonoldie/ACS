 function [torques, B, C_vec, G] = newtonEuler(roboticStructure, he, w0, dw0, ddp0)
    %NEWTONEULER Summary of this function goes here
    %   Detailed explanation goes here

    jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));
    
    z0 = [0;0;1];
    
    w = cell(1,3);
    dw = cell(1,3);
    dw_m = cell(1,3);
    ddp = cell(1,3);
    ddp_c = cell(1,3);

    ddp0 = ddp0 - roboticStructure.T_b_i{min(1, jointsIndex(1)-1)}(1:3, 1:3)' * roboticStructure.g;

    r = cell(1,3);
    r_c = cell(1,3);

    % forward
    for i=1:roboticStructure.DOF

        R = roboticStructure.T{jointsIndex(i)}(1:3, 1:3);
        R_b_i = roboticStructure.T_b_i{jointsIndex(i)}(1:3, 1:3);
        
        r{i} = R' * roboticStructure.T{jointsIndex(i)}(1:3, 4);
        r_c{i} = R_b_i' * (roboticStructure.CoM{jointsIndex(i)} - roboticStructure.T_b_i{jointsIndex(i)}(1:3,4));

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
            dw{i} = dw{i} + R' * ((roboticStructure.jointsSymbol(i, 3) * z0) + cross(roboticStructure.jointsSymbol(i, 2)*w_i_minus_one, z0));
        end

        
        ddp{i} = R' * ddp_i_minus_one + cross(dw{i},r{i}) + cross(w{i}, cross(w{i}, r{i}));

        if strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Prismatic')
            ddp{i} = ddp{i} + R' * roboticStructure.jointsSymbol(i, 3) * z0 + cross(2 * roboticStructure.jointsSymbol(i, 3) * w{i}, R*z0);
        end

       
        ddp_c{i} = ddp{i}  + cross(dw{i}, r_c{i}) + cross(w{i}, cross(w{i}, r_c{i}));
        
        dw_m{i} = dw_i_minus_one + ...
            roboticStructure.motors{i}.k *  roboticStructure.jointsSymbol(i, 3) * roboticStructure.motors{i}.rotationAxis + ...
            cross(roboticStructure.motors{i}.k *  roboticStructure.jointsSymbol(i, 2) * w_i_minus_one, roboticStructure.motors{i}.rotationAxis);
    end

    torques = sym(zeros(3,1));

    f = cell(1,3);
    mu = cell(1,3);

    % backword
    for i=roboticStructure.DOF:-1:1

        R_b_i = roboticStructure.T_b_i{jointsIndex(i)}(1:3, 1:3);
        R_plus = roboticStructure.T{jointsIndex(i)+1}(1:3, 1:3);
        R_minus = roboticStructure.T{jointsIndex(i)}(1:3, 1:3);
        
        % r = R' * roboticStructure.T{jointsIndex(i)}(1:3, 4);
        % r_c = R_b_i' * (roboticStructure.CoM{jointsIndex(i)} - roboticStructure.T_b_i{jointsIndex(i)}(1:3,4));

        if i == roboticStructure.DOF
            f_i_plus_one = he(1:3);
            mu_i_plus_one = he(4:6);
        else
            f_i_plus_one = f{i+1};
            mu_i_plus_one = mu{i+1};
        end
        
        f{i} = R_plus * f_i_plus_one + roboticStructure.mass(jointsIndex(i)) * ddp_c{i};
        mu{i} = - cross(f{i}, (r{i} + r_c{i})) +  R_plus * mu_i_plus_one + cross(R_plus * f_i_plus_one, r_c{i}) + roboticStructure.MoI{jointsIndex(i)} * dw{i} + ...
            + cross(w{i}, roboticStructure.MoI{jointsIndex(i)} * w{i});
        
        % TODO: add motor components
        
        if strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Revolute')
            torques(i) = mu{i}'*R_minus'*z0 + roboticStructure.frictions{i}.Fv * roboticStructure.jointsSymbol(i, 2) + roboticStructure.frictions{i}.Fs * sign(roboticStructure.jointsSymbol(i, 2));
        else
            torques(i) = f{i}'*R_minus'*z0 + roboticStructure.frictions{i}.Fv * roboticStructure.jointsSymbol(i, 2) + roboticStructure.frictions{i}.Fs * sign(roboticStructure.jointsSymbol(i, 2));
        end
    end

    % Dynamics matrices from the NE result

    qSyms = roboticStructure.jointsSymbol(:,1);
    dqSyms = roboticStructure.jointsSymbol(:,2);

    G = subs(torques, roboticStructure.jointsSymbol(:), [qSyms; zeros(roboticStructure.DOF * 2, 1)]);
    G = simplify(G);

    C_vec = subs(torques, roboticStructure.jointsSymbol(:), [qSyms;dqSyms;zeros(roboticStructure.DOF, 1)]) - G;
    C_vec = simplify(C_vec);

    B = sym(zeros(roboticStructure.DOF));

    for i=1:roboticStructure.DOF
         B(1:3, i) = subs(torques, roboticStructure.jointsSymbol(:), [qSyms; zeros(roboticStructure.DOF, 1);  double((1:3)'==i)]) - G;
    end
    
    B = simplify(B);
end