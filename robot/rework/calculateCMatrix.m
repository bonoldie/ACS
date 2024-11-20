function [C] = calculateCMatrix(roboticStructure)
    C_expanded = sym(zeros(roboticStructure.DOF, roboticStructure.DOF, roboticStructure.DOF));
    
    for i=1:roboticStructure.DOF
        for j=1:roboticStructure.DOF
            for k=1:roboticStructure.DOF
                % C_expanded(i, j, k) = diff(B(i,j), q(k)) * q_dot(k) - 0.5 * diff(B(j,k), q(i)) * q_dot(k);
                C_expanded(i,j,k) = 0.5 * (diff(roboticStructure.dynamics.B(i,j), roboticStructure.jointsSymbol(k, 1)) + diff(roboticStructure.dynamics.B(i,k), roboticStructure.jointsSymbol(j, 1)) - diff(roboticStructure.dynamics.B(j,k), roboticStructure.jointsSymbol(i,1))) * roboticStructure.jointsSymbol(k, 2);
            end
        end
    end
    
    C = simplify(sum(C_expanded, 3));

end