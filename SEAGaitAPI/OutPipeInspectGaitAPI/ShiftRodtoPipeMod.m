% This function returns the joint angles of the SEA snake robot for the shifting (originated from the rolling Helix)
%- by Donghyung Kim, 2016.12.15
% Ref) W. Zhen, C. Gong, and H. Choset, "Modeling Rolling Gaits of A Snake Robot", ICRA 2015.

function theta = ShiftRodtoPipeMod(snakeData, sm, PoleOnLeft)

    % Parameters
    %  1) Original value of A
    %    -> 여기서 sm.r은 constant, 즉 Shiftinf from Rod to Pipe 모드 시작 할 때
    %    radius 값을 상수로 기억해 놓은 것임.
    r = sm.r;
    p = sm.p;
    m = sm.m;

    k_bar = r/(r^2 + p^2);
    tau_bar = p/(r^2 + p^2);
    A = 2*k_bar*sin(tau_bar*m)/tau_bar;
    
    % 2) Modified value of A
    r_ShiftRtP = sm.r_ShiftRtP;
    p = sm.p;
    m = sm.m;

    k_bar = r_ShiftRtP/(r_ShiftRtP^2 + p^2);
    tau_bar = p/(r_ShiftRtP^2 + p^2);
    A_ShiftRtP = 2*k_bar*sin(tau_bar*m)/tau_bar;    
    
    
    
    
    
 	Ot = sm.Ot;   
    
    % This is to change the direction of the snake's coil
    if PoleOnLeft
        signDorJoint = -1;      % change the sign of the angle of dorsal joints
    else
        signDorJoint = 1;
    end
    
    % Generate the joint waves (Rolling Helix)
    for i=1:(snakeData.num_modules / 2)
        if mod(i,2)	% (odd) for dorsal (yaw-axis) joints
            theta(i) = signDorJoint*A_ShiftRtP*cos(Ot + tau_bar*m*i);
        else        % (even) for lateral (pitch-axis) joints
            theta(i) = A_ShiftRtP*sin(Ot + tau_bar*m*i);
        end        
        % Apply joint limit
        if theta(i) < -pi/2 + 0.02
            theta(i) = -pi/2 + 0.02;
        end
        if theta(i) > pi/2 - 0.02
            theta(i) = pi/2 - 0.02;
        end
    end
    
    for i=((snakeData.num_modules / 2)+1):snakeData.num_modules
        if mod(i,2)	% (odd) for dorsal (yaw-axis) joints
            theta(i) = signDorJoint*A*cos(Ot + tau_bar*m*i);
        else        % (even) for lateral (pitch-axis) joints
            theta(i) = A*sin(Ot + tau_bar*m*i);
        end        
        % Apply joint limit
        if theta(i) < -pi/2 + 0.02
            theta(i) = -pi/2 + 0.02;
        end
        if theta(i) > pi/2 - 0.02
            theta(i) = pi/2 - 0.02;
        end
    end
    

    % Convert the joint angles (head to tail) to the SEA snake's joint angles
    theta = anglesUtoSEA(snakeData, theta);  

end