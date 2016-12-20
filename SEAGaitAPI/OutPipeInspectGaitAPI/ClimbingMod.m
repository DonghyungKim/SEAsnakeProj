% This function returns the joint angles of the SEA snake robot for the climbing mode (Rolling Helix)
%- by Donghyung Kim, 2016.8.16
% Ref) W. Zhen, C. Gong, and H. Choset, "Modeling Rolling Gaits of A Snake Robot", ICRA 2015.

function theta = ClimbingMod(snakeData, sm, PoleOnLeft)

    % Parameters (Rolling Helix)
    r = sm.r;
    p = sm.p;
    m = sm.m;
    
	Ot = sm.Ot;

    k_bar = r/(r^2 + p^2);
    tau_bar = p/(r^2 + p^2);
    A = 2*k_bar*sin(tau_bar*m)/tau_bar;
    
    % This is to change the direction of the snake's coil
    if PoleOnLeft
        signDorJoint = -1;      % change the sign of the angle of dorsal joints
    else
        signDorJoint = 1;
    end
    
    % Generate the joint waves (Rolling Helix)
    for i=1:snakeData.num_modules
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