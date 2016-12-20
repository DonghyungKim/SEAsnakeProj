% This function returns the joint angles of the SEA snake robot for rotating in place
%- by Donghyung Kim, 2016.9.17
% [Inputs]
% - snakeData: data needed to run the matlab control code on the SEA snake robot using
%               ...\SnakePlotter\matlab_SEA-master\utils\SEAsnakeTools\setupSnakeData.m
% - sm: snake modulation (sm)
% - fbk: SEA snake's snesor feedback. Usually it obtained by 'fbk = snake.getNextFeedback();'

function theta = RotInPlaceMod(snakeData, sm, PoleOnLeft, Ot_RiP, RotInPlaceMotDir_Odd_OPI)

    % Parameters (Rolling Helix)
    r = sm.r;
    p = sm.p;
    m = sm.m;
    
	Ot = sm.Ot;

    k_bar = r/(r^2 + p^2);
    tau_bar = p/(r^2 + p^2);
    A = 2*k_bar*sin(tau_bar*m)/tau_bar;
    
    % Rotating in place gait paremeters
    A_RiP = sm.A_RiP;
    w_RiP = sm.w_RiP;
    v_RiP = sm.v_RiP;

    
    % This is to change the direction of the snake's coil
    if PoleOnLeft
        signDorJoint = -1;      % change the sign of the angle of dorsal joints
    else
        signDorJoint = 1;
    end
    
    % Generate the joint waves
    for i=1:snakeData.num_modules
 
        if (i>=1)&&(i<snakeData.num_modules/2)
                if mod(i,2)	% (odd) for dorsal (yaw-axis) joints
                    theta(i) = signDorJoint*A*cos(Ot + tau_bar*m*i) + RotInPlaceMotDir_Odd_OPI*signDorJoint*A_RiP*sin(w_RiP*Ot_RiP + v_RiP*i);
                else        % (even) for lateral (pitch-axis) joints
                    theta(i) = A*sin(Ot + tau_bar*m*i) + (~RotInPlaceMotDir_Odd_OPI)*signDorJoint*A_RiP*sin(w_RiP*Ot_RiP + v_RiP*i);
                end
        else
                if mod(i,2)	% (odd) for dorsal (yaw-axis) joints
                    theta(i) = signDorJoint*A*cos(Ot + tau_bar*m*i) + (~RotInPlaceMotDir_Odd_OPI)*signDorJoint*A_RiP*sin(w_RiP*Ot_RiP + v_RiP*i);
                else        % (even) for lateral (pitch-axis) joints
                    theta(i) = A*sin(Ot + tau_bar*m*i) + RotInPlaceMotDir_Odd_OPI*signDorJoint*A_RiP*sin(w_RiP*Ot_RiP + v_RiP*i);
                end

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