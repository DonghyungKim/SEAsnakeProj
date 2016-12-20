function theta = SideWindMod(snakeData,sm)
  
    beta_lat = sm.beta_lat;
    beta_dor = sm.beta_dor;
    
    A_lat = sm.A_lat;
    A_dor= sm.A_dor;

    w_lat = sm.w_lat;
    w_dor = sm.w_dor;

    v_lat = sm.v_lat;
    v_dor = sm.v_dor;

    delta = sm.delta;
    
    Ot = sm.Ot;

    % Generate the joint waves - Sidewinding
    for i=1:snakeData.num_modules
        % Gait equation
        if mod(i,2)	% (odd) for dorsal (yaw-axis) joints
            xi_dor = w_dor*Ot + v_dor*i;
            theta(i) = beta_dor + A_dor*sin(xi_dor + delta);
        else        % (even) for lateral (pitch-axis) joints
            xi_lat = w_lat*Ot + v_lat*i;
            theta(i) = beta_lat + A_lat*sin(xi_lat);
        end
        
        % Apply joint limit
        if theta(i) < -pi/2 + 0.05
            theta(i) = -pi/2 + 0.05;
        end
        if theta(i) > pi/2 - 0.05
            theta(i) = pi/2 - 0.05;
        end
    end
    
    % Convert the joint angles (head to tail) to the SEA snake's joint angles
    theta = anglesUtoSEA(snakeData, theta);  
    
end