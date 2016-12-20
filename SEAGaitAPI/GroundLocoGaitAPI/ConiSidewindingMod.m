function theta = ConiSidewindingMod(snakeData,sm)
  
    beta_lat = sm.beta_lat;
    beta_dor = sm.beta_dor;
    
    A_lat = sm.A_lat;
    A_dor= sm.A_dor;

    w_lat = sm.w_lat;
    w_dor = sm.w_dor;

    v_lat = sm.v_lat;
    v_dor = sm.v_dor;

    delta = sm.delta;
    
    taper = sm.taper;
    
    Ot = sm.Ot;

    % Generate the joint waves - Conical sidewinding
    for i=1:snakeData.num_modules
        % Gait equation
        if mod(i,2)	% (odd) for dorsal (yaw-axis) joints
            xi_dor = w_dor*Ot + v_dor*i;
            theta(i) = beta_dor + A_dor*sin(xi_dor + delta);
        else        % (even) for lateral (pitch-axis) joints
            xi_lat = w_lat*Ot + v_lat*i;
            theta(i) = beta_lat + A_lat*sin(xi_lat);
        end 
        
        % taper the gait
        theta(i) = theta(i)*(snakeData.num_modules - taper*i)/snakeData.num_modules;
        
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