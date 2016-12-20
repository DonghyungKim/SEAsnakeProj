% (For real robot) Run the gait transition motion when user change the gait - by Donghyung Kim, 2016.8.26
%  - Find a smooth trajectory from the the current pose to the initial pose of the next gait
%  - And moves the robot along the trajectory
%  - Note that 
%           SEA_angles: initial pose of the next gait (or newly computed joint angles from the wave equation)
%      SEA_angles_prev: current pose

%% Gait transition motion

% Condition to execute the gait transition motion
ErrTh = (pi/180)*11*(1+VelFactor);

% norm of error
err_norm = norm(SEA_angles - SEA_angles_prev);

if err_norm > ErrTh
    % time length of trajectory T is set propotional to 'err_norm'
    T=0.8*err_norm;
    % SEA_angles_des becomes the goal pose
    SEA_angles_des = SEA_angles;
    % find the coefficients for 5th order polynomial trajectory
    for i=1:numModules 
        coef=[1,    0,     0,     0,      0,       0;
              0,    1,     0,     0,      0,       0;
              0,    0,     2,     0,      0,       0;
              1,    T,     T^2,   T^3,    T^4,     T^5;
              0,    1,     2*T,   3*T^2,  4*T^3,   5*T^4;
              0,    0,     2,     6*T,    12*T^2,  20*T^3];
        coef=coef\[SEA_angles_prev(i); 0; 0; SEA_angles_des(i) ; 0 ; 0];
        a(i)=coef(1);
        b(i)=coef(2);
        c(i)=coef(3);
        d(i)=coef(4);
        e(i)=coef(5);
        f(i)=coef(6);
    end

    % Initial time (sec)
    lastTime_Trans = fbk.time;

    % Current time (sec) --> Start from 0
    t=0;

    % Previous time(sec)
    t_prev = t;

    % run the gait transition motion
    while t <= T+dt
        % Read sensor feedback
        fbk = snake.getNextFeedback(fbk);

        % Update the current time (sec)
        t = fbk.time - lastTime_Trans;

        % Calculate time interval (sec)  -> it does nothing with this example. Only for check to see whether the consistency
        dt = t - t_prev;

        % 5-th order polynomial
        for j=1:numModules
            SEA_angles_prev(j) = a(j) + t*b(j) + t^2*c(j) + t^3*d(j) + t^4*e(j) + t^5*f(j);
        end

        cmd.position = SEA_angles_prev;

        % send the position command
        snake.set(cmd);

        % Save previous time
        t_prev = t;
    end
    
    % Since the transition is occured, change 'bTransHappened' to true
    bTransHappened = true;
    
end