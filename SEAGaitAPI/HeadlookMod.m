% This function returns the joint angles for the head look mode

function theta = HeadlookMod(snakeData, SEA_angles, axes, dt, lambda)

        % Convert to the U-snake's joint angles
        theta = anglesSEAtoU(snakeData, SEA_angles);

        % Substitute the current joint angles only for the headlook into th1~th3
        th_Head(1) = theta(4);      % th1 is the 4th joint angle
        th_Head(2) = theta(3);
        th_Head(3) = theta(2);
        th_Head(4) = theta(1);     % th4 is the 1st joint angle, which means, the joint angle of the head module

        % Module length
        l=[snakeData.moduleLen; snakeData.moduleLen; snakeData.moduleLen];

        % Get Jacobian
        J = Jacobi_HeadLook(th_Head, l);

        % Determine the Head's velocity from the joysick input
        x_dot = zeros(5,1);

        x_dot(2) = 0.04*axes(1);    % Y-axis translation
        x_dot(3) = -0.04*axes(2);   % Z-axis translation

        x_dot(5) = 0.30*axes(3);    % Z-axis (Yaw) rotation
        x_dot(4) = -0.30*axes(4);   % Y-axis (Pitch) rotation     


        % DLS Inverse    
        J_inv = (J'*J + (lambda^2).*eye(4,4))\(J');
        
        % Find joint velocities th_dot corresponding to task-space velocities x_dot
        th_dot = J_inv*x_dot;

        % Integrate the joint position
        theta(4) = theta(4) + th_dot(1)*dt;
        theta(3) = theta(3) + th_dot(2)*dt;
        theta(2) = theta(2) + th_dot(3)*dt;
        theta(1) = theta(1) + th_dot(4)*dt;
        
        % Apply joint limit
        for i=1:4
            if theta(i) < -pi/2 + 0.05
                theta(i) = -pi/2 + 0.05;
            end
            if theta(i) > pi/2 - 0.05
                theta(i) = pi/2 - 0.05;
            end
        end

        % Convert to the SEA snake's joint angles
        theta = anglesUtoSEA(snakeData, theta);

end