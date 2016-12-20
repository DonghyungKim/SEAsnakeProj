% This function returns the joint angles of the SEA snake robot for the pipe crawling mode (Rolling Helix)
%- by Donghyung Kim, 2016.8.16
% Ref) W. Zhen, C. Gong, and H. Choset, "Modeling Rolling Gaits of A Snake Robot", ICRA 2015.

function theta = PipeCrawMod(snakeData, sm, PoleOnLeft, povs, buttons, Ot_offset_ShiftPipe)

    % Parameters (Rolling Helix)
    r = sm.r;
    p = sm.p;
    m = sm.m;
    
	Ot = sm.Ot;

    k_bar = r/(r^2 + p^2);
    tau_bar = p/(r^2 + p^2);
    A = 2*k_bar*sin(tau_bar*m)/tau_bar;
    
    % This is to change the direction of direction of the snake's coil
    if PoleOnLeft
        signDorJoint = -1;      % change the sign of the angle of dorsal joints
    else
        signDorJoint = 1;
    end
    
    % Generate the joint waves (Rolling Helix)
    for i=1:snakeData.num_modules
        
        % [Repulsive force] when user press <- or -> from the D-pad, generate repulsive force to lateral direction
        if (povs == 90)
            if ((i>0)&&(i<=round(snakeData.num_modules/4))) ...
                    || ((i>=round((snakeData.num_modules/2) + 1))&&(i<=round(snakeData.num_modules*(3/4))))
                r = 1.10*sm.r;%1.05*sm.r;
                k_bar = r/(r^2 + p^2);
                tau_bar = p/(r^2 + p^2);
                A = 2*k_bar*sin(tau_bar*m)/tau_bar;
            else
                r = 0.90*sm.r;%0.95*sm.r;
                k_bar = r/(r^2 + p^2);
                tau_bar = p/(r^2 + p^2);
                A = 2*k_bar*sin(tau_bar*m)/tau_bar;
            end         
        elseif (povs == 270)
            if ((i>0)&&(i<=round(snakeData.num_modules/4))) ...
                    || ((i>=round((snakeData.num_modules/2) + 1))&&(i<=round(snakeData.num_modules*(3/4))))
                r = 0.90*sm.r;%0.95*sm.r;
                k_bar = r/(r^2 + p^2);
                tau_bar = p/(r^2 + p^2);
                A = 2*k_bar*sin(tau_bar*m)/tau_bar;
            else
                r = 1.10*sm.r;%1.05*sm.r;
                k_bar = r/(r^2 + p^2);
                tau_bar = p/(r^2 + p^2);
                A = 2*k_bar*sin(tau_bar*m)/tau_bar;
            end     
        end
        
        % [Shifting from pipe to pipe] when user keep pressing '1'
        %  generate the motion to shift the robot from pipe to pipe during crawing
        if buttons(1)
            r_max =  1.96*sm.r;%2.08*sm.r; % 2.07*sm.r;%1.97*sm.r;%1.87*sm.r;%2.02*sm.r;%1.87*sm.r; %1.9*sm.r; % 2.07*sm.r; %2.07*sm.r;
            r = GetRadiusHelixForShiftPipe(i, Ot - Ot_offset_ShiftPipe, 0.9*sm.r, r_max, 6.7*pi, snakeData.num_modules);  % 0.94*sm.r, r_max, 5.7*pi, snakeData.num_modules);  % 0.34*sm.r, r_max, 6.2*pi, snakeData.num_modules); %0.14*sm.r, r_max, 5.7*pi, snakeData.num_modules);% 0.08*sm.r, r_max, 4.7*pi, snakeData.num_modules);  %7.7*pi, snakeData.num_modules);  %4.7*pi, snakeData.num_modules); %6.2*pi, snakeData.num_modules); %7.6*pi, snakeData.num_modules);
            k_bar = r/(r^2 + p^2);
            tau_bar = p/(r^2 + p^2);
            A = 2*k_bar*sin(tau_bar*m)/tau_bar;
            
            
%             % taper the amplitude
%             if i > round(snakeData.num_modules/2)
%                A = A*(1 - 0.6*(i-round(snakeData.num_modules/2)) / round(snakeData.num_modules/2) );
%             end

            
        end

        
        % Wave equations
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