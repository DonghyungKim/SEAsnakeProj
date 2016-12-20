% This function returns the two joint angles of the head of SEA snake robot 
% to keep head's orientation stationary - by Donghyung Kim, 2016.09.02
% Remark) For the SEA snake robot, the last index number means the head module (not the tail module). 
% Thus this function only affect (num_modules)th and (num_modules - 1)th modules 
% [Inputs]
%  - theta: (1 x num_modules) vector of joint angles
%  - x_head_init: (6 x 1) vector of initial head's pose, which means (x, y, z, roll, pitch, yaw), measured by the gravity frame
%  - x_head: (6 x 1) vector of current head's pose, which means (x, y, z, roll, pitch, yaw), measured by the gravity frame
%  - num_modules: number of SEA modules
% [Output]
%  - SEA_angles: (1 x num_modules) vector of joint angles 

function  SEA_angles = HeadTwoMods_KeepFoward(theta, x_head_init, x_head, num_modules)

	% 
    for i=1:num_modules
        SEA_angles(i) = theta(i);
    end


    % start the robot with the head module's 'model number' side up
    if (x_head_init(4) > 0)  
         %  - when the head module's 'model number' side up
         if x_head(4) > 0 %((x_head(4) > 70*D2R) && (x_head(4) < 110*D2R))
            head_comp_alpha = 1.3*(x_head(4) - x_head_init(4));% 90*D2R);
            head_comp_gamma = 1.2*(x_head(6) - x_head_init(6));

            SEA_angles(num_modules) = head_comp_alpha;
            SEA_angles(num_modules-1) = head_comp_gamma;
        
        %  - when the head module's 'LED only' side up
         elseif x_head(4) < 0  %((x_head(4) > -110*D2R) && (x_head(4) < -70*D2R))
            head_comp_alpha = 1.3*(x_head(4) + x_head_init(4));%90*D2R);
            SEA_angles(num_modules) = head_comp_alpha;

            if (x_head(6) > pi/2) && (x_head(6) < pi)
                head_comp_gamma = 1.2*(pi - x_head(6) + x_head_init(6));
                SEA_angles(num_modules-1) = head_comp_gamma;
            elseif (x_head(6) < -pi/2) && (x_head(6) > -pi)
                head_comp_gamma = 1.2*(pi + x_head(6) - x_head_init(6));
                SEA_angles(num_modules-1) = -head_comp_gamma;
            end
        end   

        
    % start the robot with the head module's 'LED only' side up
    else
        %  - when the head module's 'LED only' side up
        if x_head(4) < 0  %((x_head(4) > -110*D2R) && (x_head(4) < -70*D2R))  % LED만 있는 면 (모델번호 없음)이 위로 온 경우
            head_comp_alpha = 1.3*(x_head(4) - x_head_init(4));%90*D2R);
            head_comp_gamma = 1.2*(x_head(6) - x_head_init(6));

            SEA_angles(16) = head_comp_alpha;
            SEA_angles(15) = -head_comp_gamma;
        
        %  - when the head module's 'model number' side up    
        elseif x_head(4) > 0 %((x_head(4) > 70*D2R) && (x_head(4) < 110*D2R))
            head_comp_alpha = 1.3*(x_head(4) + x_head_init(4));%90*D2R);
            SEA_angles(num_modules) = head_comp_alpha;

            if (x_head(6) > pi/2) && (x_head(6) < pi)
                head_comp_gamma = 1.2*(pi - x_head(6) + x_head_init(6));
                SEA_angles(num_modules-1) = -head_comp_gamma;
            elseif (x_head(6) < -pi/2) && (x_head(6) > -pi)
                head_comp_gamma = 1.2*(pi + x_head(6) - x_head_init(6));
                SEA_angles(num_modules-1) = head_comp_gamma;
            end  
        end
    end
    
    
	% Apply joint limit
    if SEA_angles(i) < -pi/2 + 0.05
        SEA_angles(i) = -pi/2 + 0.05;
    end
    if SEA_angles(i) > pi/2 - 0.05
        SEA_angles(i) = pi/2 - 0.05;
    end
    
end