% This function returns the snake modulation (sm) with the joystick inputs
% and the gain parameters - by Donghyung Kim, 2016.12.16
% [Inputs]
%  - VelFactor: parameters for the speed control obtained at '%% Initialize the joystick and its control parameters'
%  - radi: parameters related to the radius of the helix, such as 
%    -> radi.init: initial or maximum value of radius of the helix (Unit: m)
%    -> radi.min_ClimbPole: minimum value of radius of the helix (Unit: m)
%  - pitch: pitch values of the helix such as
%    -> pitch.init: initial or minimum value of pitch of the helix
%    -> pitch.max: maximum value of pitch of the helix
%  - sm: snake module including (at least) sm_res.m, sm.r, and sm.p
%  - buttons: Current status of buttons from the joystic reading
% [Output]
%  - sm_res: updated value of sm including:
%    -> sm_res.m : module length (Unit: m)   --> constant
%    -> sm_res.Ot: updated internal time
%    -> sm_res.r: updated radius of the helix (Unit: m)
%    -> sm_res.r_ShiftRtP : updated radius of the helix only for this specific mode, 'Shifting from Rod to Pipe' (Unit: m)
%    -> sm_res.p: updated pitch of the helix (Unit: m)

function sm_res = GetSM_ShiftRodToPipeModeWithJoystick(snakeData, fbk, VelFactor,radi, pitch, sm, povs, buttons)

    % inilialize sm_res by sm
    sm_res = sm;
    
    % For changing the radius with buttons 5 and 6
    kappa = 1/sm.r_ShiftRtP;
    
	%% Manual control of radius of helix, r
    % Loosen with button '5'
    if (sm.r <= radi.init)&&(buttons(5))&&(~buttons(6))
        kappa = kappa - 0.020*(1+VelFactor); 
        sm_res.r_ShiftRtP = 1/kappa;
        
    % Tighten with button '6' 
    elseif (sm.r_ShiftRtP >= radi.min_ClimbPole)&&(buttons(6))&&(~buttons(5))
        if(isempty(fbk)) ||  ((~isempty(fbk))&&( sm_res.r_ShiftRtP > radi.init/2)) % -> If feedback is empty, or large radius of helix 
            kappa = kappa + 0.020*(1+VelFactor);                          % -> then just reduce the radius without using the feedback
            sm_res.r_ShiftRtP = 1/kappa;
        else                                        % ->In this case, the feedback exists and the radius of helix is small enough to cover something 
            if (GetSSofTorque(fbk, snakeData.num_modules) < sm.TssDes_CC)       % -> If square sum of torque is smaller than the limit, allow the robot tightens
                kappa = kappa + 0.020*(1+VelFactor);% -> otherwise, prevent over-tightening by ignoring the joystick input
                sm_res.r_ShiftRtP = 1/kappa;
            end
        end
    end

 	%% Manual control of pitch of helix, p
    % Decrease or increase pitch with button '2' or '3'     
    if (sm.p >= pitch.init)&&(buttons(2))           % Decrease p value
        sm_res.p = sm.p - 0.00010*(1+VelFactor); %0.00015*(1+VelFactor);%0.0003*(1+VelFactor);
    elseif (sm.p <= pitch.max)&&(buttons(3))        % Increase p value
        sm_res.p = sm.p + 0.00010*(1+VelFactor); %0.00015*(1+VelFactor);%0.0003*(1+VelFactor);
    end
    
%     %% Compliance control of radius of helix
%     % Apply compliance curvature during the robot climbs up / down the pole
%     if (~isempty(fbk))
%         if ((povs== 0)||(povs == 180))&&(sm_res.r < radi.init/5)
%             kappa = kappa + sm.Pgain_CC*(sm.TssDes_CC - GetSSofTorque(fbk, snakeData.num_modules));
%             sm_res.r = 1/kappa;
%             if sm_res.r <= radi.min_ClimbPole   % prevent the radius of helix becomes smaller than its minimum value
%                 sm_res.r = radi.min_ClimbPole;
%             end
%         end
%     end

    
end