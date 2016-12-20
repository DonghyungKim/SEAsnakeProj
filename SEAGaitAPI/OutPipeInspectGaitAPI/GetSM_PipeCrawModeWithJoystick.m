% This function returns the snake modulation (sm) with the joystick inputs
% and the gain parameters - by Donghyung Kim, 2016.08.25
% [Inputs]
%  - VelFactor: velocity gain factor
%  - radi: parameters related to the radius of the helix, such as 
%    -> radi.init: initial or minimum value of radius of the helix (Unit: m)
%    -> radi.des_Roll_between_Pipes: desired value for r  (Unit: m)  --> this is for the rolling between two pipes
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
%    -> sm_res.p: updated pitch of the helix (Unit: m)

function sm_res = GetSM_PipeCrawModeWithJoystick(VelFactor,radi, pitch, sm, buttons)
    % inilialize sm_res by sm
    sm_res = sm;
    
    % For changing the radius with buttons 5 and 6
    kappa = 1/sm.r;
    if (sm.r <= radi.init)&&(buttons(6))&&(~buttons(5))               % Loosen
        kappa = kappa - 0.020*(1+VelFactor);  % 0.020*(1+VelFactor); 
        sm_res.r = 1/kappa;
    elseif (sm.r >= radi.des_Roll_between_Pipes)&&(buttons(5))&&(~buttons(6)) % Tighten
        kappa = kappa + 0.020*(1+VelFactor);  % 0.020*(1+VelFactor);
        sm_res.r = 1/kappa;          
    end

    % For changing the pitch with buttons 2 and 3     
    if (sm.p >= pitch.init)&&(buttons(2))           % Decrease p value
        sm_res.p = sm.p - 0.00008*(1+VelFactor); %0.00015*(1+VelFactor);%0.0003*(1+VelFactor);
    elseif (sm.p <= pitch.max)&&(buttons(3))        % Increase p value
        sm_res.p = sm.p + 0.00008*(1+VelFactor); %0.00015*(1+VelFactor);%0.0003*(1+VelFactor);
    end

end