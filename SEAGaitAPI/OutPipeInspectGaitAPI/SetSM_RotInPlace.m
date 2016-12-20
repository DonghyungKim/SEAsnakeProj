% This function returns 'snake moduleation' (or 'sm') for the rotation in place  -  by Donghyung Kim, 16.09.22
% [Inputs]
%  - snakeData: data needed to run the matlab control code on the SEA snake robot using
%               ...\SnakePlotter\matlab_SEA-master\utils\SEAsnakeTools\setupSnakeData.m
%  - fbk: SEA snake's snesor feedback. Usually it obtained by 'fbk = snake.getNextFeedback();'
%  - radi: parameters related to the radius of the helix, such as 
%    -> radi.init: initial or minimum value of radius of the helix (Unit: m)
%    -> radi.des_Roll_between_Pipes: desired value for r  (Unit: m)  --> this is for the rolling between two pipes
%  - sm: current snake modulation (sm)
%  - povs: joystick reading, for example, [axes, buttons, povs] = read( joy );
% [Output]
%  - sm_res: updated snake modulation (sm) including gait parameters for rotation in place
%    -> sm_res.A_Rip: magnitude of wave
%    -> sm_res.w_RiP: speed of waves
%    -> sm_res.v_RiP: frequency of the actuator cycles w.r.t time t
 
function sm_res = SetSM_RotInPlace(snakeData, fbk, radi, sm, povs)
    % inilialize sm_res by sm
    sm_res = sm;
    
	% curvature of helix
    kappa = 1/sm_res.r;
    
    % sm only for rotating in place
    sm_res.A_RiP = 0.38;%0.49;        % magnitude of wave
    sm_res.w_RiP = 0.8*2*pi;	% speed of waves  -> how fast the wave is generated per one sec, for instance, (# of wave per 1 sec x 2pi)
    sm_res.v_RiP = 5.1;%4.7;	% frequency of the actuator cycles w.r.t time t -> how many waves along the robot, for instance, (# of waves along the robot / 2)
    
    
	% Compliance control of radius of helix
    % Apply compliance curvature to the spinning on pipe gait
    TssDes_CC_RotInPlace = 0.72*sm.TssDes_CC;   % Robot should loosen its coil to spin on the pipe so that robot can spin easily with less grip.
    
    if (~isempty(fbk))
        if (povs == 90)||(povs == 270)
            kappa = kappa + sm.Pgain_CC*(TssDes_CC_RotInPlace - GetSSofTorque(fbk, snakeData.num_modules));
            sm_res.r = 1/kappa;
            if sm_res.r <= radi.min_ClimbPole   % prevent the radius of helix becomes smaller than its minimum value
                sm_res.r = radi.min_ClimbPole;
            end
        end
    end
    
end