% This function returns the joint angles of the SEA snake robot for the outer-pipe inspection
%- by Donghyung Kim, 2016.12.15 (revised)
% [Inputs]
% - sm_input: snake modulation (sm)
% - snakeData: data needed to run the matlab control code on the SEA snake robot using
%               ...\SnakePlotter\matlab_SEA-master\utils\SEAsnakeTools\setupSnakeData.m
% - fbk: SEA snake's snesor feedback. Usually it obtained by 'fbk = snake.getNextFeedback();'
% - VelFactor: velocity gain factor
% - radi: parameters related to the radius of the helix, such as 
%    -> radi.init: initial or minimum value of radius of the helix (Unit: m)
%    -> radi.des_Roll_between_Pipes: desired value for r  (Unit: m)  --> this is for the rolling between two pipes
% - pitch: pitch values of the helix such as
%    -> pitch.init: initial or minimum value of pitch of the helix
%    -> pitch.max: maximum value of pitch of the helix
% - PoleOnLeft: boolean to denote the direction of pole with respect to the robot's heading
% - Theta: current joint angles, normally given by 'SEA_angles' in the main source code
% - EnableSubFcn: booleans to enable/disable the sub-functions of the outer pipe inspection such as
%    -> EnableSubFcn.HeadLook_OPI: enable/disable the head look
%    -> EnableSubFcn.PipeCraw_OPI: enable/disable the crawling between pipes
%    -> EnableSubFcn.RotInPlace_OPI: enable/disable the rotating in place
%    -> EnableSubFcn.ShiftRtoP_OPI: enable/disable the shifting from rod to pipe
% - axes, buttons, povs : joystick reading, for example, [axes, buttons, povs] = read( joy );
% - buttons_prev: buttons from the previous iteration
% - dt: time inteval
% - Ot_SoP: internal time only for the rotating in place gait
% [Output]
% - res
%    -> res.sm: updated snake modulation (sm)
%    -> res.Ot_offset_ShiftPipe: offset value of the internal time for the shifting from pipe to pipe
%    -> res.Angles: joint angles from the gait equation

function res = GetJointAngles_SM_OuterPipeInspection(sm_input, snakeData, fbk, VelFactor, radi, pitch, PoleOnLeft, Theta, EnableSubFcn, Ot_offset_ShiftPipe, axes, buttons, buttons_prev, povs, dt, Ot_RiP)

    % initialize snake modulation (sm) and joint angles
    res.sm = sm_input;
    res.Ot_offset_ShiftPipe = Ot_offset_ShiftPipe;

    % initialize the flag for sub-functions
    HeadLookOn = EnableSubFcn.HeadLook_OPI;
    PipeCrawOn = EnableSubFcn.PipeCraw_OPI;
    RotInPlaceOn = EnableSubFcn.RotInPlace_OPI;
    ShiftRtoPOn = EnableSubFcn.ShiftRtoP_OPI;
    
    % Get joint angles, SEA_angles for each sub functions
    %  - [Sub-function] Head-look
    if HeadLookOn
        % Joint angles for head-look mode
        res.Angles = HeadlookMod(snakeData, Theta, axes,  dt, 0.02);

    %  - [Sub-function] Crawling between two pipes
    elseif PipeCrawOn
        % Initialize the offset valule of Ot as the current time
        if buttons(1)&&(~buttons_prev(1))	% Enable the shifting from pipe to pipe when the user press '1'
            disp('[Outer-pipe Inspection: Pipe Crawling] Shifting from Pipe to Pipe')
            res.Ot_offset_ShiftPipe = sm_input.Ot;
        end

        % Get the snake modulation with joystick inputs
        sm = GetSM_PipeCrawModeWithJoystick(VelFactor, radi, pitch, sm_input, buttons);
        % Joint angles for pipe crawling mode   
        res.Angles = PipeCrawMod(snakeData, sm, PoleOnLeft, povs, buttons, res.Ot_offset_ShiftPipe);
        % Update sm
        res.sm = sm;
        
    %  - [Sub-function] Rotating in place    
    elseif RotInPlaceOn
        % Get the snake modulation with joystick inputs
        sm = SetSM_RotInPlace(snakeData, fbk, radi, sm_input, povs);
        % Joint angles for rotating in place
        res.Angles = RotInPlaceMod(snakeData, sm, PoleOnLeft, Ot_RiP, EnableSubFcn.RotInPlaceMotDir_Odd_OPI);
        % Update sm
        res.sm = sm;
 
    %  - [Sub-function] Shifting from Rod to Pipe
    elseif ShiftRtoPOn
        % Get the snake modulation with joystick inputs
        sm = GetSM_ShiftRodToPipeModeWithJoystick(snakeData, fbk, VelFactor, radi, pitch, sm_input, povs, buttons);
        % Joint angles for shifting from rod to pipe     
        res.Angles = ShiftRodtoPipeMod(snakeData, sm, PoleOnLeft);       
        % Update sm
        res.sm = sm;       

    %  -> (Default) Pole climbing using compliance curvature
    else
        % Get the snake modulation with joystick inputs
        sm = GetSM_ClimbingModeWithJoystick(snakeData, fbk, VelFactor, radi, pitch, sm_input, povs, buttons);
        % Joint angles for climbing mode      
        res.Angles = ClimbingMod(snakeData, sm, PoleOnLeft);
        % Update sm
        res.sm = sm;
    end
    

end