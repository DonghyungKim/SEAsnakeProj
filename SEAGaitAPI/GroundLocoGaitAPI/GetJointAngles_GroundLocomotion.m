% This function returns the joint angles of the SEA snake robot for the ground locomotion (Sidewinding, Slithering, Lateral rolling, Turn-in-place)
%- by Donghyung Kim, 2016.9.23 (revised)
% [Inputs]
% - Ot: Internal time for the gait equation
% - snakeData: data needed to run the matlab control code on the SEA snake robot using
%               ...\SnakePlotter\matlab_SEA-master\utils\SEAsnakeTools\setupSnakeData.m
% - MagFact: parameters for the magnitute control, MagFact.Lat and MagFact.Dor
% - SEA_angles_prev: previous value of joint angles
% - axes, buttons, povs : joystick reading, for example, [axes, buttons, povs] = read( joy );
% [Output]
% - SEA_angles: joint angles from the gait equation

function SEA_angles = GetJointAngles_GroundLocomotion(Ot, snakeData, MagFact, EnableSubFcn, SEA_angles_prev, axes, buttons, povs, dt)
    % Determine joint angles for gait selected by the controller or the joystick

    % - (Sub-function) Head Look
    if EnableSubFcn.HeadLook_GL
        % Joint angles for head-look mode
        SEA_angles = HeadlookMod(snakeData, SEA_angles_prev, axes,  dt, 0.02);
    
    % - Slithering
    elseif (povs == 0)||(povs == 180)
        if (~(buttons(5)|buttons(6)))&&(sign(axes(3))==0)
            sm = SetSM_SlitheringMod(MagFact,Ot);
            SEA_angles = SlitheringMod(snakeData,sm);
        else
            SEA_angles = SEA_angles_prev; 
        end

    % - Sidewinding    
    elseif (povs == 90)||(povs == 270)
        if (~(buttons(5)|buttons(6)))&&(sign(axes(3))==0)
            sm = SetSM_SidewindMod(MagFact,Ot);
            SEA_angles = SideWindMod(snakeData,sm);
        else
            SEA_angles = SEA_angles_prev; 
        end

    % - Lateral rolling    
    elseif ((buttons(5))||(buttons(6)))&&(~(buttons(5)&buttons(6)))
        if (povs == -1)&&(sign(axes(3))==0)
            sm = SetSM_LatRollingMod(MagFact,Ot);
            SEA_angles = LatRollingMod(snakeData,sm);
        else
            SEA_angles = SEA_angles_prev;                 
        end
        
    % - Conical sidewinding
    elseif (sign(axes(1)) == -1)||(sign(axes(1)) == 1)      
        if (~(buttons(5)|buttons(6)))&&(povs == -1)
            sm = SetSM_ConiSidewindingMod(MagFact,Ot);
            SEA_angles = ConiSidewindingMod(snakeData,sm);       
        else
            SEA_angles = SEA_angles_prev;                 
        end        

    % - Turn-in-place
    elseif (sign(axes(3)) == -1)||(sign(axes(3)) == 1)  
        if (~(buttons(5)|buttons(6)))&&(povs == -1)
            sm = SetSM_TurnInPlaceMod(MagFact,Ot);
            SEA_angles = TurnInPlaceMod(snakeData,sm);       
        else
            SEA_angles = SEA_angles_prev;                 
        end

    % - Stop
    else
        SEA_angles = SEA_angles_prev;   % Keep current joint angles

    end

end