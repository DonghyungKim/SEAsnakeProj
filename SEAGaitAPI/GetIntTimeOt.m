% This function is to control the direction of movement of the robot with the internal time, Ot  -  by Donghyung Kim, 16.08.10
% [Inputs] 
%  - dir (direction value): this value should be true, false, or [] (empty)
%                          where 'true' increase Ot, 'false' decrease Ot, and '[]' do noting with Ot which means stops the robot 
%  - Ot (current value of internal time (sec)): you need to bring this from the snake modulation, for example, sm.Ot 
%  - dt (time period (sec))
% [Output]
%  - Ot_ret: Updated value of 'Ot'
% [Description]
% The internal time 'Ot' is used for the wave equation independent from 't' in the main code.
% Thus one can control the direction or speed of the wave by changing 'dir' or 'dt'
% To update 'Ot' in the main code, you may write sm.Ot = GetIntTimeOt(dir, sm.Ot, dt);

function Ot_ret = GetIntTimeOt(dir, Ot, dt)
    if dir
        Ot_ret = Ot + dt;
    elseif ~dir
        Ot_ret = Ot - dt;
    else
        Ot_ret = Ot;
    end
end