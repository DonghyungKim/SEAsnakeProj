% This is the struct 'snake moduleation' (or 'sm') for the slithering  -  by Donghyung Kim, 16.08.16
% Note that the internal time, Ot, should be defined before you run this function.
% For the meaning or the details on the following parameters, see Rollinson, David S., "Control and Design of Snake Robots" (2014). Dissertations
% [Inputs]
%  - MagFact: parameters for the magnitute control, MagFact.Lat and MagFact.Dor
%            => use MagFact.Lat=0 and MagFact.Dor=0 if you want to use
%            default value of the magnitude
% - Ot: Internal time for the gait equation
% [Output]
% - sm


function sm = SetSM_SlitheringMod(MagFact, Ot)

    %======create struct SnakeModulation=============== 
    sm.beta_lat = 0;
    sm.beta_dor = 0;

    sm.A_lat = 0.7*0.6*(MagFact.Lat+1);             % This is the magnitude of wave
    sm.A_dor = 0.7*1*(MagFact.Dor+1);

    sm.w_dor = 2*pi;%8*pi;          %  -> how fast the wave is generated per one sec, for instance, (# of wave per 1 sec x 2pi)

    sm.w_lat = (1/2)*sm.w_dor;      % Roughly, this is the speed of waves or robot

    sm.v_lat = 0.8*0.4;             % determines the frequency of the actuator cycles w.r.t time t
    sm.v_dor = 0.8*1;%2*pi/12;      %  -> how many waves along the robot, for instance, (# of waves along the robot / 2)

    sm.delta = 0;

    sm.taper = 0.6;          % --> different with other gait, slithering requires this, 'taper'
                             % Ref) DefaultGaitParams.m and runGait.m from Nelson Wong

    sm.Ot = Ot;
    %==================================================

end