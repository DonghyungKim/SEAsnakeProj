% GetFlagsSubFcn_GroundLocomotion

function Res = GetFlagSubFcn_GroundLocomotion(EnableSubFcn, buttons, buttons_prev)
    % initialize  
    Res.HeadLook_GL = EnableSubFcn.HeadLook_GL;
    
    HeadLookOn = EnableSubFcn.HeadLook_GL;


    % Enable/disable the head look during the pole climbing with joystick
    %  -> Caution! It stops the robot's movement and let you operate only four modules at the head
    if ((~buttons_prev(12))&&buttons(12))&&(~HeadLookOn)
        disp('[Ground Locomotion] Headlook mode On')
        Res.HeadLook_GL=true;
    elseif ((~buttons_prev(12))&&buttons(12))&&HeadLookOn
        disp('[Ground Locomotion] Headlook mode Off')
        Res.HeadLook_GL=false;
    end

        
end