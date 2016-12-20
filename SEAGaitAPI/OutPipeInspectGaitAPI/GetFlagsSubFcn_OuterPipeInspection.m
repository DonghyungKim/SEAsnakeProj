% This function returns the flags to enable/disable sub-functions of the outer-pipe inspection
% with a given joystick input
% - by Donghyung Kim, 2016.9.17 (revised)
% [Inputs]
% - fbk: SEA snake's snesor feedback. Usually it obtained by 'fbk = snake.getNextFeedback();'
% - EnableSubFcn: flags to enable/disable the sub-functions such as
%  -> EnableSubFcn.HeadLook_OPI: Head Look mode
%  -> EnableSubFcn.PipeCraw_OPI: Crawling between pipes
%  -> EnableSubFcn.RotInPlace_OPI: Rotation in place
%  -> EnableSubFcn.RotInPlaceMotDir_Odd_OPI: direction of movement for
%  rotation in place => true <-> odd joints,  false <-> even joints
%

function Res = GetFlagsSubFcn_OuterPipeInspection(fbk, EnableSubFcn, buttons, buttons_prev)
    % initialize  
    Res.HeadLook_OPI = EnableSubFcn.HeadLook_OPI;
    Res.PipeCraw_OPI = EnableSubFcn.PipeCraw_OPI;
    Res.RotInPlace_OPI = EnableSubFcn.RotInPlace_OPI;
    Res.ShiftRtoP_OPI = EnableSubFcn.ShiftRtoP_OPI;
    
	Res.RotInPlaceMotDir_Odd_OPI = true;    
    

    HeadLookOn = EnableSubFcn.HeadLook_OPI;
    PipeCrawOn = EnableSubFcn.PipeCraw_OPI;
    RotInPlaceOn = EnableSubFcn.RotInPlace_OPI;
    ShiftRtoPOn = EnableSubFcn.ShiftRtoP_OPI;


    % Enable/disable 'HeadLook_OPI' <=>  head-look during the pole climbing with joystick
    %  -> Caution! It stops the robot's movement and let you operate only four modules at the head
    if ((~buttons_prev(12))&&buttons(12))&&(~HeadLookOn)
        disp('[Outer-pipe Inspection] Headlook mode On')
        Res.HeadLook_OPI=true;
    elseif ((~buttons_prev(12))&&buttons(12))&&HeadLookOn
        disp('[Outer-pipe Inspection] Headlook mode Off')
        Res.HeadLook_OPI=false;
    end

    % Enable/disable 'PipeCraw_OPI' <=> pipe crawling between two pipes with joystick
    if ((~buttons_prev(11))&&buttons(11))&&(~PipeCrawOn)
        disp('[Outer-pipe Inspection] Pipe Crawling mode On')
        Res.PipeCraw_OPI=true;
    elseif ((~buttons_prev(11))&&buttons(11))&&PipeCrawOn
        disp('[Outer-pipe Inspection] Pipe Crawling mode Off')
        Res.PipeCraw_OPI=false;
    end

    % Enable/disable 'RotInPlace_OPI' <=> rotating in place gait with joystick
    if (~HeadLookOn)&&(~PipeCrawOn) % do not allow rotating in place during Headlook or Pipe crawling 
        if (buttons(9))&&(~buttons_prev(5)&&buttons(5))&&(~RotInPlaceOn)
            disp('[Outer-pipe Inspection] Rotating in place On')
            Res.RotInPlace_OPI=true;
            
            if (~isempty(fbk))
                Teven = 0;
                Todd = 0;
                for i=1:16
                    if mod(i,2) %odd
                        Todd = Todd + fbk.torque(i)^2;
                    else        %even
                        Teven = Teven + fbk.torque(i)^2;
                    end
                    Todd = sqrt(Todd);
                    Teven = sqrt(Teven);
                end

                if Todd > Teven
                    Res.RotInPlaceMotDir_Odd_OPI = true;
                else
                    Res.RotInPlaceMotDir_Odd_OPI = false;
                end
            end
            
        elseif (buttons(9))&&(~buttons_prev(5)&&buttons(5))&&RotInPlaceOn
            disp('[Outer-pipe Inspection] Rotating in place Off')
            Res.RotInPlace_OPI=false;
        end    

    end
    
    % Enable/disable 'ShiftRtoP_OPI' <=> shifting from rod to pipe
    if (buttons(9))&&((~buttons_prev(6))&&buttons(6))&&(~ShiftRtoPOn)
        disp('[Outer-pipe Inspection] Shifting from Rod to Pipe mode On')
        Res.ShiftRtoP_OPI=true;
    elseif (buttons(9))&&((~buttons_prev(6))&&buttons(6))&&ShiftRtoPOn
        disp('[Outer-pipe Inspection] Shifting from Rod to Pipe mode Off')
        Res.ShiftRtoP_OPI=false;
    end
        
end