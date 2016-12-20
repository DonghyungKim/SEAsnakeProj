% Class for handling the button press counter
classdef JoyButtonCnt < handle
    properties       
        Count = 0;
        
        axes
        buttons
        povs
    
        buttons_prev = zeros(1,12);     % This is only valid when the number of buttons is 12
    end
    
    methods
        function obj = JoystickCtr(axes, buttons, povs, buttons_prev)
            if nargin == 4               
                obj.axes = axes;
                obj.buttons = buttons;
                obj.povs = povs;
                
                obj.buttons_prev = buttons_prev;
            end
        end
        
        function Update(obj, axes, buttons, povs)
            obj.axes = axes;
            obj.buttons = buttons;
            obj.povs = povs;
        end
        
        function res = ButtonPushed(obj, n)
            if (n < 1)||(n > 12)  % This is only valid when the number of buttons is 12
                error('Invalid button number')
            elseif isnumeric(n)
                if (obj.buttons(n))&&(~obj.buttons_prev(n))
                    res = true;
                else
                    res = false;
                end
            else
                error('Button number must be numeric')
            end
        end
        
        function CntWithTwoButtons(obj, decButtonID, incButtonID, CntMin, CntMax)
            if (obj.ButtonPushed(decButtonID))&&(obj.Count > CntMin)
                obj.Count = obj.Count - 1;
            elseif (obj.ButtonPushed(incButtonID))&&(obj.Count < CntMax)
                obj.Count = obj.Count + 1;              
            end
            
        end
    end
end