% Button push counter with two buttons; one decrease and the other increase the number of button presses 
% - by Donghyung Kim, 2016.8.13
% [Inputs]
%   - Count: Current value of the button push counter
%   - buttons: Current status of buttons from the joystic reading
%   - buttons_prev: Previous value of 'buttons'
%   - buttonCount: Current value of the number of button presses 
%   - decButtonID: ID of button for decreasing 'Count'
%   - incButtonID: ID of button for increasing 'Count'
%   - CntLim: Limit value of the button push counter
% [Outputs]
%   - res.buttonCount: Updated value of buttonCount
%   - res.Count = Count : Updated value of Count
% [Description]
%  -> User clicks the buttons, 'decButtonID' and 'incButtonID', then this
%  decrease or increase 'Count' within -CntLim and +CntLim

function res = GetJoyCntWithTwoButtons(Count, buttons, buttons_prev, buttonCount, decButtonID, incButtonID, CntLim)

	if (abs(Count) <= CntLim)
        if ((Count == CntLim)&&(buttons(decButtonID)==1)) || ((Count == -CntLim)&&(buttons(incButtonID)==1)) || (abs(Count) < CntLim)

            if (buttons(incButtonID)==1)&&(buttons_prev(incButtonID)==0)  % check the rising edge at 'incButtonID'th button signal
                buttonCount(incButtonID) = buttonCount(incButtonID) + 1;
            end

            if (buttons(decButtonID)==1)&&(buttons_prev(decButtonID)==0)  % check the rising edge at 'decButtonID'th button signal
                buttonCount(decButtonID) = buttonCount(decButtonID) + 1;
            end

            Count = buttonCount(incButtonID) - buttonCount(decButtonID);
        end
    end  

    res.buttonCount = buttonCount;
    
    res.Count = Count;

end
