function res = DeadzoneJoyErr(AnalogVal, joyError)

    if abs(AnalogVal) < joyError   % AnalogVal becomes 0 if it is smaller than absolute value of joyError
        res = 0;
    else
        res = AnalogVal;            % Otherwise, return AnalogVal
    end

end