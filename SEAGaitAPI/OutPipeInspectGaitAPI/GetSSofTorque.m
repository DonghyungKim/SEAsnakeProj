% This function returns the square sum of the joint torques with a given
% feedback from SEA snake robot - by Dong-Hyung Kim, 2016.09.14
function Tss = GetSSofTorque(fbk, numModules)
    if(~isempty(fbk))
        Tsum = 0;
        for i=1:numModules
            Tsum = Tsum + fbk.torque(i)^2;
        end
        Tss = sqrt(Tsum);
    else
        error('error: fbk is empty!!')
    end
end