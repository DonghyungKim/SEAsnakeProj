function simQ = GetUserSelect_Sim_or_Exp

    while 1
        str=input('Select the operation mode (s = simulation, e = experiment) : ','s');
        if ((str == 's')||(str == 'S'))
            simQ = true;
            break;
        elseif ((str == 'e')||(str == 'E'))
            simQ = false;
            break;
        else
            fprintf('Not valid input!\n');
        end
    end

end