function [jointAngles] = anglesUtoSEA(snakeData,jointAngles)
%ANGLESUTOSEA Sets U snake joint angles as if it were a SEASnake
%   Switches the signs of all joint angles and flips the module order to
%   be head to tail.
    for n = 1:length(jointAngles(:,1))
%         jointAngles(n,:) = snakeData.reversals.*jointAngles(n,:);
%         jointAngles(n,:) = fliplr(jointAngles(n,:));
        jointAngles(n,:) = fliplr(jointAngles(n,:));        % -> changed the order: flip and then swithe - by Donghyung Kim, 2016.08.09
        jointAngles(n,:) = snakeData.reversals.*jointAngles(n,:);
    end
end