% This is to initilize SEA snake robot for the simulation - by Donghyung Kim, 2016.8.10
%  - ask a user to input the number of SEA modules
%  - construct 'snakeData' for the simulation
while 1
    snakeData.num_modules = input('Input the number of SEA modules = ');
    snakeData.num_modules = ceil(snakeData.num_modules);
    if snakeData.num_modules > 1
        break;
    else
        fprintf('Not valid number! You need at least two modules!\n');
    end
end

snakeType = 'SEA Snake';
%numModules = snake.getInfo.numModules;
numModules = snakeData.num_modules;

snakeData = setupSnakeData( snakeType, numModules);
%snakeData.firmwareType = snake.getInfo.firmwareType;

