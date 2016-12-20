%snake = HebiLookup.newConnectedGroupFromName('*', 'SA008')
%snake = HebiLookup.newConnectedGroupFromName('*', 'SA002'); % -> changed from SA008 to SA002 - by Donghyung Kim, 2016.08.09
snake = HebiLookup.newConnectedGroupFromName('*', 'S-0002');

snakeType = 'SEA Snake';
%numModules = snake.getInfo.numModules;
numModules = snake.getNumModules;

snakeData = setupSnakeData( snakeType, numModules);
snakeData.firmwareType = snake.getInfo.firmwareType;

