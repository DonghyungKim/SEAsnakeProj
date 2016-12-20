% Get user's direction input

while 1
    if ~simQ
        % Read sensor feedback
        fbk = snake.getNextFeedback(fbk);

        % Calculate the time interval (sec)
        dt = (fbk.time - lastTime);
        lastTime = fbk.time;

        % Send the position command
        cmd.position = SEA_angles;
        snake.set(cmd);
    end

    % Read the joystick inputs
    [axes, buttons, povs] = read( joy );

    % When user selects the direction input
    if (povs==270)
        disp('[Outer-pipe Inspection: Pole Climbing] Direction: Left')
        PoleOnLeft = true;
        break;
    elseif (povs==90)
        disp('[Outer-pipe Inspection: Pole Climbing] Direction: Right')
        PoleOnLeft = false;
        break;

    % When user want to exit the program or change to the ground locomotion,
    elseif (buttons(9)&&buttons(10))
        bTerminate = false;
        break;
    elseif ((buttons(10))&&(~buttons_prev(10)))&&(~buttons(9))
        OperMode = 0;
        disp('[Ground Locomotion]')
        break;
    end

    % Update the previous button values
    buttons_prev = buttons;
end