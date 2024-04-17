%% forward motion
function forward(nb1, tim)
   % PID Tuning to match a desired RPM lab 11.
    maxEncoderDuty = 17;
    % Modify these (Tip: tune proportional first, then integral or derivative)
    %for motor 1. 
    kp = 3.9;%proportional gain
    ki = 0.3;           %integral gain
    kd = -0.07; %derivative gain

    %for motor 2. 
    kp2 = 3.9;
    ki2 = 0.3;
    kd2 = -0.07;

    rpm_targ = 100;  %The goal RPM

    %target RPM 
    target_rpm = 100;
    integral = 0;
    integral2 = 0;
    prevError = 0;
    prevError2 = 0;
    vals = 0;
    vals2 = 0;

    % For graphing
    rpms = 0;
    times = 0;
    rmps2 =0;
    times2 = 0;

    % Change me to change how long the program runs
    runtime = tim;

    vals = nb1.encoderRead(2);
    vals2 = nb1.encoderRead(2);
    tic
    pause(0.03); % Small delay to avoid initial case dt blowing up
    while toc < runtime
    pause(0.014);
    vals = nb1.encoderRead(1); % Get the counts since the last time we called
    vals2 = nb1.encoderRead(2);
   
    times(end+1) = toc; % Collect for graphing
    %times2(end+1) = toc; % collect for graphing 2
    dt = times(end) - times(end - 1); % Compute the time difference between the last loop and this one.
    dt2 = times(end) - times(end -1); % compute the time diffrence bwteen two for second 
    %Calculate RPM:
    % Hint 1: You can get your counts per second either by dividing your
    % counts since last read by dt, or use the val.countspersec
    % estimation multiplied by 100.

    % Hint 2: There are about 1440 encoder counts per revolution of the wheel 
    
    % Hint 3: Remember, we want this result in revolutions per minute (RPM)
    rpm = (abs(vals.counts*60)/(1440*dt));
    rpm2 = (abs(vals2.counts*60)/(1440*dt));
    
    rpms(end+1) = rpm; % Collect for graphing
    %rpms2(end+1) = rpm2;
    
    error = target_rpm - rpm; % Difference between our target and current rpm 
    error2 =  target_rpm - rpm2;
    integral = integral + (error * dt); % Running sum adding the error over the time step
    integral2 = integral + (error2* dt2);
    derivative = (error-prevError) / dt; % Rate of change estimate using the difference
    derivative2  =  (error-prevError) / dt2;                     % between the current and prior error over the
                                         % time between them
                                         
    prevError = error;
    prevError2 = error2;

    %Write the code for your controller output here, using the gain
    %variables and the three errors computed above:
    control = error*kp + integral*ki + derivative*kd; % Put the error terms and coefficients together
                                                                  % into one control signal!
    control2 = error2*kp2 + integral2*ki2 + derivative2*kd2;
    % Caps the motor duty cycle at +/- max encoder duty cycle to prevent
    % aliasing
    if control > maxEncoderDuty
        control = maxEncoderDuty;
    end 
    if control < -maxEncoderDuty
        control = -maxEncoderDuty;
    end
     if control2 > maxEncoderDuty
        control2 = maxEncoderDuty;
    end 
    if control2 < -maxEncoderDuty
        control2 = -maxEncoderDuty;
    end
    nb1.setMotor(1, control); % Send the control signal to the motor.
    nb1.setMotor(2, control2);
    end
    stop(nb1);
end
