% Created 8/13/2023 By Brevin Banks
% Modified 8/20/2023 By Brevin Banks
% This script utilizes the FK and IK functions for the Revyn Arm to
% demonstrate how orientations and positions can arbitrarily be chosen
% within reach of the robot and achieved on the actual hardware.
% Communication with an arduino on COM3 at 9600 baud running the
% RevynMat2ArdController.ino script is estabilished and commands are sent
% A figure is opened which shows the goal frames for the End Effector,
% Joint 5, Joint 3, and the Base for each desired frame. After the figure
% shows the desired frame, the command is sent to the arduino to move the
% robot to that frame. The scripts waits for the robot to move before
% iterating the control loop to start a new goal position command. After
% all commands, the robot stays in the last position and the script is
% ended.

% Clear the command window and restart the arduino communication
clc
if exist("arduinoObj")
    clear arduinoObj
end
arduinoObj = serialport("COM3",9600)

% define what end-line characters look like as commands come in from the
% arduino serial monitor ASCII characters
configureTerminator(arduinoObj,"CR/LF");

% clear the serial monitor
flush(arduinoObj)

% Create variable to maintain the loop iteration count
k =0;

% Read the ASCII data from the serialport object.
data1 = readline(arduinoObj); % initialization lines
data2 = readline(arduinoObj); % initialization lines
data3 = readline(arduinoObj); % initialization lines
data = readline(arduinoObj); % initialization lines
% Show the data lines in the command window
fprintf(data1+newline);
fprintf(data2+newline);
fprintf(data3+newline);
fprintf(data+newline);

% Create the goal position figure. Start by just showing the base frame
% before goal positions are selected
figure(1)
plot3(0,0,0)
plotf2(eye(4))
xlim([-200,200])
ylim([-200,200])
zlim([-50,550])
title('Base Robot is at home R = eye(4) P = [0,0,486.8750]mm EF:Open') % 486.8750mm is the arm straight up in the air (i.e. joints = [0,0,90,0,0,0,0])
% Wait a 5 seconds before starting the Control loop
pause(5)



while(1)


    % Wait to read that the arduino wants joint angles input
    while strcmp(data,"Enter Desired Joint Angles") ==false
        try
            data = readline(arduinoObj);
        end
    end

    % handle the figure if it has not been cleared, clear it before
    % plotting the new configuration.

    if ishandle(1)
        plot3(0,0,0)
    end

    % The first configuration
    if k==0
        F = [1 0 0 -20;
            0 1 0 0;
            0 0 1 480;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);0]';
        title('Goal 1 R = eye(4) P = [-20,0,480]mm EF:Open')
    end

    % The second configuration
    if k==1
        F = [1 0 0 0;
            0 1 0 0;
            0 0 1 480;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);2]';
        title('Goal 2 R = eye(4) P = [0,0,480]mm EF:Closed')
    end

    % The third configuration
    if k==2
        F = [1 0 0 50;
            0 1 0 0;
            0 0 1 480;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);0]';
        title('Goal 3 R = eye(4) P = [50,0,480]mm EF:Open')
    end

    % The fourth configuration
    if k==3
        F = [1 0 0 65;
            0 1 0 0;
            0 0 1 475;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);2]';
        title('Goal 4 R = eye(4) P = [65,0,475]mm EF:Closed')
    end

    % The fifth configuration
    if k==4
        F=[       0.8160   -0.3272    0.4766  80.0000;
            0.4500    0.8770   -0.1684        25.0000;
            -0.3628    0.3519    0.8628       425.0000;
            0         0         0    1.0000];
        ard_angle = IK_Revyn(F,false,true);
        ard_angle = [rad2deg(ard_angle);0]';
        [X,theta] = extract_axis_angle(F(1:3,1:3));
        X = round(X,2);
        axis_string = "["+string(X(1))+","+string(X(2))+","+string(X(3))+"]";
        theta_string = round(rad2deg(theta),2);

        title("Goal 5 R: axis="+axis_string+" angle="+theta_string+" P = [80.0,25.0,425.0]mm EF:Open")
    end



    % Plot the new configuration goal
    plotf2(eye(4))
    plotf2(F,'EF')
    plotf2(FK_Revyn(IK_Revyn(F,false,true),5),'J5')
    plotf2(FK_Revyn(IK_Revyn(F,false,true),3),'J3')
    plotf2(FK_Revyn(IK_Revyn(F,false,true),2),'J2')
    hold off
    view(45,45)
    xlim([-200,200])
    ylim([-200,200])
    zlim([-50,550])

    % Inform the user that the script has completed if all 5 are iterations
    % complete
    if k>4
        fprintf(newline+"All goal positions achieved. Communication Ending")
        clear arduinoObj
        break;
    end

    % Send the joint angles to the arduino
    writeline(arduinoObj,string(ard_angle(1)));
    writeline(arduinoObj,string(ard_angle(2)));
    writeline(arduinoObj,string(ard_angle(3)));
    writeline(arduinoObj,string(ard_angle(4)));
    writeline(arduinoObj,string(ard_angle(5)));
    writeline(arduinoObj,string(ard_angle(6)));
    writeline(arduinoObj,string(ard_angle(7)));

    % Listen for the arduino's response to the input
    data = readline(arduinoObj);

    % If there is an error with the joint angles we will skip the iteration
    while strcmp(data,"Pose Achieved") == false
        try
            data = readline(arduinoObj);
            if strcmp(data," Joint angle out of range")
                break;
            end
        end
    end

    % Print the goal joint values in the command window
    fprintf(data+newline)

    % Debugging help:
    % If the joints are out of bounds we will do the same error checking
    % found in the arduino code here again
    J1_new = ard_angle(1)+90.0;
    J2_new = ard_angle(2)+90.0;
    J3_new = 180.0-ard_angle(3);
    J4_new = ard_angle(4)+90.0;
    J5_new = ard_angle(5)+90;
    J6_new = ard_angle(6)+90;

    J_new =[J1_new,J2_new,J3_new,J4_new,J5_new,J6_new]
    if J1_new>181.0 || J2_new>181.0 || J3_new>181.0 || J4_new>181.0 || J5_new>181.0 || J6_new>181.0
        bad_joints = find(J_new>181)
    end
    if J1_new<-1.0 || J2_new<-1.0|| J3_new<-1.0|| J4_new<-1.0 || J5_new<-1.0 || J6_new<-1.0
        bad_joints = find(J_new<-1.0)
    end
    if J2_new>120.0
        fprintf("Joint 2 angle out of range: "+string(J2_new)+newline);
    end
    if J2_new<60.0
        fprintf("Joint 2 angle out of range: "+string(J2_new)+newline);
    end

    % Pause for a few seconds and increase iterator every loop
    pause(5);
    k=k+1;
end


