% Created 8/13/2023 By Brevin Banks
% Modified 8/20/2023 By Brevin Banks
% This script utilizes the FK and IK functions for the Revyn Arm to
% demonstrate how a linear trajectory can be found between 2 points within
% the robot's possible joint space. We use a starting frame and ending
% frame and evaluate if the robot can maintain its end effector orientation
% while moving between the two possible positions. We can use this to move
% the robot in a tracked straight line trajectory. We will draw a small
% rectangle with the end effector

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

% % The following commented frames and goals draw a rectangle in the ZY plane
% % of the robot base.

% Start_frame = [0 0 -1  -300.169610181001;
%                 0 -1 0 -50;
%                 -1 0  0 210;
%                 0 0 0   1];
% Goal_position = [-300.537657749172;50;210];
% jointtraj = Lin_Traj_Revyn(Start_frame,Goal_position,true);

% Start_frame2 = FK_Revyn(jointtraj(:,end),7);
% Goal_position2 = [-300.537657749172;50;250];
% jointtraj2 = Lin_Traj_Revyn(Start_frame2,Goal_position2,true);
%
%
% Start_frame3 = FK_Revyn(jointtraj2(:,end),7);
% Goal_position3 = [-300.537657749172;-50;250];
% jointtraj3 = Lin_Traj_Revyn(Start_frame3,Goal_position3,true);
%
%
% Start_frame4 = FK_Revyn(jointtraj3(:,end),7);
% Goal_position4 = [-300.537657749172;-50;210];
% jointtraj4 = Lin_Traj_Revyn(Start_frame4,Goal_position4,true);


% The following frames and goals draw a rectangle in the XY plane of the
% robot base
Start_frame = [0 0 -1  -300.169610181001;
    0 -1 0 -50;
    -1 0  0 210;
    0 0 0   1];
Goal_position = [-300.537657749172;50;210];
jointtraj = Lin_Traj_Revyn(Start_frame,Goal_position,true);

Start_frame2 = FK_Revyn(jointtraj(:,end),7);
Goal_position2 = [-330.537657749172;50;210];
jointtraj2 = Lin_Traj_Revyn(Start_frame2,Goal_position2,true);


Start_frame3 = FK_Revyn(jointtraj2(:,end),7);
Goal_position3 = [-330.537657749172;-50;210];
jointtraj3 = Lin_Traj_Revyn(Start_frame3,Goal_position3,true);


Start_frame4 = FK_Revyn(jointtraj3(:,end),7);
Goal_position4 = [-300.537657749172;-50;210];
jointtraj4 = Lin_Traj_Revyn(Start_frame4,Goal_position4,true);

%  _____
% |     |
% |_____|

while(1)


    % Wait to read that the arduino wants joint angles input
    while strcmp(data,"Enter Desired Joint Angles") ==false
        try
            data = readline(arduinoObj);
        end
    end

    % First position starts by ensuring we are in the home position. We close
    % the end effector slightly in case we wanted to hold a pen or something of
    % the sorts while the robot move in the rectangular pattern
    if k==0
        ard_angle = [0,0,90,0,0,0,0.5]';
    end
    % Assume the first corner of the rectangle
    if k==1
        ard_angle = [rad2deg(jointtraj(:,k));0.5];
    end
    % Make sure the robot End effector is closed
    if k==2
        ard_angle(7)=0.5;
    end
    % Move along the first trajectory
    if k<=length(jointtraj) && k>2
        ard_angle = [rad2deg(jointtraj(:,k));0.5];
        fprintf("1st trajectory.."+newline)
    end
    % Move along the second trajectory
    if k<=length(jointtraj)+length(jointtraj2) && k>length(jointtraj)
        ard_angle = [rad2deg(jointtraj2(:,k-length(jointtraj)));0.5];
        fprintf("2nd trajectory.."+newline)
    end
    % Move along the third trajectory
    if k<=length(jointtraj)+length(jointtraj2)+length(jointtraj3) && k>length(jointtraj)+length(jointtraj2)
        ard_angle = [rad2deg(jointtraj3(:,k-length(jointtraj)-length(jointtraj2)));0.5];
        fprintf("3rd trajectory.."+newline)
    end
    % Move along the fourth trajectory
    if k<=length(jointtraj)+length(jointtraj2)+length(jointtraj3)+length(jointtraj4) && k>length(jointtraj)+length(jointtraj2)+length(jointtraj3)
        ard_angle = [rad2deg(jointtraj4(:,k-length(jointtraj)-length(jointtraj2)-length(jointtraj3)));0.5];
        fprintf("4th trajectory.."+newline)
    end

    % Send the joint commands to the arduino
    writeline(arduinoObj,string(ard_angle(1)));
    writeline(arduinoObj,string(ard_angle(2)));
    writeline(arduinoObj,string(ard_angle(3)));
    writeline(arduinoObj,string(ard_angle(4)));
    writeline(arduinoObj,string(ard_angle(5)));
    writeline(arduinoObj,string(ard_angle(6)));
    writeline(arduinoObj,string(ard_angle(7)));


    % Listen for the arduino's response to the input
    data = readline(arduinoObj)


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

    % Send a large pause between each corner achievement of the rectangle
    if k==0 || k==length(jointtraj) || k == length(jointtraj2)+length(jointtraj) || k == length(jointtraj3)+length(jointtraj2)+length(jointtraj) || k == length(jointtraj4)+length(jointtraj3)+length(jointtraj2)+length(jointtraj)
        pause(5)
    end
    % increase iterator each loop
    k=k+1;
end


