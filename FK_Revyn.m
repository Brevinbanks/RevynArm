% Created 8/13/2023 By Brevin Banks
% Modified 8/13/2023 By Brevin Banks
% This function takes an input joint angles Frame desired forward kinematic
% frame transform for the Revyn Arm robot to achieve at the end effector.
% INPUT
%   angles - a 6x1 vector of angles bounded between [-pi,pi]
%   frame_num - a number 0 to 7 for returning a frame along the path from
%   the base to the end effector. 0-base 1-joint1 2-joint2 3-joint3
%   4-joint4 5-joint5 6-joint6 7-end effector
% OUTPUT
%   angles - a 4x4 SE3 matrix with rotation matrix and position vecotor of
%   the end effector to achieve the desired joint position of the robot with
%   the Swaytail Premium Metal Robot Mechanical Claw/Clamp Arm/Gripper relative
%   to the Base
function Tf = FK_Revyn(angles,frame_num)

% assign all the angles a short hand variable t_v
t1v = angles(1);
t2v = angles(2);
t3v = angles(3);
t4v = angles(4);
t5v = angles(5);
t6v = angles(6);

% define the variables used from lengths of the Revyn arm joints
d1f = 88.95; % mm length between base and joint 1 (joint 1 and 2 have the same position)
d4f = 142.183; % mm length between joint 4 and 5 (joint 5 and 6 have the same position)
d6f = 90.881 + 111; % mm length between joint 6 and the end effector (joint 5 and 6 have the same position)
a2f = 53.861; % mm length between joint 2 and 3 (joint 3 and 4 have the same position)

% Perform the forward kinematics given the DH parameters
% DH parameters
% Theta - z rotation
% d - z translation
% alpha - x rotation
% a - x translation
% I don't care so much the order of things so long as it ends up in the
% correct spot and the last event to occur in a transform is the input
% joint motion. You could reverse the process and have it work too. I
% prefer following joint motion last. Comma separated DH values do not
% occur simultaneously. They happen during separate phases.
% From       Theta        d      a      alpha
% 0 to 1      t1v        d1f     0        0
% 1 to 2      t2v         0      0       pi/2
% 2 to 3    pi/2,t3v      0      a2f      0
% 3 to 4      t4v         0      0       pi/2
% 4 to 5      t5v        d4f     0      -pi/2
% 5 to 6      t6v         0      0       pi/2
% 6 to Ef      0         d6f     0        0

A0f = eye(4);
A1f = tranzse3(d1f)*rotzse3(t1v);
A2f = rotxse3(pi/2)*rotzse3(t2v);
A3f = rotzse3(pi/2)*tranxse3(a2f)*rotzse3(t3v);
A4f = rotxse3(pi/2)*rotzse3(t4v);
A5f = tranzse3(d4f)*rotxse3(-pi/2)*rotzse3(t5v);
A6f = rotxse3(pi/2)*rotzse3(t6v);
Aef = tranzse3(d6f);

% Depending on the given frame_num, calculate and return the Forward
% kinematics up to that frame number relative to the base
switch(frame_num)
    case 0
        Tf = A0f; %Base
    case 1
        Tf = A0f*A1f;
    case 2
        Tf = A0f*A1f*A2f;
    case 3
        Tf = A0f*A1f*A2f*A3f;
    case 4
        Tf = A0f*A1f*A2f*A3f*A4f;
    case 5
        Tf = A0f*A1f*A2f*A3f*A4f*A5f;
    case 6
        Tf = A0f*A1f*A2f*A3f*A4f*A5f*A6f;
    case 7
        Tf = A0f*A1f*A2f*A3f*A4f*A5f*A6f*Aef;
end

% Error check
% Check if the angles are within range
range_angles = [angles(1),angles(2),angles(4),angles(5),angles(6)];
Tfef = A0f*A1f*A2f*A3f*A4f*A5f*A6f*Aef;
if Tfef(3,4)<0 % must be above the flat plane of the ground
    error('The given joints result in a frame that is out of reach of the robot. The End effector is below the base')
elseif norm(Tfef(1:3,4))<202.635 %%mm radius of circle around the base
    error('The given joints result in a frame that is out of reach of the robot. The End effector position pushes the joint limits')
elseif norm(Tfef(1:3,4)-[0;0;d1f])<205.5 %mm radius of circle around joint 1
    error('The given joints result in a frame that is out of reach of the robot. The End effector position pushes the joint limits')
elseif norm(Tfef(1:3,4)-[0;0;d1f])>a2f+d4f+d6f
    error('The given joints result in a frame that is out of reach of the robot. The End effector position is longer than the arm is capable of')
end

if any(abs(round(range_angles,3))>pi+0.01)
    % Throw error if any angle is a problem
    angles_out = find(abs(round(range_angles,3))>pi+0.01);
    ao_string = string(angles_out);
    er_str = "Angle "+ao_string+" given to Fk_Revyn was out of range for [-pi,pi] "+newline;
    er_str = strcat(er_str(1),er_str(3),er_str(2));
    error(er_str)
end
if angles(3)>2*pi+0.01 || angles(3)<0-0.01
    % Throw error if any angle is a problem
    error('The angle for Joint 3 given to Fk_Revyn was out of range for [0,2pi]')
end

end