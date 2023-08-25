% Created 8/13/2023 By Brevin Banks
% Modified 8/13/2023 By Brevin Banks
% The following functions are back up code that was used in the fundamental
% derivation of the FK and IK for the revyn arm.

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
function Tf = Fk_Revyn_original(angles,frame_num)
% Check if the angles are within range
if any(abs(angles)>pi)
    % Throw error if any angle is a problem
    error('One of the angles given to Fk_Revyn was out of range for [-pi,pi]')
end
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
end


% This function takes an input reference SE3 Frame transformation and
% produces the expected 6 joint angles for the Revyn Arm robot to achieve
% that desired position and orientation in Frame. The solution is not
% necessarily unique.
% INPUT
%   Frame - a 4x4 SE3 matrix 3:3,3:3 is a rotation matrix and 1:3,4 is a
%   collumn of positions
%   plotit - boolean use to turn on debug plotting of vectors and frames
% OUTPUT
%   angles - a 6x1 column vector of the joint angles 1 to 6 that can be
%   input to achieve the input Frame at the end effector with the Swaytail
%   Premium Metal Robot Mechanical Claw/Clamp Arm/Gripperrelative to the
%   Base
function angles = IK_Revyn_original(Frame,plotit)

% The following resources were used to help debug the geometric
% construction of the inverse kinematics for the Revyn arm
% They are good references, but the final solution contains a more
% thourough conditional approach

% https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwi--sqnmNiAAxU2k4kEHTmgAgwQFnoECCoQAQ&url=https%3A%2F%2Fdergipark.org.tr%2Fen%2Fdownload%2Farticle-file%2F2289226&usg=AOvVaw0t8ubsj5CZnWb2fr4bldMA&opi=89978449
% https://www.frontiersin.org/articles/10.3389/fnbot.2022.791796/full
% https://www.hindawi.com/journals/mpe/2021/6647035/

% The following are quick reference variables used to grab the different
% values within the desired frame transform

if nargin<2
    plotit = false;
end
nxi = Frame(1,1);
nyi = Frame(2,1);
nzi = Frame(3,1);
oxi = Frame(1,2);
oyi = Frame(2,2);
ozi = Frame(3,2);
axi = Frame(1,3);
ayi = Frame(2,3);
azi = Frame(3,3);
pxi = Frame(1,4);
pyi = Frame(2,4);
pzi = Frame(3,4);

% The physical lengths of arms on the Revyn arm
d1i = 88.95; % mm length between base and joint 1 (joint 1 and 2 have the same position)
d4i = 142.183; % mm length between joint 4 and 5 (joint 5 and 6 have the same position)
d6i = 90.881 + 111; % mm length between joint 6 and the end effector (joint 5 and 6 have the same position)
a2i = 53.861; % mm length between joint 2 and 3 (joint 3 and 4 have the same position)

% We will extract ZYZ euler angles from the given frame transformation to
% identify what the position of joint 5 is relative to the end effector
%ZYZ Euler
Zei = atan2(ayi,axi); % The first Z
Yei = atan2(sqrt(1-azi^2),azi); % Y
Z2ei = atan2(ozi,-nzi); % the second Z unused. Just observed for reference. Possibly useful in solution to theta 4.

axi = cos(Zei)*sin(Yei);% rotation angle of end effector in x from given ZYZ
ayi = sin(Zei)*sin(Yei); % roation angle of end effector in y from given ZYZ
azi = cos(Yei); %rotation angle of end effector in z from given ZYZ

% We find the first angle by projecting the robot down to the X and Y
% plane. Looking down from the top if the top of the robot it perpendicular to the z axis of the robot base.
% The angle is obtained by drawing a vector between the end effector and
% joint 5. The given pi positions of the end effector and backed out
% positions using the magnitude d6i and the angles ai help determine the
% angle between the x axis of the base frame relative to the end effector.
% This is for joint 1
t1i = atan2(pyi-d6i*ayi,pxi-d6i*axi); % Theta 1 angle of joint 1 (forearm style joint)

% To get theta 2 and theta 3 we will use the law of cosines with the
% position of joint 5 relative to the base. We can also issolate the
% position of the 1st joint since it's z position is fixed and use it to
% get the position of the 1st joint relative to the 5th joint.
P0ef = [pxi;pyi;pzi]; % The given Position of the desired frame for the end effector
alpha = [axi;ayi;azi]; % The given ZYZ euler angles rotation vector for the desired frame from the base to Joint 5
P5ef = d6i*alpha; %position of joint 5 with respect to the end effector
P05 = P0ef-P5ef; %vector from base to joint 5 from base perspective
P15 = P05-[0;0;d1i]; %vector from joint 1 to joint 5 from base perspective

% The third joint  is the joint on the opposite side of the arm formed by
% P15, thus after applying the law of cosines we subtract pi/2
phi = acos((a2i^2+d4i^2-norm(P15)^2)/(2*a2i*d4i)); % Joint 3 solve angle between a2i and d4i using law of cosines
t3i = phi-(pi/2); % Theta 3 angle of joint 3 (elbow style joint)

% Joint 2 uses many conditions to grab the correct angle. Since the
% solution to the law of cosines uses acos that is bounded by 0 to pi we
% need to find a way to correct for the quadrant that the joint is truly
% in. We could use atan2 to bound the solution between [-pi,pi], however
% the following approach is also valid with the set of conditional
% statements that follow that will also help provide insight as to what is
% happening with joint 2. Two variables, gamma, and psi are used to find a
% combination with pi/2 that issolates theta 2.
gamma = acos((a2i^2-d4i^2+norm(P15)^2)/(2*a2i*norm(P15))); % Joint 2 solve angle between a2i and P15
psi = acos((-norm(P05)^2+d1i^2+norm(P15)^2)/(2*d1i*norm(P15))); % Joint 2 solve angle between P15 and P05

% Part of the analysis of joint 2 requires understanding of joint 5's x
% position relative to joint 1. Is x5 on top, in front, or behind x1's
P15in1 = ROTZ(round(t1i))'*P15; % See joint 5 position from perspective of joint 1 (rotational perspective only. Not including position of set of d1i)
X5 = P15in1(1); % relative X position of joint 5 to joint 1


% The following conditions help us understand where theta 2 should be
% selected from for joint 2
if abs(gamma+psi-pi)<0.001 % Check that joint 2 isn't straight up or down
    t2i=0; % Theta 2 angle of joint 2 (elbow style joint)
else
    if round(pi-psi,3)~=0 % Check if the psi component of the solution to joint 2 is equal to pi
        if X5>0 % Check if the x position of joint 5 is more than the x of joint 1
            psi = -pi+psi; % if so then the psi component of the solution is as such
        elseif X5<0 % Check if the x position of joint 5 is less than the x of joint 1
            psi = pi-psi; % if so then the psi component of the solution is as such
        end
    end
    % We round to the 3rd decimal for the following operations to allow for wiggle room in the
    % precision of the calculations. The servos and potentiometers can only be
    % so accurate, and these calculations are prone to minor errors.
    if round(t3i,3) < 0 % check if Joint 3 is negative
        gamma = gamma; % if so the gamma component to the joint 2 solution is normal gamma
    elseif round(t3i,3)>0 % check if Joint 3 is positive
        gamma = -gamma; % if so the gamma component to the joint 2 solution is negative gamma
    elseif round(t3i,3)==round(pi/2,3) || round(t3i,3)==round(-pi/2,3) % if Joint 3 is equal to positive or negative 90 degrees
        gamma = 0; % then gamma should be zero. If not already zero set it to zero
    elseif round(t3i,3)==0 % if Joint 3 is exactly equal to zero
        if X5>0 % Check if the x position of joint 5 is more than the x of joint 1
            gamma=gamma; % if so the gamma component to the joint 2 solution is normal gamma
        end
        if X5<0 % Check if the x position of joint 5 is less than the x of joint 1
            gamma=-gamma;  % if so the gamma component to the joint 2 solution is negative gamma
        end
    end
    t2i = gamma + psi; % Theta 2 angle of joint 2 (elbow style joint)
end

% Joint 5 requires an understanding of the position of joint 3. We will use
% forward kinematics with the joints 1 to 3 that we previously obtained to
% get P03 and P3ef, position vectors relative to the base that describe the
% angles near joint 5 and 3
A0i = eye(4); % Base Frame
A1i = tranzse3(d1i)*rotzse3(t1i); % Frame from joint 1 to the base frame
A2i = rotxse3(pi/2)*rotzse3(t2i); % Frame from joint 2 to joint 1
A3i = rotzse3(pi/2)*tranxse3(a2i)*rotzse3(t3i); % Frame from joint 3 to joint 2
A01i = A0i*A1i; % Frame from joint 1 to the base frame
A02i = A01i*A2i; % Frame from joint 2 to the base frame
A03i = A02i*A3i; % Frame from joint 3 to the base frame
P03 = A03i(1:3,4); % Position vector from joint 3 to the base frame
P3ef = P0ef-P03; % Position vectro from joint 3 to the end effector relative to the base frame
cost5i = real(((d4i^2 + d6i^2 - norm(P3ef)^2)/(2*d4i*d6i))); % The cos(theta 5) using the law of cosines between d4i and d6i
% We use the law sin^2 + cos^2 = 1 to solve for sin (sin = sqrt(1-cos^2). Then divide cos by
% this answer to get sin/cos = tan. thus we can use atan2 as such to get
% the output theta5 bounded between [-pi,pi]. However there are two
% possible solutions for theta 5 that depend on theta 4 and 6 which we
% don't have yet. We will do an initial guess where theta 5 is the solution
% to atan2 - pi. the second solution is pi - atan2 which we will test if
% the solution to theta 4 and 6 yield the wrong final frame.
t5i = atan2(real(sqrt(1-(cost5i^2))),cost5i)-pi; % the solutio to theta 5, the rotation of the 5th joint (elbow style joint)

% Joint 4 can have near infinite amount of solutions respective of joint 5
% and 6. One such solution is to observe the frame transform between joint
% 4 and end the end effector. We can isolate this by taking the
% transformation from the base with the angles 1 to 3 that we have
% previously issolated up to joint 3. Then we orient the frame from frame 3
% relative to the base into the orientation of frame 4 before it would be
% rotated by theta 4. Thus we have A03i*rotxse3(pi/2), which is the next
% step in the forward kinematics to frame 4 before rotating by theta 4.
% With the given Frame, which is really the frame of the end effector
% relative to the base, we can left multiply it by the inverse of this new
% frame we issolated at joint 4 to get the joint 4 frame relative to the
% end effector
A4efi = real((A03i*rotxse3(pi/2))\Frame);

% With this frame we can observe the angle between the X and Y values of
% the joint 4 frame and end effector frame and use atan2 to resolve the
% angle
if norm(cross(P3ef,P5ef))<0.001 % Check that the P3ef and P5ef vectors are not parallel
    t4i = 0; % if they are parallel, then we won't assign any value to theta 4. We will assign it to theta 6 instead (forearm style joint)
else
    t4i = atan2(A4efi(2,4),A4efi(1,4)); % theta 4, We will find the angle between the arm of d6i and the position of frame 4's x axis to get the 4th joint (forearm style joint)
end

% With theta 4 we will correct the orientation of the robot to coincide
% with the given Frame input using theta 6. We will use forward kinematics
% with all the joints 1 to 5 that we know now and use the result to find
% the difference in angle between the given Frame and our frame at joint 6
% relative to the base.
A4i = rotxse3(pi/2)*rotzse3(t4i); % Frame from joint 4 to joint 5
A5i = tranzse3(d4i)*rotxse3(-pi/2)*rotzse3(t5i); % Frame from joint 5 to joint 6
A04i = A03i*A4i; % Frame from joint 4 to the base frame
A05i = A04i*A5i; % Frame from joint 5 to the base frame

% We add the rotation of rotxse3(pi/2) to get the frame from 5 to the frame at 6 before rotating by theta 6.
% Then using the resultant frame we will take the inverse and multiply it
% with the given frame to obtain the frame transform between frame 6 and
% the end effector. The rotation matrix of this result should be a z
% rotation and thus the position in (1,1) is cos(theta6) and in (2,1) is
% sin(theta6) with this we can use atan2 to get theta6 out of the result.
A6efi = real((A05i*rotxse3(pi/2))\Frame); % solution to the forward kinematics up to joint 6 and inverse multiplied between the given frame
t6i = atan2(A6efi(2,1),A6efi(1,1)); % the joint angle theta 6 (forearm style joint)

% We will do a single sure pass test to see if the solution to theta 4,5,
% and 6, agree with the input desired frame. First we will find the
% complete forward kinematics with out Joints 1 to 6 up to the end effector
% and then check if the resultant frame relative to the base is about equal to
% the given Frame.
A6i = rotxse3(pi/2)*rotzse3(t6i); % Frame from joint 6 to joint ef
Aei = tranzse3(d6i); % Frame from ef at 6 to ef at claw
A06i = A05i*A6i; % Frame from joint 6 to joint ef at 6
A0Efi = A06i*Aei; % Frame from joint efat6 to joint ef claw

% Test if the found frame is about equal to the given frame
if abs(sum(sum(round(A0Efi(1:3,1:3)*Frame(1:3,1:3)',3)-eye(3))))>0.1 %Observe the absolute difference of the sum of all elements between the rotation matrices multiplied together (one inverted) subtract the identity.
    % if they are not close to equal than use the second solution to theta 5
    % and find a new theta 4 and 6.
    t5i = pi + atan2(real(sqrt(1-cost5i^2)),cost5i);

    % The same steps for 4 and 6 as above
    A4efi = real((A03i*rotxse3(pi/2))\Frame);
    if norm(cross(P3ef,P5ef))<0.001
        t4i = 0;
    else
        t4i = atan2(A4efi(2,4),A4efi(1,4))+pi;
    end
    A4i = rotxse3(pi/2)*rotzse3(t4i);
    A5i = tranzse3(d4i)*rotxse3(-pi/2)*rotzse3(t5i);
    A04i = A03i*A4i;
    A05i = A04i*A5i;
    A6efi = real((A05i*rotxse3(pi/2))\Frame);
    t6i = atan2(A6efi(2,1),A6efi(1,1));
    A6i = rotxse3(pi/2)*rotzse3(t6i);
    Aei = tranzse3(d6i);
    A06i = A05i*A6i;
    A0Efi = A06i*Aei;

end

% if debugging you can use the following to plot helpful vectors in 3D
% space
if plotit==true
    subplot(1,2,1) % Forward Kinematics results
    hold on
    plotv3(A0i(1:3,4),A01i(1:3,4))
    plotv3(A01i(1:3,4),A02i(1:3,4))
    plotv3(A02i(1:3,4),A03i(1:3,4))
    plotv3(A03i(1:3,4),A04i(1:3,4))
    plotv3(A04i(1:3,4),A05i(1:3,4))
    plotv3(A05i(1:3,4),A06i(1:3,4))
    plotv3(A06i(1:3,4),A0Efi(1:3,4))
    plotv3([0;0;0],P0ef,'m')
    plotv3([0;0;0],P03,'b')
    plotv3([0;0;0],P05,'r')
    subplot(1,2,2) % Inverse Kinematics results
    hold on
    plotv3(A0i(1:3,4),A01i(1:3,4))
    plotv3(A01i(1:3,4),A02i(1:3,4))
    plotv3(A02i(1:3,4),A03i(1:3,4))
    plotv3(A03i(1:3,4),A04i(1:3,4))
    plotv3(A04i(1:3,4),A05i(1:3,4))
    plotv3(A05i(1:3,4),A06i(1:3,4))
    plotv3(A06i(1:3,4),A0Efi(1:3,4))
    plotv3([0;0;0],P0ef,'m')
    plotv3([0;0;0],P03,'b')
    plotv3([0;0;0],P05,'r')
    plotv3(P05,[0;0;d1i],"#D95319")
end

% Bound all solutions between [-pi,pi]
if t1i>pi
    t1i = t1i-2*pi;
end
if t2i>pi
    t2i = t2i-2*pi;
end
if t3i>pi
    t3i = t3i-2*pi;
end
if t4i>pi
    t4i = t4i-2*pi;
end
if t5i>pi
    t5i = t5i-2*pi;
end
if t6i>pi
    t6i = t6i-2*pi;
end
% Round and generate the output
angles = round(real([t1i;t2i;t3i;t4i;t5i;t6i]),4);

end

