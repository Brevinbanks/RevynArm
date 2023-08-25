% Created 8/13/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% Revyn Jacobian Derivation

% We will calcuate the Jacobian of the Revyn arm relative to the Base from
% the End effector in the Base frame by using the concept of J(q) = q_dot.
% This will give us the Spatial Jacobian.
% If we consider the Forward kinematics to the End Effector, called
% grip_frame in this script, and called E in this description, we can
% derive a upper left skew symetric matrix from of each collumn of the 
% Jacobian matrix as the following for each i joint variable:

% Jskewi = dE/dqi * E^-1

% if Jskew takes the form of a 4x4 matrix, then the 6 elements that make up
% a single column of the jacobian (relating the potentially velocity
% direction and magnitude of motion for the i'th joint relative to the
% base) can be obtained from Jskew as:
% Jskewi = [0  -Wzi  Wyi Vxi;
%           Wz  0  -Wxi Vyi;
%          -Wyi  Wxi  0  Vzi;
%           0   0   0   0];
%
% Where Wxi,Wyi,and Wzi are the relative spatial angular velocities, and
% Vxi,Vyi,and Vzi are the relative spatial linear velocities for the i'th
% joint


% Initialize the symbolic variables for for the joint angles
syms q1 q2 q3 q4 q5 q6
symq = [q1,q2,q3,q4,q5,q6];

% Find E or grip_frame, the End effector relative to the base, using
% forward kinematics
grip_frame = FK_Revyn(symq,7);
% Find the inverse of the end effector forward kinematics
grip_frameinv = Finv(grip_frame);
% Calculate each column for the upper left semi skew jacobian matrices
% i'th joints.
Jskew1 = diff(grip_frame,symq(1))*grip_frameinv
Jskew2 = diff(grip_frame,symq(2))*grip_frameinv
Jskew3 = diff(grip_frame,symq(3))*grip_frameinv
Jskew4 = diff(grip_frame,symq(4))*grip_frameinv
Jskew5 = diff(grip_frame,symq(5))*grip_frameinv
Jskew6 = diff(grip_frame,symq(6))*grip_frameinv

% Unskew the upper left semi symmetric matrix and the linear velocities to
% form the 6x6 standard spatial jacobian
J = sym(zeros(6,6));
J(1:6,1) = [Jskew1(1,4); Jskew1(2,4); Jskew1(3,4); Jskew1(3,2); Jskew1(1,3); Jskew1(2,1)];
J(1:6,2) = [Jskew2(1,4); Jskew2(2,4); Jskew2(3,4); Jskew2(3,2); Jskew2(1,3); Jskew2(2,1)];
J(1:6,3) = [Jskew3(1,4); Jskew3(2,4); Jskew3(3,4); Jskew3(3,2); Jskew3(1,3); Jskew3(2,1)];
J(1:6,4) = [Jskew4(1,4); Jskew4(2,4); Jskew4(3,4); Jskew4(3,2); Jskew4(1,3); Jskew4(2,1)];
J(1:6,5) = [Jskew5(1,4); Jskew5(2,4); Jskew5(3,4); Jskew5(3,2); Jskew5(1,3); Jskew5(2,1)];
J(1:6,6) = [Jskew6(1,4); Jskew6(2,4); Jskew6(3,4); Jskew6(3,2); Jskew6(1,3); Jskew6(2,1)];
% jacobian, J could be simplified using matlab symplify for the symbolic
% toolbox, however the process takes an excessive amount of time and memory
% so this has been skipped for my purposes.

% This jacobian has been implimented as the Jac_Revyn function that can
% find the jacobian for any given set of joint angles.
% This jacobian can be used to create velocity potential ellipsoids,
% observe the determinant to see how close the robot is to a singularity,
% or for IK iterative control, etc.
