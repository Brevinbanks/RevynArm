% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 3x3 so3 rotation matrix about z given an angle psi
% Input
%   psi - angle in radians of desired rotation
% Output
%   rz - a 3x3 so3 matrix of a rotation about the z axis
function rz = rotz(psi) 

    rz = [cos(psi) -sin(psi) 0; 
        sin(psi) cos(psi) 0; 
        0 0 1];
    
end