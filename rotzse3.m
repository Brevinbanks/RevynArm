% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 4x4 se3 rotation matrix about z given an angle psi
% Input
%   psi - angle in radians of desired rotation
% Output
%   Tf - a 4x4 se3 matrix of a rotation about the z axis
function Tf = rotzse3(psi)
    Tf = [cos(psi) -sin(psi) 0 0; 
        sin(psi) cos(psi) 0 0; 
        0 0 1 0;
        0 0 0 1];
end