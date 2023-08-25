% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 4x4 se3 translation transformation matrix 
% along y given a distance beta
% Input
%   beta - desired distance to translate in y
% Output
%   Tf - a 4x4 se3 matrix of a translation
function Tf = tranyse3(beta)
    Tf = [1 0 0 0;
          0 1 0 beta; 
         0 0 1  0;
          0 0 0 1];
end