% Created 8/13/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function produces a 3x3 skew matrix from a 3x1 vector of the
% following form [0 -w3 w2;
%                w3 0 -w1;
%               -w2 w1 0];
% Input
%   vec - a 3x1 vector
% Output
%   skewmat - a 3x3 skew symmetrix matrix
function skewmat = skew3(vec) 

skewmat = [0 -vec(3) vec(2); vec(3) 0 -vec(1); -vec(2) vec(1) 0];

end
