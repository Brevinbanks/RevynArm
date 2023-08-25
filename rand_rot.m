% Created 8/15/2023 By Brevin Banks
% Modified 8/15/2023 By Brevin Banks
% Create a random unit vector for an axis of rotation. This uses the
% exp_rot_gen function
% Input
%   none
% Output
%   Rand_R - a randomly generated 3x3 rotation matrix
function Rand_R = rand_rot()

mat_found = false;

while(mat_found==false)
    % Generate a random unit vector
    x = rand(3,1);
    x = x./norm(x);
    
    % generate a random angle between [-pi,pi]
    theta = -pi +(pi+pi)*rand(1);

    % generate the matrix
    Rand_R = exp_rot_gen(x,theta);

    % Test matrix to make sure it meets the conditions of a rotation matrix
    if det(Rand_R) == 1 && sum(sum(abs(Rand_R\Rand_R-eye(3)))) == 0 && sum(sum(abs(Rand_R'*Rand_R-eye(3)))) == 0
        mat_found = true;
    end
end