% Created 8/13/2023 By Brevin Banks
% Modified 8/13/2023 By Brevin Banks
% This function draws a vector between two 3x1 vectors in 3D space in a
% live figure
% Input
%   Vec1 - a 3x1 vector from (0,0) the origin to the point you want the
%   drawn vector to start
%   Vec2 - a 3x1 vector from (0,0) the origin to the point you want the
%   drawn vector to end
%   veccollor - a matlab shorthand or hex collor value as a string
%   line_width - thickness of the desired vector line
% Output
%   NONE - check the live figure
function plotv3(Vec1, Vec2, veccollor,line_width)

% check argument dimension
[rows, cols] = size(Vec1);
if ((rows ~= 3) || (cols ~= 1))
  warning('Vector Recieved: ')
  Vec1
  error('plotv3 did not recieve a 3x1 vector for the first vector.');
end

% check argument dimension
[rows, cols] = size(Vec2);
if ((rows ~= 3) | (cols ~= 1))
  warning('Vector Recieved: ')
  Vec2
  error('plotv3 did not recieve a 3x1 vector for the second vector.');
end

% check if the input selected a vector collor
if nargin<3 
   veccollor = 'k';
end

% check if the input selected a line_width
if nargin<4
    line_width = 1;
end
% construct array of vectors
vector_vals = [Vec1'; Vec2'];

plot3(vector_vals(:,1), vector_vals(:,2), vector_vals(:,3),'Color', veccollor,'LineWidth',line_width);

