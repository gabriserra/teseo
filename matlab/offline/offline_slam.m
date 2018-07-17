function offline_slam(filename)
% OFFLINE_SLAM Execute EKF_SLAM algorithm on logged data.
%
% This function is based on the system developed by Joan Sola to use it
% with our modified version of his slam algorithm and to iterate on the
% data logged by teseo.slx during its execution.
%
% In:
%   filename:   the name of the log file to be loaded

% Any part of code that can be taken back to the original implementation by
% Joan Sola is protected by his original copyright.
%
% (c) 2010, 2011, 2012 Joan Sola
%
% Copyright (c) 2018, Gabriele Ara, Gabriele Serra
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% * Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
% 
% * Redistributions in binary form must reproduce the above copyright notice,
%   this list of conditions and the following disclaimer in the documentation
%   and/or other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% I. INITIALIZATION

% Maximum number of landmarks at any time
N = 16;

% I.2 ESTIMATOR VARIABLES

%   x: state vector's mean
x = zeros(3 + 2*N, 1);

%   P: state vector's covariances matrix
P = zeros(numel(x),numel(x));

%   m: number of active landmarks for each iteration
m = 0;

%   landcounters: counters used to decide which landmarks will still be
%   active at each iteration
landcounters  = zeros(1,N);

% Loading robot positions and observations from log file
[robot, rho] = loaddata(filename);

% Iteration counter
iter = 1;

% We now place the robot in the map using first iteration data
r           = [1 2 3];          % set robot pointer
x(r)        = robot(iter, :)';	% initialize robot states
P(r,r)      = 0;                % initialize robot covariance

% I.3 GRAPHICS -- use the variable names of simulated and estimated
%   variables, followed by a capital G to indicate 'graphics'.
%   NOTE: the graphics code is long but absolutely necessary.

% Set figure and axes for Map
mapFig = figure(1); % create figure
cla                 % clear axes
axis([-1 5 -1 5])   % set axes limits
axis square         % set 1:1 aspect ratio

% Robot shape
Rshape0 = .08 * ...
    [...
    2 -1 -1 2; ...
    0 1 -1 0];                      % a triangle at the origin
Rshape = fromFrame(x(r), Rshape0);  % a triangle at the robot pose

% Estimated robot, blue triangle
rG = line(...
    'linestyle','-',...
    'marker','none',...
    'color','b',...
    'xdata',Rshape(1,:),...
    'ydata',Rshape(2,:));

% Estimated landmark means, blue crosses
lG = line(...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',[ ],...
    'ydata',[ ]);

% II. TEMPORAL LOOP

%   q: position gaussian noise specification
q = [.01; .02];      % amplitude or standard deviation
%   s: observation gaussian noise specification
s = [.1;  1*pi/180];  % amplitude or standard deviation

% Angles used for LIDAR observations
Nangles = size(rho, 2);
alpha = (0:Nangles-1);
alpha = alpha * pi * 2 / Nangles;

for iter = 2:size(robot,1)
    
    % II.1 SIMULATOR
    % a. motion
    % Motion is now estimated as difference from previous and current robot
    % position as detected by the odometry
    dxy = toFrame(x(r), robot(iter, 1:2)');
    dt  = robot(iter, 3) - x(3);
    
    % u is composed by motion along x axis and angular motion
    u = [dxy(1); dt];
    
    % b. observations
    Y = [rho(iter, :); alpha];
    
    indexes = isfinite(Y(1, :));
    Y = Y(:, indexes);
    
    % We remove some observations that are too far for the sake of clarity
    % of the plot, this way landmarks will change more often
    indexes = abs(Y(1, :)) < 0.8;
    Y = Y(:, indexes);
    
    % Random permutation, used to add randomness to the choice of the
    % landmarks
    Y = Y(:,randperm(size(Y,2)));
    
    % We now have a set of observed points stored in Y, but we can't know
    % in advance to which landmark they will be associated. The algorithm
    % shall take care of that for us.
    
    % SLAM algorithm
    [x, P, m, landcounters] = ekf_slam(x, P, m, landcounters, u, Y, s, q);
    
    % II.3 GRAPHICS
        
    % Estimated robot
    Rshape = fromFrame(x(r), Rshape0);
    set(rG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    
    % Active estimated landmarks
    lids = 3+1:3+m*2;   % All indices of mapped landmarks
    idxx = mod(lids,2);
    
    idx = find(~idxx);
    idy = find(idxx);
    
    lx   = x(3+idx);    % All x-coordinates
    ly   = x(3+idy);    % All y-coordinates
    set(lG, 'xdata', lx, 'ydata', ly);
    
    % Force Matlab to draw all graphic objects before next iteration
    drawnow
    
    % pause(0.1)
    
end

end