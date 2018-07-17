function simulation()
% SIMULATION Simulate a robot in a virtual environment.
%
% This function extends the system developed by Joan Sola to use it with
% our modified version of his slam algorithm.

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
% I.1 SIMULATED VARIABLES -- use capital letters for variable names

%   W: set of external landmarks
W = cloister(-4,4,-4,4,40);	% Type 'help cloister' for help

%   NN: number of landmarks
NN = size(W,2);

%   R: robot pose [x ; y ; alpha]
R = [0;-3.5;0];

%   U: control [d x ; d alpha]
U = [0.05 ; 0.015];   % fixing advance and turn increments creates a circle

%   N: maximum number of active landmarks at any time
N = 32;

% I.2 ESTIMATOR VARIABLES

%   x: state vector's mean
x = zeros(numel(R) + 2*N, 1);

%   P: state vector's covariances matrix
P = zeros(numel(x),numel(x));

%   m: number of active landmarks for each iteration
m = 0;

%   landcounters: counters used to decide which landmarks will still be
%   active at each iteration
landcounters  = zeros(1,N);

% We now place the robot in the map
r           = [1 2 3];  % set robot pointer
x(r)        = R;        % initialize robot states
P(r,r)      = 0;        % initialize robot covariance

% I.3 GRAPHICS -- use the variable names of simulated and estimated
%   variables, followed by a capital G to indicate 'graphics'.

% Set figure and axes for Map
mapFig = figure(1); % create figure
cla                 % clear axes
axis([-6 6 -6 6])   % set axes limits
axis square         % set 1:1 aspect ratio

% Simulated World -- set of all landmarks, red crosses
WG = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',W(1,:),...
    'ydata',W(2,:));

% Simulated robot, red triangle
Rshape0 = .2*[...
    2 -1 -1 2; ...
    0 1 -1 0]; % a triangle at the origin
Rshape = fromFrame(R, Rshape0); % a triangle at the robot pose
RG = line(...
    'linestyle','-',...
    'marker','none',...
    'color','r',...
    'xdata',Rshape(1,:),...
    'ydata',Rshape(2,:));

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

% Simulate for 2000 iterations
for t = 1:2000
    
    % II.1 SIMULATOR
    % a. motion
    n = q .* randn(2,1);            % Perturbation vector
    R = move(R, U, zeros(2,1) );    % We will perturb the estimator instead
                                    % of the simulator
    
    % b. observations
    npoints = 1;
    observeDist = 5;                % Maximum observable distance
    Y = zeros(2, NN);               % Observations preallocation
    for j = 1:NN
        v = s .* randn(2,1);        % Noise
        obs_j = W(:, j) + v;        % Observation
        dist_j = R(1:2) - obs_j;    % Distance from robot
        
        % If the point is within the maximum observable distance, add it to
        % the set
        if norm(dist_j) < observeDist
            Y(:,npoints) = observe(R, obs_j);
            npoints = npoints + 1;
        end
    end
    
    % Remove all non-observable points
    Y(:, npoints:end) = [];
    
    % Random permutation, used to add randomness to the choice of the
    % landmarks
    Y = Y(:,randperm(size(Y,2)));
    
    % We now have a set of observed points stored in Y, but we can't know
    % in advance to which landmark they will be associated. The algorithm
    % shall take care of that for us.
    
    % Notice that motion is perturbated when applying the algorithm
    [x, P, m, landcounters] = ekf_slam(x, P, m, landcounters, U+n, Y, s, q);
    
    % II.3 GRAPHICS
    
    % Simulated robot
    Rshape = fromFrame(R, Rshape0);
    set(RG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    
    % Estimated robot
    Rshape = fromFrame(x(r), Rshape0);
    set(rG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    
    % Active estimated landmarks
    lids = 3+1:3+m*2;
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