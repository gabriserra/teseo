% SLAM2D A 2D EKF-SLAM algorithm with simulation and graphics.
%

% I. INITIALIZE

% I.1 SIMULATOR -- use capital letters for variable names
%   W: set of external landmarks
W = cloister(-4,4,-4,4,40);	% Type 'help cloister' for help
%   N: number of landmarks
N = size(W,2);
%   R: robot pose [x ; y ; alpha]
% R = [0;-2;0];
R = [0;-3.5;0];
%   U: control [d x ; d alpha]
U = [0.05 ; 0.015];   % fixing advance and turn increments creates a circle
%U = [0.02 ; 0.005];   % fixing advance and turn increments creates a circle

% I.2 ESTIMATOR
% Map: Gaussian {x,P}
% x: state vector's mean
x = zeros(numel(R), 1);
% P: state vector's covariances matrix
P = zeros(numel(x),numel(x));

landmarks   = [];        % See Help Note #11 above
landmarksc  = [];

% Place robot in map
r           = [1 2 3];  % set robot pointer
x(r)        = R;        % initialize robot states
P(r,r)      = 0;        % initialize robot covariance

% I.3 GRAPHICS -- use the variable names of simulated and estimated
%   variables, followed by a capital G to indicate 'graphics'.
%   NOTE: the graphics code is long but absolutely necessary.

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

% Estimated robot ellipse, magenta
reG = line(...
    'linestyle','-',...
    'marker','none',...
    'color','m',...
    'xdata',[ ],...
    'ydata',[ ]);

% Estimated landmark means, blue crosses
lG = line(...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',[ ],...
    'ydata',[ ]);

% Estimated landmark ellipses, green
leG = zeros(1,N);
for i = 1:numel(leG)
    leG(i) = line(...
        'linestyle','-',...
        'marker','none',...
        'color','g',...
        'xdata',[ ],...
        'ydata',[ ]);
end

% II. TEMPORAL LOOP

m = [];

% NOISE USED FOR POSITION
q = [.01;.02];      % amplitude or standard deviation
% NOISE USED FOR DETECTION
s = [.1;1*pi/180];  % amplitude or standard deviation

for t = 1:2000
    
    % II.1 SIMULATOR
    % a. motion
    n = q .* randn(2,1);            % perturbation vector
    R = move(R, U, zeros(2,1) );    % we will perturb the estimator
    % instead of the simulator
    
    % b. observations
    npoints = 1;
    observeDist = 5;
    Y = zeros(2, N);
    for j = 1:N
        v = s .* randn(2,1);
        obs_j = W(:, j) + v;
        dist_j = R(1:2) - obs_j;
        if norm(dist_j) < observeDist
            Y(:,npoints) = observe(R, obs_j);
            npoints = npoints + 1;
        end
    end
    
    Y(:, npoints:end) = [];
    Y = Y(:,randperm(size(Y,2)));
    
    % I now have i points detected stored in Y, but I don't know to
    % which landmarks they are associated with
    
    [x, P, m, landmarks, landmarksc] = ekf_slam(x, P, m, landmarks, landmarksc, U+n, Y);
    
    % II.3 GRAPHICS
    
    % Simulated robot
    Rshape = fromFrame(R, Rshape0);
    set(RG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    
    % Estimated robot
    Rshape = fromFrame(x(r), Rshape0);
    set(rG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    
    % Estimated robot ellipse
    re      = x(r(1:2));            % robot position mean
    RE      = P(r(1:2),r(1:2));     % robot position covariance
    %[xx,yy] = cov2elli(re,RE,3,16); % x- and y- coordinates of contour
    %set(reG, 'xdata', xx, 'ydata', yy);
    
    % Estimated landmarks
%     lids = find(landmarks(1,:)); % all indices of mapped landmarks
%     lx   = x(landmarks(1,lids)); % all x-coordinates
%     ly   = x(landmarks(2,lids)); % all y-coordinates
%     set(lG, 'xdata', lx, 'ydata', ly);
    
    % ACTIVE Estimated landmarks
    % lids = find(landmarks(1,:)); % all indices of mapped landmarks
    lids = m;
    lx   = x(landmarks(1,lids)); % all x-coordinates
    ly   = x(landmarks(2,lids)); % all y-coordinates
    set(lG, 'xdata', lx, 'ydata', ly);
    
    % Estimated landmark ellipses -- one per landmark
    for i = lids
        l       = landmarks(:,i);
        le      = x(l);
        LE      = P(l,l);
        %[xx,yy] = cov2elli(le,LE,3,16);
        %set(leG(i), 'xdata', xx, 'ydata', yy);
    end
    
    % force Matlab to draw all graphic objects before next iteration
    drawnow
    % pause(0.1)
    
end

