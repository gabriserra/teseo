% Suppose a maximum number of landmarks equal to N, the size of each
% elemement used by this code is the following one:

% N         = maximum number of landmarks

% x         = [N+3 x  1 ]
% P         = [N+3 x N+3]
% m         = [ 1  x  N ] -> 1 -> number of active landmarks
% landmarksc= [ 1  x  N ]
% u         = [ 2  x  1 ]
% Y         = [ 2  x  ? ] -> [ 2  x 360] -> np = 360 (number of points)
% s         = [ 2  x  1 ] = [.1; 1*pi/180];
% q         = [ 2  x  1 ] = [.01;.02];

% S         = diag(s.^2);
% Q         = diag(q.^2);

function [x, P, m, landmarksc] = ekf_slam(x, P, m, landmarksc, u, Y, s, q)

N = (size(x,1) - 3) / 2;

S = diag(s.^2);     % covariances matrix
r = [1 2 3];

[x,P] = slam2d_move_estimator(x,P,u,m,q);

% For each point, calculate its closes landmark with Mahalanolis distance,
% then if the distance is under a certain treshold update said landmark
% (then skip said landmark in the future)

np = size(Y, 2); % Number of points

landmarks_corrected = false(1, m);
points_used = false(1, np);

% Indexes of all active landmarks in x
ll = 3+1 : 3 + m*2;

% a. create dynamic map pointers to be used hereafter
rm = [r, ll];	% all used states: robot and landmarks

% accepted = false;

z       = zeros(2, m, np);
Z       = zeros(2, 2, m, np);
E_rl    = zeros(2, 5, m, np);
dist    = inf(m, np);

% For each point
for j = 1:np
    
    % For each landmark
    for i = 1:m
        Yj = Y(:, j);
        
        %landmark_index = i;
        l = landmark_i(i);
        
        rl	= [r, l'];               % pointers to robot and lmk.
        
        % Compute E matrix
        [ei, E_r_ij, E_l_ij] = observe(x(r), x(l));  % this is h(x) in EKF
        E_rl_ij	= [E_r_ij , E_l_ij];          % expectation Jacobian
        E_ij	= E_rl_ij * P(rl, rl) * E_rl_ij';
        
        % innovation: Gaussian {z,Z}
        z_ij = Yj - ei; % this is z = y - h(x) in EKF
        % we need values around zero for angles:
        if z_ij(2) > pi
            z_ij(2) = z_ij(2) - 2*pi;
        end
        if z_ij(2) < -pi
            z_ij(2) = z_ij(2) + 2*pi;
        end
        Z_ij = S + E_ij;
        
        dist(i, j) = z_ij' * Z_ij^-1 * z_ij;
        z(:, i, j) = z_ij;
        Z(:, :, i, j) = Z_ij;
        E_rl(:, :, i, j) = E_rl_ij;
    end
end

% Now that I calculated all the distances and matrices, I can use the least
% one for each landmark to update it

for i = 1:m
    % This check does not check wether a point as already been used or not
    [distmin, j] = min(dist(i, :));
    
    % This landmark will be updated only if there is a sufficiently near
    % point to update it
    
    % Individual compatibility check at Mahalanobis distance of 3-sigma
    % (See appendix of documentation file 'SLAM course.pdf')
    if ~isempty(distmin) && distmin < 4 % 9
        
        l = landmark_i(i);
        
        rl	= [r , l'];              % pointers to robot and lmk.
        
        % Compute E matrix
        [ei, E_r_ij, E_l_ij] = observe(x(r), x(l));  % this is h(x) in EKF
        
        E_rl_ij          = [E_r_ij , E_l_ij];          % expectation Jacobian
        E_ij             = E_rl_ij * P(rl, rl) * E_rl_ij';
        
        % innovation: Gaussian {z,Z}
        z_ij = Y(:, j) - ei; % this is z = y - h(x) in EKF
        
        % we need values around zero for angles:
        if z_ij(2) > pi
            z_ij(2) = z_ij(2) - 2*pi;
        end
        if z_ij(2) < -pi
            z_ij(2) = z_ij(2) + 2*pi;
        end
        Z_ij = S + E_ij;
        
        % Kalman gain
        K = P(rm, rl) * E_rl_ij' * Z_ij^-1; % this is K = P*H'*Z^-1 in EKF
        % map update (use pointer rm)
        
        x(rm)    = x(rm)    + K*z_ij;
        P(rm,rm) = P(rm,rm) - K*Z_ij*K';
        
        landmarks_corrected(i) = true;
        points_used(j) = true;
    end
    
    % Else skip landmark
end

% Now I need to pick one of remaining points and check wether I should add
% it to the set of landmarks

% remaining_points_idx = find(~points_used);
%
% mindistances = inf(1, length(remaining_points_idx));
%
% if num_landmarks > 0
%     for idx = 1:length(remaining_points_idx)
%        j = remaining_points_idx(idx);
%        mindistances(idx) = min(dist(:, j));
%     end
%
%     [distmin,maxidx] = max(mindistances(idx));
% else
%     distmin = inf;
%     maxidx = 1;
% end
%
% % Consider using euclidean distance instead of Maholanian one
% if ~isempty(distmin) && num_landmarks < 32 && distmin > 40
%     j = remaining_points_idx(maxidx);
%     Y_NEW = Y(:, j);
%     accepted = true;
% end

%% TODO: ADD PARAMETER FOR NUMBER OF ACCEPTED POINTS
%% TODO: CHANGE ITERATION ON REMAINING_POINTS_IDX

remaining_points_idx = find(~points_used);
accepted = 0;
while accepted < 1 && ~isempty(remaining_points_idx) && m < N
    j = remaining_points_idx(1);
    
    distmin = min(dist(:, j));
    
    if isempty(distmin) || distmin > 40
        Yj = Y(:, j);
        accepted = accepted+1;
        
        % Should add a new point to the landmarks set, only if its Mahalonis
        % distance with respect to previous points is greater than a certain
        % treshold
        
        m = m+1;
        
        l = landmark_i(m);
        
        landmarksc(m) = 2;
        
        % initialization
        [x(l), L_r, L_y] = invObserve(x(r), Yj);
        P(l,rm)          = L_r * P(r,rm);
        P(rm,l)          = P(l,rm)';
        P(l,l)           = L_r * P(r,r) * L_r' + L_y * S * L_y';
    end
    
    remaining_points_idx(1) = [];
end

landmarksc(landmarks_corrected) = landmarksc(landmarks_corrected) + 1;
landmarksc(~landmarks_corrected) = landmarksc(~landmarks_corrected) - 0.5;

%% TODO: CHANGE TO A PARAMETER
landmarksc(landmarksc > 2) = 2;

%% TODO: CHANGE REMOVAL

remove_these = find(landmarksc < 0); % <= ?
remove_these(4:end) = [];

if ~isempty(remove_these)
    
    other_rows = 1:N;
    other_rows = other_rows(~ismember(other_rows, remove_these));
    
    new_order = [other_rows remove_these];
    
    x_new_order = [r zeros(1, N*2)];
    for i = 1:N % length(new_order)
        l = landmark_i(i);
        newl = landmark_i(new_order(i));
        x_new_order(l) = newl;
    end
    
    % x_new_order = 3 + new_order; %% WRONG
    
    x           = x(x_new_order, :);
    P           = P(x_new_order, x_new_order);
    
    landmarksc  = landmarksc(:, new_order); %% TODO: CHECK
    
    % Then I need to set to zero last rows and columns
    N_remove = numel(remove_these);
    remove_indexes = N-N_remove+1: N;
    x_remove_indexes = N*2+3 - (N_remove*2) +1 : N*2+3 ;
    
    
    x(x_remove_indexes, :) = 0;
    P(x_remove_indexes, :) = 0;
    P(:, x_remove_indexes) = 0;
    
    landmarksc(remove_indexes) = 0;
    
    m = m-N_remove;
end

end

function [x,P] = slam2d_move_estimator(x,P,u,m,q)

Q = diag(q.^2);     % covariances matrix
r = [1 2 3];

if m > 0
    ll = 3 + 2*m;
    ll = 3+1 : ll;
    ll = reshape(ll', 1, numel(ll));
else
    ll = [];
end

% b. Prediction -- robot motion
[x(r), R_r, R_n] = move(x(r), u, zeros(2,1));	% Estimator perturbed with n

P(r,ll) = R_r * P(r,ll);                        % See PDF notes 'SLAM course.pdf'
P(ll,r) = P(r,ll)';
P(r,r)  = R_r * P(r,r) * R_r' + R_n * Q * R_n';

end

function l = landmark_i(i)
    l = 3 + 2*(i-1);
    l = [
        l+1;
        l+2
        ];
end