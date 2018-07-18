function [x, P, m, landmarksc] = ekf_slam(x, P, m, landmarksc, u, Y, s, q)
% EKF_SLAM Execute SLAM algorithm iteration for unknown landmarks associations.
%
% This function is based on the system developed by Joan Sola to use it
% with our modified version of his slam algorithm and to iterate on the
% data logged by teseo.slx during its execution.
%
% InOut:
%   x:          state vector means, first three values are estimated robot
%               position, following ones are landmarks xy coordinates,
%               alternated
%   P:          state vector's covariances matrix
%   m:          number of active landmarks
%   landmarksc:	counters used to decide which landmarks will still be
%               active at each iteration
% In:
%   u:          action performed from previous iteration to next one
%   Y:          LIDAR observations, one per column, expressed as range-and-
%               bearing data
%   s:          observation gaussian noise specification
%   q:          position gaussian noise specification

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

% IMPLEMENTATION DETAILS
%
% Suppose a maximum number of landmarks equal to N, the size of each
% elemement used by this code is the following one:

% N         = maximum number of landmarks

% x         = [N+3 x  1 ]
% P         = [N+3 x N+3]
% m         = [1]
% landmarksc= [ 1  x  N ]
% u         = [ 2  x  1 ]
% Y         = [ 2  x  ? ] -> [ 2  x np ] -> np <= 360 (number of points)
% s         = [ 2  x  1 ] = [.1; 1*pi/180];
% q         = [ 2  x  1 ] = [.01;.02];

% Maximum number of landmarks
N = (size(x,1) - 3) / 2;

% Observation's covariance matrix
S = diag(s.^2);

% Robot pointer
r = [1 2 3];

% Estimate robot movement
[x,P] = slam2d_move_estimator(x,P,u,m,q);

% For each point, calculate its closes landmark with Mahalanobis distance,
% then if the distance is under a certain treshold update said landmark
% (then skip said landmark in the future)

% Number of points detected by the LIDAR
np = size(Y, 2);

% Conditions used to check visited points and landmarks during the
% execution
landmarks_corrected = false(1, m);
points_used         = false(1, np);

% Indexes of all active landmarks in x
ll = 3+1 : 3 + m*2;

% All used states, robot pose and active landmarks indexes
rm = [r, ll];

% Storing all variables used to evaluate the best association between
% points and landmarks
z       = zeros(2, m, np);
Z       = zeros(2, 2, m, np);
E_rl    = zeros(2, 5, m, np);
dist    = inf(m, np);

% For each point
for j = 1:np
    
    % For each landmark
    for i = 1:m
        Yj = Y(:, j);
        
        % Pointer to current landmark
        l = landmark_i(i);
        
        % Pointer to robot and current landmark
        rl	= [r, l'];
        
        % Compute E matrix
        [ei, E_r_ij, E_l_ij] = observe(x(r), x(l));
        E_rl_ij	= [E_r_ij , E_l_ij];                % Expectation Jacobian
        E_ij	= E_rl_ij * P(rl, rl) * E_rl_ij';
        
        % Compute innovation Gaussian: z and Z
        z_ij = Yj - ei; % This is z = y - h(x) in EKF
        
        % We need values around zero for angles:
        if z_ij(2) > pi
            z_ij(2) = z_ij(2) - 2*pi;
        end
        if z_ij(2) < -pi
            z_ij(2) = z_ij(2) + 2*pi;
        end
        
        % Innovation matrix for point j and landmark i
        Z_ij = S + E_ij;
        
        % Distances between point j and landmark i
        dist(i, j) = z_ij' * Z_ij^-1 * z_ij;
        z(:, i, j) = z_ij;
        Z(:, :, i, j) = Z_ij;
        E_rl(:, :, i, j) = E_rl_ij;
    end
end

% Now that I calculated all the distances and matrices, I can use the least
% one for each landmark to update it

% For each landmark
for i = 1:m
    % Calculate its closest point
    % This check does not check wether a point as already been used or not
    [distmin, j] = min(dist(i, :));
    
    % This landmark will be updated only if there is a sufficiently near
    % point to update it
    
    % Individual compatibility check using Mahalanobis distance
    if ~isempty(distmin) && distmin < 4
        % Pointers to robot and landmark
        l = landmark_i(i);
        rl	= [r , l'];
        
        % Compute E matrix again
        [ei, E_r_ij, E_l_ij] = observe(x(r), x(l));
        E_rl_ij          = [E_r_ij , E_l_ij];
        E_ij             = E_rl_ij * P(rl, rl) * E_rl_ij';
        
        % Compute innovation Gaussian: z and Z
        z_ij = Y(:, j) - ei;
        
        % We need values around zero for angles:
        if z_ij(2) > pi
            z_ij(2) = z_ij(2) - 2*pi;
        end
        if z_ij(2) < -pi
            z_ij(2) = z_ij(2) + 2*pi;
        end
        
        % Innovation matrix for point j and landmark i
        Z_ij = S + E_ij;
        
        % Kalman gain
        K = P(rm, rl) * E_rl_ij' * Z_ij^-1; % This is K = P*H'*Z^-1 in EKF
        
        % Map update (using pointer rm)
        x(rm)    = x(rm)    + K*z_ij;
        P(rm,rm) = P(rm,rm) - K*Z_ij*K';
        
        landmarks_corrected(i) = true;
        points_used(j) = true;
    end
    
    % Else skip landmark
end

% Now I need to pick one of remaining points and check wether I should add
% it to the set of landmarks

% Right now, we accept at most one new point for each iteration

remaining_points_idx = find(~points_used);
accepted = 0;

% For each unused point
while accepted < 1 && ~isempty(remaining_points_idx) && m < N
    j = remaining_points_idx(1);
    
    distmin = min(dist(:, j));
    
    % If the Mahalanobis distance is over a certain treshold we add the
    % new point to landmarks set
    if isempty(distmin) || distmin > 40
        Yj = Y(:, j);
        accepted = accepted+1;
        
        m               = m + 1;
        l               = landmark_i(m);
        landmarksc(m)   = 2;
        
        % New landmark Initialization
        [x(l), L_r, L_y] = invObserve(x(r), Yj);
        P(l,rm)          = L_r * P(r,rm);
        P(rm,l)          = P(l,rm)';
        P(l,l)           = L_r * P(r,r) * L_r' + L_y * S * L_y';
    end
    
    remaining_points_idx(1) = [];
end

% Update counters, to remove unused landmarks
landmarksc(landmarks_corrected) = landmarksc(landmarks_corrected) + 1;
landmarksc(~landmarks_corrected) = landmarksc(~landmarks_corrected) - 0.5;

% Maximum counter value
landmarksc(landmarksc > 2) = 2;

% Removing points from the set if the counter is below zero
remove_these = find(landmarksc < 0);
remove_these(4:end) = [];

if ~isempty(remove_these)
    
    % There are points to remove, let's scramble rows and columns so that
    % the last ones are the rows and columns associated to removed
    % landmarks and then we can set these last rows/columns to zero
    other_rows = 1:N;
    other_rows = other_rows(~ismember(other_rows, remove_these));
    
    new_order = [other_rows remove_these];
    x_new_order = [r zeros(1, N*2)];
    
    for i = 1:N
        l = landmark_i(i);
        newl = landmark_i(new_order(i));
        x_new_order(l) = newl;
    end
    
    % Scrambling columns
    x           = x(x_new_order, :);
    P           = P(x_new_order, x_new_order);
    
    landmarksc  = landmarksc(:, new_order);
    
    % Finally I need to set to zero last rows and columns
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

% Position noise covariances matrix
Q = diag(q.^2);
r = [1 2 3];

% If there are active landmarks, build the pointer to all landmarks states
if m > 0
    ll = 3 + 2*m;
    ll = 3+1 : ll;
    ll = reshape(ll', 1, numel(ll));
else
    ll = [];
end

% Prediction of robot motion
% We use legacy move function which accepted a gaussian noise, giving zero
[x(r), R_r, R_n] = move(x(r), u, zeros(2,1));

% Updating estimation of all landmarks and robot based on motion prediction
P(r,ll) = R_r * P(r,ll);
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