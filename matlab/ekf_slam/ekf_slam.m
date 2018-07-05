function [x, P, m, landmarks, landmarksc] = slam2d_unknown(x, P, m, landmarks, landmarksc, u, Y)

% CONSTANTS
persistent s S r;

if isempty(s)
    % Measurement noise: Gaussian {0,S}
    s = [.1;1*pi/180];  % amplitude or standard deviation
    S = diag(s.^2);     % covariances matrix
    
    r = [1 2 3];
end
% END CONSTANTS

[x,P] = slam2d_move_estimator(x,P,u,m,landmarks);

% For each point, calculate its closes landmark with Mahalanolis distance,
% then if the distance is under a certain treshold update said landmark
% (then skip said landmark in the future)

num_landmarks = numel(m)
num_points = size(Y, 2);

landmarks_corrected = false(1, num_landmarks);
points_used = false(1, num_points);

% Indexes of all active landmarks in x
landmarks_active = landmarks(:, m);
landmarks_active = reshape(landmarks_active', 1, numel(landmarks_active));

% a. create dynamic map pointers to be used hereafter
rm = [r, landmarks_active];	% all used states: robot and landmarks

accepted = false;

z = zeros(2, num_landmarks, num_points);
Z = zeros(2, 2, num_landmarks, num_points);
E_rl = zeros(2, 5, num_landmarks, num_points);
dist = inf(num_landmarks, num_points);

% For each point
for j = 1:num_points
    
    % For each landmark
    for i = 1:num_landmarks
        Yj = Y(:, j);
        
        landmark_index = m(i);
        l = landmarks(:, landmark_index);
        rl            = [r , l'];               % pointers to robot and lmk.
        
        % Compute E matrix
        [ei, E_r_ij, E_l_ij] = observe(x(r), x(l));  % this is h(x) in EKF
        E_rl_ij          = [E_r_ij , E_l_ij];          % expectation Jacobian
        E_ij             = E_rl_ij * P(rl, rl) * E_rl_ij';
        
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

for i = 1:num_landmarks
    % This check does not check wether a point as already been used or not
    [distmin, j] = min(dist(i, :));
    
    % This landmark will be updated only if there is a sufficiently near
    % point to update it
    
    % Individual compatibility check at Mahalanobis distance of 3-sigma
    % (See appendix of documentation file 'SLAM course.pdf')
    if ~isempty(distmin) && distmin < 4 % 9
        landmark_index = m(i);
        l = landmarks(:, landmark_index);
        rl            = [r , l'];              % pointers to robot and lmk.
        
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

remaining_points_idx = find(~points_used);
accepted = 0;
while accepted < 1 && ~isempty(remaining_points_idx) && num_landmarks < 32
    j = remaining_points_idx(1);
    
    [distmin] = min(dist(:, j));
    
    if isempty(distmin)
        distmin = inf;
    end
    
    if distmin > 40
        Y_NEW = Y(:, j);
        accepted = accepted+1;
        
        % Should add a new point to the landmarks set, only if its Mahalonis
        % distance with respect to previous points is greater than a certain
        % treshold

        Yj = Y_NEW;

        l = length(x);
        l = [
            l+1;
            l+2];

        landmarks(:, end+1) = l;
        landmarksc(end+1)   = 2;
        m(end+1) = size(landmarks, 2);

        % initialization
        [x(l), L_r, L_y] = invObserve(x(r), Yj);
        P(l,rm)          = L_r * P(r,rm);
        P(rm,l)          = P(l,rm)';
        P(l,l)           = L_r * P(r,r) * L_r' + L_y * S * L_y';
    end
    
    remaining_points_idx(1) = [];
end
accepted

% TODO: improve this, because it decrements all non-active landmarks if
% there are any
landmarksc(m(landmarks_corrected)) = landmarksc(m(landmarks_corrected)) + 1;
landmarksc(m(~landmarks_corrected)) = landmarksc(m(~landmarks_corrected)) - 0.5;
landmarksc(landmarksc > 2) = 2;

% if accepted
%  % Should add a new point to the landmarks set, only if its Mahalonis
%     % distance with respect to previous points is greater than a certain
%     % treshold
%     
%     Yj = Y_NEW;
%     
%     l = length(x);
%     l = [
%         l+1;
%         l+2];
%     
%     landmarks(:, end+1) = l;
%     landmarksc(end+1)   = 1;
%     m(end+1) = size(landmarks, 2);
%     
%     % initialization
%     [x(l), L_r, L_y] = invObserve(x(r), Yj);
%     P(l,rm)          = L_r * P(r,rm);
%     P(rm,l)          = P(l,rm)';
%     P(l,l)           = L_r * P(r,r) * L_r' + L_y * S * L_y';   
% end

% [val, i] = min(landmarksc);
% 
% %% TODO: change to a new value/constant
% if val == 0 && num_landmarks == 15
%     % Should remove one point, the one with index i
%     m(i) = []; % Is this enough?
% end

% Removing all unseen points
% indexes = find(landmarksc <= 0);
% if ~isempty(indexes)
%     m(ismember(m,indexes)) = [];
% end

indexes = find(landmarksc <= 0);
indexes = indexes(:,randperm(size(indexes,2)));
indexes(4:end) = [];
if ~isempty(indexes)
    m(ismember(m,indexes)) = [];
end


end

function [x,P] = slam2d_move_estimator(x,P,u,m,landmarks)

% CONSTANTS
persistent q Q r;

if isempty(q)
    % System noise: Gaussian {0,Q}
    q = [.01;.02];      % amplitude or standard deviation
    Q = diag(q.^2);     % covariances matrix
    
    r = [1 2 3];
end
% END CONSTANTS

% Indexes of all active landmarks in x
if ~isempty(m)
    landmarks_active = landmarks(:, m);
    landmarks_active = reshape(landmarks_active', 1, numel(landmarks_active));

    m = landmarks_active;
end

% a. create dynamic map pointers to be used hereafter
% rm = [r, m];	% all used states: robot and landmarks

% b. Prediction -- robot motion
[x(r), R_r, R_n] = move(x(r), u, zeros(2,1));	% Estimator perturbed with n
P(r,m) = R_r * P(r,m);                          % See PDF notes 'SLAM course.pdf'
P(m,r) = P(r,m)';
P(r,r) = R_r * P(r,r) * R_r' + R_n * Q * R_n';

end