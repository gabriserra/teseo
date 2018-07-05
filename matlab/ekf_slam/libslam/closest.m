function [dist,closest_i] = closest(vec,mat)
%CLOSEST Determines the index of closest vector to the one provided in
%input as range-and-bearing coordinates
%   Vectors are stored as columns

r = 1;
b = 2;

[~,m] = size(mat);

if m == 0
    closest_i = 0;
    dist = inf;
    return;
end

distances = zeros(1,m);

for i = 1:m
    % Polar distance
    distances(i) = vec(r)^2 + mat(r,i)^2 - 2 * vec(r) * mat(r,i) * cos(vec(b) - mat(b,i));
end

[dist, closest_i] = min(distances);

dist = sqrt(dist);

end

