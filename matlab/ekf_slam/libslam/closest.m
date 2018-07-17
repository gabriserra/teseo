function [dist, idx] = closest(vec,mat)
%CLOSEST Determines the index of closest vector to the one provided in
% input as range-and-bearing coordinates.
%
% In:
%   vec:    the basic vector expressed in range-and-bearing coordinates
%   mat:    the complete list of range-and-bearing coordinates
% Out:
%   dist:   the minimum distance of vec from any point in mat
%   idx:    the index of the closest point in mat
%
%   Notice: all vectors are expected to be column vectors.

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

% Pointers to range and bearing values
r = 1;
b = 2;

[~,m] = size(mat);

if m == 0
    % If the list is empty then the distance is infinite and the index is
    % invalid
    idx = 0;
    dist = inf;
    return;
end

% Preallocation
distances = zeros(1,m);

for i = 1:m
    % Square of the distance calculated in polar coordinates
    distances(i) = vec(r)^2 + mat(r,i)^2 - 2 * vec(r) * mat(r,i) * cos(vec(b) - mat(b,i));
end

[dist, idx] = min(distances);

% Actual distance is the square root of the calculated one
dist = sqrt(dist);

end

