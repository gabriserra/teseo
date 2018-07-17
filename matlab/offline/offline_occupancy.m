function map = offline_occupancy(filename, width, height, resolution)
% OFFLINE_OCCUPANCY Generate an occupancy map using logged data.
%
% This function is based on the class available on MathWorks website called
% "Implement Simultaneous Localization And Mapping (SLAM) with Lidar Scans"
% which can be reached easily using the following link:
%   http://bit.ly/matlab_slam
%
% In:
%   filename:   the name of the log file to be loaded
%   width:      the width of the map in meters
%   height:     the height of the map in meters
%   resolution: the resolution of the map, higher resolutions provide
%               better results, recommended value is 20
% Out:
%   map:        the generated map

% Any part of code that can be taken back to the original implementation by
% MathWorks is protected by his original copyright.
%
% Copyright 2017 The MathWorks, Inc.
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

% Loading robot positions and observations from log file
[robot, rho] = loaddata(filename);

% Creating the map object
map = robotics.OccupancyGrid(width, height, resolution);

% Figure handle used to visualize the map
figureHandle    = figure('Name', 'Map');
axesHandle      = axes('Parent', figureHandle);
mapHandle       = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');

% Number of iterations
Niter   = size(robot, 1);

% Angles associated to LIDAR observations
Nangles = size(rho, 2);
angles  = (0:Nangles-1);
angles  = angles * pi * 2 / Nangles;

% Offset of the robot, used because the occupancy map does not accept any
% negative value
OFFSET = [2 2 0];

for iter = 1:Niter
    
    % Current robot position
    robotPose = robot(iter, :) + OFFSET;

    % LIDAR observations
    ranges = rho(iter, :);
    modScan = lidarScan(ranges, angles);

    % Insert LIDAR range observation in the map
    % The last argument is maximum distance to be used when infinite value
    % is provided
    insertRay(map, robotPose, modScan, 2);

    % Visualize the map after every 25-th update.
    if ~mod(iter,150)%25)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(iter)]);
        drawnow
    end
end

mapHandle.CData = occupancyMatrix(map);
title(axesHandle, ['OccupancyGrid: Update ' num2str(iter)])

end