format compact;

slam_dir = pwd;

p = {
    [ slam_dir '/lib']
    [ slam_dir '/libslam']
    %[ slam_dir '/simulate:']
    [ slam_dir '']
    };

for i = 1:size(p, 1)
    path(p{i},path);
end

clear p slam_dir;