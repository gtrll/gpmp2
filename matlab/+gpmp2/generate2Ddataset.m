function dataset = generate2Ddataset(dataset_str)
%GENERATE2DDATASET Generate 2D dataset evidence grid
%
%   Usage: dataset = GENERATE2DDATASET(dataset_str)
%   @dataset_str       dataset string, existing datasets:
%                      'OneObstacleDataset', 'TwoObstaclesDataset'
%
%   Output Format:
%   dataset.map        ground truth evidence grid
%   dataset.rows       number of rows (y)
%   dataset.cols       number of cols (x)
%   dataset.origin_x   origin of map x
%   dataset.origin_y   origin of map y
%   dataset.cell_size  cell size


% dataset 5: 1 obs dataset for 2D Arm obs avoid
if strcmp(dataset_str, 'OneObstacleDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.origin_x = -1;
    dataset.origin_y = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle([190, 160], [60, 80], dataset.map);
    
% dataset 6: obs dataset for 3D Arm obs avoid
elseif strcmp(dataset_str, 'TwoObstaclesDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.origin_x = -1;
    dataset.origin_y = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle([200, 200], [80, 100], dataset.map);
    dataset.map = add_obstacle([160, 80], [30, 80], dataset.map);
    
% dataset 7: multiple obs dataset for 2D Arm obs avoid
elseif strcmp(dataset_str, 'MultiObstacleDataset')
    % params
    dataset.cols = 400; %x
    dataset.rows = 300; %y
    dataset.origin_x = -20;
    dataset.origin_y = -10;
    dataset.cell_size = 0.1;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle(get_center(12,10,dataset), get_dim(5,7,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(-7,10,dataset), get_dim(10,7,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(0,-5,dataset), get_dim(10,5,dataset), dataset.map);

% mobile 2d map
elseif strcmp(dataset_str, 'MobileMap1')
    % params
    dataset.cols = 500; %x
    dataset.rows = 500; %y
    dataset.origin_x = -5;
    dataset.origin_y = -5;
    dataset.cell_size = 0.01;
    % map
    dataset.map = zeros(dataset.rows, dataset.cols);
    % obstacles
    dataset.map = add_obstacle(get_center(0,0,dataset), get_dim(1,5,dataset), dataset.map);
%     dataset.map = add_obstacle(get_center(-2.5,-2,dataset), get_dim(5,1,dataset), dataset.map);
    % wall
    dataset.map = add_obstacle(get_center(0,4.5,dataset), get_dim(10,1,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(0,-4.5,dataset), get_dim(10,1,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(4.5,0,dataset), get_dim(1,10,dataset), dataset.map);
    dataset.map = add_obstacle(get_center(-4.5,0,dataset), get_dim(1,10,dataset), dataset.map);

    
% no such dataset
else
    error('No such dataset exist');
end

end

function [map, landmarks] = add_obstacle(position, size, map, landmarks, origin_x, origin_y, cell_size)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col) ...
    = ones(2*half_size_row+1, 2*half_size_col+1); 

% landmarks
if nargin == 7
    for x = position(1)-half_size_row-1 : 4 : position(1)+half_size_row-1
        y = position(2)-half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        y = position(2)+half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
    
    for y = position(2)-half_size_col+3 : 4 : position(2)+half_size_col-5
        x = position(1)-half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        x = position(1)+half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
end

end

function center = get_center(x,y,dataset)

center = [y - dataset.origin_y, x - dataset.origin_x]./dataset.cell_size;

end

function dim = get_dim(w,h,dataset)

dim = [h, w]./dataset.cell_size;

end
