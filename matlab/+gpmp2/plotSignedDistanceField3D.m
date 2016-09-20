function h = plotSignedDistanceField3D(field, origin, cell_size, epsilon_dist, marker_size)
%PLOTSIGNEDDISTANCEFIELD3D plot 3D SignedDistanceField
%
%   Usage: PLOTSIGNEDDISTANCEFIELD3D(field, origin, cell_size, epsilon_dist)
%   @field                  field 3D matrix
%   @origin                 origin of the map
%   @cell_size              cell size
%   @epsilon_dist           optional plot obstacle safety distance, default = 0
%   @marker_size            marker size, default = 0

if nargin < 4
    epsilon_dist = 0;
    marker_size = 10;
elseif nargin < 5
    marker_size = 10;
end

% get X-Y coordinates
grid_rows = size(field, 1);
grid_cols = size(field, 2);
grid_z = size(field, 3);
grid_corner_x = origin(1) + (grid_cols-1)*cell_size;
grid_corner_y = origin(2) + (grid_rows-1)*cell_size;
grid_corner_z = origin(3) + (grid_z-1)*cell_size;
grid_X = origin(1) : cell_size : grid_corner_x;
grid_Y = origin(2) : cell_size : grid_corner_y;
grid_Z = origin(3) : cell_size : grid_corner_z;

idx = find(field<epsilon_dist);
[x, y, z] = ind2sub(size(field),idx);
h = plot3(grid_X(y), grid_Y(x), grid_Z(z), '.', 'Color', [0.4, 0.4, 0.4], 'MarkerSize', marker_size);

axis equal
axis([origin(1)-cell_size/2, grid_corner_x+cell_size/2, ...
    origin(2)-cell_size/2, grid_corner_y+cell_size/2, ...
    origin(3)-cell_size/2, grid_corner_z+cell_size/2])

end
