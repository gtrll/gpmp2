function h = plotSignedDistanceField2D(field, origin_x, origin_y, cell_size, epsilon_dist)
%PLOTSIGNEDDISTANCEFIELD2D plot 2D SignedDistanceField
%
%   Usage: PLOTSIGNEDDISTANCEFIELD2D(field, origin_x, origin_y, cell_size, epsilon_dist)
%   @field                  field matrix
%   @origin_x, origin_y     origin (down-left) corner of the map
%   @cell_size              cell size
%   @epsilon_dist           optional plot obstacle safety distance, default = 0

if nargin < 5
    epsilon_dist = 0;
end

% get X-Y coordinates
grid_rows = size(field, 1);
grid_cols = size(field, 2);
grid_corner_x = origin_x + (grid_cols-1)*cell_size;
grid_corner_y = origin_y + (grid_rows-1)*cell_size;
grid_X = origin_x : cell_size : grid_corner_x;
grid_Y = origin_y : cell_size : grid_corner_y;

h = imagesc(grid_X, grid_Y, field);

colormap(hsv(64))

% colormap(gray(64))
% caxis([min(min(field)), epsilon_dist]);

set(gca,'YDir','normal')

axis equal
axis([origin_x-cell_size/2, grid_corner_x+cell_size/2, ...
    origin_y-cell_size/2, grid_corner_y+cell_size/2])

colorbar

xlabel('X/m')
ylabel('Y/m')


end

