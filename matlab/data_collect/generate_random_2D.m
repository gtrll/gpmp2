function dataset = generate_random_2D()

cols = 300; rows = 300;
dataset.cols = cols;
dataset.rows = rows;
dataset.cell_size = 0.01;
dataset.origin_x = -rows*dataset.cell_size/2;
dataset.origin_y = -cols*dataset.cell_size/2;

dataset.map = zeros(dataset.rows, dataset.cols);
rect1 = random_rect(dataset.rows, dataset.cols, 1, 1);
rect2 = random_rect(dataset.rows, dataset.cols, rows/2+1, cols/2+1);
dataset.map(rect1(1):rect1(2), rect1(3):rect1(4)) = 1;
dataset.map(rect2(1):rect2(2), rect2(3):rect2(4)) = 1;

end

function rect = random_rect(rows, cols, start_x, start_y)

% Returns [x_bot, x_top, y_left, y_right]

height = randi([20, rows/2]);
width = randi([20, cols/2]);
x_bot = randi([start_x, rows-height+1]);
y_left = randi([start_y, cols-width+1]);

rect = [x_bot, x_bot+height, y_left, y_left+width];
end