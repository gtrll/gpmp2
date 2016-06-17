function set3DPlotRange(dataset)
%SET3DPLOTRANGE Set figure axis range for 3D dataset 
%
%   Usage: SET3DPLOTRANGE(dataset)
%   @dataset    dataset output by generate3Ddataset

x1 = dataset.origin_x;
x2 = dataset.cols * dataset.cell_size + dataset.origin_x;
y1 = dataset.origin_y;
y2 = dataset.rows * dataset.cell_size + dataset.origin_y;
z1 = dataset.origin_z;
z2 = dataset.z * dataset.cell_size + dataset.origin_z;

axis([x1 x2 y1 y2 z1 z2]);

end

