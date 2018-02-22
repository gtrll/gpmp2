function ImG = getGoalImage(x_target, axes, img_size)

    goal_rad = 5; % in pixels

    x_lim = axes.XLim; y_lim = axes.YLim;
    c_gpix = ceil(img_size(1) * (x_target(1) - x_lim(1))/(x_lim(2)-x_lim(1)));
    r_gpix = ceil(img_size(2) * (x_target(2) - y_lim(1))/(y_lim(2)-y_lim(1)));
    y_g = img_size(1) - r_gpix + 1;
    x_g = c_gpix;

    ImG = zeros(img_size(1),img_size(2));
   
    x=1:img_size(2);
    y=1:img_size(1);
    [xx,yy] = meshgrid(x,y);

    ImG((xx-x_g).^2+(yy-y_g).^2 < goal_rad^2) = 1.0;

end
