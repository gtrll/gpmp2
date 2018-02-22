function saveData(x,u,Im,Im_obst,Im_goal,case_path)

    iter = [1:size(x,2)]'; 
    data = [iter,x',u'];

    % Save state, control data
    header = '%% iter, q_1, q_2, q_3, u_1, u_2, u3 \n ';
    fid = fopen(fullfile(case_path,'traj_data.txt'),'w');
    fprintf(fid,header);
    fmt = repmat('%8.5f, ',1,size(data,2));
    fmt = fmt(1:end-2);
    fmt = strcat(fmt,'\n');
    fprintf(fid,fmt,data');
    fclose(fid);
    
    % Save Goal 
    img_file = fullfile(case_path,'goal.png');
    imwrite(Im_goal,img_file);
    
    % Save Obstacles
    Im_obst = im2bw(Im_obst); %#ok<IM2BW>
    img_file = fullfile(case_path,'obstacles.png');
    imwrite(Im_obst,img_file);
    
    % Save Images
    img_path = fullfile(case_path,'img');
    assert(~exist(img_path,'dir'));
    mkdir(img_path);   
    for i=1:size(x,2)
        img_file = fullfile(img_path,sprintf('%05d.png',i));
        imwrite(Im(:,:,:,i),img_file);
    end
    

end