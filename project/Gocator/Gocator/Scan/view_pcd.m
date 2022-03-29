%ptCloud = pcread('Point_Cloud_Gocator_2022_03_01_18_44_12_3.pcd');
%pcshow(ptCloud);

folder = 'scan9/';
current_folder = strcat('C:\Users\Gabriele\Documents\GitHub\computer-vision\project\Gocator\Gocator\Scan\', folder);

files = dir(strcat(folder, '*.pcd'));

for file = files'
    fprintf(1, '%s\n', file.name);
    ptCloud = pcread(strcat(current_folder, file.name));
    pcshow(ptCloud);
    pause;
end