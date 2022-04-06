folder = 'scan10\';
%current_folder = strcat('C:\Users\Gabriele\Documents\GitHub\computer-vision\project\Scan\', folder);
current_folder = strcat('D:\Projects\GitHub-Projects\computer-vision\project\Scan\', folder);

files = dir(strcat(folder, '*.pcd'));

for file = files'
    fprintf(1, '%s\n', file.name);
    ptCloud = pcread(strcat(current_folder, file.name));
    pcshow(ptCloud);
    pause;
end