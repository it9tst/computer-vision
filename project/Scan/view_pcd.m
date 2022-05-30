folder = 'scan10\';
%current_folder = strcat('C:\Users\Gabriele\Documents\GitHub\computer-vision\project\Scan\', folder);
current_folder = strcat('D:\Projects\GitHub-Projects\computer-vision\project\Scan\', folder);

files = dir(strcat(folder, '*.pcd'));

for file = files'
    fprintf(1, '%s\n', file.name);
    ptCloud = pcread(strcat(current_folder, file.name));
    pcshow(ptCloud);
    set(gcf, 'InvertHardCopy', 'off');
    %set(gcf, 'PaperPosition', [0 0 50 30]); %Position plot at left hand corner with width 5 and height 5. 
    %set(gcf, 'PaperSize', [50 30]); %Set the paper to have width 5 and height 5. 
    %saveas(gcf, 'test', 'pdf') %Save figure
    %set(gcf,'color','w');
    %set(gca, 'XColor', [0.4 0.4 0.4], 'YColor', [0.4 0.4 0.4], 'ZColor', [0.4 0.4 0.4]);
    pause;
end