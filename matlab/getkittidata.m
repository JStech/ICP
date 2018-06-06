function [cloud] = getkittidata(frame, sequence)
cloud_filename = sprintf('../data/dataset/sequences/%02d/velodyne/%06d.bin', ...
    sequence, frame-1);
fid = fopen(cloud_filename);
cloud = fread(fid, [4 1000000], 'float');
fclose(fid);
cloud = cloud(1:3, :)';
end
