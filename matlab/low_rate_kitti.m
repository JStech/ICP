clear
dataset = 'kitti';
load_data;
load_poses;

params = icp_params;
params.neighbor_structure = 'unstructured';
params.mode = 'hmrf';
params.verbose = true;
skip = 10;

clouds = {};
for i=1:n_kitti_clouds/skip
  clouds{i} = unproject(getcloud((i-1)*skip+1));
end

T = {eye(4)};
for i=1:n_kitti_clouds/skip-1
  % TODO: add random translation
  params.t_init = inv(pmats{(i-1)*skip+1})*pmats{i*skip+1};
  T{i} = hmrf_icp(clouds{i}, clouds{i+1}, params);
end
