function [tf, elapsed] = s4pcs(ref, src, overlap, params)
  tf = nan(4); elapsed = NaN;
  src = (params.t_init * src')';
  % write ref points to tmp_ref.ply and src points to tmp_src.ply
  pcwrite(pointCloud(ref(:,1:3)), 'tmp_ref.ply', 'PLYFormat', 'ascii');
  pcwrite(pointCloud(src(:,1:3)), 'tmp_src.ply', 'PLYFormat', 'ascii');

  % run it
  cmd = sprintf('./Super4PCS -i tmp_ref.ply tmp_src.ply -o %f -n 200 -d 0.01 -t 150 -m tmp_mat.txt', overlap);
  tic
  [s, ~] = system(cmd);
  elapsed = toc;
  if s > 0
    delete tmp_*.ply
    return;
  end

  % read results
  tmp_output = fopen('tmp_mat.txt', 'r');
  fgets(tmp_output);
  fgets(tmp_output);
  tf = fscanf(tmp_output, '%f', [4,4]);
  fclose(tmp_output);
  tf = tf' * params.t_init;
  delete tmp_*.txt
end
