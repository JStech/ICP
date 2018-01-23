function [tf, elapsed] = goicp(ref, src, params)
  tf = eye(4); elapsed = 0;
  src = (params.t_init * src')';
  % write ref points to tmp_ref.txt
  tmp_ref = fopen('tmp_ref.txt', 'w');
  fprintf(tmp_ref, '%d\n', sum(~isnan(ref(:,1))));
  for i=1:length(ref)
    if isnan(ref(i, 1))
      continue
    end
    fprintf(tmp_ref, '%d %d %d\n', ref(i, 1), ref(i, 2), ref(i, 3));
  end
  fclose(tmp_ref);

  % write src points to tmp_src.txt in random order
  tmp_src = fopen('tmp_src.txt', 'w');
  fprintf(tmp_src, '%d\n', sum(~isnan(src(:,1))));
  p = randperm(length(src));
  for i=1:length(src)
    if isnan(src(p(i), 1))
      continue
    end
    fprintf(tmp_src, '%d %d %d\n', src(p(i), 1), src(p(i), 2), src(p(i), 3));
  end
  fclose(tmp_src);

  % run it
  cmd = 'timeout 150 ./GoICP tmp_ref.txt tmp_src.txt 1000 Go-ICP-config.txt tmp_output.txt';
  s = system(cmd);
  if s > 0
    tf = nan(4);
    elapsed = nan;
    delete tmp_*.txt
    return;
  end

  % read results
  tmp_output = fopen('tmp_output.txt', 'r');
  elapsed = fscanf(tmp_output, '%f', 1);
  R = fscanf(tmp_output, '%f', [3,3])';
  T = fscanf(tmp_output, '%f', [3,1]);
  fclose(tmp_output);
  tf = eye(4);
  tf(1:3,1:3) = R;
  tf(1:3,4) = T;
  tf = tf * params.t_init;
  delete tmp_*.txt
end
