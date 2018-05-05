close all
hmrf_style = 'ko';
all_style = 'r.';
pct_style = 'g.';
sigma_style = 'b.';
x84_style = 'c.';
dynamic_style = 'm.';
goicp_style = 'r+';
s4pcs_style = 'b+';
r_clip = 1.0;
t_clip = 0.6;
e_clip = 20;

if exist('shark_results')
  res_shark_all = cell2mat(shark_results.all');
  res_shark_pct = cell2mat(shark_results.pct');
  res_shark_sigma = cell2mat(shark_results.sigma');
  res_shark_x84 = cell2mat(shark_results.x84');
  res_shark_dynamic = cell2mat(shark_results.dynamic');
  res_shark_goicp = cell2mat(shark_results.goicp');
  res_shark_s4pcs = cell2mat(shark_results.s4pcs');
  extended_hmrf_shark = cellfun(@(x) [x nan nan nan], shark_results.hmrf, ...
      'UniformOutput', false);
  truncated_hmrf_shark = cellfun(@(x) x(1:4), extended_hmrf_shark, ...
      'UniformOutput', false);
  res_shark_hmrf = cell2mat(truncated_hmrf_shark');
  res_shark_params = cell2mat(shark_results.params');

  % clip values
  res_shark_all(res_shark_all(:,3)>t_clip, 3) = t_clip;
  res_shark_pct(res_shark_pct(:,3)>t_clip, 3) = t_clip;
  res_shark_sigma(res_shark_sigma(:,3)>t_clip, 3) = t_clip;
  res_shark_x84(res_shark_x84(:,3)>t_clip, 3) = t_clip;
  res_shark_dynamic(res_shark_dynamic(:,3)>t_clip, 3) = t_clip;
  res_shark_goicp(res_shark_goicp(:,3)>t_clip, 3) = t_clip;
  res_shark_s4pcs(res_shark_s4pcs(:,3)>t_clip, 3) = t_clip;
  res_shark_hmrf(res_shark_hmrf(:,3)>t_clip, 3) = t_clip;
  res_shark_all(res_shark_all(:,4)>r_clip, 4) = r_clip;
  res_shark_pct(res_shark_pct(:,4)>r_clip, 4) = r_clip;
  res_shark_sigma(res_shark_sigma(:,4)>r_clip, 4) = r_clip;
  res_shark_x84(res_shark_x84(:,4)>r_clip, 4) = r_clip;
  res_shark_dynamic(res_shark_dynamic(:,4)>r_clip, 4) = r_clip;
  res_shark_goicp(res_shark_goicp(:,4)>r_clip, 4) = r_clip;
  res_shark_s4pcs(res_shark_s4pcs(:,4)>r_clip, 4) = r_clip;
  res_shark_hmrf(res_shark_hmrf(:,4)>r_clip, 4) = r_clip;
  res_shark_all(res_shark_all(:,2)>e_clip, 2) = e_clip;
  res_shark_pct(res_shark_pct(:,2)>e_clip, 2) = e_clip;
  res_shark_sigma(res_shark_sigma(:,2)>e_clip, 2) = e_clip;
  res_shark_x84(res_shark_x84(:,2)>e_clip, 2) = e_clip;
  res_shark_dynamic(res_shark_dynamic(:,2)>e_clip, 2) = e_clip;
  res_shark_goicp(res_shark_goicp(:,2)>e_clip, 2) = e_clip;
  res_shark_s4pcs(res_shark_s4pcs(:,2)>e_clip, 2) = e_clip;
  res_shark_hmrf(res_shark_hmrf(:,2)>e_clip, 2) = e_clip;

  figure
  subplot(1,3,1)
  hold on
  plot(res_shark_params(:,3), res_shark_hmrf(:,3), hmrf_style)
  plot(res_shark_params(:,3), res_shark_all(:,3), all_style)
  plot(res_shark_params(:,3), res_shark_pct(:,3), pct_style)
  plot(res_shark_params(:,3), res_shark_sigma(:,3), sigma_style)
  plot(res_shark_params(:,3), res_shark_x84(:,3), x84_style)
  plot(res_shark_params(:,3), res_shark_dynamic(:,3), dynamic_style)
  plot(res_shark_params(:,3), res_shark_goicp(:,3), goicp_style)
  plot(res_shark_params(:,3), res_shark_s4pcs(:,3), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('Shark sequence, translation error');
  %ylim([0, 1]);

  subplot(1,3,2)
  hold on
  plot(res_shark_params(:,3), res_shark_hmrf(:,4), hmrf_style)
  plot(res_shark_params(:,3), res_shark_all(:,4), all_style)
  plot(res_shark_params(:,3), res_shark_pct(:,4), pct_style)
  plot(res_shark_params(:,3), res_shark_sigma(:,4), sigma_style)
  plot(res_shark_params(:,3), res_shark_x84(:,4), x84_style)
  plot(res_shark_params(:,3), res_shark_dynamic(:,4), dynamic_style)
  plot(res_shark_params(:,3), res_shark_goicp(:,4), goicp_style)
  plot(res_shark_params(:,3), res_shark_s4pcs(:,4), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('Shark sequence, rotation error');
  %ylim([0, 1]);


  subplot(1,3,3)
  hold on
  plot(res_shark_params(:,3), res_shark_hmrf(:,2), hmrf_style)
  plot(res_shark_params(:,3), res_shark_all(:,2), all_style)
  plot(res_shark_params(:,3), res_shark_pct(:,2), pct_style)
  plot(res_shark_params(:,3), res_shark_sigma(:,2), sigma_style)
  plot(res_shark_params(:,3), res_shark_x84(:,2), x84_style)
  plot(res_shark_params(:,3), res_shark_dynamic(:,2), dynamic_style)
  plot(res_shark_params(:,3), res_shark_goicp(:,2), goicp_style)
  plot(res_shark_params(:,3), res_shark_s4pcs(:,4), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('Shark sequence, elapsed time');
  %ylim([0, 12]);
end

if exist('desk_results')
  res_desk_all = cell2mat(desk_results.all');
  res_desk_pct = cell2mat(desk_results.pct');
  res_desk_sigma = cell2mat(desk_results.sigma');
  res_desk_x84 = cell2mat(desk_results.x84');
  res_desk_dynamic = cell2mat(desk_results.dynamic');
  res_desk_goicp = cell2mat(desk_results.goicp');
  res_desk_s4pcs = cell2mat(desk_results.s4pcs');
  extended_hmrf_desk = cellfun(@(x) [x nan nan nan], desk_results.hmrf, ...
      'UniformOutput', false);
  truncated_hmrf_desk = cellfun(@(x) x(1:4), extended_hmrf_desk, ...
      'UniformOutput', false);
  res_desk_hmrf = cell2mat(truncated_hmrf_desk');
  res_desk_params = cell2mat(desk_results.params');

  % clip values
  res_desk_all(res_desk_all(:,3)>t_clip, 3) = t_clip;
  res_desk_pct(res_desk_pct(:,3)>t_clip, 3) = t_clip;
  res_desk_sigma(res_desk_sigma(:,3)>t_clip, 3) = t_clip;
  res_desk_x84(res_desk_x84(:,3)>t_clip, 3) = t_clip;
  res_desk_dynamic(res_desk_dynamic(:,3)>t_clip, 3) = t_clip;
  res_desk_goicp(res_desk_goicp(:,3)>t_clip, 3) = t_clip;
  res_desk_s4pcs(res_desk_s4pcs(:,3)>t_clip, 3) = t_clip;
  res_desk_hmrf(res_desk_hmrf(:,3)>t_clip, 3) = t_clip;
  res_desk_all(res_desk_all(:,4)>r_clip, 4) = r_clip;
  res_desk_pct(res_desk_pct(:,4)>r_clip, 4) = r_clip;
  res_desk_sigma(res_desk_sigma(:,4)>r_clip, 4) = r_clip;
  res_desk_x84(res_desk_x84(:,4)>r_clip, 4) = r_clip;
  res_desk_dynamic(res_desk_dynamic(:,4)>r_clip, 4) = r_clip;
  res_desk_goicp(res_desk_goicp(:,4)>r_clip, 4) = r_clip;
  res_desk_s4pcs(res_desk_s4pcs(:,4)>r_clip, 4) = r_clip;
  res_desk_hmrf(res_desk_hmrf(:,4)>r_clip, 4) = r_clip;
  res_desk_all(res_desk_all(:,2)>e_clip, 2) = e_clip;
  res_desk_pct(res_desk_pct(:,2)>e_clip, 2) = e_clip;
  res_desk_sigma(res_desk_sigma(:,2)>e_clip, 2) = e_clip;
  res_desk_x84(res_desk_x84(:,2)>e_clip, 2) = e_clip;
  res_desk_dynamic(res_desk_dynamic(:,2)>e_clip, 2) = e_clip;
  res_desk_goicp(res_desk_goicp(:,2)>e_clip, 2) = e_clip;
  res_desk_s4pcs(res_desk_s4pcs(:,2)>e_clip, 2) = e_clip;
  res_desk_hmrf(res_desk_hmrf(:,2)>e_clip, 2) = e_clip;

  figure
  subplot(1,3,1)
  hold on
  plot(res_desk_params(:,3), res_desk_hmrf(:,3), hmrf_style)
  plot(res_desk_params(:,3), res_desk_all(:,3), all_style)
  plot(res_desk_params(:,3), res_desk_pct(:,3), pct_style)
  plot(res_desk_params(:,3), res_desk_sigma(:,3), sigma_style)
  plot(res_desk_params(:,3), res_desk_x84(:,3), x84_style)
  plot(res_desk_params(:,3), res_desk_dynamic(:,3), dynamic_style)
  plot(res_desk_params(:,3), res_desk_goicp(:,3), goicp_style)
  plot(res_desk_params(:,3), res_desk_s4pcs(:,3), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('Desk sequence, translation error');
  %ylim([0, 1]);

  subplot(1,3,2)
  hold on
  plot(res_desk_params(:,3), res_desk_hmrf(:,4), hmrf_style)
  plot(res_desk_params(:,3), res_desk_all(:,4), all_style)
  plot(res_desk_params(:,3), res_desk_pct(:,4), pct_style)
  plot(res_desk_params(:,3), res_desk_sigma(:,4), sigma_style)
  plot(res_desk_params(:,3), res_desk_x84(:,4), x84_style)
  plot(res_desk_params(:,3), res_desk_dynamic(:,4), dynamic_style)
  plot(res_desk_params(:,3), res_desk_goicp(:,4), goicp_style)
  plot(res_desk_params(:,3), res_desk_s4pcs(:,4), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('Desk sequence, rotation error');
  %ylim([0, 1]);

  subplot(1,3,3)
  hold on
  plot(res_desk_params(:,3), res_desk_hmrf(:,2), hmrf_style)
  plot(res_desk_params(:,3), res_desk_all(:,2), all_style)
  plot(res_desk_params(:,3), res_desk_pct(:,2), pct_style)
  plot(res_desk_params(:,3), res_desk_sigma(:,2), sigma_style)
  plot(res_desk_params(:,3), res_desk_x84(:,2), x84_style)
  plot(res_desk_params(:,3), res_desk_dynamic(:,2), dynamic_style)
  plot(res_desk_params(:,3), res_desk_goicp(:,2), goicp_style)
  plot(res_desk_params(:,3), res_desk_s4pcs(:,2), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('Desk sequence, elapsed time');
  %ylim([0, 12]);
end

if exist('room_results')
  res_room_all = cell2mat(room_results.all');
  res_room_pct = cell2mat(room_results.pct');
  res_room_sigma = cell2mat(room_results.sigma');
  res_room_x84 = cell2mat(room_results.x84');
  res_room_dynamic = cell2mat(room_results.dynamic');
  res_room_goicp = cell2mat(room_results.goicp');
  res_room_s4pcs = cell2mat(room_results.s4pcs');
  extended_hmrf_room = cellfun(@(x) [x nan nan nan], room_results.hmrf, ...
      'UniformOutput', false);
  truncated_hmrf_room = cellfun(@(x) x(1:4), extended_hmrf_room, ...
      'UniformOutput', false);
  res_room_hmrf = cell2mat(truncated_hmrf_room');
  res_room_params = cell2mat(room_results.params');

  % clip values
  res_room_all(res_room_all(:,3)>t_clip, 3) = t_clip;
  res_room_pct(res_room_pct(:,3)>t_clip, 3) = t_clip;
  res_room_sigma(res_room_sigma(:,3)>t_clip, 3) = t_clip;
  res_room_x84(res_room_x84(:,3)>t_clip, 3) = t_clip;
  res_room_dynamic(res_room_dynamic(:,3)>t_clip, 3) = t_clip;
  res_room_goicp(res_room_goicp(:,3)>t_clip, 3) = t_clip;
  res_room_s4pcs(res_room_s4pcs(:,3)>t_clip, 3) = t_clip;
  res_room_hmrf(res_room_hmrf(:,3)>t_clip, 3) = t_clip;
  res_room_all(res_room_all(:,4)>r_clip, 4) = r_clip;
  res_room_pct(res_room_pct(:,4)>r_clip, 4) = r_clip;
  res_room_sigma(res_room_sigma(:,4)>r_clip, 4) = r_clip;
  res_room_x84(res_room_x84(:,4)>r_clip, 4) = r_clip;
  res_room_dynamic(res_room_dynamic(:,4)>r_clip, 4) = r_clip;
  res_room_goicp(res_room_goicp(:,4)>r_clip, 4) = r_clip;
  res_room_s4pcs(res_room_s4pcs(:,4)>r_clip, 4) = r_clip;
  res_room_hmrf(res_room_hmrf(:,4)>r_clip, 4) = r_clip;
  res_room_all(res_room_all(:,2)>e_clip, 2) = e_clip;
  res_room_pct(res_room_pct(:,2)>e_clip, 2) = e_clip;
  res_room_sigma(res_room_sigma(:,2)>e_clip, 2) = e_clip;
  res_room_x84(res_room_x84(:,2)>e_clip, 2) = e_clip;
  res_room_dynamic(res_room_dynamic(:,2)>e_clip, 2) = e_clip;
  res_room_goicp(res_room_goicp(:,2)>e_clip, 2) = e_clip;
  res_room_s4pcs(res_room_s4pcs(:,2)>e_clip, 2) = e_clip;
  res_room_hmrf(res_room_hmrf(:,2)>e_clip, 2) = e_clip;

  figure
  subplot(1,3,1)
  hold on
  plot(res_room_params(:,3), res_room_hmrf(:,3), hmrf_style)
  plot(res_room_params(:,3), res_room_all(:,3), all_style)
  plot(res_room_params(:,3), res_room_pct(:,3), pct_style)
  plot(res_room_params(:,3), res_room_sigma(:,3), sigma_style)
  plot(res_room_params(:,3), res_room_x84(:,3), x84_style)
  plot(res_room_params(:,3), res_room_dynamic(:,3), dynamic_style)
  plot(res_room_params(:,3), res_room_goicp(:,3), goicp_style)
  plot(res_room_params(:,3), res_room_s4pcs(:,3), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('room sequence, translation error');
  %ylim([0, 1]);

  subplot(1,3,2)
  hold on
  plot(res_room_params(:,3), res_room_hmrf(:,4), hmrf_style)
  plot(res_room_params(:,3), res_room_all(:,4), all_style)
  plot(res_room_params(:,3), res_room_pct(:,4), pct_style)
  plot(res_room_params(:,3), res_room_sigma(:,4), sigma_style)
  plot(res_room_params(:,3), res_room_x84(:,4), x84_style)
  plot(res_room_params(:,3), res_room_dynamic(:,4), dynamic_style)
  plot(res_room_params(:,3), res_room_goicp(:,4), goicp_style)
  plot(res_room_params(:,3), res_room_s4pcs(:,4), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('room sequence, rotation error');
  %ylim([0, 1]);


  subplot(1,3,3)
  hold on
  plot(res_room_params(:,3), res_room_hmrf(:,2), hmrf_style)
  plot(res_room_params(:,3), res_room_all(:,2), all_style)
  plot(res_room_params(:,3), res_room_pct(:,2), pct_style)
  plot(res_room_params(:,3), res_room_sigma(:,2), sigma_style)
  plot(res_room_params(:,3), res_room_x84(:,2), x84_style)
  plot(res_room_params(:,3), res_room_dynamic(:,2), dynamic_style)
  plot(res_room_params(:,3), res_room_goicp(:,2), goicp_style)
  plot(res_room_params(:,3), res_room_s4pcs(:,2), s4pcs_style)
  legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP', 'Super4PCS')
  title('room sequence, elapsed time');
  %ylim([0, 12]);
end
