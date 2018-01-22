res_desk_all = cell2mat(desk_results.all');
res_desk_pct = cell2mat(desk_results.pct');
res_desk_sigma = cell2mat(desk_results.sigma');
res_desk_x84 = cell2mat(desk_results.x84');
res_desk_dynamic = cell2mat(desk_results.dynamic');
res_desk_goicp = cell2mat(desk_results.goicp');
extended_hmrf_desk = cellfun(@(x) [x nan nan nan], desk_results.hmrf, ...
    'UniformOutput', false);
truncated_hmrf_desk = cellfun(@(x) x(1:4), extended_hmrf_desk, ...
    'UniformOutput', false);
res_desk_hmrf = cell2mat(truncated_hmrf_desk');
res_desk_params = cell2mat(desk_results.params');
[~, desk_overlap_sort] = sort(res_desk_params(:,3));
%desk_overlap_sort = desk_overlap_sort(50:80);
[~, desk_axis_sort] = sort(res_desk_params(:,5));

res_shark_all = cell2mat(shark_results.all');
res_shark_pct = cell2mat(shark_results.pct');
res_shark_sigma = cell2mat(shark_results.sigma');
res_shark_x84 = cell2mat(shark_results.x84');
res_shark_dynamic = cell2mat(shark_results.dynamic');
res_shark_goicp = cell2mat(shark_results.goicp');
extended_hmrf_shark = cellfun(@(x) [x nan nan nan], shark_results.hmrf, ...
    'UniformOutput', false);
truncated_hmrf_shark = cellfun(@(x) x(1:4), extended_hmrf_shark, ...
    'UniformOutput', false);
res_shark_hmrf = cell2mat(truncated_hmrf_shark');
res_shark_params = cell2mat(shark_results.params');
[~, shark_overlap_sort] = sort(res_shark_params(:,3));
%shark_overlap_sort = shark_overlap_sort(30:70);
[~, shark_axis_sort] = sort(res_shark_params(:,5));

close all
%hold on
%plot(res_desk_params(desk_axis_sort,5), res_desk_hmrf(desk_axis_sort,3), 'ko')
%plot(res_desk_params(desk_axis_sort,5), res_desk_all(desk_axis_sort,3), 'r.')
%plot(res_desk_params(desk_axis_sort,5), res_desk_pct(desk_axis_sort,3), 'g.')
%plot(res_desk_params(desk_axis_sort,5), res_desk_sigma(desk_axis_sort,3), 'b.')
%plot(res_desk_params(desk_axis_sort,5), res_desk_x84(desk_axis_sort,3), 'r+')
%plot(res_desk_params(desk_axis_sort,5), res_desk_dynamic(desk_axis_sort,3), 'g+')
%plot(res_desk_params(desk_axis_sort,5), res_desk_goicp(desk_axis_sort,3), 'b+')
%legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
%title('Desk sequence, translation error');

subplot(2,3,1)
hold on
plot(res_desk_params(desk_overlap_sort,3), res_desk_hmrf(desk_overlap_sort,3), '-ko')
plot(res_desk_params(desk_overlap_sort,3), res_desk_all(desk_overlap_sort,3), '-r.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_pct(desk_overlap_sort,3), '-g.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_sigma(desk_overlap_sort,3), '-b.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_x84(desk_overlap_sort,3), '-r+')
plot(res_desk_params(desk_overlap_sort,3), res_desk_dynamic(desk_overlap_sort,3), '-g+')
plot(res_desk_params(desk_overlap_sort,3), res_desk_goicp(desk_overlap_sort,3), '-b+')
legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
title('Desk sequence, translation error');

subplot(2,3,4)
hold on
plot(res_shark_params(shark_overlap_sort,3), res_shark_hmrf(shark_overlap_sort,3), '-ko')
plot(res_shark_params(shark_overlap_sort,3), res_shark_all(shark_overlap_sort,3), '-r.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_pct(shark_overlap_sort,3), '-g.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_sigma(shark_overlap_sort,3), '-b.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_x84(shark_overlap_sort,3), '-r+')
plot(res_shark_params(shark_overlap_sort,3), res_shark_dynamic(shark_overlap_sort,3), '-g+')
plot(res_shark_params(shark_overlap_sort,3), res_shark_goicp(shark_overlap_sort,3), '-b+')
legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
title('Shark sequence, translation error');

subplot(2,3,2)
hold on
plot(res_desk_params(desk_overlap_sort,3), res_desk_hmrf(desk_overlap_sort,4), '-ko')
plot(res_desk_params(desk_overlap_sort,3), res_desk_all(desk_overlap_sort,4), '-r.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_pct(desk_overlap_sort,4), '-g.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_sigma(desk_overlap_sort,4), '-b.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_x84(desk_overlap_sort,4), '-r+')
plot(res_desk_params(desk_overlap_sort,3), res_desk_dynamic(desk_overlap_sort,4), '-g+')
plot(res_desk_params(desk_overlap_sort,3), res_desk_goicp(desk_overlap_sort,4), '-b+')
legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
title('Desk sequence, rotation error');

subplot(2,3,5)
hold on
plot(res_shark_params(shark_overlap_sort,3), res_shark_hmrf(shark_overlap_sort,4), '-ko')
plot(res_shark_params(shark_overlap_sort,3), res_shark_all(shark_overlap_sort,4), '-r.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_pct(shark_overlap_sort,4), '-g.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_sigma(shark_overlap_sort,4), '-b.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_x84(shark_overlap_sort,4), '-r+')
plot(res_shark_params(shark_overlap_sort,3), res_shark_dynamic(shark_overlap_sort,4), '-g+')
plot(res_shark_params(shark_overlap_sort,3), res_shark_goicp(shark_overlap_sort,4), '-b+')
legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
title('Shark sequence, rotation error');

subplot(2,3,3)
hold on
plot(res_desk_params(desk_overlap_sort,3), res_desk_hmrf(desk_overlap_sort,2), '-ko')
plot(res_desk_params(desk_overlap_sort,3), res_desk_all(desk_overlap_sort,2), '-r.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_pct(desk_overlap_sort,2), '-g.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_sigma(desk_overlap_sort,2), '-b.')
plot(res_desk_params(desk_overlap_sort,3), res_desk_x84(desk_overlap_sort,2), '-r+')
plot(res_desk_params(desk_overlap_sort,3), res_desk_dynamic(desk_overlap_sort,2), '-g+')
plot(res_desk_params(desk_overlap_sort,3), res_desk_goicp(desk_overlap_sort,2), '-b+')
legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
title('Desk sequence, elapsed time');

subplot(2,3,6)
hold on
plot(res_shark_params(shark_overlap_sort,3), res_shark_hmrf(shark_overlap_sort,2), '-ko')
plot(res_shark_params(shark_overlap_sort,3), res_shark_all(shark_overlap_sort,2), '-r.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_pct(shark_overlap_sort,2), '-g.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_sigma(shark_overlap_sort,2), '-b.')
plot(res_shark_params(shark_overlap_sort,3), res_shark_x84(shark_overlap_sort,2), '-r+')
plot(res_shark_params(shark_overlap_sort,3), res_shark_dynamic(shark_overlap_sort,2), '-g+')
plot(res_shark_params(shark_overlap_sort,3), res_shark_goicp(shark_overlap_sort,2), '-b+')
legend('HMRF', 'All', 'Percent', 'Sigma', 'X84', 'Dynamic', 'GoICP')
title('Shark sequence, elapsed time');
