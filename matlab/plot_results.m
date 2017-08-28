subplot(1, 3, 1)
plot( ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,4) +.8*rand(640,1)-0.4, 'r.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,7) +.8*rand(640,1)-0.4, 'b.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,10)+.8*rand(640,1)-0.4, 'g.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,13)+.8*rand(640,1)-0.4, 'm.', ...
  newresults(:,2)+.02*rand(640,1)-0.01, newresults(:,5)+.8*rand(640,1)-0.4, 'ko')

subplot(1, 3, 2)
plot( ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,5), 'r.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,8), 'b.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,11), 'g.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,14), 'm.', ...
  newresults(:,2)+.02*rand(640,1)-0.01, newresults(:,6), 'ko')

subplot(1, 3, 3)
plot( ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,6), 'r.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,9), 'b.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,12), 'g.', ...
  results(:,2)+.02*rand(640,1)-0.01, results(:,15), 'm.', ...
  newresults(:,2)+.02*rand(640,1)-0.01, newresults(:,7), 'ko')

fig = gcf;
fig.PaperPositionMode = 'auto'
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

%rsummary.ols = zeros(1, 40);
%rsummary.all_t = zeros(1, 40);
%rsummary.all_r = zeros(1, 40);
%rsummary.all_i = zeros(1, 40);
%rsummary.pct_t = zeros(1, 40);
%rsummary.pct_r = zeros(1, 40);
%rsummary.pct_i = zeros(1, 40);
%rsummary.sigma_t = zeros(1, 40);
%rsummary.sigma_r = zeros(1, 40);
%rsummary.sigma_i = zeros(1, 40);
%rsummary.dyn_t = zeros(1, 40);
%rsummary.dyn_r = zeros(1, 40);
%rsummary.dyn_i = zeros(1, 40);
%rsummary.hmrf_t = zeros(1, 40);
%rsummary.hmrf_r = zeros(1, 40);
%rsummary.hmrf_i = zeros(1, 40);
%rsummary.hmrf_is = zeros(1, 40);
%
%for frame=1:40
%  rsummary.ols(frame) = results(16*frame, 2);
%  rsummary.all_i(frame) = mean(results(16*frame-15:16*frame, 4));
%  rsummary.all_t(frame) = mean(results(16*frame-15:16*frame, 5));
%  rsummary.all_r(frame) = mean(results(16*frame-15:16*frame, 6));
%  rsummary.pct_i(frame) = mean(results(16*frame-15:16*frame, 7));
%  rsummary.pct_t(frame) = mean(results(16*frame-15:16*frame, 8));
%  rsummary.pct_r(frame) = mean(results(16*frame-15:16*frame, 9));
%  rsummary.sigma_i(frame) = mean(results(16*frame-15:16*frame, 10));
%  rsummary.sigma_t(frame) = mean(results(16*frame-15:16*frame, 11));
%  rsummary.sigma_r(frame) = mean(results(16*frame-15:16*frame, 12));
%  rsummary.dyn_i(frame) = mean(results(16*frame-15:16*frame, 13));
%  rsummary.dyn_t(frame) = mean(results(16*frame-15:16*frame, 14));
%  rsummary.dyn_r(frame) = mean(results(16*frame-15:16*frame, 15));
%  rsummary.hmrf_is(frame) = mean(results(16*frame-15:16*frame, 16));
%  rsummary.hmrf_i(frame) = mean(results(16*frame-15:16*frame, 17));
%  rsummary.hmrf_t(frame) = mean(results(16*frame-15:16*frame, 18));
%  rsummary.hmrf_r(frame) = mean(results(16*frame-15:16*frame, 19));
%end
%
%figure
%plot(rsummary.ols, rsummary.all_i, 'r', ...
%  rsummary.ols, rsummary.pct_i, 'g', ...
%  rsummary.ols, rsummary.sigma_i, 'b', ...
%  rsummary.ols, rsummary.dyn_i, 'm', ...
%  rsummary.ols, rsummary.hmrf_i, 'k')
%legend('All', 'Percent', 'Sigma', 'Dynamic', 'HMRF')
%xlabel('Overlap')
%ylabel('ICP iterations')
%title('ICP iterations')
%
%figure
%plot(rsummary.ols, rsummary.all_t, 'r', ...
%  rsummary.ols, rsummary.pct_t, 'g', ...
%  rsummary.ols, rsummary.sigma_t, 'b', ...
%  rsummary.ols, rsummary.dyn_t, 'm', ...
%  rsummary.ols, rsummary.hmrf_t, 'k')
%legend('All', 'Percent', 'Sigma', 'Dynamic', 'HMRF')
%xlabel('Overlap')
%ylabel('translation error')
%title('Translation error')
%
%figure
%plot(rsummary.ols, rsummary.all_t, 'r', ...
%  rsummary.ols, rsummary.pct_t, 'g', ...
%  rsummary.ols, rsummary.sigma_t, 'b', ...
%  rsummary.ols, rsummary.dyn_t, 'm', ...
%  rsummary.ols, rsummary.hmrf_t, 'k')
%legend('All', 'Percent', 'Sigma', 'Dynamic', 'HMRF')
%xlabel('Overlap')
%ylabel('rotation error')
%title('Rotation error')
