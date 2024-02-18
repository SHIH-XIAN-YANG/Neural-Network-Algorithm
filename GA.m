%% GA PID
function [X_opt, fval, Elapsed_time] = GA(cost_function, LB, UB, nvars, popSize, max_iter,crossOver)
options = optimoptions( ...
             'ga', ...                                    % 最佳化算法
             'PopulationSize', popSize, ...                    % 染色體數量
             'MaxGenerations', max_iter, ...                   % 最大繁衍代數
             'PlotFcn', {@gaplotbestf}, ...               % 繪圖函數
             'CrossoverFraction', crossOver, ...                % 交配率
             'Display', 'iter');                          % 結果展示方式

tic
[X_opt, fval] = ga(@cost_function, nvars, [], [], [], [], LB, UB, [], [], options);
Elapsed_time = toc;
disp(['Error: ',num2str(fval),'   Elapsed Time: ',num2str(Elapsed_time)]);
