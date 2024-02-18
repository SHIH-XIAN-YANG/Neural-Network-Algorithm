%GA PID
clc; clear; close all;
%% GA PID
options = optimoptions( ...
             'ga', ...                                    % 最佳化算法
             'PopulationSize', 100, ...                    % 染色體數量
             'MaxGenerations', 100, ...                   % 最大繁衍代數
             'PlotFcn', {@gaplotbestf}, ...               % 繪圖函數
             'CrossoverFraction', 0.8, ...                % 交配率
             'Display', 'iter');                          % 結果展示方式


% options = optimoptions('MutationFcn', {@mutationuniform, 0.1})
nvar = 6;
LB = [0,0,0.1,0,0,0.1];
UB = [500, 1, 20, 500,1, 10];

fitness = zeros(100,1);

tic
[X_opt, fval] = ga(@cost_function, nvar, [], [], [], [], LB, UB, [], [], options);
Elapsed_time = toc;
disp(['Error: ',num2str(fval),'   Elapsed Time: ',num2str(Elapsed_time)]);
%% plot the result of global best

plot_result(X_opt);
tracking_test(X_opt);