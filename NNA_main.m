%%
clc; 
clear all; close all;

%% 

global nvars LB UB

a=1;             % Number of Independent runs
b=1;             % Starting dimension
c=1;             % Finishing dimension

objective_function = @cost_function;


for ii=b:c
    nvars = 6;
    disp(['*************************NNA*************************',' nvars: ',num2str(nvars)]);
    mm=0;
    FF = zeros(a,1);
    NFEss = zeros(a,1);
    
    for k=1:a
        close all; 
        LB = [0,0,0.1,0,0,0.1];
        UB = [250, 1, 20, 250, 1, 10];
        
        
        [Xmin,Fmin,NFEs,FMIN,Elapsed_Time]=NNA(objective_function,LB,UB,nvars,50,100); %obj, LB, UB, PID parameter, 50 populaiton, 100 iteration
        %[Xmin,Fmin,Elapsed_Time]=PSO(objective_function,LB,UB,nvars,100,100,1.5,0.12,0.99);
        %[Xmin,Fmin,Elapsed_Time]=GA(objective_function,LB,UB,nvars,100,100,0.8);
        % INPUTS:

        % objective_function:           Objective function which you wish to minimize or maximize
        % LB:                           Lower bound of a problem
        % UB:                           Upper bound of a problem
        % nvars:                        Number of design variables
        % npop                          Population size
        % max_it:                       Maximum number of iterations

        % OUTPUTS:

        % Xmin:                         Global optimum solution
        % Fmin:                         Cost of global optimum solution
        % NFEs:                         Number of function evaluations
        % FMIN:                         Cost reduction history
        % Elapsed_Time:                 Elasped time for solving an optimization problem
        
        
        figure;
        plot(FMIN,'Linewidth',2);
        legend('NNA');
        title('F1');
        grid on;
        xlabel('Number of Iterations','FontSize',14);
        ylabel('Objective Function Value','FontSize',14);
        pause(0.1)
        
        FF(k)=Fmin;
        NFEss(k)=NFEs;
        disp(['Run:  ',num2str(k),'   Error: ',num2str(Fmin),'   NFEs: ',num2str(NFEs),'    Elapsed_Time:  ',num2str(Elapsed_Time)]);
        disp(['Run:  ',num2str(k),'   Error: ',num2str(Fmin),'    Elapsed_Time:  ',num2str(Elapsed_Time)]);
        figure;
        plot_result(Xmin);
        save('NNA_opt.mat','Xmin');
        tracking_test(Xmin);
    end
    
    Min_NFEs=min(NFEss);
    Ave_NFEs=mean(NFEss);
    Max_NFEs=max(NFEss);
    SD_NFEs=std(NFEss);

    Min=min(FF);
    Ave=mean(FF);
    Median=median(FF);
    Max=max(FF);
    SD=std(FF);
    CV=SD/Ave;

    Results=[Min Ave Median Max SD CV Min_NFEs Ave_NFEs Max_NFEs SD_NFEs];
    
    disp('=================Result================');
    disp(['Min:   ',num2str(Min)]);
    disp(['Ave:   ',num2str(Ave)]);
    disp(['Median:',num2str(Median)]);
    disp(['Max:   ',num2str(Max)]);
    disp(['SD:    ',num2str(SD)]);
    disp(['CV:    ',num2str(CV)]);
    
    
    
    if mm==0
        heaNNAr={''  'Best Sol' 'Average Sol' 'Median Sol' 'Worse Sol' 'SD Sol' 'CV Sol' 'Best NFEs' 'Average NFEs' 'Worse NFEs' 'SD NFEs'};
        mm=mm+1;
    end
    
    g='B';
    f=num2str(ii+1);
    f=strcat(g,f);
    xlswrite('NNA.xls',heaNNAr,hh);
    xlswrite('NNA.xls',Results,hh,f);
    FF=FF';
    xlswrite('NNA.xls',FF,hh,'B6');
    clear FF
    
end