function [Xoptimum,Objective,NFEs,FMIN,Elapsed_Time]=NNA(objective_function,LB,UB,nvars,npop,max_it)
% INPUTS:

% objective_function:           Objective function which you wish to minimize or maximize
% LB:                           Lower bound of a problem
% UB:                           Upper bound of a problem
% nvars:                        Number of design variables
% npop                          Population size
% max_it:                       Maximum number of iterations

% OUTPUTS:

% Xoptimum:                     Global optimum solution
% Objective:                    Cost of global optimum solution
% NFEs:                         Number of function evaluations
% FMIN:                         Cost reduction history
% Elapsed_Time:                 Elasped time for solving an optimization problem

%% ==========================================================================
% Default values for the NNA
format long g
if (nargin <5 || isempty(npop)), npop=50; end
if (nargin <6 || isempty(max_it)), max_it=round((5000*nvars)/npop); end
%==========================================================================
%% --------------------Initialization----------------------------------------
X_LB=repmat(LB,npop,1);
X_UB=repmat(UB,npop,1);

beta=1;
x_pattern=zeros(npop,nvars);
cost=zeros(npop,1);

% Creat random initial population
for i=1:npop
    x_pattern(i,:)=LB+((UB-LB).*rand(1,nvars));
    cost(i)=objective_function(x_pattern(i,:));
end

[COST,index]=min(cost); %get initial best wieght and bias
%--------------------------------------------------------------------------
% Creat random initial weights with constraint of Summation each column = 1
ww=ones(1,npop)*0.5;
w=diag(ww);
for i=1:npop
    t=rand(1,npop-1)*0.5;
    t=(t./sum(t))*0.5; %nomalize to sum(t) = 0.5 
    w(w(:,i)==0,i)=t;  %assign value to index that not on daignal position
end
%--------------------------------------------------------------------------
% w=rand(npop,npop);       %% An alternative way of initializing weights
% for i=1:npop
%     w(:,i)=w(:,i)./sum(w(:,i));       % Summation of each column = 1
% end
%--------------------------------------------------------------------------
XTarget=x_pattern(index,:);   % Best obtained solution
Target=COST;                  % Best obtained objetive function value
wtarget=w(:,index);           % Best obtained weight (weight target) ??

%% -------------------- Main Loop for the NNA -------------------------------
FMIN=zeros(max_it,1);
tic
for ii=1:max_it
    
    %------------------ Creating new solutions ----------------------------
    x_new=w*x_pattern;
    x_pattern=x_new+x_pattern;
    %------------------- Updating the weights -----------------------------
    for i=1:npop
        w(:,i)=abs(w(:,i)+((wtarget-w(:,i))*2.*rand(npop,1)));
    end
    
    for i=1:npop
        w(:,i)=w(:,i)./sum(w(:,i));    % Summation of each column = 1
    end
    
    %----------------------- Creat new input solutions --------------------
    for i=1:npop
        
        if rand<beta
            
            %------------- Bias for input solutions -----------------------
            N_Rotate=ceil(beta*nvars);
            
            xx=LB+(UB-LB).*rand(1,nvars);
            rotate_postion=randperm(nvars);rotate_postion=rotate_postion(1:N_Rotate);
            
            for m=1:N_Rotate
                x_pattern(i,rotate_postion(m))=xx(m);
            end
            %---------- Bias for weights ----------------------------------
            N_wRotate=ceil(beta*npop);
            
            w_new=rand(N_wRotate,npop);
            rotate_position=randperm(npop);rotate_position=rotate_position(1:N_wRotate);
            
            for j=1:N_wRotate
                w(rotate_position(j),:)=w_new(j,:);
            end
            
            for iii=1:npop
                w(:,iii)=w(:,iii)./sum(w(:,iii));   % Summation of each column = 1
            end
        else
            %------------ Transfer Function Operator ----------------------
            x_pattern(i,:)=x_pattern(i,:)+(XTarget-x_pattern(i,:))*2.*rand(1,nvars);
        end
    end
    % ---------------------- Bias Reduction -------------------------------
    beta=beta*0.99;
    
    if beta<0.01
        beta=0.05; 
    end
    
    % beta=1-((1/max_it)*ii);        % An alternative way of reducing the value of beta
    %----------------------------------------------------------------------
    x_pattern=max(x_pattern,X_LB);    x_pattern=min(x_pattern,X_UB);     % Check the side constraints
    %-------------- Calculating objective function values -----------------
    for i=1:npop
        cost(i)=objective_function(x_pattern(i,:));
    end
    %% ------ Selection ---------------------------------------------------
    [FF,Index]=min(cost);
    
    if FF<Target % Update X_target and W_target
        Target=FF;
        XTarget=x_pattern(Index,:);
        wtarget=w(:,Index);
    else
        [~,Indexx]=max(cost);  % small change here 
        x_pattern(Indexx,:)=XTarget;
        w(:,Indexx)=wtarget;
    end
    
    %% Display
    disp(['Iteration: ',num2str(ii),'   Objective= ',num2str(Target),'   beta= ',num2str(beta)]);
    FMIN(ii)=Target;
end
%% -------------------------------- NNA Finishes ----------------------------
plot(FMIN,'Linewidth',3);
xlabel('Number of Iterations','FontSize',24,'FontWeight','bold');
ylabel('Objective Function Value','FontSize',24,'FontWeight','bold');
Xoptimum=XTarget;
Objective=objective_function(Xoptimum);  % Best obtained optimum solution
NFEs=npop*max_it;
Elapsed_Time=toc;
end