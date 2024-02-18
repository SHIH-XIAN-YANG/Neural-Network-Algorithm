%%
clear
clc
close all;
%% Initialisation

n = 100;          % Population Size (Swarm Size)
bird_step =100;   % Maximum Number of Iterations
dim = 6;          % Number of Decision Variables

dt = 0.3;
VarMin= [0,0,0.1,0,0,0.1]; % Lower Bound of Variables
VarMax= [500,1,20,500,1,10]; % Upper Bound of Variables

c2 = 1.5;         % Personal Learning Coefficient
c1 = 0.12;        % Global Learning Coefficient
w =0.99;          % Inertia Weight Damping Ratio
fitness=zeros(1,bird_step); 

% Velocity Limits
VelMax=1;
VelMin=-VelMax;

current_fitness =zeros(n,1);
globl_best_position = zeros(dim,n);

% Initialisation position
current_position = ((VarMax - VarMin)').*rand(dim,n) + VarMin';
velocity = 0.3*randn(dim, n) ; %Normally distributed with mean=0 standard deviation=0.3
local_best_position  = current_position ;    

% evaluate the initial population
for i = 1:n
    current_fitness(i) = cost_function(current_position(:,i)); 
end


local_best_fitness  = current_fitness ;
[global_best_fitness,g] = min(local_best_fitness) ;

for i=1:n
    globl_best_position(:,i) = local_best_position(:,g) ;
end


%  VELOCITY UPDATE  %
velocity = w *velocity...
    + c1*((rand(dim, n)).*(local_best_position-current_position))...
    + c2*((rand(dim, n)).*(globl_best_position-current_position));

% Apply Velocity Limits
velocity = max(velocity, VelMin);
velocity = min(velocity, VelMax);
                       
% swarm update            
current_position = current_position + velocity*dt ;

% Velocity Mirror Effect if over the boundary the velocity nagative
for i=1:3    
    IsOutside=(current_position(i)<VarMin(i) | current_position(i)>VarMax(i));
    if IsOutside
       velocity(i) = -velocity(i);
    end
end

% Apply Position Limits
for i=1:3
    current_position(i) = max(current_position(i),VarMin(i));
    current_position(i) = min(current_position(i),VarMax(i));
end
                                               

%% Main Loop
iter = 0 ; 
tic;
while  ( iter < bird_step )
    iter = iter + 1;

    for i = 1:n
        current_fitness(i) = cost_function(current_position(:,i)) ;    
    end


    for i = 1 : n
        if current_fitness(i) < local_best_fitness(i)
           local_best_fitness(i)  = current_fitness(i);  
           local_best_position(:,i) = current_position(:,i)   ;
        end   
    end


    [current_global_best_fitness,g] = min(local_best_fitness);
    fitness(iter) = current_global_best_fitness;

    if current_global_best_fitness < global_best_fitness
       global_best_fitness = current_global_best_fitness;

        for i=1:n
            globl_best_position(:,i) = local_best_position(:,g);
        end
    end



     %  VELOCITY UPDATE  
     velocity = w *velocity...
         + c1*((rand(dim, n)).*(local_best_position-current_position))...
         + c2*((rand(dim, n)).*(globl_best_position-current_position));

     % Apply Velocity Limits
     velocity = max(velocity, VelMin);
     velocity = min(velocity, VelMax);

     % SWARMUPDATE 
     current_position = current_position + velocity*dt; 

     % Velocity Mirror Effect if over the boundary the velocity nagative
     for i=1:3    
        IsOutside=(current_position(i)<VarMin(i) | current_position(i)>VarMax(i));
        if IsOutside
            velocity(i) = -velocity(i);
        end
     end

     % Apply Position Limits
     for i=1:3
        current_position(i) = max(current_position(i),VarMin(i));
        current_position(i) = min(current_position(i),VarMax(i));
     end
     %current_position
     disp(['iter: ', num2str(iter),' fitness: ',num2str(current_global_best_fitness)]);
     %sprintf('The value of interation iter %3.0f ', iter );
     %disp('The value of interation iter %3.0f\n');

end % end of while loop its mean the end of all step that the birds move it 
Elapsed_Time = toc;

xx=globl_best_position(:,g);
current_position=globl_best_position(:,g);
xx = abs(globl_best_position(:,g));
figure;
plot(1:bird_step,fitness,'Linewidth',2);
grid on;
xlabel('iteration');
ylabel('fitness');

disp(['Error: ',num2str(global_best_fitness),'   Elapsed Time:',num2str(Elapsed_Time)]);

%% plot the result of global best

 plot_result(xx);
%tracking_test(xx);
