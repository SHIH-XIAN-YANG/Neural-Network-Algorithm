function [X_opt, global_best_fitness, Elapsed_Time] = PSO(objective_function, LB, UB, nvars, popSize, max_iter, c1, c2, w)

n = popSize;          % Population Size (Swarm Size)
bird_step =max_iter;   % Maximum Number of Iterations
dim = nvars;          % Number of Decision Variables

dt = 0.3;
VarMin= LB; % Lower Bound of Variables
VarMax= UB; % Upper Bound of Variables

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
    current_fitness(i) = objective_function(current_position(:,i)); 
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
        current_fitness(i) = objective_function(current_position(:,i)) ;    
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

X_opt=globl_best_position(:,g);
current_position=globl_best_position(:,g);

disp(['Error: ',num2str(global_best_fitness),'   Elapsed Time:',num2str(Elapsed_Time)]);
end
