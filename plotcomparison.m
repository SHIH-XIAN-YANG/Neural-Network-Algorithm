clc;clear; close all;

PSO_opt = load('PSO_opt.mat');
GA_opt = load('GA_opt.mat');
NNA_opt = load('NNA_opt.mat');

for round=1:3
    if round==1
        PID = PSO_opt.Xmin;
    elseif round == 2
        PID = GA_opt.Xmin;
    else
        PID = NNA_opt.Xmin;
    end
    theta_r = [1;
               1];
    %theta_r = get_ref_trajectory();
    m1 = 0.1;%(kg)
    m2 = 0.1;%(kg)
    l1 = 0.8;%(m)
    l2 = 0.4;%(m)
    g = 9.81;%m/s^2

    dt = 0.001; %sampling time
    
    
     Kp = [PID(1), 0;
            0,  PID(4)];
    Ki = [PID(2), 0;
            0, PID(5)];
    Kd = [PID(3), 0;
               0, PID(6)];   
      
    t = 0:dt:5;
    
    arr_length = length(t);
    e = zeros(2,arr_length);
    sum_e = zeros(2,arr_length);
    torque = zeros(2,arr_length);
    theta = zeros(2,arr_length);
    dtheta = zeros(2,arr_length);
    ddtheta = zeros(2,arr_length);
    M = zeros(2,2,arr_length);
    C = zeros(2,1,arr_length);
    G = zeros(2,1,arr_length);
    P = zeros(2,arr_length);
    I = zeros(2,arr_length);
    D = zeros(2,arr_length);
    
    
    
    %% main loop
    e(:,1) = theta_r - theta(:,1);
    sum_e(:,1) = e(:,1)*dt;
    torque(:,1) = Kp*e(:,1) + Kd*(e(:,1)-e(:,1))/dt + Ki*sum_e(:,1);
    k=2;
    P(:,1) = Kp*e(:,1);
    I(:,1) = Ki*sum_e(:,1);
    D(:,1) = Kd*(e(:,1)-e(:,1))/dt;
    
    for t=dt:dt:(arr_length-1)*dt

        e(:,k) = theta_r - theta(:,k-1);
        sum_e(:,k) = sum_e(:,k-1) + (e(:,k))*dt;
        P(:,k) = Kp*e(:,k);
        I(:,k) = Ki*sum_e(:,k);
        D(:,k) = Kd*(e(:,k)-e(:,k-1))/dt;
        
        torque(:,k) = Kp*e(:,k) + Kd*(e(:,k)-e(:,k-1))/dt + Ki*sum_e(:,k);
        

        c1 = cos(theta(1,k-1));
        c2 = cos(theta(2,k-1));
        %s1 = sin(theta(1,k-1));
        s2 = sin(theta(2,k-1));

        c12 = cos(theta(1,k-1) + theta(2,k-1));
        %s12 = sin(theta(1,k-1) + theta(2,k-1));

        M(:,:,k-1) =  [(m1 + m2)*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*c2,   m2*l2*l2 + m2*l1*l2*c2;
                    m2*l2*l2 + m2*l1*l2*c2,                         m2*l2*l2];

        C(:,:,k-1) = [-2*m2*l1*l2*s2*dtheta(1,k-1)*dtheta(2,k-1) - ...
                        m2*l1*l2*s2*dtheta(2,k-1)*dtheta(2,k-1);
                             m2*l1*l2*c2*dtheta(1,k-1)*dtheta(1,k-1)];

        G(:,:,k-1) =  [(m1+m2)*g*l1*c1 + m2*g*l2*c12;
                                m2*g*l1*c12];

%         ddtheta(:,k) = inv(M(:,:,k-1))*(torque(k-1) - C(:,:,k-1)- G(:,:,k-1));
        ddtheta(:,k) = M(:,:,k-1)\(torque(:, k)-C(:,:,k-1)-G(:,:,k-1));

        dtheta(:,k) = dtheta(:,k-1) + (ddtheta(:,k) )*dt;

        theta(:,k) = theta(:,k-1) + (dtheta(:,k) )*dt;
        
        k=k+1;
    end
    %% plot result theta
    t = 0:dt:(arr_length-1)*dt;
    hold on;
    %subplot(2,1,1);
    %{
    plot(t,(theta(1,:)),'Linewidth',1.5);
    title('q1');
    xlabel('t(s)'); ylabel('$\theta(rad)$','Interpreter','Latex');
    %ylim([-0.2 1.2]);
    ylim([0 2]);
    grid on;
    legend('PSO-PID','GA-PID','NNA-PID');
    %}
    
    
    %subplot(2,1,2);
    
    plot(t,(theta(2,:)),'Linewidth',1.5);
    title('q2');
    xlabel('t(s)'); ylabel('$\theta(rad)$','Interpreter','Latex');
    ylim([-0.2 1.2]);
    grid on;
    legend('PSO-PID','GA-PID','NNA-PID');
    
    %{
    plot(t,e(1,:));
    title('error(rad)');
    xlabel('t(s)'); 
    legend('PSO-PID','GA-PID','NNA-PID');
    grid on;
    %}
    %{
    plot(t,e(2,:));
    title('error(rad)');
    xlabel('t(s)'); 
    legend('PSO-PID','GA-PID','NNA-PID');
    grid on;
    %}
end


%plot(t,theta_r(2,:));
%ylim([0 2]);
%grid on;
%legend('theta1','theta2');
%xlabel('t'); ylabel('$\theta_1, \theta_2(rad)$','Interpreter','Latex');
