%% tracking test

function [] = tracking_test(PID)
    theta_r = get_ref_trajectory();
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
    e(:,1) = theta_r(:,1) - theta(:,1);
    sum_e(:,1) = e(:,1)*dt;
    torque(:,1) = Kp*e(:,1) + Kd*(e(:,1)-e(:,1))/dt + Ki*sum_e(:,1);
    k=2;
    P(:,1) = Kp*e(:,1);
    I(:,1) = Ki*sum_e(:,1);
    D(:,1) = Kd*(e(:,1)-e(:,1))/dt;
    
    for t=dt:dt:(arr_length-1)*dt

        e(:,k) = theta_r(:,k) - theta(:,k-1);
        sum_e(:,k) = sum_e(:,k-1) + (e(:,k))*dt;
        P(:,k) = Kp*e(:,k);
        I(:,k) = Ki*sum_e(:,k);
        D(:,k) = Kd*(e(:,k)-e(:,k-1))/dt;
        
        torque(:,k) = Kp*e(:,k) + Kd*(e(:,k)-e(:,k-1))/dt + Ki*sum_e(:,k);
        

        c1 = cos(theta(1,k-1));
        c2 = cos(theta(2,k-1));
        s1 = sin(theta(1,k-1));
        s2 = sin(theta(2,k-1));

        c12 = cos(theta(1,k-1) + theta(2,k-1));
        s12 = sin(theta(1,k-1) + theta(2,k-1));

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
    figure;
    
    subplot(2,1,1);
    
    plot(t,theta(1,:),t,theta_r(1,:));
    title('q1');
    xlabel('t'); ylabel('$\theta$','Interpreter','Latex');
    grid on;
    
    subplot(2,1,2);
    
    plot(t,theta(2,:),t,theta_r(2,:));
    title('q2');
    xlabel('t'); ylabel('$\theta$','Interpreter','Latex');
    grid on;
    figure;
    
    
    
    subplot(2,1,1);
    plot(t,rad2deg(dtheta(1,:)));
    title('dq');
    xlabel('t');
    grid on;
    subplot(2,1,2);
    plot(t,rad2deg(dtheta(2,:)));
    xlabel('t');
    grid on;
    
    figure;
    title('ddq');
    subplot(2,1,1);
    plot(t,rad2deg(ddtheta(1,:)));
    title('ddq')
    xlabel('t');
    grid on;
    subplot(2,1,2);
    plot(t,rad2deg(ddtheta(2,:)));
    
    xlabel('t'); 
    grid on;
    figure
    plot(t,rad2deg(e));
    title('error');
    xlabel('t'); 
    grid on;
    
    figure;
    subplot(2,1,1);
    plot(t,torque(1,:));
    title('torque1');
    xlabel('t');
    grid on;
    subplot(2,1,2);
    plot(t,torque(2,:));
    title('torque2');
    xlabel('t');
    grid on;
    %%
    overshoot = zeros(2,1);
    overshoot(1) = max(theta(1,:))-theta_r(1);
    
    overshoot(2) = max(theta(2,:))-theta_r(2);
    if overshoot(1)<0
        overshoot(1) = 0;
    end
    if overshoot(2)<0
        overshoot(2) = 0;
    end
    Risetime = zeros(2,1);
    i=1;
    for t=0:dt:arr_length        
        if (theta(1,i)>=theta_r(1)*0.9)
            Risetime(1) = t;
            break;
        end
        i = i+1;
    end
    i=1;
    for t=0:dt:arr_length
        if (theta(2,i)>=theta_r(2)*0.9)
            Risetime(2) = t;
            break;
        end
        i = i+1;
    end
end