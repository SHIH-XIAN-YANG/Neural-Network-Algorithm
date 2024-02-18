function J = cost_function(PID)
    
    theta_r = [1;
               1];
    
   Kp = [PID(1), 0;
            0,  PID(4)];
    Ki = [PID(2), 0;
            0, PID(5)];
    Kd = [PID(3), 0;
               0, PID(6)];                
                   
    m1 = 0.1;%(kg)
    m2 = 0.1;%(kg)
    l1 = 0.8;%(m)
    l2 = 0.4;%(m)
    g = 9.81;%m/s^2

    dt = 0.001; %sampling time
      
    t = 0:dt:1;
    
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
    
    
    
    %% main loop
    e(:,1) = theta_r - theta(:,1);
    sum_e(:,1) = e(:,1)*dt;
    torque(:,1) = Kp*e(:,1) + Kd*(e(:,1)-e(:,1))/dt + Ki*sum_e(:,1);
    k=2;

    
    for t=dt:dt:(arr_length-1)*dt

        e(:,k) = theta_r - theta(:,k-1);
        sum_e(:,k) = sum_e(:,k-1) + (e(:,k))*dt;
        
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

        ddtheta(:,k) = M(:,:,k-1)\(torque(:, k)-C(:,:,k-1)-G(:,:,k-1));

        dtheta(:,k) = dtheta(:,k-1) + (ddtheta(:,k) )*dt;

        theta(:,k) = theta(:,k-1) + (dtheta(:,k) )*dt;
        
        k=k+1;
    end
    
    t = 0:dt:1;
    theta1_step = stepinfo(theta(1,:),t)
    theta2_step = stepinfo(theta(2,:),t)
    Ess1 = abs(theta(1,arr_length-1)-theta_r(1))
    Ess2 = abs(theta(2,arr_length-1)-theta_r(2))
    phi = 0;
    J = (1-exp(-phi))*(theta1_step.Overshoot + Ess1) + exp(-phi)*(theta1_step.SettlingTime+theta1_step.RiseTime)...
        +(1-exp(-phi))*(theta2_step.Overshoot + Ess2) + exp(-phi)*(theta2_step.SettlingTime+theta2_step.RiseTime);
    %J = sum(abs(e(1,:))) + sum(abs(e(2,:)));
end