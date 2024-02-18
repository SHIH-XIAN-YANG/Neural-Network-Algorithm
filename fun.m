function J = fun(pid)

global hh nvars o

    % Track the output of optsim to a signal of 1

    % Variables a1 and a2 are shared with RUNTRACKLSQ
    Kp = abs(pid(1));
    Ki = abs(pid(2));
    Kd = abs(pid(3));

    sprintf('The value of interation Kp= %3.0f, Kd= %3.0f', pid(1),pid(2)); 

    % Compute function value
    %simopt = simset('solver','ode5','SrcWorkspace','Current','DstWorkspace','Current');  % Initialize sim options
    %[tout,xout,yout] = sim('Systeme_a_Optimse',[0 10],simopt);
    s = tf('s');
    
    %plant
    %sys = exp(-s)/(10*s+1);
    sys = 1/(s^2 - 4*s -5);
    %controller
    cont = Kp + Ki/s + Kd*s;
    
    %closed-loop system
    cl_sys = feedback(cont*sys,1);
    dt = 0.001;
    t = 0:dt:20;

    [yout] = step(cl_sys,t);

    Y = stepinfo(cl_sys);



    e=1 - yout;  % compute the error 

    overshoot=max(yout)-1; % compute the overshoot
    %sys_overshoot = Y.Overshoot;


    alpha=100;beta=0.8;

    %J=sum(abs(e)*t)*beta+sum(abs(e))+sum((e.*e)*t)+sys_overshoot*alpha; %cost funciton
    J=0;
    for i=1:length(t)
        J = J + abs(e(i))*t(i);
    end
    
    J = J + 100*overshoot;
    
    
    %J=sum((e.*e));
    %J=sum(abs(e));
    %J=log(Y.SettlingTime/5e-2+1)+log(Y.Overshoot/1e-4+1) + log(sum(abs(e)));
    %phi = 0.4;
    %J = (1-exp(-phi))*(Y.SettlingTime - Y.RiseTime) + exp(-phi)*(Y.Overshoot + (Y.SettlingMax));


end