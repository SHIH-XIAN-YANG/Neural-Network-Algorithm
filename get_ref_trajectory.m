%% multi- trjectory
function theta_r = get_ref_trajectory()
    dt=0.001;
    t = 0:dt:5;

    theta_r = zeros(2,length(t));

    i=1;
    for t=0:dt:1
        theta_r(1,i) = 1;
        theta_r(2,:) = 1.5;
        i = i+1;
    end


    for t=0+dt:dt:1
        theta_r(1,i) = theta_r(1,length(0:dt:1-dt)) + 0.5*t;
        theta_r(2,i) = theta_r(2,length(0:dt:1-dt)) - t;
        i=i+1;
    end

    for t=0+dt:dt:1
        theta_r(1,i) = theta_r(1,i-1);
        theta_r(2,i) = theta_r(2,i-1);
        i = i+1;
    end

    for t=0+dt:dt:1
        theta_r(1,i) = theta_r(1,length(0:dt:3-dt)) - t;
        theta_r(2,i) = theta_r(2,length(0:dt:3-dt)) + 0.5*t;
        i=i+1;
    end

    for t=0+dt:dt:1
        theta_r(1,i) = theta_r(1,i-1);
        theta_r(2,i) = theta_r(2,i-1);
        i=i+1;
    end

%{
t=0:dt:5;
figure;
plot(t,theta_r);
ylim([0 2])
grid on;
legend('theta1','theta2');
xlabel('t'); ylabel('$\theta_1, \theta_2(rad)$','Interpreter','Latex');
%}
end
    