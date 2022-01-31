%% APPENDIX
clc;clear all;close all;
%% Traction control system with Fuzzy PID
GR_ax1 = 2.41;                                                      % Gear ratio of Axle                     
GR_tran1 = 4.56;                                                    % Gear ratio of different gears

I_tran1 = 0.147;                                                    % Inertia of different gears
I_e1 = 0.09;                                                        % Inertia of engine
I_d1 = 0.12;                                                        % Inertia of driveshaft
I_ax1 = 0.003;                                                      % Inertia of Axle
I_tire1 = 1.2;                                                      % Inertia of tire
I_rear1 = I_tire1*2;                                                 % Inertia of rear wheels
thetaderedline_dot1 = 6500;                                         % Readline theta [RPM]
M1 = 1665;                                                          % mass of car
car_h1 = 1.23;                                                      % car's height
car_w1 = 1.97;                                                      % car's width
car_area1 = car_h1*car_w1;                                           % car's area
drag_c1 = 0.28;                                                     % Drag coefficient
resi_c1 = 0.015;                                                    % Friction coefficient
rho1 = 1.225;                                                       % air density
I_dt1 = I_tire1 + GR_ax1^2*I_d1 + (GR_ax1^2*GR_tran1^2*(I_e1 + I_tran1));  % Drivetrain Inertia

% based on Tire specification
Rt_front1 = ((19*0.0254) + (2*((30*285)/100000)))/2;                % Front tire radius 
Rt_rear1 = ((20*0.0254) + (2*((25*335)/100000)))/2;                 % Rear tire radius

Meq_FT1 = (2*I_tire1)/(Rt_front1^2);
Meq1 = M1 + Meq_FT1;

fis = readfis;

% evalute torque from engine specification
P_hp1 = [0 10 40 80 160 220 240 270 330 400 480 540 620 660 670 675];
P_watt1 = P_hp1*746;
thetae_dot_rpm1 = [0 500 1000 1500 2000 2100 2300 2500 3000 3500 4000 4500 5000 5500 6000 6500];
thetae_dot_rad1 = (2*pi/60)*thetae_dot_rpm1;
T1 = P_watt1./thetae_dot_rad1;

% evalute rolling resistance force
F_fric1 = resi_c1*M1*9.8;

N1 = 45001;                      % vector length
dt1 = 0.0001;                     % time increment
t1 = (0:dt1:(N1-1)*dt1);

xdot1 = zeros(N1,1);          % velocity vector for plot
xdot1(1) = 5/2.23694;            % initial velocity = 5mph

x1 = zeros(N1,1);                 % distance vector for plot
x1(1) = 0;                       % initial distance = 0m

thetae_dot_rad_instant1 = zeros(N1,1);
P_watt_instant1 = zeros(N1,1);
T_instant1 = zeros(N1,1);

thetaw_dot_req1 = zeros(N1,1);
sr_instant1 = zeros(N1,1);
thetaw_dot_req1(1) = xdot1(1)/Rt_rear1;
thetaw_dot_instant1 = zeros(N1,1);
t_force_instant1 = zeros(N1,1);
xddot1 = zeros(N1,1);
thetaw_ddot_instant1 = zeros(N1,1); 
sr_instant1(1) = 0.01;
% calculate intial wheel angular velocity from slip ratio
thetaw_dot_instant1(1) = xdot1(1)*(sr_instant1(1)+1)*(1/Rt_rear1);

% input traction force vs slip ratio
t_force1 = [0 2000 3333 4777 6000 6100 6400 6450 6500 6300 6000 5700 5200 5000 4800 4700 4600 4500 4400 4300];
sr1 = [0 0.01 0.02 0.03 0.04 0.05 0.06 0.07 0.1 0.15 0.2 0.3 0.4 0.45 0.5 0.6 0.7 0.8 0.9 1];

sr_desired1 = 0.1;
error1=zeros(N1,1);
error1(1) = sr_desired1 - sr_instant1(1);
Kp1 = 8000; Kd1 = 0; Ki1 = 7550; 
de1 = 0; error_sum1 = 0;
for k = 2:N1
% 1st gear only                
% get engine speed from wheel speed
        thetae_dot_rad_instant1(k-1) = (thetaw_dot_instant1(k-1)*GR_ax1*GR_tran1(1)); 
% PID Controller        
        if k == 2
            de1 = error1(k-1);
        else
            de1 = error1(k-1) - error1(k-2);
        end 
        error_sum1 = error1(k-1)*dt1 + error_sum1;
% get Kp and Ki from FIS       
        [K1] = evalfis(fis,[error1(k-1) de1/dt1]);
        
        de_wrt_t1(k-1) = de1/dt1;
        kp1(k-1) = K1(1);
        kd1(k-1) = Kd1;
        ki1(k-1) = K1(2);
        
% Controlled Torque        
         T_instant1(k-1) = kp1(k-1)*error1(k-1) + kd1(k-1)*(de1/dt1) + ki1(k-1)*error_sum1;
         % The torque can't drop's below zero, it's not possible 
         if T_instant1(k-1) < 0
            T_instant1(k-1) = 0;
         end
             
% logic to get tractive force        
        if sr_instant1(k-1) >= 0
            t_force_instant1(k-1) = interp1(sr1,t_force1, ...
                                              sr_instant1(k-1));
        else                              
            t_force_instant1(k-1) = 0;
        end
% calculate linear acceleration (to get linear velocity for next iteration)        
        xddot1(k-1) = ((t_force_instant1(k-1)/Meq1(1)) - ...
                      (F_fric1/Meq1(1)) - ((0.5*rho1*drag_c1*car_area1/Meq1(1))* ...
                      (xdot1(k-1).^2)));

% calculate angular wheel acceleration (to get angular wheel velocity for next iteration)        
        thetaw_ddot_instant1(k-1) = (-(t_force_instant1(k-1)*Rt_rear1/I_dt1) + ...
                                        (GR_ax1*GR_tran1*T_instant1(k-1)/I_dt1));
                                    
% use euler method to calculate speed and distance & angular wheel speed
        xdot1(k) = xdot1(k-1) + xddot1(k-1)*dt1;
        x1(k) = x1(k-1) + xdot1(k-1)*dt1;
        thetaw_dot_instant1(k) = thetaw_dot_instant1(k-1) + thetaw_ddot_instant1(k-1)*dt1;
    
% get the actual velocity of car     
        thetaw_dot_req1(k) = xdot1(k)/Rt_rear1;
% get the new Slip ratio    
        sr_instant1(k) = (thetaw_dot_instant1(k) - thetaw_dot_req1(k))/thetaw_dot_req1(k); 
        error1(k) = sr_desired1 - sr_instant1(k);
end 

% for fuzzy
    de_wrt_t1(k) = de_wrt_t1(k-1);
    kp1(k) = Kp1;
    kd1(k) = Kd1;
    ki1(k) = Ki1;

%% Traction control system with PID

GR_ax = 2.41;                                                      % Gear ratio of Axle                     
GR_tran = 4.56;                                                    % Gear ratio of different gears

I_tran = 0.147;                                                    % Inertia of different gears
I_e = 0.09;                                                        % Inertia of engine
I_d = 0.12;                                                        % Inertia of driveshaft
I_ax = 0.003;                                                      % Inertia of Axle
I_tire = 1.2;                                                      % Inertia of tire
I_rear = I_tire*2;                                                 % Inertia of rear wheels
thetaderedline_dot = 6500;                                         % Readline theta [RPM]
M = 1665;                                                          % mass of car
car_h = 1.23;                                                      % car's height
car_w = 1.97;                                                      % car's width
car_area = car_h*car_w;                                            % car's area
drag_c = 0.28;                                                     % Drag coefficient
resi_c = 0.015;                                                    % Friction coefficient
rho = 1.225;                                                       % air density
I_dt = I_tire + GR_ax^2*I_d + (GR_ax^2*GR_tran^2*(I_e + I_tran));  % Drivetrain Inertia

% based on Tire specification
Rt_front = ((19*0.0254) + (2*((30*285)/100000)))/2;                % Front tire radius 
Rt_rear = ((20*0.0254) + (2*((25*335)/100000)))/2;                 % Rear tire radius

Meq_FT = (2*I_tire)/(Rt_front^2);
Meq = M + Meq_FT;

% evalute torque from engine specification
P_hp = [0 10 40 80 160 220 240 270 330 400 480 540 620 660 670 675];
P_watt = P_hp*746;
thetae_dot_rpm = [0 500 1000 1500 2000 2100 2300 2500 3000 3500 4000 4500 5000 5500 6000 6500];
thetae_dot_rad = (2*pi/60)*thetae_dot_rpm;
T = P_watt./thetae_dot_rad;

% evalute rolling resistance force
F_fric = resi_c*M*9.8;

N = 45001;                      % vector length
dt = 0.0001;                     % time increment
t = (0:dt:(N-1)*dt);

xdot = zeros(N,1);          % velocity vector for plot
xdot(1) = 5/2.23694;            % initial velocity = 5mph

x = zeros(N,1);                 % distance vector for plot
x(1) = 0;                       % initial distance = 0m

thetae_dot_rad_instant = zeros(N,1);
P_watt_instant = zeros(N,1);
T_instant = zeros(N,1);

thetaw_dot_req = zeros(N,1);
sr_instant = zeros(N,1);
thetaw_dot_req(1) = xdot(1)/Rt_rear;
thetaw_dot_instant = zeros(N,1);
t_force_instant = zeros(N,1);
xddot = zeros(N,1);
thetaw_ddot_instant = zeros(N,1); 
sr_instant(1) = 0.01;
% calculate intial wheel angular velocity from slip ratio
thetaw_dot_instant(1) = xdot(1)*(sr_instant(1)+1)*(1/Rt_rear);

% input traction force vs slip ratio
t_force = [0 2000 3333 4777 6000 6100 6400 6450 6500 6300 6000 5700 5200 5000 4800 4700 4600 4500 4400 4300];
sr = [0 0.01 0.02 0.03 0.04 0.05 0.06 0.07 0.1 0.15 0.2 0.3 0.4 0.45 0.5 0.6 0.7 0.8 0.9 1];

sr_desired = 0.1;
error=zeros(N,1);
error(1) = sr_desired - sr_instant(1);
Kp = 8000; Kd = 0; Ki = 7550; 
de = 0; error_sum = 0;
for k = 2:N
% 1st gear only                
% get engine speed from wheel speed
        thetae_dot_rad_instant(k-1) = (thetaw_dot_instant(k-1)*GR_ax*GR_tran(1)); 
% Instead of getting torque from Engine speed, torque is controlled through
% PID Controller        
        if k == 2
            de = error(k-1);
        else
            de = error(k-1) - error(k-2);
        end 
        error_sum = error(k-1)*dt + error_sum;
% related to fuzzy        
        de_wrt_t(k-1) = de/dt;
        kp(k-1) = Kp;
        kd(k-1) = Kd;
        ki(k-1) = Ki;
        
% Controlled Torque        
         T_instant(k-1) = Kp*error(k-1) + Kd*(de/dt) + Ki*error_sum;
         % The torque can't drop's below zero, it's not possible 
         if T_instant(k-1) < 0
            T_instant(k-1) = 0;
         end
             
% logic to get tractive force        
        if sr_instant(k-1) >= 0
            t_force_instant(k-1) = interp1(sr,t_force, ...
                                              sr_instant(k-1));
        else                              
            t_force_instant(k-1) = 0;
        end
% calculate linear acceleration (to get linear velocity for next iteration)        
        xddot(k-1) = ((t_force_instant(k-1)/Meq(1)) - ...
                      (F_fric/Meq(1)) - ((0.5*rho*drag_c*car_area/Meq(1))* ...
                      (xdot(k-1).^2)));

% calculate angular wheel acceleration (to get angular wheel velocity for next iteration)        
        thetaw_ddot_instant(k-1) = (-(t_force_instant(k-1)*Rt_rear/I_dt) + ...
                                        (GR_ax*GR_tran*T_instant(k-1)/I_dt));
                                    
% use euler method to calculate speed and distance & angular wheel speed
        xdot(k) = xdot(k-1) + xddot(k-1)*dt;
        x(k) = x(k-1) + xdot(k-1)*dt;
        thetaw_dot_instant(k) = thetaw_dot_instant(k-1) + thetaw_ddot_instant(k-1)*dt;
    
% get the actual velocity of car     
        thetaw_dot_req(k) = xdot(k)/Rt_rear;
% get the new Slip ratio    
        sr_instant(k) = (thetaw_dot_instant(k) - thetaw_dot_req(k))/thetaw_dot_req(k); 
        error(k) = sr_desired - sr_instant(k);
end 

% for fuzzy
    de_wrt_t(k) = de_wrt_t(k-1);
    kp(k) = Kp;
    kd(k) = Kd;
    ki(k) = Ki;

figure
plot(sr,t_force,'linewi',2),xlabel('Slip Ratio'),ylabel('Tractive Force [N]'),
title('Tractive force vs. Slip Ratio'),grid on

%% comparsion between PID and Fuzzy-PID Controller

figure
plot(t,thetaw_dot_instant*Rt_rear*2.23694,t,thetaw_dot_instant1*Rt_rear*2.23694,'linewi',2)
xlabel('time [s]'),ylabel('velocity [mph]')
title('ZR1 velocity, 5 mph rolling start'),grid on
legend('PID','Fuzzy PID')

figure
plot(t,sr_instant,t,sr_instant1,'linewi',2)
xlabel('time [s]'),ylabel('Current Slip ratio'),grid on
title('Slip ratio')
legend('PID','Fuzzy PID')

figure
plot(t,t_force_instant,t,t_force_instant1,'linewi',2),grid on
xlabel('time [s]'),ylabel('Tractive force [N]')
title('Tractive force vs. time')
legend('PID','Fuzzy PID')

figure
plot(t,T_instant,t,T_instant1,'linewi',2),grid on
xlabel('time [s]'),ylabel('Torque [N-m]')
title('Torque vs. time')
legend('PID','Fuzzy PID')

figure
plot(t,kp,t,ki,t,kp1,t,ki1,'linewi',2),grid on
xlabel('time [s]')
legend('Kp(PID)','Ki(PID)','Kp(Fuzzy PID)','Ki(Fuzzy PID)')
