% Problem 2

clear variables;
clc;

%% Part a

S1 = 0.016;
S2 = 0.017; 
AR1 = 0.798;
AR2 = 1.241;
e = 0.9;
m1 = 0.003;
m2 = 0.003;
g = 9.8;
rho = 1.225;
rhoalts = [1.225 1.21328 1.20165 1.19011 1.17864 1.16727 1.15598 1.14477 1.13364 1.12260 1.11164 1.10077 1.08997 1.07925 1.06862 1.05807 1.04759 1.03720 1.02688 1.01665 1.00649];
CLa1 = 3.141592 * AR1/(1 + sqrt(1 + (AR1 / 2)^2));
CLa2 = 3.141502 * AR2/(1 + sqrt(1 + (AR2 / 2)^2));
CDo1 = 0.02;
CDo2 = 0.02;
epsilon1 = 1 / (3.141592 * e * AR1);
epsilon2 = 1 / (3.141592 * e * AR2);

alpha = linspace(-2,16,1000);

CL1 = alpha*CLa1;
CL2 = alpha*CLa2;

CD1 = CDo1 + epsilon1 * CL1.^2;
CD2 = CDo2 + epsilon2 * CL2.^2;

LD1 = CL1./CD1;
LD2 = CL2./CD2;

MaxLD1 = maxk(LD1,1);
MaxLD2 = maxk(LD2,1);

LDalpha1 = alpha(find(LD1==MaxLD1));
LDalpha2 = alpha(find(LD2==MaxLD2));

CLLDmax1 = sqrt(CDo1 / epsilon1);
CLLDmax2 = sqrt(CDo2 / epsilon2);
CDLDmax1 = CDo1 + epsilon1 * CLLDmax1^2;
CDLDmax2 = CDo2 + epsilon2 * CLLDmax2^2;
MaxLD1 = CLLDmax1 / CDLDmax1;
MaxLD2 = CLLDmax2 / CDLDmax2;
LDalpha1 = CLLDmax1 / CLa1;
LDalpha2 = CLLDmax2 / CLa2;

disp("Max L/D for the Dart: "+MaxLD1)
disp("AoA for Max L/D: "+LDalpha1)

disp("Max L/D for the Modified Dart: "+MaxLD2)
disp("AoA for Max L/D: "+LDalpha2)

figure
subplot(3,1,1)
plot(alpha,CL1,alpha,CL2)
xlabel('alpha'), ylabel('CL'), grid
subplot(3,1,2)
plot(alpha,CD1,alpha,CD2)
xlabel('alpha'), ylabel('CD'), grid
subplot(3,1,3)
plot(CL1,CD1,'-',CL2,CD2,'-',CLLDmax1,CDLDmax1,'o',CLLDmax2,CDLDmax2,'o');
xlabel('CL'), ylabel('CD'), grid

%% Part b

Gam1 = -atan(1 / MaxLD1);
Gam2 = -atan(1 / MaxLD2);

V1 = sqrt(2 * m1 * g ./(rhoalts * S1 * (CLLDmax1 * cos(Gam1) - CDLDmax1 * sin(Gam1))));
V2 = sqrt(2 * m2 * g ./(rhoalts * S2 * (CLLDmax2 * cos(Gam2) - CDLDmax2 * sin(Gam2))));

figure
plot(rhoalts,V1,rhoalts,V2)

%% Part c

VLDmax1 = sqrt(2 * m1 * g /(rho * S1 * (CLLDmax1 * cos(Gam1) - CDLDmax1 * sin(Gam1))));
VLDmax2 = sqrt(2 * m2 * g /(rho * S2 * (CLLDmax2 * cos(Gam2) - CDLDmax2 * sin(Gam2))));

H = 10; % Initial Height, m
R = 0; % Initial Range, m
to = 0; % Initial Time, sec
tf = 10; % Final Time, sec
tspan = [to tf];
xo1 = [VLDmax1;Gam1;H;R];
xo2 = [VLDmax2;Gam2;H;R];
[ta1,xa1] = ode23('EqMotion',tspan,xo1);
[ta2,xa2] = ode23('EqMotion',tspan,xo2);
% b) Oscillating Glide due to Zero Initial Flight Path Angle
xo1 = [VLDmax1;0;H;R];
xo2 = [VLDmax2;0;H;R];
[tb1,xb1] = ode23('EqMotion',tspan,xo1);
[tb2,xb2] = ode23('EqMotion',tspan,xo2);
% c) Effect of Increased Initial Velocity
xo1 = [1.5*VLDmax1;0;H;R];
xo2 = [1.5*VLDmax2;0;H;R];
[tc1,xc1] = ode23('EqMotion',tspan,xo1);
[tc2,xc2] = ode23('EqMotion',tspan,xo2);
% d) Effect of Further Increase in Initial Velocity
xo1 = [3*VLDmax1;0;H;R];
xo2 = [3*VLDmax2;0;H;R];
[td1,xd1] = ode23('EqMotion',tspan,xo1);
[td2,xd2] = ode23('EqMotion',tspan,xo2);
figure
plot(xa1(:,4),xa1(:,3),'-',xa2(:,4),xa2(:,3),'--',xb1(:,4),xb1(:,3),'-',xb2(:,4),xb2(:,3),'--',xc1(:,4),xc1(:,3),'-',xc2(:,4),xc2(:,3),'--',xd1(:,4),xd1(:,3),'-',xd2(:,4),xd2(:,3),'--')
xlabel('Range, m'), ylabel('Height, m'), grid, legend('Dart, Equ Values','Mod-Dart, Equ Values','Dart, Zero Angle','Mod-Dart, Zero Angle','Dart, 150% Airspeed','Mod-Dart, 150% Airspeed','Dart, 300% Airspeed','Mod-Dart, 300% Airspeed')
figure
subplot(2,2,1)
plot(ta1,xa1(:,1),'-',ta2,xa2(:,1),'--',tb1,xb1(:,1),'-',tb2,xb2(:,1),'--',tc1,xc1(:,1),'-',tc2,xc2(:,1),'--',td1,xd1(:,1),'-',td2,xd2(:,1),'--')
xlabel('Time, s'), ylabel('Velocity, m/s'), grid,  legend('Dart, Equ Values','Mod-Dart, Equ Values','Dart, Zero Angle','Mod-Dart, Zero Angle','Dart, 150% Airspeed','Mod-Dart, 150% Airspeed','Dart, 300% Airspeed','Mod-Dart, 300% Airspeed')
subplot(2,2,2)
plot(ta1,xa1(:,2),'-',ta2,xa2(:,2),'--',tb1,xb1(:,2),'-',tb2,xb2(:,2),'--',tc1,xc1(:,2),'-',tc2,xc2(:,2),'--',td1,xd1(:,2),'-',td2,xd2(:,2),'--')
xlabel('Time, s'), ylabel('Flight Path Angle, rad'), grid,  legend('Dart, Equ Values','Mod-Dart, Equ Values','Dart, Zero Angle','Mod-Dart, Zero Angle','Dart, 150% Airspeed','Mod-Dart, 150% Airspeed','Dart, 300% Airspeed','Mod-Dart, 300% Airspeed')
subplot(2,2,3)
plot(ta1,xa1(:,3),'-',ta2,xa2(:,3),'--',tb1,xb1(:,3),'-',tb2,xb2(:,3),'--',tc1,xc1(:,3),'-',tc2,xc2(:,3),'--',td1,xd1(:,3),'-',td2,xd2(:,3),'--')
xlabel('Time, s'), ylabel('Altitude, m'), grid,  legend('Dart, Equ Values','Mod-Dart, Equ Values','Dart, Zero Angle','Mod-Dart, Zero Angle','Dart, 150% Airspeed','Mod-Dart, 150% Airspeed','Dart, 300% Airspeed','Mod-Dart, 300% Airspeed')
subplot(2,2,4)
plot(ta1,xa1(:,4),'-',ta2,xa2(:,4),'--',tb1,xb1(:,4),'-',tb2,xb2(:,4),'--',tc1,xc1(:,4),'-',tc2,xc2(:,4),'--',td1,xd1(:,4),'-',td2,xd2(:,4),'--')
xlabel('Time, s'), ylabel('Range, m'), grid,  legend('Dart, Equ Values','Mod-Dart, Equ Values','Dart, Zero Angle','Mod-Dart, Zero Angle','Dart, 150% Airspeed','Mod-Dart, 150% Airspeed','Dart, 300% Airspeed','Mod-Dart, 300% Airspeed')

