%% HW3

clc;

%% 6.1

clear variables;

syms U V W P Q R

V_B = [U; V; W];
omega_B = [P; Q; R];

a_B = [diff(U); diff(V); diff(W)] + cross(omega_B, V_B);

disp("6.1")
disp(" ")
disp("Expressions for Intertial Acceleration in the rotating body axis system:")
disp(" ")
disp(a_B)

%% 6.2

clear variables;

syms U V W Ud Vd Wd P Q R

V_b = [U; V; W];
Vd_b = [Ud; Vd; Wd];
omega_b = [P; Q; R];

aI_b = Vd_b + cross(omega_b, V_b);

disp("6.2")
disp(" ")
disp("Expressions for Intertial Acceleration in the rotating body axis system:")
disp(" ")

aIsol = subs(aI_b, [U V W Ud Vd Wd P Q R], [200 0 10 5 0 0 0 0.1 0.1]);

disp("Solution:")
disp(aIsol)
disp("ft/s^2")
disp(" ")

%% 6.3

clear variables;

syms m R

I_xx = 2*m*(R^2);

Is_xx = subs(I_xx, [m R], [500, 10]);

disp("6.3")
disp(" ")
disp("The change in Ixx is:")
fprintf("%d lb-ft^2\n",Is_xx)
disp(" ")

%% 6.4

clear variables;

syms m ym zm

I_yz = m*ym*zm;

Is_yz = subs(I_yz, [m ym zm], [500, 2, 10]);

disp("6.4")
disp(" ")
disp("The value of Iyz is:")
fprintf("%d lb-ft^2\n",Is_yz)
disp(" ")

%% 6.5



