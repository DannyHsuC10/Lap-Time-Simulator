function [N,q,dz,f] = Suspension_3dof(ax,ay,F_ext,r_ext,car)

% =========================================================
% Suspension 3DOF
%
% q =
% [ Heave
%   Roll
%   Pitch ]
%
% Inputs
% ---------------------------------------------------------
% ax      : longitudinal acceleration [m/s^2]
% ay      : lateral acceleration [m/s^2]
%
% F_ext   : external force [Fx Fy Fz]'
% r_ext   : application point [x y z]'
%
% Outputs
% ---------------------------------------------------------
% N       : wheel loads
% q       : modal displacement
% dz      : wheel displacement
% f       : generalized force
%
% =========================================================


g = 9.81;

%% =========================================================
% Vehicle Forces
%% =========================================================

Fz_vehicle = car.mass*g;

Mx_vehicle = car.mass*ay*(car.h-car.h_rc);

My_vehicle = car.mass*ax*(car.h-car.h_rc);

f_vehicle = [
    Fz_vehicle
    Mx_vehicle
    My_vehicle
];

%% =========================================================
% External Force
%% =========================================================
%
% x : forward
% y : left
% z : upward
%
% Only Fz contributes to current 3DOF model
%

f_ext = [
     -F_ext(3)
     r_ext(2)*F_ext(3)
    -r_ext(1)*F_ext(3)
];

%% =========================================================
% Total Generalized Force
%% =========================================================

f = f_vehicle + f_ext;

%% =========================================================
% Modal Stiffness
%% =========================================================

Km = diag([
    car.K_heave
    car.K_roll
    car.K_pitch
]);

%% =========================================================
% Modal Displacement
%% =========================================================

q = Km\f;

%% =========================================================
% Wheel Displacement Matrix
%% =========================================================

B = [
     1  -car.t_front/2   -car.L_front
     1   car.t_front/2   -car.L_front

     1  -car.t_rear/2     car.L_rear
     1   car.t_rear/2     car.L_rear
];

%% =========================================================
% Wheel Vertical Displacement
%% =========================================================

dz = B*q;

%% =========================================================
% Wheel Load Distribution
%% =========================================================

x = [
     car.L_front
     car.L_front
    -car.L_rear
    -car.L_rear
];

y = [
    -car.t_front/2
     car.t_front/2
    -car.t_rear/2
     car.t_rear/2
];

A = [
    1 1 1 1
    y'
    x'
];

%% =========================================================
% Static Equilibrium
%% =========================================================

b = [
    -f(1)
     f(2)
     f(3)
];

N = A'*((A*A')\(-b));

end