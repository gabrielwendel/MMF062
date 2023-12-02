function  [vDot,omegaDotf,omegaDotr,Fzf,Fzr]=...
    Sub_vehicle_dynamics(v,Tdrivf,Tdrivr,slipf,slipr,slope,road_cond,CONST)

% Read parameters
c_d=CONST.c_d;
c_lfCoG=CONST.c_lfCoG;
c_lrCoG=CONST.c_lrCoG;
A_f=CONST.A_f;
air=CONST.air;

M=CONST.M;
J_w=CONST.J_w; %[kg*m^2] Moment of inertia for two wheels and axle 
g=CONST.g;

h=CONST.h;
L=CONST.L;
L_f=CONST.Lf;
L_r=CONST.Lr;

R=CONST.R;
f_r=CONST.f_r; % rolling resistance coefficient

% Calculate air resistance & lift
Rair=0.5*c_d*A_f*air*v^2;
F_air_fz = 0.5*c_lfCoG*A_f*air*v^2;
F_air_rz = 0.5*c_lrCoG*A_f*air*v^2;

% Magic Tire Formula
ff=Sub_magic_tireformula(slipf,road_cond);
fr=Sub_magic_tireformula(slipr,road_cond);

%----Write your solution below---------------------------------------------
% Fzf = -(F_air_fz*L_f + F_air_fz*L_r + F_air_fz*R*fr + F_air_rz*R*fr - F_air_fz*fr*h - F_air_rz*fr*h - L_r*M*g*cos(slope) - M*R*fr*g*cos(slope) + M*fr*g*h*cos(slope))/(L_f + L_r - R*ff + R*fr + ff*h - fr*h);
% Fzr = -(F_air_rz*L_f + F_air_rz*L_r - F_air_fz*R*ff - F_air_rz*R*ff + F_air_fz*ff*h + F_air_rz*ff*h - L_f*M*g*cos(slope) + M*R*ff*g*cos(slope) - M*ff*g*h*cos(slope))/(L_f + L_r - R*ff + R*fr + ff*h - fr*h);
% vDot = -(L_f*Rair + L_r*Rair + F_air_fz*L_f*ff + F_air_fz*L_r*ff + F_air_rz*L_f*fr + F_air_rz*L_r*fr - R*Rair*ff + R*Rair*fr + Rair*ff*h - Rair*fr*h + L_f*M*g*sin(slope) + L_r*M*g*sin(slope) - L_r*M*ff*g*cos(slope) - L_f*M*fr*g*cos(slope) - M*R*ff*g*sin(slope) + M*R*fr*g*sin(slope) + M*ff*g*h*sin(slope) - M*fr*g*h*sin(slope))/(M*(L_f + L_r - R*ff + R*fr + ff*h - fr*h));
% omegaDotf = (L_f*Tdrivf + L_r*Tdrivf - R*Tdrivf*ff + R*Tdrivf*fr + Tdrivf*ff*h - Tdrivf*fr*h + F_air_fz*R^2*f_r*fr + F_air_rz*R^2*f_r*fr + F_air_fz*R^2*ff*fr + F_air_rz*R^2*ff*fr + F_air_fz*L_f*R*f_r + F_air_fz*L_r*R*f_r + F_air_fz*L_f*R*ff + F_air_fz*L_r*R*ff - F_air_fz*R*f_r*fr*h - F_air_rz*R*f_r*fr*h - F_air_fz*R*ff*fr*h - F_air_rz*R*ff*fr*h - M*R^2*f_r*fr*g*cos(slope) - M*R^2*ff*fr*g*cos(slope) - L_r*M*R*f_r*g*cos(slope) - L_r*M*R*ff*g*cos(slope) + M*R*f_r*fr*g*h*cos(slope) + M*R*ff*fr*g*h*cos(slope))/(J_w*(L_f + L_r - R*ff + R*fr + ff*h - fr*h));
% omegaDotr = (L_f*Tdrivr + L_r*Tdrivr - R*Tdrivr*ff + R*Tdrivr*fr + Tdrivr*ff*h - Tdrivr*fr*h - F_air_fz*R^2*f_r*ff - F_air_rz*R^2*f_r*ff - F_air_fz*R^2*ff*fr - F_air_rz*R^2*ff*fr + F_air_rz*L_f*R*f_r + F_air_rz*L_r*R*f_r + F_air_rz*L_f*R*fr + F_air_rz*L_r*R*fr + F_air_fz*R*f_r*ff*h + F_air_rz*R*f_r*ff*h + F_air_fz*R*ff*fr*h + F_air_rz*R*ff*fr*h + M*R^2*f_r*ff*g*cos(slope) + M*R^2*ff*fr*g*cos(slope) - L_f*M*R*f_r*g*cos(slope) - L_f*M*R*fr*g*cos(slope) - M*R*f_r*ff*g*h*cos(slope) - M*R*ff*fr*g*h*cos(slope))/(J_w*(L_f + L_r - R*ff + R*fr + ff*h - fr*h));

Fzf = -(F_air_fz*L_f + F_air_rz*L_f + F_air_fz*L_r + F_air_rz*L_r - F_air_fz*fr*h - F_air_rz*fr*h - L_r*M*g*cos(slope) + M*fr*g*h*cos(slope))/(L_f + L_r + ff*h - fr*h);
Fzr = -(F_air_fz*ff*h + F_air_rz*ff*h - L_f*M*g*cos(slope) - M*ff*g*h*cos(slope))/(L_f + L_r + ff*h - fr*h);
vDot = -(L_f*Rair + L_r*Rair + F_air_fz*L_f*ff + F_air_rz*L_f*ff + F_air_fz*L_r*ff + F_air_rz*L_r*ff + Rair*ff*h - Rair*fr*h + L_f*M*g*sin(slope) + L_r*M*g*sin(slope) - L_r*M*ff*g*cos(slope) - L_f*M*fr*g*cos(slope) + M*ff*g*h*sin(slope) - M*fr*g*h*sin(slope))/(M*(L_f + L_r + ff*h - fr*h));
omegaDotf = (L_f*Tdrivf + L_r*Tdrivf + Tdrivf*ff*h - Tdrivf*fr*h + F_air_fz*L_f*R*f_r + F_air_rz*L_f*R*f_r + F_air_fz*L_r*R*f_r + F_air_rz*L_r*R*f_r + F_air_fz*L_f*R*ff + F_air_rz*L_f*R*ff + F_air_fz*L_r*R*ff + F_air_rz*L_r*R*ff - F_air_fz*R*f_r*fr*h - F_air_rz*R*f_r*fr*h - F_air_fz*R*ff*fr*h - F_air_rz*R*ff*fr*h - L_r*M*R*f_r*g*cos(slope) - L_r*M*R*ff*g*cos(slope) + M*R*f_r*fr*g*h*cos(slope) + M*R*ff*fr*g*h*cos(slope))/(J_w*(L_f + L_r + ff*h - fr*h));
omegaDotr = (L_f*Tdrivr + L_r*Tdrivr + Tdrivr*ff*h - Tdrivr*fr*h + F_air_fz*R*f_r*ff*h + F_air_rz*R*f_r*ff*h + F_air_fz*R*ff*fr*h + F_air_rz*R*ff*fr*h - L_f*M*R*f_r*g*cos(slope) - L_f*M*R*fr*g*cos(slope) - M*R*f_r*ff*g*h*cos(slope) - M*R*ff*fr*g*h*cos(slope))/(J_w*(L_f + L_r + ff*h - fr*h));

%--------------------------------------------------------------------------
end