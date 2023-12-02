clear all
close all
clc

%% Derive Explicit Form:

% Parameters:
syms g M J_w R f_r h Rair_x L_f L_r Rair F_air_fz F_air_rz slope Tdrivf Tdrivr ff fr; 

% Variables:
syms Fzr Fzf Fxr Fxf v_x w_r w_f;

% Independent variables:
syms t;

% Derivatives:
syms derv_x derw_r derw_f;

% Insert equations and use "solve" command to derive the Explicit Form:

% (Fzr + F_air_rz)*L_r - (Fzf + F_air_fz)*L_f - (Fxr + Fxf)*(h - R) == 0, ...

sol = solve( ... 
    Fzr + F_air_rz + Fzf + F_air_fz == M*g*cos(slope), ...
    M*derv_x + Rair + M*g*sin(slope) == Fxr + Fxf, ...
    Fzr*(L_f+L_r) - h*(M*derv_x + M*g*sin(slope)) - Rair*h - L_f*M*g*cos(slope) == 0, ...
    Tdrivf - J_w*derw_f - Fxf*R - Fzf*R*f_r == 0, ...
    Tdrivr - J_w*derw_r - Fxr*R - Fzr*R*f_r == 0, ...
    Fxf == Fzf*ff, ...
    Fxr == Fzr*fr, ...
    Fzf, derv_x, Fzr, derw_f, derw_r, Fxf, Fxr);

disp('Fzf = ') 
disp(sol.Fzf)

disp('Fzr = ') 
disp(sol.Fzr)

disp('derv_x = ') 
disp(sol.derv_x)

disp('derw_r = ') 
disp(sol.derw_r)

disp('derw_f = ') 
disp(sol.derw_f)



