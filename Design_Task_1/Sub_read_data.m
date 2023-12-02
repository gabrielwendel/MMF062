function CONST=Sub_read_data

%--------------------------------------------------------------------------
% Read vehicle data with use of specific vehicle data file
%--------------------------------------------------------------------------

%----Loading vehicle parameters--------------------------------------------
Saab_93_datasheet;    % Load data from the Saab 9-3 data file.

%----Chassis---------------------------------------------------------------
CONST.M   = m;            %[kg] Curb weight
CONST.mb  = m-m_unspr;    %[kg] Vehicle sprung mass
CONST.mf  = m_unspr/2;    %[kg] Unsprung mass front wheels
CONST.mr  = m_unspr/2;    %[kg] Unsprung mass rear wheels
CONST.L   = L;            %[m]  Wheel base 
CONST.Lf  = l_f;          %[m]  Distance along X-axis from CoG to front axle Default Data
CONST.Lr   = L-CONST.Lf;  %[m]  Distance along X-axis from CoG to rear axle
CONST.h   = h;            %[m]  Distance along Z-axis from CoG to front axle.
CONST.A_f = A_f;          %[m^2]Front area
CONST.c_d = c_d;          %[-]  Air drag coefficient 
CONST.c_lfCoG = c_lfCoG;  %[-]  Air lift coefficient over front axle
CONST.c_lrCoG = c_lrCoG;  %[-]  Air lift coefficient over front axle

%----Tire/Wheel------------------------------------------------------------
CONST.R   = radius_wheel; %[m]  Wheel radius
CONST.J_w  = J_wheel;     %[kg*m^2] Moment of inertia for two wheels and axle 
CONST.f_r = RRconst;      %[-]  Rolling resistance coefficient

%----Environment-----------------------------------------------------------
CONST.g   = g;            %[m/s^2]  Gravitational constant
CONST.air = air;          %[kg/m^3] Air density 

%--------------------------------------------------------------------------
%-----------------------------Added for Task 3-----------------------------
%----Driveline-------------------------------------------------------------
CONST.Tmax_vecf    = Tmax_vecf;    %[Nm] Front motor maximum torque vector, corresponding to w_vecf
CONST.w_vecf       = w_vecf;       %[rad/s] Front motor rotational speed, corresponding to Tmax_vecf
CONST.ratio_vecf   = ratio_f;      %[-] Front drive motor gear ratio(s)
CONST.Tmax_vecr    = Tmax_vecr;    %[Nm] Rear motor maximum torque vector, corresponding to w_vecr
CONST.w_vecr       = w_vecr;       %[rad/s] Rear motor rotational speed, corresponding to Tmax_vecr
CONST.ratio_vecr   = ratio_r;      %[-] Rear drive motor gear ratio(s)

end
