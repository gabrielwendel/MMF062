%=== Saab_9-3_data ========================================================
% Numerical vehicle data representative for a typical Saab 9-3 Sedan
%	Ref: 2003 Saab 9-3 Production No. 1787
%   Created by Matthijs Klomp 2006-08-08
%   Update 2023: Zhaohui Ge
%               - Remove unused parameters
%               - Add air lift coefficient over the front and the rear
%               axles
%               - Introduce electrified driveline 
%==========================================================================

g=9.80665;             %[m/s^2]  Gravitational constant

%% --- Chassis ------------------------------------------------------------
m=1675;             %[kg]     Curb weight (full tank, no driver or pass.)
m_unspr=185;        %[kg]     Unsprung mass all four wheels.
m_maxload=450;      %[kg]     Maximum load (driver not included)
Ixx = 540;          %[kg*m^2] Total mass moment of inertia, around CoG
Iyy = 2398;         %[kg*m^2] Total mass moment of inertia, around CoG
Izz = 2617;         %[kg*m^2] Total mass moment of inertia, around CoG
I(1,1)=Ixx; I(2,2)=Iyy; I(3,3)=Izz; % Inertia matrix
L=2.675;            %[m]      Wheel base
l_f=0.45*L;         %0.4*L %[m]      Distance along X-axis from CoG to front axle, MODIFY WEIGHT HERE!!!!
l_r=L-l_f;          %[m]      Distance along X-axis from CoG to rear axle
B_f=1.517;          %[m]      Track width front
B_r=1.505;          %[m]      Track width rear
h=0.343;           %0.543 %[m]      Distance along Z-axis from CoG to ground.
A_f=2.17;           %[m^2]    Front area
c_d=0.30;           %[-]      Air drag coefficient 
c_lfCoG=0.04;       %[-]      Air lift coefficient over front axle if air drag force on CoG heigth
c_lrCoG=0.13;       %[-]      Air lift coefficient over rear  axle if air drag force on CoG heigth
C_zf=30.8e3;        %[N/m]    Front Suspension Ride Rate (w/o tire)/ side**
C_zr=29.9e3;        %[N/m]    Rear Suspension Ride rate (w/o tire)/ side**
D_zf=4500;          %[Ns/m]   Suspension damping front, per side**
D_zr=3500;          %[Ns/m]   Suspension damping rear, per side**
C_af=16.22;         %[N/rad]  Front auxiliary roll stiffness (w/o springs)
C_ar=7.837;         %[N/rad]  Rear  auxiliary roll stiffness (w/o springs)
hr_f=0.045;         %[m]      Front roll center height above ground.
hr_r=0.101;         %[m]      Rear  roll center height above ground.
                    %         ** If bicycle model is used, multiply by 2                

%% --- Tire / Wheel -------------------------------------------------------
RRconst=0.0164;     %[-]      Rolling resistance coefficient
radius_wheel=0.316; %[m]      Radius of wheel
J_wheel=2;          %[kg*m^2] Moment of inertia for two wheels and axle

%% --- Environment * ------------------------------------------------------
air=1.3;            %[kg/m^3] Air density

%% --- Driveline ----------------------------------------------------------
% Gearbox/Retarder: 1 speed
ratio_f=8.57;       %[-]      Front motor gear ratio, Polestar 2 Dual Motors;
ratio_r=8.57;       %[-]      Rear  motor gear ratio, Polestar 2 Dual Motors;

% Motor
Tmax_f=330;         %[Nm]     Front motor max. torque, Polestar 2 Dual Motors;
Tmax_r=330;         %[Nm]     Rear motor max. torque, Polestar 2 Dual Motors;
Pmax_f=150;         %[kW]     Front motor max. power, Polestar 2 Dual Motors;
Pmax_r=150;         %[kW]     Rear motor max. power, Polestar 2 Dual Motors;

omega_Tmax_f=Pmax_f*1000/Tmax_f;                    %[rad/s] Front max. motor rotational speed where max. torqur can maintained 
omega_Low_f=(0:1:round(omega_Tmax_f));              %[rad/s] Front motor rotational speed vector before field weakening
omega_High_f=(round(omega_Tmax_f)+1:1:round(300/3.6/radius_wheel)*ratio_f); %[rad/s] Front motor rotational speed vector after field weakening, up to vehicle speed 300kph
w_vecf=[omega_Low_f,omega_Tmax_f,omega_High_f];     %[rad/s] Front motor speed vector
T_constantf = Tmax_f*ones(size(omega_Low_f));       %[Nm] Front motor torque vector within the constant torque region
T_fieldWeakeningf = Pmax_f*1000./omega_High_f;      %[Nm] Front motor torque vector within the field weakening torque region
Tmax_vecf = [T_constantf,Tmax_f,T_fieldWeakeningf]; %[Nm] Front motor torque vector corrspanding to the speed vector

omega_Tmax_r=Pmax_r*1000/Tmax_r;                    %[rad/s] Rear max. motor rotational speed where max. torqur can maintained 
omega_Low_r=(0:1:round(omega_Tmax_r));              %[rad/s] Rear motor rotational speed vector before field weakening
omega_High_r=(round(omega_Tmax_r)+1:1:round(300/3.6/radius_wheel)*ratio_r); %[rad/s] Rear motor rotational speed vector after field weakening, up to vehicle speed 300kph
w_vecr=[omega_Low_r,omega_Tmax_r,omega_High_r];     %[rad/s] Rear motor speed vector
T_constantr = Tmax_r*ones(size(omega_Low_r));       %[Nm] Rear motor torque vector within the constant torque region
T_rieldWeakeningr = Pmax_r*1000./omega_High_r;      %[Nm] Rear motor torque vector within the field weakening torque region
Tmax_vecr = [T_constantr,Tmax_r,T_rieldWeakeningr]; %[Nm] Rear motor torque vector corrspanding to the speed vector

%% --- End of file --------------------------------------------------------