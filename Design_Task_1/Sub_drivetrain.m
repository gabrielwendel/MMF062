function [Tdrivf,Tdrivr,gearf,gearr,wengf,wengr]=Sub_drivetrain(wf,wr,gearf_old,gearr_old,CONST)
% This function finds a gear that provides highest engine torque. 
% This function knowing the wheel speed, gear ratios and final gear finds
% all possible engine speeds and corresponding engine torques (based on the 
% given engine map). Then from
% engine torques using all gears calculates all wheel torques (one for
% each gear). Then selects the highest one and corresponding gear as output.
% i.e. Tdriv (for front or rear depending on the value of 
% CONST.torqf). Down-shifting is avoided. /Toheed

% 2023 Introduced indenpedent front and rear motors.

% Read parameters of front & rear motors
w_vecf=CONST.w_vecf;
Tmax_vecf=CONST.Tmax_vecf;
ratio_vecf=CONST.ratio_vecf;

w_vecr=CONST.w_vecr;
Tmax_vecr=CONST.Tmax_vecr;
ratio_vecr=CONST.ratio_vecr;

% Maximum engine speed
w_engf_max=CONST.w_vecf(end);
w_engr_max=CONST.w_vecr(end);

% Engine speed at maximum torque
[~, bf]=max(Tmax_vecf);
[~, br]=max(Tmax_vecr);

% Minimum engine speed is set to speed at maximum torque
w_engf_min=w_vecf(bf); % Code inherits from ICE, probably can be used for noideal e-motor in the future 
w_engr_min=w_vecr(br); % 

% Engine speed in all gears
w_engf_all_gears = (ratio_vecf*wf);
w_engr_all_gears = (ratio_vecr*wr);

% Set minimum engine speed in all gears
for i=1:length(w_engf_all_gears)    % Only one gear in 2023, can be used for multi-gears in the future     
    if (w_engf_all_gears(i) < w_engf_min)
        w_engf_all_gears(i) = w_engf_min;
    end
end

for i=1:length(w_engr_all_gears)    % Only one gear in 2023, can be used for multi-gears in the future     
    if (w_engr_all_gears(i) < w_engr_min)
        w_engr_all_gears(i) = w_engr_min;
    end
end

% Calculate drive torque in all gears
Tengf_all_gears=interp1(w_vecf,Tmax_vecf,w_engf_all_gears);   % Front motor torque for all gears
Tengr_all_gears=interp1(w_vecr,Tmax_vecr,w_engr_all_gears);   % Rear motor torque for all gears
Tdrivf_all_gears=ratio_vecf.*Tengf_all_gears;                 % Front Tdriv for all gears
Tdrivr_all_gears=ratio_vecr.*Tengr_all_gears;                 % Rear Tdriv for all gears

% Limit drive torque to maximum engine speed
for i=1:length(Tdrivf_all_gears)
    if (w_engf_all_gears(i) > w_engf_max)
        Tdrivf_all_gears(i) = 0;     
    end
end

for i=1:length(Tdrivr_all_gears)
    if (w_engr_all_gears(i) > w_engr_max)
        Tdrivr_all_gears(i) = 0;     
    end
end

% Check to avoid down shifting
[~,gearf]=max(Tdrivf_all_gears);  % Front axle Max Tdriv and corresponding gear
[~,gearr]=max(Tdrivr_all_gears);  % Rear axle Max Tdriv and corresponding gear
gearf = max(gearf, gearf_old);    % Select higher gear or use previous gear
gearr = max(gearr, gearr_old);    % Select higher gear or use previous gear
switch CONST.drive_mode
    case 0 %RWD
        Tdrivf = 0;
        Tdrivr = Tdrivr_all_gears(gearr);  % Set drive torque for selected gear
    case 1 %FWD
        Tdrivf = Tdrivf_all_gears(gearf);  % Set drive torque for selected gear
        Tdrivr = 0;
    case 2 %AWD
        Tdrivf = Tdrivf_all_gears(gearf);  % Set drive torque for selected gear
        Tdrivr = Tdrivr_all_gears(gearr);  % Set drive torque for selected gear
    otherwise
        Tdrivf = 0;
        Tdrivr = 0;
end

wengf=w_engf_all_gears(gearf);         % Set engine speed for selected gear
wengr=w_engr_all_gears(gearr);         % Set engine speed for selected gear
end