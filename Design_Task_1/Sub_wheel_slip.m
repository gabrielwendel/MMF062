function slip=Sub_wheel_slip(v,w,CONST)

%----Calculate the slip----------------------------------------------------
%----Write your solution below---------------------------------------------
slip= (CONST.R*w - v)/(abs(CONST.R*w));
%--------------------------------------------------------------------------
end