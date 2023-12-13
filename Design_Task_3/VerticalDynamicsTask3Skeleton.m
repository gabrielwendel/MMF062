%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 3

clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

InitParametersSkeleton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 3.1

numberOfTripsPerDay = totalDrivingTime/...
    (distanceSmoothRoad/vehicleVelocitySmooth+distanceRoughRoad/vehicleVelocityRough+distanceVeryRoughRoad/vehicleVelocityVeryRough);

% i)
% Calculate road spectrum
roadSpectrumSmooth = zeros(length(angularFrequencyVector),1);
roadSpectrumRough = zeros(length(angularFrequencyVector),1);
roadSpectrumVeryRough = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    roadSpectrumSmooth(j,:) = vehicleVelocitySmooth^(roadWavinessSmooth-1)*roadSeveritySmooth*angularFrequencyVector(j)^(-roadWavinessSmooth);
    roadSpectrumRough(j,:) = vehicleVelocityRough^(roadWavinessRough-1)*roadSeverityRough*angularFrequencyVector(j)^(-roadWavinessRough);
    roadSpectrumVeryRough(j,:) = vehicleVelocityVeryRough^(roadWavinessVeryRough-1)*roadSeverityVeryRough*angularFrequencyVector(j)^(-roadWavinessVeryRough);
end

% Calculate transfer functions for front wheel Zr to Ride
% You can use result from Task 1

sprungMassFront = 0.5*(distanceCogToRearAxle/wheelBase)*totalSprungMass;
unsprungMassFront =  0.25*totalUnsprungMass;

Af =  [0 0 1 0; 
       0 0 0 1;
       -(tireStiff + frontWheelSuspStiff)/unsprungMassFront frontWheelSuspStiff/unsprungMassFront -frontWheelSuspDamp/unsprungMassFront frontWheelSuspDamp/unsprungMassFront;
       frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
Bf =  [0; 0; tireStiff/unsprungMassFront; 0];

C1f = [frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
D1f = 0;

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    transferFunctionFrontZrToRide(j,:) = C1f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D1f;
end

% Calculate acceleration response spectrum for all roads

psdAccelerationSmooth = zeros(length(angularFrequencyVector),1);
psdAccelerationRough = zeros(length(angularFrequencyVector),1);
psdAccelerationVeryRough = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    psdAccelerationSmooth(j,:) = abs(transferFunctionFrontZrToRide(j))^2*roadSpectrumSmooth(j);
    psdAccelerationRough(j,:) = abs(transferFunctionFrontZrToRide(j))^2*roadSpectrumRough(j);
    psdAccelerationVeryRough(j,:) = abs(transferFunctionFrontZrToRide(j))^2*roadSpectrumVeryRough(j);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ii)
% Calculate rms values weighted according to ISO2631 
[weightedRmsAccelerationSmooth] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationSmooth)
[weightedRmsAccelerationRough] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationRough)
[weightedRmsAccelerationVeryRough] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationVeryRough)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% iii)
% Calculate time averaged vibration exposure value for 8h period

timeOnSmoothRoad = (distanceSmoothRoad / vehicleVelocitySmooth)*numberOfTripsPerDay;
timeOnRoughRoad = (distanceRoughRoad / vehicleVelocityRough)*numberOfTripsPerDay;
timeOnVeryRoughRoad = (distanceVeryRoughRoad / vehicleVelocityVeryRough)*numberOfTripsPerDay;

timeWeightedMsAcceleration = (weightedRmsAccelerationSmooth^2*timeOnSmoothRoad + ...
    weightedRmsAccelerationRough^2*timeOnRoughRoad + weightedRmsAccelerationVeryRough^2*timeOnVeryRoughRoad) ...
    / (timeOnSmoothRoad + timeOnRoughRoad + timeOnVeryRoughRoad);

timeWeightedRmsAcceleration = sqrt(timeWeightedMsAcceleration);

disp(['timeWeightedRmsAcceleration =' num2str(timeWeightedRmsAcceleration),' m/s2 (1.15)',...
    ', numberOfTripsPerDay = ' num2str(numberOfTripsPerDay)]);
disp(['vehicleVelocitySmooth = ' num2str(vehicleVelocitySmooth*3.6),' km/h',...
    ', vehicleVelocityRough = ' num2str(vehicleVelocityRough*3.6),' km/h',...
    ', vehicleVelocityVeryRough = ' num2str(vehicleVelocityVeryRough*3.6),' km/h'])


