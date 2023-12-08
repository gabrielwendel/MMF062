%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 1
%
% Add your own code where "%ADD YOUR CODE HERE" is stated
%
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

InitParametersSkeleton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.3
%
% Consider one single front wheel, identify sprung mass and unsprung mass

% sprungMassFront = (totalSprungMass * (wheelBase - distanceCogToFrontAxle) / wheelBase)/2;
% unsprungMassFront = (totalUnsprungMass * (wheelBase - distanceCogToFrontAxle) / wheelBase)/2;

sprungMassFront = 0.5*(distanceCogToRearAxle/wheelBase)*totalSprungMass;
unsprungMassFront =  0.25*totalUnsprungMass;

% Identify indiviual A and B matrix
Af =  [0 0 1 0; 
       0 0 0 1;
       -(tireStiff + frontWheelSuspStiff)/unsprungMassFront frontWheelSuspStiff/unsprungMassFront -frontWheelSuspDamp/unsprungMassFront frontWheelSuspDamp/unsprungMassFront;
       frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
Bf =  [0; 0; tireStiff/unsprungMassFront; 0];

% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 2)front wheel Zr to Suspension travel and 
% 3) front wheel Zr to Tyre force

% matrices Zr to Ride,front wheel:
% C1f = 1/sprungMassFront*[frontWheelSuspStiff -frontWheelSuspStiff frontWheelSuspDamp - frontWheelSuspDamp];
C1f = [frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
D1f = 0;

% matrices for Zr to Suspension travel, front wheel:
C2f = [1 -1 0 0];
D2f = 0;

% matrices for Zr to Tyre force, front wheel:
C3f = [-tireStiff 0 0 0];
D3f = tireStiff;

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionFrontZrToRide(j,:) = C1f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D1f;
    transferFunctionFrontZrToTravel(j,:) = C2f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D2f;
    transferFunctionFrontZrToForce(j,:) = C3f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D3f;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Consider one single rear wheel, identify sprung mass and unsprung mass

% sprungMassRear = 0.5*totalSprungMass * (wheelBase - distanceCogToRearAxle) / wheelBase;
% unsprungMassRear = 0.5*totalUnsprungMass * (wheelBase - distanceCogToRearAxle) / wheelBase;

sprungMassRear = 0.5*(distanceCogToFrontAxle/wheelBase)*totalSprungMass;
unsprungMassRear = 0.25*totalUnsprungMass;

% Identify indiviual A and B matrix
Ar =  [0 0 1 0; 
       0 0 0 1;
       -(tireStiff + rearWheelSuspStiff)/unsprungMassRear rearWheelSuspStiff/unsprungMassRear -rearWheelSuspDamp/unsprungMassRear rearWheelSuspDamp/unsprungMassRear;
       rearWheelSuspStiff/sprungMassRear -rearWheelSuspStiff/sprungMassRear rearWheelSuspDamp/sprungMassRear -rearWheelSuspDamp/sprungMassRear];
Br =  [0; 0; tireStiff/unsprungMassRear; 0];

% Calculate transfer functions for 
% 1) rear wheel Zr to Ride, 
% 2) rear wheel Zr to Suspension travel and 
% 3) rear wheel Zr to Tyre force

% matrices Zr to Ride, rear wheel:
C1r = [rearWheelSuspStiff/sprungMassRear -rearWheelSuspStiff/sprungMassRear rearWheelSuspDamp/sprungMassRear -rearWheelSuspDamp/sprungMassRear];
D1r = 0;

% matrices for Zr to Suspension travel, rear wheel:
C2r = [1 -1 0 0];
D2r = 0;

% matrices for Zr to Zr to Tyre force, rear wheel:
C3r = [-tireStiff 0 0 0];
D3r = tireStiff;

% Rear wheel
transferFunctionRearZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToForce = zeros(length(angularFrequencyVector),1);


for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionRearZrToRide(j,:) = C1r*inv(1i*angularFrequencyVector(j)*eye(4) - Ar)*Br + D1r;
    transferFunctionRearZrToTravel(j,:) = C2r*inv(1i*angularFrequencyVector(j)*eye(4) - Ar)*Br + D2r;
    transferFunctionRearZrToForce(j,:) = C3r*inv(1i*angularFrequencyVector(j)*eye(4) - Ar)*Br + D3r;
end

%% Task 1.4
%
% Identify natural frequencies

% Front wheel
resonanceFreqFrontBounce = sqrt((1 /(1 / frontWheelSuspStiff + 1/tireStiff)) / sprungMassFront);
resonanceFreqFrontHop = sqrt((frontWheelSuspStiff + tireStiff) / unsprungMassFront);

% Rear wheel
resonanceFreqRearBounce = sqrt((1 /(1 / rearWheelSuspStiff + 1/tireStiff)) / sprungMassRear);
resonanceFreqRearHop = sqrt((rearWheelSuspStiff + tireStiff) / unsprungMassRear);

% Convert from rad to Hertz
resonanceFreqFrontBounceInHertz = (resonanceFreqFrontBounce)/(2*pi)
resonanceFreqFrontHopInHertz = (resonanceFreqFrontHop)/(2*pi)
resonanceFreqRearBounceInHertz = (resonanceFreqRearBounce)/(2*pi)
resonanceFreqRearHopInHertz = (resonanceFreqRearHop)/(2*pi)

% Plot the transfer functions and natural frequencies
figure(1)
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToRide)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToRide)),'--r');
axis([0 50 -10 60]);grid
legend('Front','Rear','Location','northwest');
ylabel('Magnitude [dB]');%ADD YOUR CODE HERE
xlabel('f [Hz]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Ride Comfort');
hold on
% add vertical lines to plot
xline(resonanceFreqFrontBounceInHertz,'r--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency front bounce');
xline(resonanceFreqFrontHopInHertz,'g--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency front hop');
xline(resonanceFreqRearBounceInHertz,'b--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency rear bounce');
xline(resonanceFreqRearHopInHertz,'m--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency rear hop');

figure(2)
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToTravel)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToTravel)),'--r');
axis([0 50 -50 10]);grid
legend('Front','Rear','Location','northwest');
ylabel('Magnitude [dB]');%ADD YOUR CODE HERE
xlabel('f [Hz]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Suspension Travel');
hold on
% add vertical lines to plot
xline(resonanceFreqFrontBounceInHertz,'r--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency front bounce');
xline(resonanceFreqFrontHopInHertz,'g--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency front hop');
xline(resonanceFreqRearBounceInHertz,'b--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency rear bounce');
xline(resonanceFreqRearHopInHertz,'m--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency rear hop');

figure(3)
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToForce)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToForce)),'--r');
axis([0 50 40 115]);grid
legend('Front','Rear','Location','northwest');
ylabel('Magnitude [dB]');%ADD YOUR CODE HERE
xlabel('f [Hz]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Road Grip');
hold on
% add vertical lines to plot
xline(resonanceFreqFrontBounceInHertz,'r--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency front bounce');
xline(resonanceFreqFrontHopInHertz,'g--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency front hop');
xline(resonanceFreqRearBounceInHertz,'b--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency rear bounce');
xline(resonanceFreqRearHopInHertz,'m--', 'LineWidth', 1.5, 'DisplayName', 'Resonance frequency rear hop');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

