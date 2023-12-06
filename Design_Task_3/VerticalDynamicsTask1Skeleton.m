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

run('InitParametersSkeleton.m')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.3
%
% Consider one single front wheel, identify sprung mass and unsprung mass

sprungMassFront = 0.5*totalSprungMass * (wheelBase - distanceCogToFrontAxle) / wheelBase;
unsprungMassFront = 0.5*totalUnsprungMass * (wheelBase - distanceCogToFrontAxle) / wheelBase;

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
C1f = frontWheelSuspStiff/sprungMassFront*[1 -1 1 -1];
D1f = 0;

% matrices for Zr to Suspension travel, front wheel:
C2f = [-1 0 0 0];
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

sprungMassRear = 0.5 * totalSprungMass*distanceCogToFrontAxle / wheelBase;
unsprungMassRear = 0.5 * totalUnsprungMass*distanceCogToFrontAxle / wheelBase;

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
C1r = rearWheelSuspStiff/sprungMassRear*[1 -1 1 -1];
D1r = D1f;

% matrices for Zr to Suspension travel, rear wheel:
C2r = C2f;
D2r = D2f;

% matrices for Zr to Zr to Tyre force, rear wheel:
C3r = C3f;
D3r = D3f;

% Rear wheel
transferFunctionRearZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToForce = zeros(length(angularFrequencyVector),1);


for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionRearZrToRide(j,:) = C1r*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Br + D1r;
    transferFunctionRearZrToTravel(j,:) = C2r*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Br + D2r;
    transferFunctionRearZrToForce(j,:) = C3r*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Br + D3r;
end

% Plot the transfer functions
figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToRide)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToRide)),'--r');
axis([0 50 -10 60]);grid
legend('Front','Rear','Location','northwest');
ylabel('Magnitude [dB]');%ADD YOUR CODE HERE
xlabel('f [Hz]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Ride Comfort');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToTravel)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToTravel)),'--r');
axis([0 50 -50 10]);grid
legend('Front','Rear','Location','northwest');
ylabel('Magnitude [dB]');%ADD YOUR CODE HERE
xlabel('f [Hz]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Suspension Travel');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToForce)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToForce)),'--r');
axis([0 50 40 115]);grid
legend('Front','Rear','Location','northwest');
ylabel('Magnitude [dB]');%ADD YOUR CODE HERE
xlabel('f [Hz]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Road Grip');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Task 1.4
% %
% % Identify natural frequencies
% 
% % Front wheel
% resonanceFreqFrontBounce = %ADD YOUR CODE HERE
% resonanceFreqFrontHop = %ADD YOUR CODE HERE
% 
% % Rear wheel
% resonanceFreqRearBounce = %ADD YOUR CODE HERE
% resonanceFreqRearHop = %ADD YOUR CODE HERE
