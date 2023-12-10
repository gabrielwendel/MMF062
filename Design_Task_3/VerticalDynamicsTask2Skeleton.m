%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 2
% 
%
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

InitParametersSkeleton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 2.1
%
% Front wheel
%
% Calculate road spectrum
roadSpectrum = zeros(length(angularFrequencyVector),1);

for i = 1 : length(angularFrequencyVector)
    % Equation 5.29 from the compendium
    roadSpectrum(i,:) = vehicleVelocity^(roadWaviness-1)*roadSeverity*angularFrequencyVector(i)^(-roadWaviness);
end

% Calculate transfer functions for front wheel Zr to Ride and Tyre force
% You can use the code from Task 1

% Consider one single front wheel (same expressions as Task 1)
sprungMassFront = 0.5*(distanceCogToRearAxle/wheelBase)*totalSprungMass;
unsprungMassFront =  0.25*totalUnsprungMass;

% Identify A and B matrix (same matrices as task 1)

Af =  [0 0 1 0; 
       0 0 0 1;
       -(tireStiff + frontWheelSuspStiff)/unsprungMassFront frontWheelSuspStiff/unsprungMassFront -frontWheelSuspDamp/unsprungMassFront frontWheelSuspDamp/unsprungMassFront;
       frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
Bf =  [0; 0; tireStiff/unsprungMassFront; 0];

% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 3) front wheel Zr to Tyre force
% similar to Task 1 define C and D matrices for both cases
C1f = [frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
D1f = 0;

C3f = [-tireStiff 0 0 0];
D3f = tireStiff;

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)| (same as task 1)
    transferFunctionFrontZrToRide(j,:) = C1f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D1f;
    transferFunctionFrontZrToForce(j,:) = C3f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D3f;
end

% Calculate acceleration and tyre force response spectrum
psdAcceleration = zeros(length(angularFrequencyVector),1);
psdForce = zeros(length(angularFrequencyVector),1);

for m = 1 : length(angularFrequencyVector)
    % Equation 5.30 from compendium
    psdAcceleration(m,:) = abs(transferFunctionFrontZrToRide(m))^2*roadSpectrum(m);
    psdForce(m,:) = abs(transferFunctionFrontZrToForce(m))^2*roadSpectrum(m);
end

% Calculate rms values of acceleration and tyre force
msAcceleration = 0;
msForce = 0;

for n = 1 : length(angularFrequencyVector)
    % Equation 3.51 from compendium
    msAcceleration = msAcceleration + psdAcceleration(n)*deltaAngularFrequency;
    msForce = msForce + psdForce(n)*deltaAngularFrequency;
end

rmsAcceleration = sqrt(msAcceleration);
rmsForce = sqrt(msForce);

figure(100);
semilogy(frequencyVector,psdAcceleration);grid
xlabel('Frequency [Hz]');
ylabel('Acceleration [(m/s^2)^2/Hz]');
title('Sprung mass acceleration PSD');
legend(['rms value = ',num2str(rmsAcceleration)])

figure;
semilogy(frequencyVector,psdForce);grid
xlabel('Frequency [Hz]');
ylabel('Acceleration [(N)^2/Hz]');
title('Tyre force PSD');
legend(['rms value = ',num2str(rmsForce)])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 2.2
%
% Front wheel
rmsAcceleration = zeros(length(frontWheelSuspStiffVector),length(frontWheelSuspDampVector));
rmsForce = zeros(length(frontWheelSuspStiffVector),length(frontWheelSuspDampVector));

for ind1 = 1 : length(frontWheelSuspStiffVector)

    for ind2 = 1 : length(frontWheelSuspDampVector)

        % Update suspension stiffness and damping
        frontWheelSuspStiff = frontWheelSuspStiffVector(ind1);
        frontWheelSuspDamp = frontWheelSuspDampVector(ind2);
        % Update A and C matrices 
        Af =  [0 0 1 0; 
               0 0 0 1;
               -(tireStiff + frontWheelSuspStiff)/unsprungMassFront frontWheelSuspStiff/unsprungMassFront -frontWheelSuspDamp/unsprungMassFront frontWheelSuspDamp/unsprungMassFront;
               frontWheelSuspStiff/sprungMassFront -frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront -frontWheelSuspDamp/sprungMassFront];
        
        % Calculate transfer functions for front wheel Zr to Ride and Tyre force

            transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
            transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

            for j = 1 : length(angularFrequencyVector)
                % Calculate H(w) not the absolut value |H(w)|
                transferFunctionFrontZrToRide(j,:) = C1f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D1f;
                transferFunctionFrontZrToForce(j,:) = C3f*inv(1i*angularFrequencyVector(j)*eye(4) - Af)*Bf + D3f;
            end


        % Calculate acceleration and tyre force response spectrum
        psdAcceleration = zeros(length(angularFrequencyVector),1);
        psdForce = zeros(length(angularFrequencyVector),1);

        for m = 1 : length(angularFrequencyVector)
            psdAcceleration(m,:) = abs(transferFunctionFrontZrToRide(m))^2*roadSpectrum(m);
            psdForce(m,:) = abs(transferFunctionFrontZrToForce(m))^2*roadSpectrum(m);
        end

        % Calculate rms values of acceleration and tyre force
        msAcceleration = 0;
        msForce = 0;

        for n = 1 : length(angularFrequencyVector)
            msAcceleration = msAcceleration + psdAcceleration(n)*deltaAngularFrequency;
            msForce = msForce + psdForce(n)*deltaAngularFrequency;
        end

        rmsAcceleration(ind1,ind2) = sqrt(msAcceleration);
        rmsForce(ind1,ind2) = sqrt(msForce);

    end
end

% Plot rms values vs damping
figure;
plot(frontWheelSuspDampVector,rmsAcceleration);grid
legend(num2str(frontWheelSuspStiffVector'));
xlabel('Damping coefficient [Ns/m]');
ylabel('Acceleration RMS [m/s^2]');
title('Sprung mass acceleration vs chassis damping for various spring stiffness');
axis([0 10000 0 3]);

figure;
plot(frontWheelSuspDampVector,rmsForce);grid
legend(num2str(frontWheelSuspStiffVector'));
xlabel('Damping coefficient [Ns/m]');
ylabel('Tyre Force RMS [N]');
title('Dynamic tyre force vs chassis damping for various spring stiffness');
axis([0 10000 0 1400]);

% Identify optimal damping values for each stiffness value
[optimalRmsAcceleration,iOptimalRmsAcceleration] = min(rmsAcceleration,[],2);
[optimalRmsForce,iOptimalRmsForce] = min(rmsForce,[],2);

% Plot optimal damping for Ride and Tyre force vs Stiffness
figure;
% Use optimal rms acc/force index
plot(frontWheelSuspStiffVector, frontWheelSuspDampVector(iOptimalRmsAcceleration));
plot(frontWheelSuspStiffVector, frontWheelSuspDampVector(iOptimalRmsForce));
grid
xlabel('Spring stiffness [N/m]'); 
ylabel('Damping coefficient [Ns/m]');
title('Optimal damping vs spring stiffness');
legend('rmsAcce','rmsForce');
axis([10000 65000 0 4000]);

