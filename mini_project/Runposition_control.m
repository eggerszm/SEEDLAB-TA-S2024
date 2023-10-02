%% Runmotor_control.m
% Zander Eggers
%
% This script runs a simulation of a motor control sytem and plots the
% results with the implemented control system on Arduino Uno.
%
% required files: motor_control.slx, stepData.m
%
close all;

%% Load stepData
load('stepData.mat')

%% Define motor parameters
Kp = 1.5;
Ki = .1;
K = 2.6; % DC gain [rad/Vs]
sigma = 9; % time constant reciprocal [1/s]

%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('position_control')

%
% run the simulation
%

out=sim('position_control.slx');

%% A Plot of the results
% These figures show the simulated vs tested results.
% The experimental results have much more error. This is mostly due to the
% quatization error. The results are within 80% of the expected results and
% the wheel moves smoothly to the human eye. To reduce the error shown, the
% sample time could be increased. For this test the sample time was 10 ms.

figure
subplot(2,1,1)
plot(out.Voltage, '--', 'linewidth', 2)
hold on
plot(data(:,1), data(:,2),'linewidth', 2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')

subplot(2,1,2)
plot(out.Position, 'linewidth', 2)
hold on
plot(data(:,1), data(:,3), 'linewidth', 2)
plot(out.DesiredPosition, '--', 'LineWidth',2)
hold off
legend('Simulated', 'Experimental', 'Desired', 'Location','southeast')
xlabel('Time (s)')
ylabel('Angular Position (rad)')