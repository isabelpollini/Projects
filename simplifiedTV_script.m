clc
clear
close all

%%%%%%%% SIMPLIFIED TORQUE VECTORING SYSTEM %%%%%%%%


% The 'simplifiedTV.slx' Simulink model is a simple torque distribution 
% algorithm that was thought to work as part of a Simulink 14-gdl-model,
% already implemented by the Dynamis PRC Vehicle Dynamics department.
% 
% Therefore this script is not intendend to support the TV Simulink model 
% alone, but to be added to the general script of the Simulink 14-gdl- 
% model in order to provide the variables needed to make the Torque 
% Vectoring work when attached to the main model.
%
% The folder 'data' must contain:
%           - a struct named Car, containing all of the car specifics 
%           - power and torque curves of the electric motors in the form 
%               of look-up tables
%

% Loading car data
addpath(genpath('data'))
car_DP12e

% Fixed parameters of the car
Vec.maxTorque   = 21;                                                   % [Nm]      maximum torque provided by a single motor
Vec.Tstraight   = 0.5;                                                  % [-]       axle torque distribution on each wheel when going straight
Vec.deltaMax    = 2.618;                                                % [rad]     maximum steering angle   
Vec.mTot        = Car.m;                                                % [kg]      car mass
loadedRadiusF   = 0.227;                                                % [m]       front wheels loaded radius
loadedRadiusR   = 0.223;                                                % [m]       rear wheels loaded radius

% Changable parameters
Vec.TouterMaxF  = 0.7;                                                  % [-]       front axle torque distribution on the outer wheel when turning at maximum steering angle
Vec.TouterMaxR  = 0.7;                                                  % [-]       rear axle torque distribution on the outer wheel when turning at maximum steering angle
Vec.rearI       = 0.4;                                                  % [-]       initial torque distribution on the rear axle
Vec.threshold   = 0.6;                                                  % [-]       throttle request value under which the longitudinal distribution is constant
v               = 90/3.6;                                               % [m/s]     average car speed when throttle request = 100%.

% Calculation of the proportional coefficient between the torque on the 
% outer wheel and the steering angle for each axle    
Vec.alphaF      = (Vec.TouterMaxF - Vec.Tstraight)/Vec.deltaMax;        % [-/rad]     
Vec.alphaR      = (Vec.TouterMaxR - Vec.Tstraight)/Vec.deltaMax;        % [-/rad]   

% Calculation of the rear torque distribution corresponding to throttle
% request = 100% and maximum torque on the rear axle
dragAero        = 0.5*Car.rho*Car.CDA*(v^2);                            % [N]       aerodynamic drag
longForce       = 2*(Car.tau*Vec.maxTorque/loadedRadiusF) +...
                  2*(Car.tau*Vec.maxTorque/loadedRadiusR) - dragAero;   % [N]       total longitudinal force
Ax              = longForce/Vec.mTot;                                   % [m/s^2]   longitudinal acceleration
longitudinalLT  = Vec.mTot*Ax*Car.hCG/Car.wb;                           % [N]       longitudinal load transfer
fzRear          = (1 - Car.mF)*Vec.mTot*9.81 + longitudinalLT;          % [N]       vertical load on the rear axle
Vec.rearF       = (fzRear/9.81)/Vec.mTot;                               % [-]       

