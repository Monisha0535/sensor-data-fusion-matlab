%This MATLAB is a mechanization function used for updating the state of 
% a vehicle, represented as AVP (Altitude, Velocity, Position), 
% based on various sensor measurements and physical models. 
function [AVP, Cbn, Omegaien, Omegaenn] = AVPMechanization (AVP, wibb, fibb, params, Dt)
%   Mecanization Function -> Input = State of the vehicle at previouse step in termes of 
%   AVP(altitude, velocity and position), Measured angular rates in body frame (wibb), 
%   measured specific force in body frame (fibb),
%   parameters of geometrical value in Project_Uno (params), sampling period (Dt).
% OUTPUT = the vector AVP, that is [heading, pithc, roll, vnorth, veast, vdown, latitude, longitude, height]
%% State vector components
Heading = AVP(1);
Pitch = AVP(2);
Roll= AVP(3);
V = AVP(4:6);
Lat = AVP(7);
Lon = AVP(8);
Alt = AVP(9);

%% Attitude update
Cbn = eul2rotm([Heading, Pitch, Roll],'ZYX'); % Euler to rot matrix for providing the HPR and the rotation sequence
Omegaibb = rotVec2Mat(wibb); % Is the matrix of measured angular rate
wien = [cos(Lat)*params.wie, 0, -sin(Lat)*params.wie]; % Components of earth angular rate in navigation frame
Omegaien = rotVec2Mat(wien); %This line converts the measured angular rates wibb into a skew-symmetric matrix
Rn = params.R0* (1-params.eccentricity^2)/(1-params.eccentricity^2*sin(Lat)^2)^1.5; % Normal radius (meridians)
Re = params.R0/sqrt (1-params.eccentricity^2*sin(Lat)^2); % Earth radius (tangent)
Lat_dot = V(1)/(Rn+Alt); % Time Derivative Lat
Lon_dot = V(2)/(cos(Lat)*(Re+Alt)); % Time Derivative Lon
wenn = [cos(Lat)*Lon_dot, -Lat_dot, -sin(Lat)*Lon_dot]; % Update w(n,en)
Omegaenn = rotVec2Mat(wenn);
Cbn = Cbn*(eye(3)+Omegaibb*Dt) - (Omegaien + Omegaenn)*Cbn*Dt; % Expression for update

%% Specific force transformation and gravity model evaluation
% Takes into account the effect of latitude on the gravity
fibn = Cbn * fibb;
gamma0 = params.gam*(1+params.kgamma*sin(Lat)*sin(Lat))/(sqrt(1-params.eccentricity^2*sin(Lat)*sin(Lat))); % m/s^2 
gamma = gamma0*(sqrt(Rn*Re)/(sqrt(Rn*Re)+Alt))^2; % m/s^2 For take into account the effect of eccentricity on the gravity
gn=[0 0 gamma]'- params.wie^2*(sqrt(Rn*Re)+Alt)*[sin(Lat)*cos(Lat) 0 cos(Lat)*cos(Lat)]'; % m/s^2 Vettore gravità del filo
% Gravity model = contribution of gravity - contribute of centrifugal acceleration

%% Velocity update
% V = previouse value + sensor value + correction of gravity model + coriolis + sample rate
V = V + (fibn + gn - (Omegaenn + 2*Omegaien)*V)*Dt;

%% Position update
Alt = Alt - V(3)*Dt; % Update altitude
Lat = Lat + V(1)/(Rn+Alt)*Dt; % Update latitude
Re = params.R0/sqrt (1-params.eccentricity^2*sin(Lat)^2); % Update value of transfers earth rate not important
Lon = Lon + V(2)/(cos(Lat)*(Re+Alt))*Dt; % Update longitude
AVP(1:3) = rotm2eul(Cbn,'ZYX'); % Update the navigation solution by rotating back the angle from rot to Euler
AVP(4:6) = V;
AVP(7) = Lat;
AVP(8) = Lon;
AVP(9) = Alt;
end

