%This MATLAB code is a function that performs the time update step for an 
% error state Kalman filter. It takes several inputs and updates 
% the filter's state and covariance matrices. 
function [dX,P] = KFTimeUpdate(fibb,Cbn, params, AVP, dX, P, Q, ts, Omega, Omegaen)

%   Function for the evaluation of the time update equations of error state
%   Kalman filter
% This line defines a MATLAB function called KFTimeUpdate that takes the following inputs:
% fibb: A vector
% Cbn: A 3x3 matrix
% params: A structure containing various parameters
% AVP: A vector
% dX: A vector
% P: A matrix
% Q: A matrix
% ts: A scalar
% Omegaie: A scalar
% Omegaen: A scalar
% The function returns two outputs: dX and P.
Lat = AVP(7);
Lat = AVP(7);
Alt = AVP(9);
V = AVP(4:6);
fn = Cbn*fibb;

F21 = rotVec2Mat (-Cbn*fibb);
Rn = params.R0* (1-params.eccentricity^2)/(1-params.eccentricity^2*sin (Lat)^2)^1.5; 
Re = params.R0/sqrt (1-params.eccentricity^2*sin (Lat)^2);
F32 = zeros(3);
F32(1,1) = 1/ (Rn + AVP (9));
F32(2,2) = 1/((Re+ AVP (9) *cos (Lat)));
F32(3,3) = -1;
gamma0 = params.gam*(1+params.kgamma*sin(Lat)*sin(Lat))/(sqrt(1-params.eccentricity^2*sin(Lat)*sin(Lat)));%m/s^2
reSe = Re*sqrt(cos(Lat)^2 + (1-params.eccentricity^2)^2*sin(Lat)^2);
F23 = zeros (3);
F23(3,3) = -2*gamma0/reSe;

% F equations 
F = zeros (15); %This line initializes a 15x15 matrix F with all zeros.
F(4:6,1:3) = F21;
F(4:6, 7:9) = F23;
F(7:9,4:6) = F32;
F(1:3, 13:15) = Cbn;
F(4:6, 10:12) = Cbn;

PHI = eye(15) + F*ts;
dX = PHI * dX;
G = eye(15); % Covariance
G(4:6,1:3) = Cbn;
G(7:9,4:6) = -Cbn;

% G(10:12, 10:12) = eye(3);
% G(13:15, 13:15) = eye(3);
P = PHI*P*PHI' + G*Q*G'*ts; %This line updates the covariance matrix P based 
                            % on the time update equations of the Kalman filter.
end

%In summary, this code implements the time update step of an error state 
% Kalman filter, which is used for state estimation in navigation and sensor 
% fusion applications. It involves several matrix operations and calculations
% related to the filter's state and covariance matrices.
