% This MATLAB function, rotVec2Mat, is designed to create a 
% rotation matrix (rotMat) from a given 3-element vector w, 
% which represents the angular velocity or angular rate. 
% This is often used in rigid body dynamics and transformations in 3D space.
% The rotation matrix can be used to perform rotations in 3D space
function [rotMat] = rotVec2Mat(w) %function [rotMat] = rotVec2Mat(w): 
% This line defines a MATLAB function called rotVec2Mat that takes one input 
% argument w and returns an output argument rotMat.

% Rotate transform vector in correspondent skewMatrix
% The purpose of the function is: arrange the enties of the vector angular rate in a simmetric matrix
rotMat = [0 -w(3) w(2)
          w(3) 0 -w(1)
         -w(2) w(1) 0];
%This line is where the skew-symmetric matrix is created. 
% It's a 3x3 matrix constructed from the elements of the input vector w.
% After this line, the function returns rotMat, which is the desired 
% skew-symmetric matrix corresponding to the input angular rate vector w. 
% This matrix can then be used in various applications, such as rigid body 
% transformations or rotations in 3D space.
end

