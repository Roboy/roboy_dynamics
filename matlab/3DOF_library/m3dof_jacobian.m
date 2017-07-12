%%
% Filename: m3dof_jacobian.m
% Desc: calculates the body Jacobian for the planar 3DOF robot
% INPUT: 
% q (3x1): joint angles in radians
% l (3x1): length of the links 
% OUTPUT:
% J (3x3): body Jacobian expressed in base frame
%
% 2015 alessandro.giordano@dlr.de
%%

function [J] = m3dof_jacobian(q,l)
% Variables extraction
l1 = l(1);
l2 = l(2);
l3 = l(3);

% Angles calculation
c1 = cos(q(1));
s1 = sin(q(1));
c12 = cos(q(1) + q(2));
s12 = sin(q(1) + q(2));
c123 = cos(q(1) + q(2) + q(3));
s123 = sin(q(1) + q(2) + q(3));

% Jacobian calculation
J(1,1) = -l1*s1 -l2*s12 -l3*s123;
J(1,2) = -l2*s12 -l3*s123;
J(1,3) = -l3*s123;
J(2,1) = l1*c1 + l2*c12 + l3*c123;
J(2,2) = l2*c12 + l3*c123;
J(2,3) = l3*c123;
J(3,1) = 1;
J(3,2) = 1;
J(3,3) = 1;