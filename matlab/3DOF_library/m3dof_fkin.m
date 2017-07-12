%%
% Filename: m3dof_fkin.m
% Desc: calculates the forward kinematics for the planar 3DOF robot
% INPUT: 
% q (3x1): joint angles in radians
% l (3x1): length of the links 
% OUTPUT:
% f (3x1): planar position [f(1:2)] and rotation angle in radians [f(3)] of EE 
%
% 2015 alessandro.giordano@dlr.de
%%

function [f] = m3dof_fkin(q,l)
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

% EE position and rotation
f(1,1) = l1*c1 + l2*c12 + l3*c123;
f(2,1) = l1*s1 + l2*s12 + l3*s123;
f(3,1) = q(1) + q(2) + q(3);