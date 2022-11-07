function [ s ] = ppvector( Structure,t )
%PPVECTOR 
%
%Summary: This function performs the evaluation of a 2D vector spline 
%         interpolation for a given parameter.
%
%Input:   Structure - Spline structures for each vector components
%         t         - Value of the parameter at which the vector is found
%
%Output:  s         - Vector value at the parameter t
%
%%
%... Calculate the vector using its X and Y interpolating splines
s(1,1) = ppval(Structure(1,1),t);
s(2,1) = ppval(Structure(2,1),t);
%
%... Finish function ppvector
end

