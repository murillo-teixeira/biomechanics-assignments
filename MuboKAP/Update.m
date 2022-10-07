function [ ] = Update()
%Update
%
%Summary: This function updates the values for the path dependent 
%         quantities.
%
%Input:   None
%
%Output:  None
%
%%
%... Access the global memory
global Jnt
%
%%
%... Evaluate bounds for the virtue function of cams
for k = 1:Jnt.NCam
    Jnt.Cam(k).LoAngle = Jnt.Cam(k).angle - 0.7854;
    Jnt.Cam(k).HiAngle = Jnt.Cam(k).angle + 0.7854;
end
%
%%
%... Finalize function Update
end