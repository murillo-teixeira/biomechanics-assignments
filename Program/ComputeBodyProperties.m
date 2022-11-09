function ComputeBodyProperties()
global NBody Body 

f2 = table2array(readtable('..\Material\Kinematics & Dynamics\trial_0001_static_f_2.tsv', 'FileType','text'));
Mass = mean(f2(:,5))/9.81;

for i = 1 : NBody
    Body(i).mass = Body(i).mass * Mass;
    Body(i).rg = Body(i).mass *(Body(i).rg * Body(i).Length)^2;
end
end