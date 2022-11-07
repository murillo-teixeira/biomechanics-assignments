function ComputeBodyProperties()
global NBody Body 

Mass = 62.5;    % Slide 70 - week 05

for i = 1 : NBody
    Body(i).mass = Body(i).mass * Mass;
    Body(i).rg = Body(i).mass *(Body(i).rg * Body(i).Length)^2;
end
end