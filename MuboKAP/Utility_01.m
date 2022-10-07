fileID = fopen('FourBarDriver.txt', 'w');
H = [t',theta'];
fprintf(fileID,'%12.8f %12.8f\n', H);
fclose(fileID);
%
global spline_type
%
k  = 1;
time = 0.02324;
Nline = 0;
H = [3  2  0  3     1.000  0.000 -2.000  0.000  0];
%
%
    Nline                  = Nline + 1;
    Jnt_Driver(k).type     = H(Nline,1);
    Jnt_Driver(k).i        = H(Nline,2);
    Jnt_Driver(k).coortype = H(Nline,3);
    Jnt_Driver(k).j        = H(Nline,4);
%
    Jnt_Driver(k).spPi     = H(Nline,5:6)';
    Jnt_Driver(k).spPj     = H(Nline,7:8)';
    Jnt_Driver(k).Filename = 'FourBarDriver.txt'; 
%
    H                       = dlmread(Jnt_Driver(k).Filename);
    spline_type             = 1;
%
t = H(:,1);
z = H(:,2);
spline_per = DSM_spline(t',z',order)
t = 0:0.001:1.0;
figure(1);
plot(t,ppval(spline_per,t));
%
    Jnt_Driver(k).Spline    = DSM_spline(H(:,1),H(:,2));
    Jnt_Driver(k).Splined   = fnder(Jnt_Driver(k).Spline);
    Jnt_Driver(k).Splinedd  = fnder(Jnt_Driver(k).Splined);
%
z     = ppval(Jnt_Driver(k).Spline,0)


aux1 = Pts_Int(2).q-Pts_Int(1).q;
aux2 = Pts_Int(4).q-Pts_Int(3).q;
aux3 = Pts_Int(6).q-Pts_Int(5).q;

s1   = sqrt(aux1(1,:).^2+aux1(2,:).^2);
s2   = sqrt(aux2(1,:).^2+aux2(2,:).^2);
s3   = sqrt(aux3(1,:).^2+aux3(2,:).^2);

H1   = [t' s1'];
H2   = [t' s2'];
H3   = [t' s3'];


    H   = dlmread('Cam_001.txt');
%
%... Evaluate the coordinates of the Cam surface control points
    angle  = H(:,1);
    radius = H(:,2);
    x      = radius.*cos(angle);
    y      = radius.*sin(angle);

