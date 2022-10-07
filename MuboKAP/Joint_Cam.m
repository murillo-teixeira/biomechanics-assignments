function [Phi,Jac,niu,gamma] = Joint_Cam (Phi,Jac,niu,gamma,k,time)
%Joint_Cam
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a cam
%         follower pair. Three types of cams are considered: point,
%         flat-faced and roller follower.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         Nline   - Number of the line for the first constraint equation
%         Body    - Body information
%         Jnt.Cam - Cam information
%         k       - Number of the Cam joint
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.Cam - Cam information
%         NBodyCoordinates - 3x number of bodies
%
%
%%
%... Access global memory
global H NConstraints NCoordinates NBodyCoordinates A90
global Flag Nline Body Jnt
global spline_type
%%
%... Read input for this joint
if Flag.ReadInput == 1
    Nline               = Nline + 1;
    Jnt.Cam(k).type     = H(Nline,1);
    Jnt.Cam(k).i        = H(Nline,2);
    Jnt.Cam(k).j        = H(Nline,3);
    Jnt.Cam(k).angle    = H(Nline,4);
    Jnt.Cam(k).roller   = H(Nline,5)^2;
    Jnt.Cam(k).spPj     = H(Nline,6:7)';
    Jnt.Cam(k).spQj     = H(Nline,8:9)';
    Jnt.Cam(k).order    = H(Nline,10);
    x                   = H(Nline,11);
    Jnt.Cam(k).Filename = sprintf(string('Cam_%03d.txt'),x);
    return
end
%%
%... Initialize data for this joint
if Flag.InitData == 1
    NConstraints      = NConstraints + 2;
    NCoordinates      = NCoordinates + 1;
%
%... Read data from input file
    H   = dlmread(Jnt.Cam(k).Filename);
%
%... Evaluate the coordinates of the Cam surface control points
    angle  = H(:,1);
    radius = H(:,2);
    x      = radius.*cos(angle);
    y      = radius.*sin(angle);
%
%... Evaluate the spline interpolation and its derivatives
    spline_type         = 2;
    Jnt.Cam(k).spP(1,1) = DSM_spline(angle',x',Jnt.Cam(k).order);
    Jnt.Cam(k).spP(2,1) = DSM_spline(angle',y',Jnt.Cam(k).order);
    Jnt.Cam(k).tpP(1,1) = fnder(Jnt.Cam(k).spP(1));
    Jnt.Cam(k).tpP(2,1) = fnder(Jnt.Cam(k).spP(2));
    Jnt.Cam(k).npP(1,1) = fnder(Jnt.Cam(k).tpP(1));
    Jnt.Cam(k).npP(2,1) = fnder(Jnt.Cam(k).tpP(2));
    Jnt.Cam(k).hpP(1,1) = fnder(Jnt.Cam(k).npP(1));
    Jnt.Cam(k).hpP(2,1) = fnder(Jnt.Cam(k).npP(2));
    %
    %... For flat faced cams evaluate the normal vector the the face
    if (Jnt.Cam(k).type == 3)
        Jnt.Cam(k).npj = A90*(Jnt.Cam(k).spPj-Jnt.Cam(k).spQj);
    end
    return
end
%%
%... Line numbers of the constraint equations & Pointer of next constraints
i1      = Nline;
i2      = i1 + 1;
Nline   = Nline + 2;
%%
%... Contribution to the r.h.s. of the velocity equations is null
if Flag.Velocity == 1
    niu(i1:i2,1) = 0.0;
    if (Flag.Jacobian == 0); return; end
end
%%
%... Initialize variables
i               = Jnt.Cam(k).i;
j               = Jnt.Cam(k).j;
%
%... Evaluate common quantities used in constraint 
Jnt.Cam(k).spPi = ppvector(Jnt.Cam(k).spP,Jnt.Cam(k).angle);
Jnt.Cam(k).tpPi = ppvector(Jnt.Cam(k).tpP,Jnt.Cam(k).angle);
tPi             = Body(i).A*Jnt.Cam(k).tpPi;
sPi             = Body(i).A*Jnt.Cam(k).spPi;
sPj             = Body(j).A*Jnt.Cam(k).spPj;
d               = Body(i).r + sPi - Body(j).r - sPj;
if (Jnt.Cam(k).type == 3); nj = Body(j).A*Jnt.Cam(k).npj; end
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    switch Jnt.Cam(k).type
        case 1    % point follower cam
            Phi(i1:i2,1) = d;
        case 2    % roller follower cam
            Phi(i1:i2,1) = [d'*tPi; ...
                            d'*d-Jnt.Cam(k).roller];
        case 3    % flat-face follower cam
            Phi(i1:i2,1) = [tPi'*nj; ...
                              d'*nj];
    end
end
%%
%... Evaluate common quantities in velocity and acceleration constraints
if (Flag.Jacobian == 1 || Flag.Acceleration == 1)
    BspPi           = Body(i).B*Jnt.Cam(k).spPi;
    BspPj           = Body(j).B*Jnt.Cam(k).spPj;
 	BtpPi           = Body(i).B*Jnt.Cam(k).tpPi;
    Jnt.Cam(k).npPi = ppvector(Jnt.Cam(k).npP,Jnt.Cam(k).angle);
	gPi             = Body(i).A*Jnt.Cam(k).npPi;
    if (Jnt.Cam(k).type == 3); Bnpj = Body(j).B*Jnt.Cam(k).npj; end
end
%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    j5 = NBodyCoordinates + k;	
%    
    switch Jnt.Cam(k).type
        case 1    % point follower cam
            Jac(i1:i2,j1:j2) = [ eye(2), BspPi];
            Jac(i1:i2,j3:j4) = [-eye(2),-BspPj];
            Jac(i1:i2,j5:j5) = [tPi];
        case 2    % roller follower cam
            Jac(i1:i2,j1:j2) = [ tPi', tPi'*BspPi+d'*BtpPi; ...
			                     2*d', 2*d'*BspPi];
            Jac(i1:i2,j3:j4) = [-tPi',-tPi'*BspPj; ...
			                    -2*d',-2*d'*BspPj];
            Jac(i1:i2,j5:j5) = [d'*gPi+tPi'*tPi; ...
			                       2*d'*tPi];
        case 3    % flat-face follower cam
            Jac(i1:i2,j1:j2) = [ 0, 0, nj'*BtpPi; ...
			                      nj', nj'*BspPi];
            Jac(i1:i2,j3:j4) = [ 0, 0, tPi'*Bnpj; ...
			                     -nj', d'*Bnpj-nj'*BspPj ];
            Jac(i1:i2,j5:j5) = [nj'*gPi; ...
			                    nj'*tPi];
    end
end
%%
%... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
%
%... Evaluate common quantities in velocity and acceleration constraints
    tid2  = Body(i).thetad.^2;
    tjd2  = Body(j).thetad.^2;
    tdad  = Body(i).thetad*Jnt.Cam(k).angled;
    ad2   = Jnt.Cam(k).angled^2;
    BnpPi = Body(i).B*Jnt.Cam(k).npPi;
    if Jnt.Cam(k).type>1
        Jnt.Cam(k).hpPi = ppvector(Jnt.Cam(k).hpP,Jnt.Cam(k).angle);
        hPi             = Body(i).A*Jnt.Cam(k).hpPi;
        dd              = Body(i).rd + BspPi*Body(i).thetad - ...
                          Body(j).rd - BspPj*Body(j).thetad;
        tPid            = BtpPi*Body(i).thetad + gPi*Jnt.Cam(k).angled;
    end
%
%... For each cam type    
    switch Jnt.Cam(k).type
        case 1    % point follower cam
            gamma(i1:i2,1) = sPi*tid2-sPj*tjd2-2*BtpPi*tdad-gPi*ad2;
        case 2    % roller follower cam
            aux            = sPi*tid2-2*BtpPi*tdad-gPi*ad2-sPj*tjd2;
            gamma(i1:i2,1) = [tPi'*aux-2*dd'*tPid+...
                              d'*(tPi*tid2-2*BnpPi*tdad-hPi*ad2); ...
                              -dd'*dd+d'*aux];
        case 3    % flat-face follower cam
            njd            = Body(j).B*Jnt.Cam(k).npj*Body(j).thetad;
            gamma(i1:i2,1) = [tPi'*nj*tid2-2*njd'*tPid-...
                              nj'*(-tPi*tid2+2*BnpPi*tdad+hPi*ad2); ...
                              d'*nj*tid2-2*dd'*njd-...
                             nj'*(gPi*ad2-sPi*tid2+2*BtpPi*tdad+sPj*tjd2)];
    end
end
%
%... Finish function Joint_Cam
end
    