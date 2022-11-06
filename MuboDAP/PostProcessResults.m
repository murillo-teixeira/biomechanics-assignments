function PostProcessResults (t,y,Filename)
%ReadInputData
%
%Summary: This function controls controls the processing of the results of
%         the model analysis and generates their graphs and general reports
%
%Input:   t        - Vector with the time steps of the analysis
%         y        - Vector with history of positions and velocities
%         Filename - Filename of input data
%
%Output:  No output specified 
%
%
%%
%... Access memory
global NBody Body NCoordinates NCoord1
global Ntime Pts Jnt NJoint Flag
global lambda acc
%
global w
%
%% ... Rename and open report files
[~,name,~]      = fileparts(Filename);
FileOutput      = strcat(name,'.out');
FilePoInterest  = strcat(name,'.poi');
FileJntReaction = strcat(name,'.jnt');
%
fileOUT = fopen(FileOutput,'w');
filePOI = fopen(FilePoInterest,'w');
fileJNT = fopen(FileJntReaction,'w');
%
%% ... Create headers for the report files
%
% ... Header for the output file with bodies kinematics
str0  = '...Time...|';
str1  = '....................................................';
str2  = '..................Body (';
str3  = ')...................';
str4  =['.......X........|.......Y........|.......O........|'];
str5  =['.......Xd.......|.......Yd.......|.......Od.......|'];
str6  =['.......Xdd......|.......Ydd......|.......Odd......|'];
strA  = str0;
strB  = '          |';
for i=1:NBody
    str  = sprintf('%03d',i);
    strA = strcat(strA,'.',str1,str2,str,str3,str1,'|');
    strB = strcat(strB,str4,str5,str6);
end
fprintf(fileOUT, '%s\n',strA);
fprintf(fileOUT, '%s\n',strB);
%
% ... Header for the output file with Points of Interest
str1 = '..............';
str2 = 'Point of Interest (';
str3 = ')...';
str4 = 'Body (';
str5 = 'Coordinates (';  
str6 = ')';  
str7 = '.......X........|.......Y........|';
str8 = '.......Xd.......|.......Yd.......|';
str9 = '.......Xdd......|.......Ydd......|';
strA = str0;
strB = '          |';
for k=1:Pts.NPointsInt
    stra = sprintf('%03d',k);
    strb = sprintf('%03d',Pts.Int(k).i);
    strc = sprintf('%8.4f , %8.4f',Pts.Int(k).spPi);
    strA = strcat(strA,'.',str1,str2,stra,str3,str4, strb, str3, ...
                  str5,strc,str6,str1,'|');
    strB = strcat(strB,str7,str8,str9);
end
fprintf(filePOI, '%s\n',strA);
fprintf(filePOI, '%s\n',strB);
%
% ... Header for the output file with Joint Reactions
%       12345678901234567890123456789012345678901234
str1 = '..............';
str2 = '...Joint (';
str3 = ').....';
str4 = 'Type (';
str5 = '......Body (';
str6 = ')';  
str7 = '.......fX.......|.......fY.......|......Mom.......|';
strA = str0;
strB = '          |';
strC = '          |';
for k=1:Jnt.NReaction
    stra = sprintf('%03d',k);
    strb = sprintf('%s'  ,Jnt.Reaction(k).Type);
    strc = sprintf('%03d',Jnt.Reaction(k).Number);
%
    strd = sprintf('%03d',Jnt.Reaction(k).i);
    strB = strcat(strB,'.',str1,str5,strd,str3,str1,'|');
    strC = strcat(strC,str7);
    if Jnt.Reaction(k).j == 0
        strA = strcat(strA,'.',str2,stra,str3,...
                               str4,strb,strc,str3,'|');
    else
        strd = sprintf('%03d',Jnt.Reaction(k).j);
        strA = strcat(strA,'.',str1,str1,str2,stra,str3,...
                               str4,strb,strc,str6,str1,str1,'|');
        strB = strcat(strB,'.',str1,str5,strd,str3,str1,'|');
        strC = strcat(strC,str7);
    end
%
end
fprintf(fileJNT, '%s\n',strA);
fprintf(fileJNT, '%s\n',strB);
fprintf(fileJNT, '%s\n',strC);
%
%% ... Create waitbar
w       = waitbar(0,'Report Progress');
%
%% ... Build the time history of accelerations, joint forces & PoI
Flag.Position     = 0;
Flag.Velocity     = 0;
Flag.Jacobian     = 1;
Flag.General      = 1;
%
for k=1:Ntime
    Flag.Transfer     = 1;
    Flag.Acceleration = 1;
    Flag.Reaction     = 0;
    [~]               = FuncEval(t(k,1),y(k,:)');
    q  (:,k)          = y(k,1:NCoordinates)';
    qd (:,k)          = y(k,NCoord1:end)';
    qdd(:,k)          = acc;
%
% ... Print information on Output file with the bodies kinematics
    fprintf(fileOUT, '%10.4f',t(k,1));
    for i=1:NBody
%
% ... Find the acceleration of the rigid bodies
        i1 = 3*i - 2;
        i2 = i1 + 1;
        i3 = i2 + 1;
        Body(i).rdd     = qdd(i1:i2,k);
        Body(i).thetadd = qdd(i3:i3,k);
%
        fprintf(fileOUT, '%17.9d',Body(i).r  ,Body(i).theta  );
        fprintf(fileOUT, '%17.9d',Body(i).rd ,Body(i).thetad );
        fprintf(fileOUT, '%17.9d',Body(i).rdd,Body(i).thetadd);
    end
    fprintf(fileOUT, '\n');
%
% ... Print information on Points of Interest file
    PointsOfInterest(k);
    fprintf(filePOI, '%10.4f',t(k,1));
    for n=1:Pts.NPointsInt
        fprintf(filePOI, '%17.9d',Pts.Int(n).q(:,k));
        fprintf(filePOI, '%17.9d',Pts.Int(n).qd(:,k));
        fprintf(filePOI, '%17.9d',Pts.Int(n).qdd(:,k));
    end
    fprintf(filePOI, '\n');
%
% ... Print information on Joint Reaction Forces file
%
%... Evaluate the Joint Reaction Forces
    NJoint            = 0;
    Flag.Transfer     = 0;
    Flag.Acceleration = 0;
    Flag.Reaction     = 1;
    [~,~,~,~]         = KinemEval(t(k,1),[],[]);
%
%... Print the Joint Reaction forces to file
    fprintf(fileJNT, '%10.4f',t(k,1));
    for NJoint=1:Jnt.NReaction
        fprintf(fileJNT, '%17.9d',Jnt.Reaction(NJoint).gi);
        if Jnt.Reaction(NJoint).j ~= 0
            fprintf(fileJNT, '%17.9d',Jnt.Reaction(NJoint).gj);
        end
    end
    fprintf(fileJNT, '\n');
end
fclose(fileOUT);
fclose(filePOI);
fclose(fileJNT);
%
%% Build the animation using the time history of the positions
% PlotModel(q);
% PlotSeatModel(q);
%
%%
%... Make the general plots
for k = 1:Pts.NPointsInt
%
%... Plot position vs time
    plot(t,Pts.Int(k).q(1,:),'k',t,Pts.Int(k).q(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Position'])
    figure
%
%... Plot the trajectory
    plot(Pts.Int(k).q(1,:),Pts.Int(k).q(2,:),'k')
    title(['Point of interest ' num2str(k) ' - Trajectory'])
    figure
%
%... Plot Velocities
    plot(t,Pts.Int(k).qd(1,:),'k',t,Pts.Int(k).qd(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Velocity'])
    figure
%
%... Plot Accelerations
    plot(t,Pts.Int(k).qdd(1,:),'k',t,Pts.Int(k).qdd(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Acceleration'])
    figure
end
%
%
%%
%... Finish function PreProcessData
end
