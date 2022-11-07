function [RefBody, RefMuscles] = ReferenceMuscleData()
%
%Summary: This function builds a structure containing information about the
%         lower limb muscles taken from the work of Stefanie Brandle. The
%         data are according to the biomechanical model of her work. 
%
%Input:   No input is required.
%
%Output:  RefBody      - Structure containing properties of the bodies in
%                      the biomechanical model of Stefanie Brandle.
%         RefMuscles   - Structure containing muscle data
%
%% Definition of body properties
% Foot
RefBody(1).name = 'foot';
RefBody(1).CoM = [0.33 * 0.271 * 0.6; 0]; % CoM translation with respect to what is considered in BM
RefBody(1).A = [0.9337, 0.3579;
    -0.3579, 0.9337]; % Rotation matrix that transforms the reference frame from the model of Stefanie Brandle to that usually considered in BM
RefBody(1).length = (0.271 * 0.6 * 0.6) / cosd(20);
% Leg
RefBody(2).name = 'leg';
RefBody(2).CoM = [0; -.1] * 0.434; % CoM translation with respect to what is considered in BM
RefBody(2).A = [0, 1;
    -1, 0];
RefBody(2).length = 0.434;
% Thigh
RefBody(3).name = 'thigh';
RefBody(3).CoM = [0; 0]; % CoM translation with respect to what is considered in BM
RefBody(3).A = [0, 1;
    -1, 0];
RefBody(3).length = 0.439;
% Torso
RefBody(4).name = 'torso';
RefBody(4).CoM = [0; 0.5 * (0.275 + 0.294) - 0.2327 * 0.275]; % CoM translation with respect to what is considered in BM
RefBody(4).A = [0, 1;
    -1, 0];
RefBody(4).length = 0.275 + 0.294;

%% Definition of the muscle properties
NMuscle = 1;
% Gluteus Maximus Anterior
RefMuscles(NMuscle).name = 'GlutMaxAnt';
RefMuscles(NMuscle).F0 = 382;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.142;
RefMuscles(NMuscle).Ls = 0.125;
RefMuscles(NMuscle).AttachP = [-0.0479, 0.0708;
    -0.0574, 0.0115;
    -0.0612, 0.2331;
    -0.0370, 0.1907];
RefMuscles(NMuscle).AttachBodies = [4; 4; 3; 3];
NMuscle = NMuscle + 1;

% Gluteus Maximus Middle
RefMuscles(NMuscle).name = 'GlutMaxMid';
RefMuscles(NMuscle).F0 = 546;
RefMuscles(NMuscle).pen = deg2rad(0);
RefMuscles(NMuscle).L0 = 0.147;
RefMuscles(NMuscle).Ls = 0.127;
RefMuscles(NMuscle).AttachP = [-0.0631, 0.0277;
    -0.0658, -0.0411;
    -0.0570, 0.1955;
    -0.0209, 0.1304];
RefMuscles(NMuscle).AttachBodies = [4; 4; 3; 3];
NMuscle = NMuscle + 1;

% Gluteus Maximus Posterior
RefMuscles(NMuscle).name = 'GlutMaxPos';
RefMuscles(NMuscle).F0 = 368;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.144;
RefMuscles(NMuscle).Ls = 0.145;
RefMuscles(NMuscle).AttachP = [-0.0835, -0.0207;
    -0.0809, -0.0936;
    -0.0400, 0.1269;
    -0.0081, 0.0763];
RefMuscles(NMuscle).AttachBodies = [4; 4; 3; 3];
NMuscle = NMuscle + 1;

% Adductor Longus
RefMuscles(NMuscle).name = 'AddLong';
RefMuscles(NMuscle).F0 = 418;
RefMuscles(NMuscle).pen = deg2rad(6);
RefMuscles(NMuscle).L0 = 0.138;
RefMuscles(NMuscle).Ls = 0.11;
RefMuscles(NMuscle).AttachP = [0.0390, -0.0723;
    0.0067, -0.0162];
RefMuscles(NMuscle).AttachBodies = [4; 3];
NMuscle = NMuscle + 1;

% Adductor Brevis
RefMuscles(NMuscle).name = 'AddBrev';
RefMuscles(NMuscle).F0 = 286;
RefMuscles(NMuscle).pen = deg2rad(0);
RefMuscles(NMuscle).L0 = 0.133;
RefMuscles(NMuscle).Ls = 0.02;
RefMuscles(NMuscle).AttachP = [0.0122, -0.0801;
    0.0012, 0.1063];
RefMuscles(NMuscle).AttachBodies = [4; 3];
NMuscle = NMuscle + 1;

% Adductor Magnus Superior
RefMuscles(NMuscle).name = 'AddMagSup';
RefMuscles(NMuscle).F0 = 346;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.087;
RefMuscles(NMuscle).Ls = 0.06;
RefMuscles(NMuscle).AttachP = [-0.0021, -0.1057;
    -0.0061, 0.1042];
RefMuscles(NMuscle).AttachBodies = [4; 3];
NMuscle = NMuscle + 1;

% Adductor Magnus Middle
RefMuscles(NMuscle).name = 'AddMagMid';
RefMuscles(NMuscle).F0 = 312;
RefMuscles(NMuscle).pen = deg2rad(3);
RefMuscles(NMuscle).L0 = 0.121;
RefMuscles(NMuscle).Ls = 0.13;
RefMuscles(NMuscle).AttachP = [-0.0119, -0.1075;
    0.0072, -0.0395];
RefMuscles(NMuscle).AttachBodies = [4; 3];
NMuscle = NMuscle + 1;

% Adductor Magnus Inferior
RefMuscles(NMuscle).name = 'AddMagInf';
RefMuscles(NMuscle).F0 = 444;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.131;
RefMuscles(NMuscle).Ls = 0.26;
RefMuscles(NMuscle).AttachP = [-0.0060, -0.1064;
    0.0094, -0.2474];
RefMuscles(NMuscle).AttachBodies = [4; 3];
NMuscle = NMuscle + 1;

% Tensor Fasciae Latae
RefMuscles(NMuscle).name = 'TenFascLat';
RefMuscles(NMuscle).F0 = 155;
RefMuscles(NMuscle).pen = deg2rad(3);
RefMuscles(NMuscle).L0 = 0.095;
RefMuscles(NMuscle).Ls = 0.425;
RefMuscles(NMuscle).AttachP = [0.0395, 0.0314;
    0.0394, 0.1332;
    0.0072, -0.2758;
    0.0062, 0.1012];
RefMuscles(NMuscle).AttachBodies = [4; 3; 3; 2];
NMuscle = NMuscle + 1;

% Pectineus
RefMuscles(NMuscle).name = 'Pectineus';
RefMuscles(NMuscle).F0 = 177;
RefMuscles(NMuscle).pen = deg2rad(0);
RefMuscles(NMuscle).L0 = 0.133;
RefMuscles(NMuscle).Ls = 0.001;
RefMuscles(NMuscle).AttachP = [0.0276, -0.0656;
    -0.0164, 0.1564];
RefMuscles(NMuscle).AttachBodies = [4; 3];
NMuscle = NMuscle + 1;

% Iliacus
RefMuscles(NMuscle).name = 'Iliacus';
RefMuscles(NMuscle).F0 = 429;
RefMuscles(NMuscle).pen = deg2rad(7);
RefMuscles(NMuscle).L0 = 0.1;
RefMuscles(NMuscle).Ls = 0.09;
RefMuscles(NMuscle).AttachP = [0.0036, 0.0464;
    0.0487, -0.0440;
    0.0411, -0.0697;
    0.0022, 0.1937;
    -0.0258, 0.1832];
RefMuscles(NMuscle).AttachBodies = [4; 4; 4; 3; 3];
NMuscle = NMuscle + 1;

% Psoas
RefMuscles(NMuscle).name = 'Psoas';
RefMuscles(NMuscle).F0 = 371;
RefMuscles(NMuscle).pen = deg2rad(8);
RefMuscles(NMuscle).L0 = 0.104;
RefMuscles(NMuscle).Ls = 0.13;
RefMuscles(NMuscle).AttachP = [0.0063, 0.0979;
    0.0467, -0.0460;
    0.0413, -0.0692;
    0.0021, 0.1985;
    -0.0252, 0.1865];
RefMuscles(NMuscle).AttachBodies = [4; 4; 4; 3; 3];
NMuscle = NMuscle + 1;

% Semitendinosus
RefMuscles(NMuscle).name = 'Semitend';
RefMuscles(NMuscle).F0 = 328;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.201;
RefMuscles(NMuscle).Ls = 0.262;
RefMuscles(NMuscle).AttachP = [-0.0520, -0.0928;
    -0.0325, 0.0951;
    -0.0116, 0.0744;
    0.0028, 0.0527];
RefMuscles(NMuscle).AttachBodies = [4; 2; 2; 2];
NMuscle = NMuscle + 1;

% Semimembranosus
RefMuscles(NMuscle).name = 'Semimemb';
RefMuscles(NMuscle).F0 = 1030;
RefMuscles(NMuscle).pen = deg2rad(15);
RefMuscles(NMuscle).L0 = 0.08;
RefMuscles(NMuscle).Ls = 0.359;
RefMuscles(NMuscle).AttachP = [-0.0476, -0.0900;
    -0.0251, 0.0960];
RefMuscles(NMuscle).AttachBodies = [4; 2];
NMuscle = NMuscle + 1;

% Biceps Femoris Long
RefMuscles(NMuscle).name = 'BicepFemLong';
RefMuscles(NMuscle).F0 = 717;
RefMuscles(NMuscle).pen = deg2rad(0);
RefMuscles(NMuscle).L0 = 0.109;
RefMuscles(NMuscle).Ls = 0.341;
RefMuscles(NMuscle).AttachP = [-0.0527, -0.0886;
    -0.0083, 0.0762];
RefMuscles(NMuscle).AttachBodies = [4; 2];
NMuscle = NMuscle + 1;

% Biceps Femoris Short
RefMuscles(NMuscle).name = 'BicepFemShort';
RefMuscles(NMuscle).F0 = 402;
RefMuscles(NMuscle).pen = deg2rad(23);
RefMuscles(NMuscle).L0 = 0.173;
RefMuscles(NMuscle).Ls = 0.1;
RefMuscles(NMuscle).AttachP = [0.0067, -0.0162;
    -0.0104, 0.0766];
RefMuscles(NMuscle).AttachBodies = [3; 2];
NMuscle = NMuscle + 1;

% Sartorius
RefMuscles(NMuscle).name = 'Sartorius';
RefMuscles(NMuscle).F0 = 104;
RefMuscles(NMuscle).pen = deg2rad(0);
RefMuscles(NMuscle).L0 = 0.579;
RefMuscles(NMuscle).Ls = 0.04;
RefMuscles(NMuscle).AttachP = [0.0551, 0.0090;
    -0.0040, -0.2113;
    -0.0058, 0.1082;
    0.0062, 0.0906;
    0.0251, 0.0646];
RefMuscles(NMuscle).AttachBodies = [4; 3; 2; 2; 2];
NMuscle = NMuscle + 1;

% Gracilis
RefMuscles(NMuscle).name = 'Gracilis';
RefMuscles(NMuscle).F0 = 108;
RefMuscles(NMuscle).pen = deg2rad(3);
RefMuscles(NMuscle).L0 = 0.352;
RefMuscles(NMuscle).Ls = 0.15;
RefMuscles(NMuscle).AttachP = [0.0146, -0.0923;
    -0.0160, 0.1024;
    0.0062, 0.0650];
RefMuscles(NMuscle).AttachBodies = [4; 2; 2];
NMuscle = NMuscle + 1;

% Rectus Femoris
RefMuscles(NMuscle).name = 'RectFem';
RefMuscles(NMuscle).F0 = 779;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.084;
RefMuscles(NMuscle).Ls = 0.346;
RefMuscles(NMuscle).AttachP = [0.0411, -0.0204;
    0.0486, -0.2713;
    0.0508, 0.1304;
    0.0405, 0.0665];
RefMuscles(NMuscle).AttachBodies = [4; 3; 2; 2];
NMuscle = NMuscle + 1;

% Vastus Medialis
RefMuscles(NMuscle).name = 'VastMed';
RefMuscles(NMuscle).F0 = 1294;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.089;
RefMuscles(NMuscle).Ls = 0.126;
RefMuscles(NMuscle).AttachP = [0.0187, -0.0146;
    0.0477, -0.1043;
    0.0546, -0.2765;
    0.0508, 0.1304;
    0.0404, 0.0665];
RefMuscles(NMuscle).AttachBodies = [3; 3; 3; 2; 2];
NMuscle = NMuscle + 1;

% Vastus Intermedius
RefMuscles(NMuscle).name = 'VastInt';
RefMuscles(NMuscle).F0 = 1365;
RefMuscles(NMuscle).pen = deg2rad(3);
RefMuscles(NMuscle).L0 = 0.087;
RefMuscles(NMuscle).Ls = 0.136;
RefMuscles(NMuscle).AttachP = [0.0388, 0.0088;
    0.0449, -0.0126;
    0.0473, -0.2712;
    0.0508, 0.1304;
    0.0404, 0.0665];
RefMuscles(NMuscle).AttachBodies = [3; 3; 3; 2; 2];
NMuscle = NMuscle + 1;

% Vastus Lateralis
RefMuscles(NMuscle).name = 'VastLat';
RefMuscles(NMuscle).F0 = 181;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.084;
RefMuscles(NMuscle).Ls = 0.157;
RefMuscles(NMuscle).AttachP = [0.0064, 0.0181;
    0.0361, -0.0805;
    0.0513, -0.2746;
    0.0508, 0.1304;
    0.0404, 0.0665];
RefMuscles(NMuscle).AttachBodies = [3; 3; 3; 2; 2];
NMuscle = NMuscle + 1;

% Gastrocnemius Medial
RefMuscles(NMuscle).name = 'GastMed';
RefMuscles(NMuscle).F0 = 1113;
RefMuscles(NMuscle).pen = deg2rad(17);
RefMuscles(NMuscle).L0 = 0.045;
RefMuscles(NMuscle).Ls = 0.408;
RefMuscles(NMuscle).AttachP = [-0.0170, -0.2597;
    -0.0338, -0.2800;
    -0.0225, 0.1012;
    -0.0439, 0.0242];
RefMuscles(NMuscle).AttachBodies = [3; 3; 2; 1];
NMuscle = NMuscle + 1;

% Gastrocnemius Lateral
RefMuscles(NMuscle).name = 'GastLat';
RefMuscles(NMuscle).F0 = 488;
RefMuscles(NMuscle).pen = deg2rad(8);
RefMuscles(NMuscle).L0 = 0.064;
RefMuscles(NMuscle).Ls = 0.385;
RefMuscles(NMuscle).AttachP = [-0.0207, -0.2619;
    -0.0375, -0.2846;
    -0.0250, 0.1018;
    -0.0439, 0.0242];
RefMuscles(NMuscle).AttachBodies = [3; 3; 2; 1];
NMuscle = NMuscle + 1;

% Soleus
RefMuscles(NMuscle).name = 'Soleus';
RefMuscles(NMuscle).F0 = 2839;
RefMuscles(NMuscle).pen = deg2rad(25);
RefMuscles(NMuscle).L0 = 0.03;
RefMuscles(NMuscle).Ls = 0.268;
RefMuscles(NMuscle).AttachP = [-0.0025, -0.0070;
    -0.0439, 0.0242];
RefMuscles(NMuscle).AttachBodies = [2; 1];
NMuscle = NMuscle + 1;

% Tibialis Anterior
RefMuscles(NMuscle).name = 'TibialAnt';
RefMuscles(NMuscle).F0 = 603;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.098;
RefMuscles(NMuscle).Ls = 0.223;
RefMuscles(NMuscle).AttachP = [0.0186, -0.0165;
    0.0340, -0.2572;
    0.0671, 0.0111];
RefMuscles(NMuscle).AttachBodies = [2; 2; 1];
NMuscle = NMuscle + 1;

% Flexor Digitorum Longus
RefMuscles(NMuscle).name = 'FlexDigLong';
RefMuscles(NMuscle).F0 = 310;
RefMuscles(NMuscle).pen = deg2rad(7);
RefMuscles(NMuscle).L0 = 0.034;
RefMuscles(NMuscle).Ls = 0.4;
RefMuscles(NMuscle).AttachP = [-0.0086, -0.0601;
    -0.0160, -0.2677;
    -0.0051, 0.0246;
    0.0218, 0.0110;
    0.1157, -0.0145];
RefMuscles(NMuscle).AttachBodies = [2; 2; 1; 1; 1];
NMuscle = NMuscle + 1;

% Flexor Hallicus Longus
RefMuscles(NMuscle).name = 'FlexHalLong';
RefMuscles(NMuscle).F0 = 322;
RefMuscles(NMuscle).pen = deg2rad(10);
RefMuscles(NMuscle).L0 = 0.043;
RefMuscles(NMuscle).Ls = 0.38;
RefMuscles(NMuscle).AttachP = [-0.0081, -0.0899;
    -0.0193, -0.2705;
    -0.0113, 0.0209;
    0.0544, 0.0002;
    0.1224, -0.0118];
RefMuscles(NMuscle).AttachBodies = [2; 2; 1; 1; 1];
NMuscle = NMuscle + 1;

% Extensor Digitorum Longus
RefMuscles(NMuscle).name = 'ExtDigLong';
RefMuscles(NMuscle).F0 = 341;
RefMuscles(NMuscle).pen = deg2rad(8);
RefMuscles(NMuscle).L0 = 0.102;
RefMuscles(NMuscle).Ls = 0.345;
RefMuscles(NMuscle).AttachP = [0.0033, 0.0086;
    0.0299, -0.2630;
    0.0429, 0.0318;
    0.1116, -0.0010];
RefMuscles(NMuscle).AttachBodies = [2; 2; 1; 1];
NMuscle = NMuscle + 1;

% Extensor Hallicus Longus
RefMuscles(NMuscle).name = 'ExtHalLong';
RefMuscles(NMuscle).F0 = 108;
RefMuscles(NMuscle).pen = deg2rad(6);
RefMuscles(NMuscle).L0 = 0.111;
RefMuscles(NMuscle).Ls = 0.305;
RefMuscles(NMuscle).AttachP = [0.0012, -0.0312;
    0.0337, -0.2607;
    0.0477, 0.0319;
    0.0796, 0.0240;
    0.1232, 0.0073];
RefMuscles(NMuscle).AttachBodies = [2; 2; 1; 1; 1];
NMuscle = NMuscle + 1;

% Peroneus Brevis
RefMuscles(NMuscle).name = 'PeronBrev';
RefMuscles(NMuscle).F0 = 348;
RefMuscles(NMuscle).pen = deg2rad(5);
RefMuscles(NMuscle).L0 = 0.05;
RefMuscles(NMuscle).Ls = 0.161;
RefMuscles(NMuscle).AttachP = [-0.0072, -0.1221;
    -0.0205, -0.2813;
    -0.0149, -0.2928;
    -0.0017, 0.0203;
    0.0187, 0.0154];
RefMuscles(NMuscle).AttachBodies = [2; 2; 2; 1; 1];
NMuscle = NMuscle + 1;

% Peroneus Longus
RefMuscles(NMuscle).name = 'PeronLong';
RefMuscles(NMuscle).F0 = 754;
RefMuscles(NMuscle).pen = deg2rad(10);
RefMuscles(NMuscle).L0 = 0.049;
RefMuscles(NMuscle).Ls = 0.345;
RefMuscles(NMuscle).AttachP = [0.0005, -0.0106;
    -0.0214, -0.2834;
    -0.0168, -0.2953;
    -0.0049, 0.0163;
    0.0191, 0.0040;
    0.0360, 0.0004;
    0.0707, 0.0019];
RefMuscles(NMuscle).AttachBodies = [2; 2; 2; 1; 1; 1; 1];
NMuscle = NMuscle + 1;

% End of function
end