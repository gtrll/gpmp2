function model = generateMobileArm(str, base_T_arm)
%GENERATEMOBILEARM Generate mobile arm model
%
%   Usage: model = GENERATEMOBILEARM(arm_str)
%   @str        dataset string, existing datasets:
%               'SimpleTwoLinksArm'
%
%   Output Format:
%   model       an ArmModel object, contains kinematics and model information


import gtsam.*
import gpmp2.*

if nargin < 2
    base_T_arm = Pose3(Rot3(eye(3)), Point3([0,0,0]'));
end

%  2 link arm 
if strcmp(str, 'SimpleTwoLinksArm')
    % abstract arm
    a = [0.3, 0.3]';
    d = [0, 0]';
    alpha = [0, 0]';
    arm = Arm(2, a, alpha, d);
    % abstract mobile arm
    marm = Pose2MobileArm(arm);
    % physical model
    % base size: 0.4 x 0.2
    spheres_data = [...
        0  -0.1  0.0  0.0  0.12
        0   0.0  0.0  0.0  0.12
        0   0.1  0.0  0.0  0.12
%         1  -0.5  0.0  0.0  0.05
%         1  -0.4  0.0  0.0  0.05
        1  -0.3  0.0  0.0  0.05
        1  -0.2  0.0  0.0  0.05
        1  -0.1  0.0  0.0  0.05
%         2  -0.5  0.0  0.0  0.05
%         2  -0.4  0.0  0.0  0.05
        2  -0.3  0.0  0.0  0.05
        2  -0.2  0.0  0.0  0.05
        2  -0.1  0.0  0.0  0.05
        2   0.0  0.0  0.0  0.05];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    model = Pose2MobileArmModel(marm, sphere_vec);


%  2 simple arms on mobile base, each arm has two links 
elseif strcmp(str, 'SimpleTwoArms')
    % abstract arm
    a = [0.6, 0.6]';
    d = [0, 0]';
    alpha = [0, 0]';
    arm = Arm(2, a, alpha, d);
    % abstract mobile arm
    % base pose to left and right 60 deg
    marm = Pose2Mobile2Arms(arm, arm, Pose3(Rot3.Yaw(-pi/3), Point3(0,0,0)), ...
        Pose3(Rot3.Yaw(pi/3), Point3(0,0,0)));
    % physical model
    % base size: 0.8 x 0.4
    spheres_data = [...
        0  -0.2  0.0  0.0  0.24
        0   0.0  0.0  0.0  0.24
        0   0.2  0.0  0.0  0.24
        1  -0.6  0.0  0.0  0.1
        1  -0.4  0.0  0.0  0.1
        1  -0.2  0.0  0.0  0.1
        2  -0.6  0.0  0.0  0.1
        2  -0.4  0.0  0.0  0.1
        2  -0.2  0.0  0.0  0.1
        2   0.0  0.0  0.0  0.1
        3  -0.6  0.0  0.0  0.1
        3  -0.4  0.0  0.0  0.1
        3  -0.2  0.0  0.0  0.1
        4  -0.6  0.0  0.0  0.1
        4  -0.4  0.0  0.0  0.1
        4  -0.2  0.0  0.0  0.1
        4   0.0  0.0  0.0  0.1];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    model = Pose2Mobile2ArmsModel(marm, sphere_vec);


%  2DMobileArm2
elseif strcmp(str, '2DMobileArm2')
    a = [1.0, 1.0]';
    d = [0, 0]';
    alpha = [0, 0]';
    arm = Arm(2, a, alpha, d);
    % abstract mobile arm
    marm = Pose2MobileArm(arm);
    % physical model
    % base size: 1.0 x 0.7
    spheres_data = [...
        0   0.2   0.0  0.0  0.35
        0  -0.2   0.0  0.0  0.35  
        1  -0.05  0.0  0.0  0.1
        1  -0.25  0.0  0.0  0.1
        1  -0.45  0.0  0.0  0.1
        2  -0.05  0.0  0.0  0.1
        2  -0.25  0.0  0.0  0.1
        2  -0.45  0.0  0.0  0.1
        2  -0.65  0.0  0.0  0.1
        2  -0.85  0.0  0.0  0.1];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    model = Pose2MobileArmModel(marm, sphere_vec);


%  Vector: omni drive base with 6 DOF Kinova JACO2 arm
elseif strcmp(str, 'Vector')
    % arm: JACO2 6DOF arm
    alpha = [pi/2, pi, pi/2, 1.0472, 1.0472, pi]';
    a = [0, 0.41, 0, 0, 0, 0]';
    d = [0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228]';
    % theta = [0, 0, 0, 0, 0, 0]';
    % abstract arm
    arm = Arm(6, a, alpha, d);
    % abstract mobile arm
    marm = Pose2MobileArm(arm, base_T_arm);
    % sphere data [id x y z r]
    spheres_data = [...
%         0 -0.01 -0.01 0.22 0.21
        0 -0.01 0 0 0.005
        0 -0.26 -0.01 0.08 0.08
        0 -0.26 0.15 0.08 0.08
        0 -0.26 -0.17 0.08 0.08
        0 0.24 -0.01 0.08 0.08
        0 0.24 0.15 0.08 0.08
        0 0.24 -0.17 0.08 0.08
        0 0.04 -0.01 0.6 0.18
        0 -0.2 -0.06 0.45 0.1
        0 -0.2  0.04 0.45 0.1
        0  0.16 -0.07 0.41 0.06
        0  0.16  0.05 0.41 0.06
        0  0.16 -0.18 0.41 0.06
        0  0.16  0.16 0.41 0.06
        0  0.33 -0.01 0.29 0.05
        
        0  -0.01  -0.24 0.31 0.05
        0  -0.12  -0.24 0.31 0.05
        0  -0.22  -0.24 0.31 0.05
        0  -0.32  -0.24 0.31 0.05
        0   0.1   -0.24 0.31 0.05
        0   0.2   -0.24 0.31 0.05
        0   0.3   -0.24 0.31 0.05
        0  -0.01   0.22 0.31 0.05
        0  -0.12   0.22 0.31 0.05
        0  -0.22   0.22 0.31 0.05
        0  -0.32   0.22 0.31 0.05
        0   0.1    0.22 0.31 0.05
        0   0.2    0.22 0.31 0.05
        0   0.3    0.22 0.31 0.05
        0  -0.32  -0.01 0.31 0.05
        0  -0.32   0.10 0.31 0.05
        0  -0.32  -0.13 0.31 0.05
        0   0.32  -0.01 0.31 0.05
        0   0.32   0.10 0.31 0.05
        0   0.32  -0.13 0.31 0.05
        
        0 0.12 -0.01 0.87 0.1
        0 0.14 -0.11 0.78 0.08
        0 0.14  0.09 0.78 0.08
        0 0.19 -0.01 1.07 0.08
        0 0.14 -0.11 0.97 0.08
        0 0.14  0.09 0.97 0.08
        0 0.175 -0.01 1.2 0.05
        0 0.175 -0.01 1.3 0.05
        0 0.175 -0.01 1.4 0.05
        0 0.175 -0.01 1.5 0.05
        0 0.175 -0.01 1.62 0.07
        0 0.27 -0.01 1.5 0.05
        0 0.37 -0.01 1.5 0.05
        0 0.37 -0.01 1.6 0.05
        0 0.37 -0.01 1.66 0.045
        0 0.37 -0.1  1.66 0.045
        0 0.37  0.08  1.66 0.045
             
        1 0.0   0.0    0.0 0.053
        1 0.0  -0.08   0.0 0.053
        1 0.0  -0.155  0.0 0.053
        1 0.0  -0.23   0.0 0.053
        
        2 0.0  0.0  0.0 0.053
        2 -0.06  0.0  0.03 0.04
        2 -0.12  0.0  0.03 0.04
        2 -0.18  0.0  0.03 0.04
        2 -0.24  0.0  0.03 0.04
        2 -0.30  0.0  0.03 0.04
        2 -0.36  0.0  0.03 0.04
        
        3 0.0  -0.01  -0.05 0.035
        3 0.0  -0.01  -0.10 0.03
        3 0.0   0.0   -0.15 0.035
        3 0.0   0.0   -0.2 0.035
        
        4 0.0  0.0  0.0 0.04
        4 0.0  0.0  -0.045 0.04
               
        5 0.0  0.0  0.0 0.04
        5 0.0  -0.008  -0.075 0.05

        6 0.0  0.05  -0.01 0.013
        6 0.0  0.05  0.01 0.013
        6 0.0  0.06  -0.039 0.018
        6 0.0  0.06  -0.067 0.018
        6 0.0  0.035  -0.042 0.018
        6 0.0  -0.05  -0.01 0.013
        6 0.0  -0.05  0.01 0.013
        6 0.0  -0.06  -0.039 0.018
        6 0.0  -0.06  -0.067 0.018
        6 0.0  -0.035  -0.042 0.018
        6 0.0  0.015  -0.055 0.02
        6 0.0  0.025  -0.08 0.02
        6 0.0  0.0  -0.08 0.02
        6 0.0  -0.025  -0.08 0.02
        6 0.0  -0.015  -0.055 0.02
        ];

    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    model = Pose2MobileArmModel(marm, sphere_vec);

% no such dataset
else
    error('No such arm exist');
end

end

