function arm_model = generateArm(arm_str, base_pose)
%GENERATEARM Generate arm model
%
%   Usage: arm_model = GENERATEARM(arm_str)
%   @arm_str       dataset string, existing datasets:
%                  'SimpleTwoLinksArm', 'SimpleThreeLinksArm', 'WAMArm', 'PR2Arm'
%   @base_pose     arm's base pose, default is origin with no rotation
%
%   Output Format:
%   arm_model      an ArmModel object, contains kinematics and model information

import gtsam.*
import gpmp2.*

if nargin < 2
    base_pose = Pose3(Rot3(eye(3)), Point3([0,0,0]'));
end

%  2 link arm 
if strcmp(arm_str, 'SimpleTwoLinksArm')
    % abstract arm
    a = [0.5, 0.5]';
    d = [0, 0]';
    alpha = [0, 0]';
    arm = Arm(2, a, alpha, d);
    % physical arm
    spheres_data = [...
        0  -0.5  0.0  0.0  0.01
        0  -0.4  0.0  0.0  0.01
        0  -0.3  0.0  0.0  0.01
        0  -0.2  0.0  0.0  0.01
        0  -0.1  0.0  0.0  0.01
        1  -0.5  0.0  0.0  0.01
        1  -0.4  0.0  0.0  0.01
        1  -0.3  0.0  0.0  0.01
        1  -0.2  0.0  0.0  0.01
        1  -0.1  0.0  0.0  0.01
        1  0.0  0.0  0.0  0.01];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    arm_model = ArmModel(arm, sphere_vec);
    
% 3 link arm
elseif strcmp(arm_str, 'SimpleThreeLinksArm')
    % abstract arm
    a = [0.5, 0.5, 0.5]';
    d = [0, 0, 0]';
    alpha = [0, 0, 0]';
    arm = Arm(3, a, alpha, d);
    
    % physical arm
    spheres_data = [...
        0  -0.5  0.0  0.0  0.01
        0  -0.4  0.0  0.0  0.01
        0  -0.3  0.0  0.0  0.01
        0  -0.2  0.0  0.0  0.01
        0  -0.1  0.0  0.0  0.01
        1  -0.5  0.0  0.0  0.01
        1  -0.4  0.0  0.0  0.01
        1  -0.3  0.0  0.0  0.01
        1  -0.2  0.0  0.0  0.01
        1  -0.1  0.0  0.0  0.01
        2  -0.5  0.0  0.0  0.01
        2  -0.4  0.0  0.0  0.01
        2  -0.3  0.0  0.0  0.01
        2  -0.2  0.0  0.0  0.01
        2  -0.1  0.0  0.0  0.01
        2  0.0  0.0  0.0  0.01];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    arm_model = ArmModel(arm, sphere_vec);

% 7 link WAM arm
elseif strcmp(arm_str, 'WAMArm')
    % arm: WAM arm
    alpha = [-pi/2,pi/2,-pi/2,pi/2,-pi/2,pi/2,0]';
    a = [0,0,0.045,-0.045,0,0,0]';
    d = [0,0,0.55,0,0.3,0,0.06]';
    theta = [0, 0, 0, 0, 0, 0, 0]';
    abs_arm = Arm(7, a, alpha, d, base_pose, theta);
    
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [...
        0 0.0  0.0  0.0 0.15
        1 0.0  0.0  0.2 0.06
        1 0.0  0.0  0.3 0.06
        1 0.0  0.0  0.4 0.06
        1 0.0  0.0  0.5 0.06
        2 0.0  0.0  0.0 0.06
        3 0.0  0.0  0.1 0.06
        3 0.0  0.0  0.2 0.06
        3 0.0  0.0  0.3 0.06
        5 0.0  0.0  0.1 0.06
        6 0.1  -0.025 0.08 0.04
        6 0.1   0.025 0.08 0.04
        6 -0.1   0     0.08 0.04
        6 0.15 -0.025 0.13 0.04
        6 0.15  0.025 0.13 0.04
        6 -0.15  0     0.13 0.04];
    
    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    arm_model = ArmModel(abs_arm, sphere_vec);

% 7 DOF PR2 right arm
elseif strcmp(arm_str, 'PR2Arm')
    % arm: PR2 arm
    alpha = [-1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0]';
    a = [0.1, 0, 0, 0, 0, 0, 0]';
    d = [0, 0, 0.4, 0, 0.321, 0, 0]';
    theta = [0, 1.5708, 0, 0, 0, 0, 0]';
    abs_arm = Arm(7, a, alpha, d, base_pose, theta);
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [...
          0 -0.010000 0.000000 0.000000 0.180000

          2 0.015000 0.220000 -0.000000 0.110000
          2 0.035000 0.140000 -0.000000 0.080000
          2 0.035000 0.072500 -0.000000 0.080000
          2 0.000000 0.000000 -0.000000 0.105000

          4 -0.005000 0.321-0.130000 -0.000000 0.075000
          4 0.010000 0.321-0.200000 -0.025000 0.055000
          4 0.010000 0.321-0.200000 0.025000 0.055000
          4 0.015000 0.321-0.265000 -0.027500 0.050000
          4 0.015000 0.321-0.265000 0.027500 0.050000
          4 0.005000 0.321-0.320000 -0.022500 0.050000
          4 0.005000 0.321-0.320000 0.022500 0.050000

          6 0 -0.017500 0.072500 0.040000
          6 0 0.017500 0.072500 0.040000
          6 0 0 0.092500 0.040000

          6 0 0.03600 0.11 0.040000
          6 0 0.027000 0.155 0.035000
          6 0 0.00900 0.18 0.030000
          6 0 0.00950 0.205 0.020000

          6 0 -0.03600 0.11 0.040000
          6 0 -0.027000 0.155 0.035000
          6 0 -0.00900 0.18 0.030000
          6 0 -0.00950 0.205 0.020000];

    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    arm_model = ArmModel(abs_arm, sphere_vec);

% 6 DOF JACO2 arm
elseif strcmp(arm_str, 'JACO2Arm')
    % arm: JACO2 6DOF arm
    alpha = [pi/2, pi, pi/2, 1.0472, 1.0472, pi]';
    a = [0, 0.41, 0, 0, 0, 0]';
    d = [0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228]';
    theta = [0, 0, 0, 0, 0, 0]';
    abs_arm = Arm(6, a, alpha, d, base_pose, theta);
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [...
        0 0.0   0.0    0.0 0.053
        0 0.0  -0.08   0.0 0.053
        0 0.0  -0.155  0.0 0.053
        0 0.0  -0.23   0.0 0.053
        
        1 0.0  0.0  0.0 0.053
        1 -0.06  0.0  0.03 0.04
        1 -0.12  0.0  0.03 0.04
        1 -0.18  0.0  0.03 0.04
        1 -0.24  0.0  0.03 0.04
        1 -0.30  0.0  0.03 0.04
        1 -0.36  0.0  0.03 0.04
        
        2 0.0  -0.01  -0.05 0.035
        2 0.0  -0.01  -0.10 0.03
        2 0.0   0.0   -0.15 0.035
        2 0.0   0.0   -0.2 0.035
        
        3 0.0  0.0  0.0 0.04
        3 0.0  0.0  -0.045 0.04
               
        4 0.0  0.0  0.0 0.04
        4 0.0  -0.008  -0.075 0.05

        5 0.0  0.05  -0.01 0.013
        5 0.0  0.05  0.01 0.013
        5 0.0  0.06  -0.039 0.018
        5 0.0  0.06  -0.067 0.018
        5 0.0  0.035  -0.042 0.018
        5 0.0  -0.05  -0.01 0.013
        5 0.0  -0.05  0.01 0.013
        5 0.0  -0.06  -0.039 0.018
        5 0.0  -0.06  -0.067 0.018
        5 0.0  -0.035  -0.042 0.018
        5 0.0  0.015  -0.055 0.02
        5 0.0  0.025  -0.08 0.02
        5 0.0  0.0  -0.08 0.02
        5 0.0  -0.025  -0.08 0.02
        5 0.0  -0.015  -0.055 0.02
        ];

    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    arm_model = ArmModel(abs_arm, sphere_vec);
    
% no such dataset
else
    error('No such arm exist');
end

end

