function arm_model = generateArm(arm_str)
%GENERATEARM Generate arm model
%
%   Usage: arm_model = GENERATEARM(arm_str)
%   @arm_str       dataset string, existing datasets:
%                  'SimpleTwoLinksArm', 'SimpleThreeLinksArm', 'WAMArm'
%
%   Output Format:
%   arm_model      an ArmModel object, contains kinematics and model information

import gtsam.*
import gpmp2.*

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
    abs_arm = Arm(7, a, alpha, d);
    
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
    
% no such dataset
else
    error('No such arm exist');
end

end

