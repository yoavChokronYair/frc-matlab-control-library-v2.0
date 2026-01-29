classdef SwerveDriveSystem < handle
    % Complete swerve drive: 4 modules, kinematics, odometry
    
    properties
        % Modules
        frontLeft
        frontRight
        backLeft
        backRight
        
        % Robot geometry (from CAD)
        wheelbaseWidth_m        % Distance between left/right wheels
        wheelbaseLength_m       % Distance between front/back wheels
        robotMass_kg
        robotInertia_kg_m2      % Rotational inertia about vertical axis
        
        % Trajectory following
        trajectoryPID           % For path tracking (x, y, theta)
    end
    
    methods
        function obj = SwerveDriveSystem()
            % Create 4 modules
            obj.frontLeft = SwerveModuleModel('FrontLeft');
            obj.frontRight = SwerveModuleModel('FrontRight');
            obj.backLeft = SwerveModuleModel('BackLeft');
            obj.backRight = SwerveModuleModel('BackRight');
            
            % Defaults (update from CAD)
            obj.wheelbaseWidth_m = 0.6;
            obj.wheelbaseLength_m = 0.6;
            obj.robotMass_kg = 50;
            obj.robotInertia_kg_m2 = 5.0;
        end
        
        function loadCADData(obj, cadData)
            % Import from SolidWorks
            obj.robotMass_kg = cadData.mass;
            obj.robotInertia_kg_m2 = cadData.inertia_zz; % Vertical axis
            obj.wheelbaseWidth_m = cadData.wheelbase_width;
            obj.wheelbaseLength_m = cadData.wheelbase_length;
            
            fprintf('Swerve Drive CAD Data Loaded:\n');
            fprintf('  Mass: %.1f kg\n', obj.robotMass_kg);
            fprintf('  Inertia: %.2f kg⋅m²\n', obj.robotInertia_kg_m2);
            fprintf('  Wheelbase: %.2f m x %.2f m\n', ...
                obj.wheelbaseLength_m, obj.wheelbaseWidth_m);
        end
        
        function calculateAllControllers(obj)
            % Calculate gains for all 4 modules
            
            modules = {obj.frontLeft, obj.frontRight, obj.backLeft, obj.backRight};
            
            for i = 1:4
                modules{i}.calculateDriveControllers(obj.robotMass_kg);
                modules{i}.calculateAzimuthControllers();
            end
            
            % Robot-level trajectory PID
            obj.calculateTrajectoryPID();
        end
        
        function calculateTrajectoryPID(obj)
            % PID for following trajectories (x, y, rotation)
            
            % Translation (x, y) - proportional to position error
            obj.trajectoryPID.kP_translation = 5.0;  % (m/s) / m
            obj.trajectoryPID.kI_translation = 0.1;
            obj.trajectoryPID.kD_translation = 0.0;
            
            % Rotation (theta) - proportional to angle error
            obj.trajectoryPID.kP_rotation = 3.0;     % (rad/s) / rad
            obj.trajectoryPID.kI_rotation = 0.05;
            obj.trajectoryPID.kD_rotation = 0.0;
            
            fprintf('Trajectory Following Gains:\n');
            fprintf('  Translation: kP=%.2f\n', obj.trajectoryPID.kP_translation);
            fprintf('  Rotation: kP=%.2f\n', obj.trajectoryPID.kP_rotation);
        end
        
        function testScrubForces(obj, testData)
            % Measure energy losses from wheel scrub during rotation
            % testData: from spin-in-place test
            
            % Expected power from kinematics
            omega = testData.angularVelocity_rad_s;
            torque_expected = obj.robotInertia_kg_m2 * 0; % Constant speed
            
            % Actual power from motor currents
            totalCurrent = sum(testData.driveCurrents_A);
            voltage = mean(testData.voltage_V);
            power_actual = voltage * totalCurrent;
            
            % Scrub loss
            scrubPower = power_actual - torque_expected * omega;
            
            fprintf('Scrub Testing:\n');
            fprintf('  Measured scrub power: %.1f W at %.2f rad/s\n', ...
                scrubPower, omega);
            
            % Update drive friction models
            % (increase kS to compensate for scrub)
        end
        
        function exportAllGains(obj, directory)
            % Export all module gains to separate files
            
            obj.frontLeft.exportGains([directory, '/FrontLeft.json']);
            obj.frontRight.exportGains([directory, '/FrontRight.json']);
            obj.backLeft.exportGains([directory, '/BackLeft.json']);
            obj.backRight.exportGains([directory, '/BackRight.json']);
            
            % Export robot-level gains
            robotGains.trajectory = obj.trajectoryPID;
            robotGains.geometry.width = obj.wheelbaseWidth_m;
            robotGains.geometry.length = obj.wheelbaseLength_m;
            
            jsonStr = jsonencode(robotGains);
            fid = fopen([directory, '/RobotConfig.json'], 'w');
            fprintf(fid, '%s', jsonStr);
            fclose(fid);
            
            fprintf('All swerve gains exported to: %s\n', directory);
        end
    end
end
