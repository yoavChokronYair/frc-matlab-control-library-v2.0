classdef SwerveModuleModel < handle
    % Single swerve module: 1 drive motor, 1 azimuth motor, 1 absolute encoder
    
    properties
        moduleName              % e.g., 'FrontLeft'
        
        % Motors
        driveMotor              % KrakenX40Model or KrakenX60Model
        azimuthMotor            % Typically KrakenX40Model
        
        % Gearing
        driveGearRatio          % Motor rotations : wheel rotations
        azimuthGearRatio        % Motor rotations : module rotations
        
        % Physical
        wheelDiameter_m
        wheelRadius_m
        moduleInertia_kg_m2     % Rotational inertia of steering
        
        % Encoder
        absoluteEncoderOffset_rad % Calibration offset
        
        % Controllers
        driveFF                 % Feedforward for drive motor
        drivePID                % PID for drive velocity
        azimuthFF               % Feedforward for azimuth
        azimuthPID              % PID for azimuth position
        azimuthMotionMagic      % Motion Magic for azimuth
    end
    
    methods
        function obj = SwerveModuleModel(name)
            obj.moduleName = name;
            
            % Typical FRC swerve configuration
            obj.driveMotor = KrakenX60Model();      % Higher torque for drive
            obj.azimuthMotor = KrakenX40Model();    % Lighter for steering
            
            obj.driveGearRatio = 6.75;              % L2 gearing typical
            obj.azimuthGearRatio = 12.8;            % Example azimuth ratio
            
            obj.wheelDiameter_m = 0.1016;           % 4 inches
            obj.wheelRadius_m = obj.wheelDiameter_m / 2;
            
            obj.moduleInertia_kg_m2 = 0.01;         % Estimate, refine from tests
            
            obj.absoluteEncoderOffset_rad = 0;      % Must calibrate
        end
        
        function calculateDriveControllers(obj, robotMass_kg)
            % Calculate feedforward and PID for drive motor
            % robotMass_kg: Total robot mass (for acceleration calc)
            
            % Effective inertia at drive motor
            % Linear: F = m*a  →  Rotational: τ = J*α
            % J_wheel = m * r²
            % J_motor = J_wheel * gearRatio²
            inertia_equiv = (robotMass_kg / 4) * obj.wheelRadius_m^2 * obj.driveGearRatio^2;
            
            friction.static = 0.3;      % Estimate, tune from tests
            friction.viscous = 0.01;
            
            obj.driveFF = FeedforwardCalculator.calculateSimpleRotational(...
                obj.driveMotor, obj.driveGearRatio, inertia_equiv, friction);
            
            % Velocity PID (drive is velocity-controlled)
            systemParams.inertia = inertia_equiv;
            systemParams.damping = friction.viscous;
            
            [kP, kI, ~] = PIDTuner.tuneVelocityPID(systemParams, 20); % 20 Hz bandwidth
            obj.drivePID.kP = kP;
            obj.drivePID.kI = kI;
            obj.drivePID.kD = 0;
            
            fprintf('%s Drive Controller:\n', obj.moduleName);
            fprintf('  FF: kS=%.3f, kV=%.4f\n', obj.driveFF.kS, obj.driveFF.kV);
            fprintf('  PID: kP=%.3f, kI=%.3f\n', obj.drivePID.kP, obj.drivePID.kI);
        end
        
        function calculateAzimuthControllers(obj)
            % Calculate feedforward, PID, and Motion Magic for azimuth
            
            friction.static = 0.2;      % Lower friction, good bearings
            friction.viscous = 0.005;
            
            obj.azimuthFF = FeedforwardCalculator.calculateSimpleRotational(...
                obj.azimuthMotor, obj.azimuthGearRatio, obj.moduleInertia_kg_m2, friction);
            
            % Position PID (azimuth is position-controlled)
            systemParams.inertia = obj.moduleInertia_kg_m2;
            systemParams.damping = friction.viscous;
            desiredResponse.settlingTime_s = 0.1;   % Fast steering
            desiredResponse.overshoot_percent = 5;
            
            [kP, kI, kD] = PIDTuner.tunePositionPID(systemParams, desiredResponse);
            obj.azimuthPID.kP = kP;
            obj.azimuthPID.kI = kI;
            obj.azimuthPID.kD = kD;
            
            % Motion Magic
            maxSpeed = obj.azimuthMotor.FREE_SPEED_RAD_S / obj.azimuthGearRatio * 0.8;
            maxAccel = 50; % rad/s² - conservative
            obj.azimuthMotionMagic.cruiseVelocity = maxSpeed;
            obj.azimuthMotionMagic.acceleration = maxAccel;
            
            fprintf('%s Azimuth Controller:\n', obj.moduleName);
            fprintf('  PID: kP=%.3f, kI=%.3f, kD=%.4f\n', ...
                obj.azimuthPID.kP, obj.azimuthPID.kI, obj.azimuthPID.kD);
        end
        
        function calibrateAbsoluteEncoder(obj, measuredAngle_rad, desiredAngle_rad)
            % Calibrate encoder offset
            % Call this with module pointed in known direction
            
            obj.absoluteEncoderOffset_rad = desiredAngle_rad - measuredAngle_rad;
            fprintf('%s: Encoder offset = %.3f rad (%.1f deg)\n', ...
                obj.moduleName, obj.absoluteEncoderOffset_rad, ...
                rad2deg(obj.absoluteEncoderOffset_rad));
        end
        
        function exportGains(obj, filename)
            % Export all controller gains
            
            output.module = obj.moduleName;
            output.drive.feedforward = obj.driveFF;
            output.drive.pid = obj.drivePID;
            output.azimuth.feedforward = obj.azimuthFF;
            output.azimuth.pid = obj.azimuthPID;
            output.azimuth.motionMagic = obj.azimuthMotionMagic;
            output.azimuth.encoderOffset = obj.absoluteEncoderOffset_rad;
            
            jsonStr = jsonencode(output);
            fid = fopen(filename, 'w');
            fprintf(fid, '%s', jsonStr);
            fclose(fid);
        end
    end
end
