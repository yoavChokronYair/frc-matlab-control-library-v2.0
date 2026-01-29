classdef SimpleRotationalMechanism < handle
    % Template for simple rotational mechanisms: intake, turret
    % Single motor or gearbox, rotation about fixed axis, no gravity effects
    
    properties
        % Configuration
        mechanismName
        motorModel          % KrakenX40Model or KrakenX60Model
        numMotors           % Number of motors
        gearRatio           % Motor rotations : output rotations
        
        % Physical properties (from CAD)
        inertia_kg_m2       % Moment of inertia
        mass_kg             % Total mass
        
        % Friction (from tests or estimates)
        staticFriction_Nm   % Coulomb friction
        viscousFriction     % Viscous damping coefficient
        
        % Limits
        minPosition_rad
        maxPosition_rad
        maxVelocity_rad_s
        maxAcceleration_rad_s2
        
        % Controllers (outputs)
        feedforward
        pidGains
        motionMagicProfile
    end
    
    methods
        function obj = SimpleRotationalMechanism(name, motorType)
            % Constructor
            obj.mechanismName = name;
            
            if strcmp(motorType, 'KrakenX40')
                obj.motorModel = KrakenX40Model();
            elseif strcmp(motorType, 'KrakenX60')
                obj.motorModel = KrakenX60Model();
            else
                error('Unknown motor type');
            end
            
            % Defaults
            obj.numMotors = 1;
            obj.gearRatio = 10.0;
            obj.staticFriction_Nm = 0.5;
            obj.viscousFriction = 0.01;
        end
        
        function loadCADData(obj, cadData)
            % Import data from SolidWorks
            % cadData: struct with fields from Mass Properties
            
            obj.inertia_kg_m2 = cadData.inertia;
            obj.mass_kg = cadData.mass;
            
            fprintf('%s: Loaded CAD data\n', obj.mechanismName);
            fprintf('  Mass: %.2f kg\n', obj.mass_kg);
            fprintf('  Inertia: %.4f kg⋅m²\n', obj.inertia_kg_m2);
        end
        
        function setLimits(obj, minPos, maxPos, maxVel, maxAccel)
            % Set motion limits
            obj.minPosition_rad = minPos;
            obj.maxPosition_rad = maxPos;
            obj.maxVelocity_rad_s = maxVel;
            obj.maxAcceleration_rad_s2 = maxAccel;
        end
        
        function calculateFeedforward(obj)
            % Calculate feedforward gains
            friction.static = obj.staticFriction_Nm;
            friction.viscous = obj.viscousFriction;
            
            obj.feedforward = FeedforwardCalculator.calculateSimpleRotational(...
                obj.motorModel, obj.gearRatio, obj.inertia_kg_m2, friction);
            
            % Scale for multiple motors
            if obj.numMotors > 1
                obj.feedforward.kS = obj.feedforward.kS / obj.numMotors;
                obj.feedforward.kV = obj.feedforward.kV / obj.numMotors;
                obj.feedforward.kA = obj.feedforward.kA / obj.numMotors;
            end
        end
        
        function calculatePID(obj, settlingTime_s, overshoot_percent)
            % Calculate PID gains
            systemParams.inertia = obj.inertia_kg_m2;
            systemParams.damping = obj.viscousFriction;
            
            desiredResponse.settlingTime_s = settlingTime_s;
            desiredResponse.overshoot_percent = overshoot_percent;
            
            [kP, kI, kD] = PIDTuner.tunePositionPID(systemParams, desiredResponse);
            
            obj.pidGains.kP = kP;
            obj.pidGains.kI = kI;
            obj.pidGains.kD = kD;
        end
        
        function calculateMotionMagic(obj, safetyFactor)
            % Calculate Motion Magic profile parameters
            % safetyFactor: 0.7-0.9 (reduce theoretical max for safety)
            
            if nargin < 2
                safetyFactor = 0.8;
            end
            
            % Maximum velocity (limited by motor speed and current limit)
            maxMotorSpeed_rad_s = obj.motorModel.FREE_SPEED_RAD_S;
            maxOutputSpeed = (maxMotorSpeed_rad_s / obj.gearRatio) * safetyFactor;
            obj.motionMagicProfile.cruiseVelocity = min(maxOutputSpeed, obj.maxVelocity_rad_s);
            
            % Maximum acceleration (limited by motor torque)
            maxTorquePerMotor = obj.motorModel.getMaxTorqueAtSpeed(0); % At stall
            totalTorque = maxTorquePerMotor * obj.numMotors * obj.gearRatio;
            maxAccel = (totalTorque / obj.inertia_kg_m2) * safetyFactor;
            obj.motionMagicProfile.acceleration = min(maxAccel, obj.maxAcceleration_rad_s2);
            
            % S-curve smoothing (jerk limit)
            obj.motionMagicProfile.jerk = obj.motionMagicProfile.acceleration * 10; % Typical ratio
            
            fprintf('%s Motion Magic Profile:\n', obj.mechanismName);
            fprintf('  Cruise velocity: %.2f rad/s\n', obj.motionMagicProfile.cruiseVelocity);
            fprintf('  Acceleration: %.2f rad/s²\n', obj.motionMagicProfile.acceleration);
            fprintf('  Jerk: %.2f rad/s³\n', obj.motionMagicProfile.jerk);
        end
        
        function refineFromTests(obj, testData)
            % Update parameters based on real test data
            
            % Extract actual inertia and friction
            [inertia_measured, friction_measured] = ...
                ParameterExtraction.extractMechanicalParams(...
                testData, obj.motorModel, obj.gearRatio);
            
            % Update properties (blend with CAD)
            obj.inertia_kg_m2 = 0.7 * obj.inertia_kg_m2 + 0.3 * inertia_measured;
            obj.staticFriction_Nm = friction_measured;
            
            % Recalculate controllers
            obj.calculateFeedforward();
            obj.calculatePID(0.5, 5.0); % Default response
            obj.calculateMotionMagic();
            
            fprintf('%s: Refined from test data\n', obj.mechanismName);
        end
        
        function exportToRobot(obj, filename)
            % Export controller gains to JSON for robot code
            
            output.mechanism = obj.mechanismName;
            output.feedforward = obj.feedforward;
            output.pid = obj.pidGains;
            output.motionMagic = obj.motionMagicProfile;
            output.limits.min = obj.minPosition_rad;
            output.limits.max = obj.maxPosition_rad;
            
            jsonStr = jsonencode(output);
            fid = fopen(filename, 'w');
            fprintf(fid, '%s', jsonStr);
            fclose(fid);
            
            fprintf('Exported to: %s\n', filename);
        end
    end
end
