classdef KrakenX40Model < handle
    % Kraken X40 Motor Model
    % Specifications based on manufacturer data
    
    properties (Constant)
        % Electrical Properties
        FREE_SPEED_RPM = 6000;          % Free speed (RPM)
        FREE_SPEED_RAD_S = 628.3;       % Free speed (rad/s)
        STALL_TORQUE_NM = 7.09;         % Stall torque (N⋅m)
        STALL_CURRENT_A = 483;          % Stall current (A)
        FREE_CURRENT_A = 2.0;           % Free current (A)
        RESISTANCE_OHM = 0.025;         % Terminal resistance (Ω)
        KV = 500;                       % Velocity constant (RPM/V)
        KT = 0.0147;                    % Torque constant (N⋅m/A)
        
        % Operational Limits
        MAX_CONTINUOUS_CURRENT_A = 40;  % FRC current limit
        NOMINAL_VOLTAGE_V = 12;         % Battery voltage
        
        % Physical Properties
        MOTOR_MASS_KG = 0.385;          % Motor mass
        ROTOR_INERTIA_KG_M2 = 0.0001;   % Rotor moment of inertia
    end
    
    properties
        currentLimit_A          % User-defined current limit
        efficiencyFactor        % Accounts for real-world losses (0.8-0.95)
        thermalModel            % Optional: thermal derating
    end
    
    methods
        function obj = KrakenX40Model(currentLimit_A)
            % Constructor
            if nargin < 1
                obj.currentLimit_A = obj.MAX_CONTINUOUS_CURRENT_A;
            else
                obj.currentLimit_A = min(currentLimit_A, obj.MAX_CONTINUOUS_CURRENT_A);
            end
            obj.efficiencyFactor = 0.90; % Conservative default
            obj.thermalModel = [];
        end
        
        function torque = getTorque(obj, current_A)
            % Calculate torque from current
            % Torque = Kt * Current
            current_A = min(current_A, obj.currentLimit_A);
            torque = obj.KT * current_A * obj.efficiencyFactor;
        end
        
        function current = getCurrent(obj, torque_Nm)
            % Calculate required current for torque
            current = (torque_Nm / obj.efficiencyFactor) / obj.KT;
            current = min(current, obj.currentLimit_A);
        end
        
        function voltage = getVoltage(obj, speed_rad_s, torque_Nm)
            % Motor voltage equation: V = I*R + Kemf*ω
            current = obj.getCurrent(torque_Nm);
            kemf = 1 / obj.KV * (60 / (2*pi)); % Back-EMF constant
            voltage = current * obj.RESISTANCE_OHM + kemf * speed_rad_s;
        end
        
        function maxTorque = getMaxTorqueAtSpeed(obj, speed_rad_s)
            % Maximum torque available at given speed (limited by current)
            % Account for back-EMF reducing available current
            max_voltage = obj.NOMINAL_VOLTAGE_V;
            kemf = 1 / obj.KV * (60 / (2*pi));
            
            % V = I*R + Kemf*ω  →  I = (V - Kemf*ω) / R
            max_current = (max_voltage - kemf * speed_rad_s) / obj.RESISTANCE_OHM;
            max_current = min(max_current, obj.currentLimit_A);
            
            maxTorque = obj.getTorque(max_current);
        end
        
        function [kV, kA, kS] = getFeedforwardConstants(obj, gearRatio, efficiency)
            % Calculate feedforward constants for velocity control
            % kV: Velocity feedforward (V/(rad/s))
            % kA: Acceleration feedforward (V/(rad/s²))
            % kS: Static friction feedforward (V)
            
            if nargin < 3
                efficiency = obj.efficiencyFactor;
            end
            
            kemf = 1 / obj.KV * (60 / (2*pi)); % V/(rad/s) at motor
            
            % Scale for gear ratio (output shaft perspective)
            kV = kemf * gearRatio / efficiency;
            
            % Acceleration constant from rotor inertia
            kA = obj.RESISTANCE_OHM * obj.KT * obj.ROTOR_INERTIA_KG_M2 * gearRatio^2;
            
            % Static friction (empirical, needs real-world tuning)
            kS = 0.1; % Placeholder, System ID will find real value
        end
        
        function powerDraw = getPowerDraw(obj, speed_rad_s, torque_Nm)
            % Calculate electrical power draw (Watts)
            current = obj.getCurrent(torque_Nm);
            voltage = obj.getVoltage(speed_rad_s, torque_Nm);
            powerDraw = voltage * current;
        end
    end
end
