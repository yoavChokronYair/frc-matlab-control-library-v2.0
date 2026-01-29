classdef KrakenX60Model < handle
    % Kraken X60 Motor Model
    % Higher torque variant
    
    properties (Constant)
        % Electrical Properties
        FREE_SPEED_RPM = 6000;
        FREE_SPEED_RAD_S = 628.3;
        STALL_TORQUE_NM = 10.5;         % Higher torque than X40
        STALL_CURRENT_A = 606;
        FREE_CURRENT_A = 2.5;
        RESISTANCE_OHM = 0.020;
        KV = 500;
        KT = 0.0173;                    % Higher torque constant
        
        MAX_CONTINUOUS_CURRENT_A = 40;
        NOMINAL_VOLTAGE_V = 12;
        
        MOTOR_MASS_KG = 0.440;          % Slightly heavier
        ROTOR_INERTIA_KG_M2 = 0.00012;
    end
    
    properties
        currentLimit_A
        efficiencyFactor
        thermalModel
    end
    
    methods
        function obj = KrakenX60Model(currentLimit_A)
            if nargin < 1
                obj.currentLimit_A = obj.MAX_CONTINUOUS_CURRENT_A;
            else
                obj.currentLimit_A = min(currentLimit_A, obj.MAX_CONTINUOUS_CURRENT_A);
            end
            obj.efficiencyFactor = 0.90;
            obj.thermalModel = [];
        end
        
        function torque = getTorque(obj, current_A)
            current_A = min(current_A, obj.currentLimit_A);
            torque = obj.KT * current_A * obj.efficiencyFactor;
        end
        
        function current = getCurrent(obj, torque_Nm)
            current = (torque_Nm / obj.efficiencyFactor) / obj.KT;
            current = min(current, obj.currentLimit_A);
        end
        
        function voltage = getVoltage(obj, speed_rad_s, torque_Nm)
            current = obj.getCurrent(torque_Nm);
            kemf = 1 / obj.KV * (60 / (2*pi));
            voltage = current * obj.RESISTANCE_OHM + kemf * speed_rad_s;
        end
        
        function maxTorque = getMaxTorqueAtSpeed(obj, speed_rad_s)
            max_voltage = obj.NOMINAL_VOLTAGE_V;
            kemf = 1 / obj.KV * (60 / (2*pi));
            
            max_current = (max_voltage - kemf * speed_rad_s) / obj.RESISTANCE_OHM;
            max_current = min(max_current, obj.currentLimit_A);
            
            maxTorque = obj.getTorque(max_current);
        end
        
        function [kV, kA, kS] = getFeedforwardConstants(obj, gearRatio, efficiency)
            if nargin < 3
                efficiency = obj.efficiencyFactor;
            end
            
            kemf = 1 / obj.KV * (60 / (2*pi));
            kV = kemf * gearRatio / efficiency;
            kA = obj.RESISTANCE_OHM * obj.KT * obj.ROTOR_INERTIA_KG_M2 * gearRatio^2;
            kS = 0.1;
        end
        
        function powerDraw = getPowerDraw(obj, speed_rad_s, torque_Nm)
            current = obj.getCurrent(torque_Nm);
            voltage = obj.getVoltage(speed_rad_s, torque_Nm);
            powerDraw = voltage * current;
        end
    end
end
