classdef VisualizationSystem < handle
    % Extensible visualization and validation plugin system
    % Allows adding visualization/validation features without modifying core classes
    %
    % This system provides a plugin-based architecture where visualization and
    % validation tools are registered and can be applied to simulation results
    % without requiring changes to the simulation engine or mechanism classes.
    
    properties (Constant)
        VERSION = '2.0'
        PLUGIN_REGISTRY_FILE = 'visualization_plugins.mat'
    end
    
    properties (Access = private)
    end
    
    methods (Static, Access = private)
        function map = getPlugins()
            persistent plugins
            if isempty(plugins)
                plugins = containers.Map();
            end
            map = plugins;
        end
        
        function setPlugins(map)
            persistent plugins
            plugins = map;
        end
        
        function map = getValidators()
            persistent validators
            if isempty(validators)
                validators = containers.Map();
            end
            map = validators;
        end
        
        function setValidators(map)
            persistent validators
            validators = map;
        end
        
        function map = getReporters()
            persistent reporters
            if isempty(reporters)
                reporters = containers.Map();
            end
            map = reporters;
        end
        
        function setReporters(map)
            persistent reporters
            reporters = map;
        end
    end
    
    methods (Static)
        % ===== PLUGIN REGISTRATION =====
        
        function registerVisualization(pluginName, functionHandle, description)
            % Register a visualization function
            %
            % Usage:
            %   VisualizationSystem.registerVisualization('myPlot', @myPlotFunction, ...
            %       'Plots my custom visualization')
            %
            % Function signature:
            %   function myPlotFunction(simData, varargin)
            %       % simData: simulation results struct
            %       % varargin: optional parameters
            %   end
            
            pluginInfo.handle = functionHandle;
            pluginInfo.description = description;
            pluginInfo.type = 'visualization';
            pluginInfo.registered = datetime('now');
            
            plugins = VisualizationSystem.getPlugins();
            plugins(pluginName) = pluginInfo;
            VisualizationSystem.setPlugins(plugins);
            fprintf('[VisualizationSystem] Registered visualization: %s\n', pluginName);
        end
        
        function registerValidator(validatorName, functionHandle, description)
            % Register a validation function
            %
            % Usage:
            %   VisualizationSystem.registerValidator('myValidation', @myValidationFunc, ...
            %       'Validates my custom metrics')
            %
            % Function signature:
            %   function results = myValidationFunc(simData, mechanism, varargin)
            %       % simData: simulation results
            %       % mechanism: mechanism object
            %       % results: struct with validation results
            %   end
            
            validatorInfo.handle = functionHandle;
            validatorInfo.description = description;
            validatorInfo.type = 'validator';
            validatorInfo.registered = datetime('now');
            
            validators = VisualizationSystem.getValidators();
            validators(validatorName) = validatorInfo;
            VisualizationSystem.setValidators(validators);
            fprintf('[VisualizationSystem] Registered validator: %s\n', validatorName);
        end
        
        function registerReporter(reporterName, functionHandle, description)
            % Register a report generator function
            %
            % Usage:
            %   VisualizationSystem.registerReporter('myReport', @myReportFunc, ...
            %       'Generates my custom report')
            %
            % Function signature:
            %   function reportText = myReportFunc(results, varargin)
            %       % results: validation/analysis results
            %       % reportText: cell array of report lines
            %   end
            
            reporterInfo.handle = functionHandle;
            reporterInfo.description = description;
            reporterInfo.type = 'reporter';
            reporterInfo.registered = datetime('now');
            
            reporters = VisualizationSystem.getReporters();
            reporters(reporterName) = reporterInfo;
            VisualizationSystem.setReporters(reporters);
            fprintf('[VisualizationSystem] Registered reporter: %s\n', reporterName);
        end
        
        % ===== PLUGIN DISCOVERY =====
        
        function list = listVisualizations()
            % List all registered visualizations
            list = keys(VisualizationSystem.getPlugins());
        end
        
        function list = listValidators()
            % List all registered validators
            list = keys(VisualizationSystem.getValidators());
        end
        
        function list = listReporters()
            % List all registered reporters
            list = keys(VisualizationSystem.getReporters());
        end
        
        function info = getVisualizationInfo(name)
            % Get info about a visualization
            plugins = VisualizationSystem.getPlugins();
            if plugins.isKey(name)
                info = plugins(name);
            else
                error('Visualization "%s" not registered', name);
            end
        end
        
        function info = getValidatorInfo(name)
            % Get info about a validator
            validators = VisualizationSystem.getValidators();
            if validators.isKey(name)
                info = validators(name);
            else
                error('Validator "%s" not registered', name);
            end
        end
        
        % ===== PLUGIN EXECUTION =====
        
        function runVisualization(pluginName, simData, varargin)
            % Execute a registered visualization
            %
            % Usage:
            %   VisualizationSystem.runVisualization('myPlot', simData);
            %   VisualizationSystem.runVisualization('myPlot', simData, 'param1', value1);
            
            plugins = VisualizationSystem.getPlugins();
            if ~plugins.isKey(pluginName)
                error('Visualization "%s" not registered. Available: %s', ...
                    pluginName, strjoin(VisualizationSystem.listVisualizations(), ', '));
            end
            
            plugin = plugins(pluginName);
            try
                plugin.handle(simData, varargin{:});
            catch ME
                fprintf('[ERROR] Visualization "%s" failed:\n', pluginName);
                rethrow(ME);
            end
        end
        
        function results = runValidator(validatorName, simData, mechanism, varargin)
            % Execute a registered validator
            %
            % Usage:
            %   results = VisualizationSystem.runValidator('myValidation', simData, mechanism);
            %   results = VisualizationSystem.runValidator('myValidation', simData, mechanism, ...
            %       'param1', value1);
            
            validators = VisualizationSystem.getValidators();
            if ~validators.isKey(validatorName)
                error('Validator "%s" not registered. Available: %s', ...
                    validatorName, strjoin(VisualizationSystem.listValidators(), ', '));
            end
            
            validator = validators(validatorName);
            try
                results = validator.handle(simData, mechanism, varargin{:});
            catch ME
                fprintf('[ERROR] Validator "%s" failed:\n', validatorName);
                rethrow(ME);
            end
        end
        
        function report = runReporter(reporterName, results, varargin)
            % Execute a registered reporter
            %
            % Usage:
            %   report = VisualizationSystem.runReporter('myReport', results);
            %   report = VisualizationSystem.runReporter('myReport', results, 'param1', value1);
            
            reporters = VisualizationSystem.getReporters();
            if ~reporters.isKey(reporterName)
                error('Reporter "%s" not registered. Available: %s', ...
                    reporterName, strjoin(VisualizationSystem.listReporters(), ', '));
            end
            
            reporter = reporters(reporterName);
            try
                report = reporter.handle(results, varargin{:});
            catch ME
                fprintf('[ERROR] Reporter "%s" failed:\n', reporterName);
                rethrow(ME);
            end
        end
        
        % ===== BATCH OPERATIONS =====
        
        function results = runAllValidators(simData, mechanism, varargin)
            % Run all registered validators on simulation data
            %
            % Output:
            %   results: struct with field for each validator
            
            results = struct();
            validators = VisualizationSystem.listValidators();
            
            for i = 1:length(validators)
                name = validators{i};
                try
                    results.(name) = VisualizationSystem.runValidator(name, simData, mechanism, varargin{:});
                    fprintf('✓ Validator "%s" completed\n', name);
                catch ME
                    fprintf('✗ Validator "%s" failed: %s\n', name, ME.message);
                    results.(name) = [];
                end
            end
        end
        
        function plotAllVisualizations(simData, varargin)
            % Run all registered visualizations on simulation data
            
            visualizations = VisualizationSystem.listVisualizations();
            
            for i = 1:length(visualizations)
                name = visualizations{i};
                try
                    VisualizationSystem.runVisualization(name, simData, varargin{:});
                    fprintf('✓ Visualization "%s" completed\n', name);
                catch ME
                    fprintf('✗ Visualization "%s" failed: %s\n', name, ME.message);
                end
            end
        end
        
        % ===== PLUGIN PERSISTENCE =====
        
        function savePluginRegistry(filename)
            % Save registered plugins to file
            
            if nargin < 1
                filename = VisualizationSystem.PLUGIN_REGISTRY_FILE;
            end
            
            plugins = VisualizationSystem.getPlugins();
            validators = VisualizationSystem.getValidators();
            reporters = VisualizationSystem.getReporters();
            
            save(filename, 'plugins', 'validators', 'reporters');
            fprintf('Plugin registry saved to: %s\n', filename);
        end
        
        function loadPluginRegistry(filename)
            % Load registered plugins from file
            
            if nargin < 1
                filename = VisualizationSystem.PLUGIN_REGISTRY_FILE;
            end
            
            if isfile(filename)
                load(filename, 'plugins', 'validators', 'reporters');
                VisualizationSystem.setPlugins(plugins);
                VisualizationSystem.setValidators(validators);
                VisualizationSystem.setReporters(reporters);
                fprintf('Plugin registry loaded from: %s\n', filename);
            else
                fprintf('Warning: Plugin registry file not found: %s\n', filename);
            end
        end
        
        % ===== UTILITY =====
        
        function printRegistry()
            % Print all registered plugins
            
            fprintf('\n');
            fprintf('====================================\n');
            fprintf('Visualization Plugin Registry\n');
            fprintf('====================================\n\n');
            
            plugins = VisualizationSystem.getPlugins();
            validators = VisualizationSystem.getValidators();
            reporters = VisualizationSystem.getReporters();
            
            % Visualizations
            if ~plugins.isempty()
                fprintf('VISUALIZATIONS:\n');
                for name = VisualizationSystem.listVisualizations()
                    info = plugins(name{1});
                    fprintf('  • %s\n', name{1});
                    fprintf('    └─ %s\n', info.description);
                end
                fprintf('\n');
            else
                fprintf('VISUALIZATIONS: (none registered)\n\n');
            end
            
            % Validators
            if ~validators.isempty()
                fprintf('VALIDATORS:\n');
                for name = VisualizationSystem.listValidators()
                    info = validators(name{1});
                    fprintf('  • %s\n', name{1});
                    fprintf('    └─ %s\n', info.description);
                end
                fprintf('\n');
            else
                fprintf('VALIDATORS: (none registered)\n\n');
            end
            
            % Reporters
            if ~reporters.isempty()
                fprintf('REPORTERS:\n');
                for name = VisualizationSystem.listReporters()
                    info = reporters(name{1});
                    fprintf('  • %s\n', name{1});
                    fprintf('    └─ %s\n', info.description);
                end
                fprintf('\n');
            else
                fprintf('REPORTERS: (none registered)\n\n');
            end
            
            fprintf('====================================\n\n');
        end
        
        function unregisterAll()
            % Clear all registered plugins (for testing/reset)
            VisualizationSystem.setPlugins(containers.Map());
            VisualizationSystem.setValidators(containers.Map());
            VisualizationSystem.setReporters(containers.Map());
            fprintf('[VisualizationSystem] All plugins unregistered\n');
        end
    end
end