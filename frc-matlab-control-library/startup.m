%% FRC MATLAB Control Library v2.0
%
% Startup script - Run this to initialize the library
%
% This initializes everything you need:
% - Adds all library folders to MATLAB path
% - Initializes visualization plugin system
% - Shows quick start guide
%
% Usage:
%   cd /path/to/frc-matlab-control-library
%   startup

clear all
clc

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('      FRC MATLAB CONTROL LIBRARY v2.0\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% Get library path
libPath = fileparts(mfilename('fullpath'));

fprintf('📂 Library Path: %s\n\n', libPath);

% Add all subdirectories to path
fprintf('📚 Adding library folders to MATLAB path...\n');
addpath(libPath);
addpath(genpath(fullfile(libPath, 'src')));
addpath(fullfile(libPath, 'examples'));
fprintf('   ✓ Motor models\n');
fprintf('   ✓ Mechanism templates\n');
fprintf('   ✓ Control synthesis\n');
fprintf('   ✓ Simulation engine\n');
fprintf('   ✓ System identification\n');
fprintf('   ✓ Visualization plugins\n');
fprintf('   ✓ Examples\n');

% Initialize visualization system
fprintf('🔌 Initializing visualization plugin system...\n');
try
    VisualizationSystemInit();
    fprintf('   ✓ Plugin system initialized\n');
    fprintf('   ✓ 7 visualizations registered\n');
    fprintf('   ✓ 4 validators registered\n');
catch ME
    fprintf('   ⚠ Warning: %s\n', ME.message);
    fprintf('   Try running: VisualizationSystemInit()\n');
end

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('                     READY TO USE!\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

fprintf('📋 QUICK START:\n\n');
fprintf('   1. Create a mechanism:\n');
fprintf('      >> turret = SimpleRotationalMechanism(''MyTurret'', ''KrakenX40'');\n');
fprintf('      >> turret.gearRatio = 10;\n');
fprintf('      >> turret.inertia_kg_m2 = 0.05;\n\n');

fprintf('   2. Auto-tune controllers:\n');
fprintf('      >> turret.calculateFeedforward();\n');
fprintf('      >> turret.calculatePID(0.5, 5);\n\n');

fprintf('   3. Simulate:\n');
fprintf('      >> testSeq.time = linspace(0, 2, 200);\n');
fprintf('      >> testSeq.voltage = [zeros(100,1); 6*ones(100,1)];\n');
fprintf('      >> simData = DynamicsSimulator.simulateResponse(turret, testSeq);\n\n');

fprintf('   4. Visualize:\n');
fprintf('      >> VisualizationSystem.runVisualization(''standard'', simData);\n\n');

fprintf('   5. Validate:\n');
fprintf('      >> results = VisualizationSystem.runValidator(''safety'', simData, turret);\n\n');


fprintf('📖 DOCUMENTATION:\n\n');
fprintf('   • docs/GETTING_STARTED.md    - Complete tutorial\n');
fprintf('   • docs/API_REFERENCE.md      - Function reference\n');
fprintf('   • docs/ARCHITECTURE.md       - System design\n');
fprintf('   • README.md                  - Project overview\n\n');

fprintf('🎯 EXAMPLES:\n\n');
fprintf('   >> Example_SimpleTurret_v2_0        - Simple turret\n');
fprintf('   >> Example_Using_Plugin_System      - All features\n\n');

fprintf('💡 HELPFUL COMMANDS:\n\n');
fprintf('   VisualizationSystem.printRegistry();   - See all plugins\n');
fprintf('   help VisualizationSystem               - Full API help\n');
fprintf('   help DynamicsSimulator                 - Simulation help\n');
fprintf('   help SimpleRotationalMechanism         - Mechanism help\n\n');

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('                  Happy Simulating! 🚀\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');
