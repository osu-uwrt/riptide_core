function generate_talos_ekf(target)
%GENERATE_TALOS_EKF Regenerate the standalone C++ estimator core.
% target is "x86_64" or "arm64". Run this file from its containing folder.

arguments
    target (1,1) string {mustBeMember(target,["x86_64","arm64"])} = "x86_64"
end

modelPath = fullfile(fileparts(mfilename('fullpath')), 'talos_ekf.slx');
open_system(modelPath);
talos_ekf_init;
if target == "arm64"
    hardware = 'ARM Compatible->ARM 64-bit (LP64)';
else
    hardware = 'Intel->x86-64 (Linux 64)';
end
config = getActiveConfigSet('talos_ekf');
set_param(config, 'ProdHWDeviceType', hardware);
set_param(config, 'GenCodeOnly', 'on');
set_param(config, 'GenerateReport', 'off');
% Keep architecture-specific generated sources side by side. CMake selects the
% matching directory from CMAKE_SYSTEM_PROCESSOR, so an ARM vehicle build never
% attempts to compile x86 SIMD intrinsics (and vice versa).
output = fullfile(fileparts(modelPath), 'generated', target);
Simulink.fileGenControl('set','CodeGenFolder',output,'createDir',true);
rtwbuild('talos_ekf');
end
