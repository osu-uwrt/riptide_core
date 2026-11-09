% Part 1 model-in-the-loop assertions. This intentionally uses ordinary
% SimulationInput simulations because this MATLAB installation has no
% Simulink Test license.
talos_ekf_init;
open_system(fullfile(fileparts(mfilename('fullpath')), 'talos_ekf.slx'));

% The gravity measurement must be exactly insensitive to global yaw.
jacobianContext = [0.31; -0.42; 0.851; 1; 1; 0; 1; 1; 0; 1; 1; 1];
jacobianState = [zeros(3,1); 0.72; -0.21; 0.43; 0.49; zeros(9,1)];
jacobianState(4:7) = jacobianState(4:7)/norm(jacobianState(4:7));
analyticJacobian = talos_measure_imu_configured_jacobian(jacobianState, jacobianContext);
numericJacobian = talos_numeric_jacobian(jacobianState, 1, jacobianContext);
assert(max(abs(analyticJacobian(:)-numericJacobian(:))) < 2e-8);
q = jacobianState(4:7);
worldYawTangent = [-q(4); -q(3); q(2); q(1)]/2;
assert(norm(analyticJacobian(1:3,4:7)*worldYawTangent) < 1e-13);

dt = talosEkfCfg.sampleTime;
t = (0:dt:2)';
N = numel(t);

stationaryImu = repmat([0 0 1 0 0 0 0 0], N, 1);
out = run_case(t, stationaryImu, true(N,1), true(N,1), true(N,1), true(N,1));
stationaryState = out.yout{1}.Values.Data;
stationaryCovariance = out.yout{2}.Values.Data;
assert(all(isfinite(stationaryState), 'all'));
assert(max(vecnorm(stationaryState(:,1:3), 2, 2)) < 1e-8);
assert_covariance(stationaryCovariance);

roll = pi * t;
flipImu = [zeros(N,1), sin(roll), cos(roll), repmat([pi 0],N,1), zeros(N,3)];
imuEnable = true(N,1);
imuEnable(t > 1.5) = false; % finish through prediction to exercise dropout
out = run_case(t, flipImu, imuEnable, false(N,1), false(N,1), false(N,1));
flipState = out.yout{1}.Values.Data;
flipCovariance = out.yout{2}.Values.Data;
quaternionNorm = vecnorm(flipState(:,4:7), 2, 2);
assert(all(isfinite(flipState), 'all'));
assert(max(abs(quaternionNorm - 1)) < 5e-3);
assert_covariance(flipCovariance);

% q and -q describe the same attitude. Gravity-direction measurements are
% deliberately identical, so no sign discontinuity can enter the innovation.
imuContext = [0; 0; 1; 1; 1; 0; 1; 1; 0; 1; 1; 1];
gPositive = talos_measure_imu_configured([zeros(3,1); 1; 0; 0; 0; zeros(9,1)], imuContext);
gNegative = talos_measure_imu_configured([zeros(3,1); -1; 0; 0; 0; zeros(9,1)], imuContext);
assert(norm(gPositive - gNegative) < 1e-12);

% A disabled channel has an exactly zero measurement row and Jacobian.
maskedContext = imuContext;
maskedContext(5) = 0;
maskedH = talos_measure_imu_configured_jacobian( ...
    [zeros(3,1); 1; 0; 0; 0; zeros(9,1)], maskedContext);
assert(norm(maskedH(2,:)) < 1e-12);

fprintf('stationary_samples=%d\n', size(stationaryState,1));
fprintf('flip_samples=%d\n', size(flipState,1));
fprintf('max_quaternion_norm_error=%.12g\n', max(abs(quaternionNorm-1)));
fprintf('minimum_covariance_eigenvalue=%.12g\n', minimum_covariance_eigenvalue(flipCovariance));

function out = run_case(t, imu, enableImu, enableFog, enableDvl, enableDepth)
cfg = evalin('base','talosEkfCfg');
N = numel(t);
ds = Simulink.SimulationData.Dataset;
Q = cfg.processNoise * cfg.sampleTime;
Q(4:7,4:7) = 0;
Q(5:7,5:7) = 0.25 * cfg.processNoise(4:6,4:6) * cfg.sampleTime;
ds{1} = timeseries(repmat(Q,1,1,N),t,'IsTimeFirst',false);
ds{2} = timeseries(repmat(cfg.sampleTime,N,1),t);
ds{3} = timeseries(enableImu,t);
imuMeasurement = [zeros(N,3), imu(:,4:5), zeros(N,1), imu(:,6:8)];
imuNoise = diag([diag(cfg.imuNoise(1:3,1:3)); ...
    diag(cfg.imuNoise(4:5,4:5)); cfg.imuNoise(4,4); diag(cfg.imuNoise(6:8,6:8))]);
ds{4} = timeseries(imuMeasurement,t);
ds{5} = timeseries(repmat(imuNoise,1,1,N),t,'IsTimeFirst',false);
ds{6} = timeseries(enableFog,t);
ds{7} = timeseries(zeros(N,3),t);
ds{8} = timeseries(repmat(cfg.fogNoise*eye(3),1,1,N),t,'IsTimeFirst',false);
ds{9} = timeseries(enableDvl,t);
ds{10} = timeseries(zeros(N,3),t);
ds{11} = timeseries(repmat(cfg.dvlNoise,1,1,N),t,'IsTimeFirst',false);
ds{12} = timeseries(enableDepth,t);
ds{13} = timeseries(zeros(N,1),t);
ds{14} = timeseries(repmat(cfg.depthNoise,N,1),t);
ds{15} = timeseries(false(N,1),t);
ds{16} = timeseries(zeros(N,16),t);
ds{17} = timeseries(repmat(cfg.resetNoise,1,1,N),t,'IsTimeFirst',false);
ds{18} = timeseries(repmat([0 0 0 1 1 1],N,1),t);
ds{19} = timeseries([imu(:,1:3), repmat([1 1 0 1 1 0 1 1 1],N,1)],t);
ds{20} = timeseries(repmat([0 0 1],N,1),t);
ds{21} = timeseries(ones(N,1),t);
in = Simulink.SimulationInput('talos_ekf');
in = in.setExternalInput(ds).setModelParameter('StopTime',num2str(t(end)));
out = sim(in);
end

function assert_covariance(data)
assert(all(isfinite(data),'all'));
for k = 1:25:size(data,3)
    covariance = 0.5 * (data(:,:,k) + data(:,:,k)');
    assert(max(abs(covariance-data(:,:,k)),[],'all') < 1e-8);
    assert(min(eig(covariance)) > -1e-8);
end
end

function result = minimum_covariance_eigenvalue(data)
result = inf;
for k = 1:25:size(data,3)
    result = min(result,min(eig(0.5*(data(:,:,k)+data(:,:,k)'))));
end
end
