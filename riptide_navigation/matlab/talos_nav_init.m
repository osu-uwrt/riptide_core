function S = talos_nav_init(p)
%TALOS_NAV_INIT Estimator state at startup: filter, reorder queue, history and counters.
%#codegen
z = talos_nav_sizes();
k = talos_nav_counters();
E = talos_nav_event();

F0 = talos_nav_filter_init(p, []);
F0.tilt_pending = p.estimator.initialize_tilt_from_imu ~= 0;
F0.initialized = false;
S.F = F0;
S.n = zeros(k.n_count, 1);
S.now = int64(0);
S.q = repmat(E, 1, 4 * z.queue);
S.qhead = 1;
S.qn = 0;
S.qcount = zeros(4, 1);
S.seq = 0;
S.recording = true;

S.hin = repmat(E, 1, z.inputs);
S.hn = 0;
S.hfirst = 0;
ck.F = F0;
ck.t = int64(0);
ck.idx = 0;
S.hck = repmat(ck, 1, z.checkpoints);
S.ck_first = 1;
S.ck_total = 0;
S.scratch = repmat(E, 1, z.inputs + 1);

S.pub_x = F0.x;
S.has_pub = false;
S.last_pub = int64(0);
S.snap = true;
S.pub_offset = zeros(2, 1);
S.last_clock = int64(0);
S.reset_stamp = int64(0);
S.late_stamp = zeros(4, 1, 'int64');
S.imu_groups = ones(3, 1);
S.dvl_sigma_hist = zeros(1, z.sigma);
S.dvl_sigma_count = 0;
S.fog_fallback = false;
S.arrival_age = zeros(4, 1);
S.arrival_min = zeros(4, 1);
S.arrival_max = zeros(4, 1);
S.arrival_count = zeros(4, 1);
S.arrival_valid = false(4, 1);
S.late_lag = zeros(4, 1);
S.late_lag_max = zeros(4, 1);
S.set_pose_error = 0;
S.last_diag = int64(0);
S.step = 0;
S.preview_events = 0;
S.preview_limited = false;
S.pub_prediction = 0;
S.base_cnt = F0.cnt;
S.base_n = zeros(k.n_count, 1);
S.base_garbage = zeros(4, 1);
S.overloaded_last = false;

end
