function k = talos_nav_counters
%TALOS_NAV_COUNTERS Index of every counter. c_* live in the replayed filter state, n_* outside it.

k.c_tilt_corrections = 1;
k.c_accepted = 2;
k.c_stale = 6;
k.c_duplicate = 7;
k.c_invalid = 8;
k.c_stale_by = 9;
k.c_duplicate_by = 13;
k.c_invalid_by = 17;
k.c_startup_epoch = 21;
k.c_tilt_init = 22;
k.c_innovation = 23;
k.c_innovation_by = 24;
k.c_recoveries = 28;
k.c_recoveries_by = 29;
k.c_frozen_by = 33;
k.c_holds_by = 37;
k.c_count = 40;

k.n_arrivals = 1;
k.n_pre_clock = 5;
k.n_future = 9;
k.n_queue_by = 13;
k.n_queue = 17;
k.n_transform = 18;
k.n_transform_by = 19;
k.n_empty_frame = 23;
k.n_empty_fallback = 27;
k.n_lagged_replays = 31;
k.n_late_fallbacks = 35;
k.n_imu_quaternions = 39;
k.n_imu_unavailable = 40;
k.n_set_pose = 43;
k.n_resets = 44;
k.n_dvl_degraded = 45;
k.n_limited = 46;
k.n_overload = 47;
k.n_replayed = 48;
k.n_ring_lost = 49;
k.n_count = 49;
end
