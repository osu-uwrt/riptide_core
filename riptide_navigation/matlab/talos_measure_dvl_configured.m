function y = talos_measure_dvl_configured(x, context)
%TALOS_MEASURE_DVL Velocity at the DVL origin, expressed in base_link.
offset = context(1:3);
mask = context(4:6);
y = mask.*(x(8:10) + cross(x(11:13), offset));
end
