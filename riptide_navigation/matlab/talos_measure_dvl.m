function y = talos_measure_dvl(x, offset)
%TALOS_MEASURE_DVL Velocity at the DVL origin, expressed in base_link.
y = x(8:10) + cross(x(11:13), offset);
end
