function H = talos_numeric_jacobian(x, measurementIndex, context)
%TALOS_NUMERIC_JACOBIAN Code-generation-safe normalized measurement Jacobian.
sizes = [9 3 3 1 16];
H = zeros(sizes(measurementIndex),16);
for k = 1:16
    h = 1e-6 * max(1, abs(x(k)));
    xp=x; xm=x; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    H(:,k) = (dispatch(xp,measurementIndex,context)- ...
              dispatch(xm,measurementIndex,context))/(2*h);
end
end

function y = dispatch(x, index, context)
switch index
    case 1, y = talos_measure_imu_configured(x, context);
    case 2, y = talos_measure_fog_configured(x, context);
    case 3, y = talos_measure_dvl_configured(x, context);
    case 4, y = talos_measure_depth_configured(x, context(1));
    otherwise, y = talos_measure_reset(x);
end
end
