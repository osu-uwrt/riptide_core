function H = talos_numeric_jacobian(x, measurementIndex)
%TALOS_NUMERIC_JACOBIAN Code-generation-safe normalized measurement Jacobian.
sizes = [8 1 3 1 16];
H = zeros(sizes(measurementIndex),16);
for k = 1:16
    h = 1e-6 * max(1, abs(x(k)));
    xp=x; xm=x; xp(k)=xp(k)+h; xm(k)=xm(k)-h;
    H(:,k) = (dispatch(xp,measurementIndex)-dispatch(xm,measurementIndex))/(2*h);
end
end

function y = dispatch(x, index)
switch index
    case 1, y = talos_measure_imu(x);
    case 2, y = talos_measure_fog(x);
    case 3, y = talos_measure_dvl(x);
    case 4, y = talos_measure_depth(x);
    otherwise, y = talos_measure_reset(x);
end
end
