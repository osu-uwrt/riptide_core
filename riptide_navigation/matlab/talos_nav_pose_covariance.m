function out = talos_nav_pose_covariance(mode, x, A, B)
%TALOS_NAV_POSE_COVARIANCE ROS pose covariance conversions with world-axis angular errors.
% 'ros': 6x6 pose covariance of state covariance A. 'fill': pose A with unspecified (zero
% diagonal) axes taken from the 6x6 B. 'apply': state covariance B with its pose block set from A.
%#codegen
w = x(4); qx = x(5); qy = x(6); qz = x(7);
G = [-qx -qy -qz; w qz -qy; -qz w qx; qy -qx w];
switch mode
    case 'ros'
        P = A;
        out = zeros(6, 6);
        out(1:3, 1:3) = P(1:3, 1:3);
        out(1:3, 4:6) = 2 * P(1:3, 4:7) * G;
        out(4:6, 1:3) = out(1:3, 4:6)';
        out(4:6, 4:6) = 4 * G' * P(4:7, 4:7) * G;
    case 'fill'
        out = A;
        un = diag(A) == 0;
        for i = 1:6
            for j = 1:6
                if ~un(i) && ~un(j)
                    continue;
                end
                if un(i) && un(j)
                    out(i, j) = B(i, j);
                else
                    out(i, j) = 0;
                end
            end
        end
    otherwise
        out = B;
        out(1:3, 1:3) = A(1:3, 1:3);
        out(1:3, 4:7) = 0.5 * A(1:3, 4:6) * G';
        out(4:7, 1:3) = out(1:3, 4:7)';
        out(4:7, 4:7) = 0.25 * G * A(4:6, 4:6) * G';
end
end
