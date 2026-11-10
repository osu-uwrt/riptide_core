function talos_navigation_setup
%TALOS_NAVIGATION_SETUP Put the estimator's MATLAB functions on the path and define the model's buses.
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'matlab'));
talos_navigation_buses;
end
