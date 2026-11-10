function p = talos_nav_params(config)
%TALOS_NAV_PARAMS Model parameters of a navigation config, as the model's param bus carries them.
% Reads through launch/navigation_config.py so the enum and bool conventions match the model.

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1
    config = fullfile(root, 'config', 'talos_navigation.yaml');
end
[status, text] = system(['python3 "' fullfile(root, 'launch', 'navigation_config.py') '" json "' config '"']);
assert(status == 0, text);
p = jsondecode(text);
end
