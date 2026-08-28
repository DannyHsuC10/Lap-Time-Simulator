function feature = find_track_features(track)
%==========================================================
% Find track feature points from curvature gradient
%
% Output
%   feature.idx        Point index
%   feature.s          Distance [m]
%   feature.x
%   feature.y
%   feature.kappa
%   feature.dkappa
%==========================================================

%% Curvature gradient

dkappa = gradient(track.kappa, track.s);

%% Find local maxima

[pks,idx] = findpeaks(abs(dkappa), ...
    'MinPeakProminence',0.02,...
    'MinPeakDistance',20);

%% Save

feature.idx    = idx;
feature.s      = track.s(idx);

feature.x      = track.x(idx);
feature.y      = track.y(idx);

feature.kappa  = track.kappa(idx);
feature.dkappa = dkappa(idx);
feature.score  = pks;

end