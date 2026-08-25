function plot_track(track)

feature = find_track_features(track);

X = track.x;
Y = track.y;
kappa = track.kappa;

figure
hold on
grid on
axis equal
axis tight

title('Track Curvature Map')

xlabel('X [m]')
ylabel('Y [m]')

scatter( ...
    X,...
    Y,...
    18,...
    kappa,...
    'filled');

colormap(jet)

cb = colorbar;
ylabel(cb,'Curvature [1/m]')

%% 畫點點

% scatter(feature.x,...
%         feature.y,...
%         90,...
%         'r',...
%         'filled');

%% Label

% for i = 1:length(feature.idx)
% 
%     text(feature.x(i),...
%          feature.y(i),...
%          sprintf('%d',feature.idx(i)),...
%          'Color','r',...
%          'FontWeight','bold',...
%          'VerticalAlignment','bottom');
% 
% end

end