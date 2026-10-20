function track = build_track(segments, ds)

    % =========================================================
    % Geometry reconstruction
    % =========================================================
    [x, y, s, kappa_raw, section, type] = ...
        track_geometry(segments, ds);

    n = length(s);

    % =========================================================
    % Curvature smoothing
    % =========================================================
    kappa = kappa_raw;
    kappa = smoothdata(kappa_raw,'gaussian',50);% 較佳點
    kappa = smoothdata(kappa,'gaussian',100);% 幾何理想化(不適合實際狀況)

    % 避免平滑後直線區出現非常小的假曲率
    kappa(abs(kappa) < 1e-8) = 0;

    % =========================================================
    % Segment length
    % =========================================================
    ds_track = [s(1); diff(s)];

    % =========================================================
    % Output
    % =========================================================
    track.x = x(:);
    track.y = y(:);
    track.s = s(:);

    track.ds = ds_track(:);

    track.kappa_raw = kappa_raw(:);
    track.kappa = kappa(:);

    track.type = type;
    track.section = section;

    track.n = n;

end

%% 幾何處理的子函式 
function [x_all, y_all, s_all, kappa_all, section_all, type_all] = ...
    track_geometry(segments, ds)

    x_all = [];
    y_all = [];
    s_all = [];
    kappa_all = [];
    section_all = [];
    type_all = strings(0);

    x = segments(1).start(1);
    y = segments(1).start(2);
    s = 0;

    for k = 1:length(segments)

        seg = segments(k);
        sec = seg.section;

        if seg.type == "LINE"

            x0 = seg.start(1);
            y0 = seg.start(2);
            x1 = seg.end(1);
            y1 = seg.end(2);

            dx = x1 - x0;
            dy = y1 - y0;

            L = hypot(dx, dy);

            if L == 0
                continue
            end

            ux = dx / L;
            uy = dy / L;

            n = max(1, ceil(L/ds));
            step = L/n;

            for i = 1:n

                x = x + ux * step;
                y = y + uy * step;
                s = s + step;

                x_all(end+1,1) = x;
                y_all(end+1,1) = y;
                s_all(end+1,1) = s;

                kappa_all(end+1,1) = 0;

                section_all(end+1,1) = sec;
                type_all(end+1,1) = "LINE";

            end

        elseif seg.type == "ARC"

            cx = seg.center(1);
            cy = seg.center(2);

            R = seg.radius;

            x0 = seg.start(1);
            y0 = seg.start(2);

            theta = atan2(y0 - cy, x0 - cx);

            if startsWith(lower(seg.direction), "clock")
                signDir = -1;
            else
                signDir = 1;
            end

            dtheta_total = seg.length / R;

            n = max(1, ceil(seg.length/ds));
            dtheta = dtheta_total / n;

            for i = 1:n

                theta = theta + signDir * dtheta;

                x = cx + R*cos(theta);
                y = cy + R*sin(theta);

                ds_local = R * dtheta;
                s = s + ds_local;

                x_all(end+1,1) = x;
                y_all(end+1,1) = y;
                s_all(end+1,1) = s;

                % Signed curvature
                kappa_all(end+1,1) = signDir / R;

                section_all(end+1,1) = sec;

                if signDir > 0
                    type_all(end+1,1) = "LEFT";
                else
                    type_all(end+1,1) = "RIGHT";
                end

            end
        end
    end
end

