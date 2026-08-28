function segments = load_segments(filename, sheet_name)

    if nargin < 2
        sheet_name = "Shape";
    end

    % =========================
    % 讀取 Excel
    % =========================
    T = readtable(filename, "Sheet", sheet_name,'VariableNamingRule', 'preserve');

    segments = struct([]);

    % =========================
    % loop rows
    % =========================
    for i = 1:height(T)

        % =========================
        % Type 清理
        % =========================
        raw_type = T.Type{i};

        % 空白 / NaN / missing -> skip
        seg_type = strtrim(upper(string(raw_type)));
        
        if strlength(seg_type) == 0 || ismissing(seg_type)
            continue;
        end
        seg_type = strtrim(upper(string(raw_type)));

        % 空字串 -> skip
        if seg_type == ""
            continue;
        end

        % =========================
        % 建立 segment
        % =========================
        seg = struct();

        seg.type   = seg_type;
        seg.length = double(T.length(i));

        seg.start = [
            double(T.Start_X(i)), ...
            double(T.Start_Y(i))
        ];

        seg.end = [
            double(T.End_X(i)), ...
            double(T.End_Y(i))
        ];

        % 預設
        seg.center    = [];
        seg.radius    = [];
        seg.direction = "";
        seg.kappa     = 0.0;
        seg.section = i;% 區段計數
        % =====================================================
        % LINE
        % =====================================================
        if seg.type == "LINE"

            seg.kappa = 0.0;

        % =====================================================
        % ARC
        % =====================================================
        elseif seg.type == "ARC"

            cx = double(T.Center_X(i));
            cy = double(T.Center_Y(i));

            r = double(T.Radius(i));

            direction = strtrim(string(T.Direction{i}));

            seg.center    = [cx, cy];
            seg.radius    = r;
            seg.direction = direction;

            if startsWith(lower(direction), "clock")
                sign = -1;
            else
                sign = +1;
            end

            seg.kappa = sign / r;

        % =====================================================
        % Unknown
        % =====================================================
        else

            warning("Skip unknown segment type at row %d : %s", ...
                i, seg.type);

            continue;

        end

        % append
        segments = [segments; seg];

    end
    disp('Pre-processing completed.')

end