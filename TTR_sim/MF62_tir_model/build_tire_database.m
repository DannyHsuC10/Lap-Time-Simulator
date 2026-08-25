function TireDB = build_tire_database(IA, P, tir) % 資料庫建立
    % ==========================
    % Database Grid
    % ==========================
    Fz_grid = 100:25:2000;
    Fy_grid = -3000:25:3000;
    
    nFz = length(Fz_grid);
    nFy = length(Fy_grid);
    
    % ==========================
    % Allocate Memory
    % ==========================
    Fx_max = nan(nFz, nFy);
    Fx_min = nan(nFz, nFy);
    
    % ==========================
    % Loop Fz
    % ==========================
    for i = 1:nFz
        FZ = Fz_grid(i);
        clc;
        fprintf('Building... %.1f%%\n', (i/nFz)*100)
        
        % 1. 取得摩擦圓外殼
        Envelope = Friction_Ellipse(FZ, IA, P, tir);
        
        % 2. 建立查表結構 (此處已修正排序與唯一性)
        Lookup = build_Fx_lookup(Envelope);
        
        % ----------------------
        % Query all Fy points
        % ----------------------
        for j = 1:nFy
            Fy = Fy_grid(j);
            
            % 透過 tire_lookup 查表，若超出物理極限會安全回傳 0 
            Fx_max(i, j) = tire_lookup(Fy, Lookup, "max");
            Fx_min(i, j) = tire_lookup(Fy, Lookup, "min");
        end
    end
    
    % ==========================
    % Store
    % ==========================
    TireDB.Fz_grid = Fz_grid;
    TireDB.Fy_grid = Fy_grid;
    TireDB.Fx_max = Fx_max;
    TireDB.Fx_min = Fx_min;
    
    current_folder = fileparts(mfilename('fullpath'));
    save_file = fullfile(current_folder, 'TireDB.mat');
    
    save(save_file, 'TireDB')
    disp("Tire database generated successfully!!")
end

%% 摩擦圓 (保持不變，你的 convhull 寫得很好)
function Envelope = Friction_Ellipse(FZ, IA, P, tir)
    sa_vec = linspace(-10, 10, 100); 
    sl_vec = linspace(-1, 1, 100); 
    [SA, SL] = meshgrid(sa_vec, sl_vec);
    FX = zeros(size(SA));
    FY = zeros(size(SA));
    for i = 1:numel(SA)
        FY(i) = MF_api("FY", FZ, SA(i), SL(i), IA, P, tir);
        FX(i) = MF_api("FX", FZ, SA(i), SL(i), IA, P, tir);
    end
    K = convhull(FX(:), FY(:));
    K = K(1:end-1);
    Envelope.FX = FX(K);
    Envelope.FY = FY(K);
end

%% 修正後的查表優化（解決重疊點與嚴格單調遞增問題）
function Lookup = build_Fx_lookup(Envelope)
    % =========================
    % Drive side (Fx >= 0)
    % =========================
    idx_drive = Envelope.FX >= 0;
    FY_raw_d = Envelope.FY(idx_drive);
    FX_raw_d = Envelope.FX(idx_drive);
    
    % 關鍵：使用 unique 去除可能重複的 Fy，並自動從小到大排序
    % 'stable' 或不加，這裡我們需要確保 Fy 唯一
    [FY_drive_uniq, u_idx] = unique(FY_raw_d);
    Lookup.FY_drive = FY_drive_uniq;
    Lookup.FX_drive = FX_raw_d(u_idx);
    
    % 記錄該 Fz 下加速邊界的 Fy 容許範圍
    Lookup.Fy_max_d = max(FY_drive_uniq);
    Lookup.Fy_min_d = min(FY_drive_uniq);

    % =========================
    % Brake side (Fx <= 0)
    % =========================
    idx_brake = Envelope.FX <= 0;
    FY_raw_b = Envelope.FY(idx_brake);
    FX_raw_b = Envelope.FX(idx_brake);
    
    [FY_brake_uniq, u_idx] = unique(FY_raw_b);
    Lookup.FY_brake = FY_brake_uniq;
    Lookup.FX_brake = FX_raw_b(u_idx);
    
    % 記錄該 Fz 下煞車邊界的 Fy 容許範圍
    Lookup.Fy_max_b = max(FY_brake_uniq);
    Lookup.Fy_min_b = min(FY_brake_uniq);
end

%% 修正後的反查機制（移除 extrap，超出物理邊界硬性截斷為 0）
function Fx_val = tire_lookup(Fy_target, Lookup, mode)
    switch lower(mode)
        case "max"
            % 檢查是否在物理極限內，超出極限直接歸零
            if Fy_target >= Lookup.Fy_min_d && Fy_target <= Lookup.Fy_max_d
                Fx_val = interp1(Lookup.FY_drive, Lookup.FX_drive, Fy_target, 'pchip');
            else
                Fx_val = 0; % 超出極限，輪胎無法提供力量
            end
            
        case "min"
            % 檢查是否在物理極限內，超出極限直接歸零
            if Fy_target >= Lookup.Fy_min_b && Fy_target <= Lookup.Fy_max_b
                Fx_val = interp1(Lookup.FY_brake, Lookup.FX_brake, Fy_target, 'pchip');
            else
                Fx_val = 0;
            end
            
        otherwise
            error("mode must be 'max' or 'min'")
    end
end