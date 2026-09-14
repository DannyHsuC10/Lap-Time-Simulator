classdef tir_model
    properties
        tire_data
    end
    
    methods
        function obj = tir_model(car)
            addpath("MF62_tir_model/Pacejka_matlab_model")
            try
                load TireDB.mat TireDB
                disp("load TireDB.mat TireDB")
            catch
                tir = load_tir("FSAE_43075R20_W8.tir");
                TireDB = build_tire_database(car.camber,car.P_tir,tir);
            end
            obj.tire_data = TireDB;
            disp("tire loading complete")
        end
        
        % ===== 算扭矩 =====
        function Fx = lim_output(obj,Fz,Fy,direction)

            if direction == "A"
                Fx = interp2( ...
                    obj.tire_data.Fy_grid,...
                    obj.tire_data.Fz_grid,...
                    obj.tire_data.Fx_max,...
                    Fy,Fz,...
                    'linear');
            elseif direction == "B"
                Fx = interp2( ...
                    obj.tire_data.Fy_grid,...
                    obj.tire_data.Fz_grid,...
                    obj.tire_data.Fx_min,...
                    Fy,Fz,...
                    'linear');
            else
                disp("direction[A:acceleration,B:brake]")
            end
        end
        %%
% ===== 方法 2：摩擦橢圓法（修正複數與越界問題） =====
        function Fx = lim_output_ellipse(obj,Fz,Fy,direction)
            output_size = size(Fz);
            Fz = Fz(:);
            Fy = Fy(:);

            % 1. 安全防護：強制轉換輸入為實數，避免上一層模型傳入複數
            Fz = real(Fz);
            Fy = real(Fy);
            
            % 2. 防止 Fz 超出查表網格範圍（ extrapolation 限制）
            max_Fz = max(obj.tire_data.Fz_grid(:));
            min_Fz = min(obj.tire_data.Fz_grid(:));
            Fz = max(min(Fz, max_Fz), min_Fz);
            
            % 3. 獲取純縱向極限 (Fy = 0)
            if direction == "A"
                Fx_pure_max = interp2(obj.tire_data.Fy_grid, obj.tire_data.Fz_grid, obj.tire_data.Fx_max, zeros(size(Fz)), Fz, 'linear');
            elseif direction == "B"
                Fx_pure_max = interp2(obj.tire_data.Fy_grid, obj.tire_data.Fz_grid, obj.tire_data.Fx_min, zeros(size(Fz)), Fz, 'linear');
            else
                disp("direction[A:acceleration,B:brake]");
                Fx = zeros(output_size);
                return;
            end
            
            % 4. 獲取純側向極限 (Fx = 0 狀況下的最大 Fy)
            if isfield(obj.tire_data, 'Fy_max_pure')
                Fy_pure_max = interp1(obj.tire_data.Fz_grid_pure, obj.tire_data.Fy_max_pure, Fz, 'linear');
            else
                Fy_pure_max = max(abs(real(obj.tire_data.Fy_grid(:)))); 
            end
            
            % 確保極限值為實數且不為零（避免除以零）
            Fx_pure_max = real(Fx_pure_max);
            Fy_pure_max = max(real(Fy_pure_max), 1e-3); 

            % 5. 實作摩擦橢圓公式
            ellipse_ratio = sqrt(max(0, 1 - (Fy ./ Fy_pure_max).^2));
            Fx = Fx_pure_max .* ellipse_ratio;
            Fx = reshape(Fx, output_size);
        end

    end
end
