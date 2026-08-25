function tir_params = load_tir(filepath) % 載入tir
     
    % 初始化 struct
    tir_params = struct();
    
    % 開啟檔案
    fid = fopen(filepath, 'r', 'n', 'UTF-8');
    if fid == -1
        error('無法開啟檔案: %s', filepath);
    end
    
    % 逐行讀取
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        
        % 忽略空行
        if isempty(line)
            continue;
        end
        
        % 忽略註解
        if startsWith(line, '$')
            continue;
        end
        
        % 忽略 section header
        if startsWith(line, '[') && endsWith(line, ']')
            continue;
        end
        
        % 移除行內註解
        dollarPos = strfind(line, '$');
        if ~isempty(dollarPos)
            line = strtrim(line(1:dollarPos(1)-1));
        end
        
        % 必須包含 '='
        eqPos = strfind(line, '=');
        if isempty(eqPos)
            continue;
        end
        
        % 分割 key 與 value
        key = strtrim(line(1:eqPos(1)-1));
        valueStr = strtrim(line(eqPos(1)+1:end));
        
        % 嘗試轉換數值
        value = parse_value(valueStr);
        
        % 存入 struct
        % 注意：MATLAB struct 的欄位名稱必須是合法識別字
        % 如果 key 有非法字元，可以用 matlab.lang.makeValidName
        key = matlab.lang.makeValidName(key);
        tir_params.(key) = value;
    end
    
    fclose(fid);
end

function val = parse_value(valueStr) % 解析數值
    % 嘗試把字串轉成數值，失敗則保留字串
    num = str2double(valueStr);
    if ~isnan(num)
        val = num;
    else
        val = valueStr;
    end
end
