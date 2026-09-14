%% OpenLAP Laptime Simulation Project
%
% OpenTRACK_InputMap
%
% Track model file creation for input_map.xlsx style geometry files.
% This script accepts a Shape sheet built from explicit LINE/ARC geometry:
% section, Type, Start_X, Start_Y, End_X, End_Y, Center_X, Center_Y,
% Radius, length, Direction.
%
% It saves the same track variables used by OpenLAP:
% info, x, dx, n, r, bank, incl, factor_grip, sector, r_apex, apex,
% X, Y, Z, arrow.

%% Clearing memory

clear
clc
close all force
diary('off')
fclose('all') ;

%% Track file selection
addpath("OpenTRACK Tracks")
filename = 'input_map.xlsx' ;

%% Settings

mesh_size = 1 ; % [m]
rotation = 0 ; % [deg]

%% Reading file

disp(['Reading input-map track file: ',filename])

info = read_info_input_map(filename,'Info') ;
segments = read_input_map_shape(filename,'Shape') ;
section_el = read_section_data(filename,'Elevation',0) ;
section_bk = read_section_data(filename,'Banking',0) ;
section_gf = read_section_data(filename,'Grip Factors',1) ;
section_sc = read_section_data(filename,'Sectors',1) ;

%% Track model name

[~,~] = mkdir('OpenTRACK Tracks') ;
trackname = "OpenTRACK Tracks/OpenTRACK_"+info.name+"_"+info.config+"_"+info.direction ;
if strcmp(info.mirror,"On")
    trackname = trackname+"_Mirrored" ;
end

%% HUD

if isfile(trackname+".log")
    delete(trackname+".log") ;
end
diary(trackname+".log") ;
disp('_______                    ____________________________________ __')
disp('__  __ \______________________  __/__  __ \__    |_  ____/__  //_/')
disp('_  / / /__  __ \  _ \_  __ \_  /  __  /_/ /_  /| |  /    __  ,<')
disp('/ /_/ /__  /_/ /  __/  / / /  /   _  _, _/_  ___ / /___  _  /| |')
disp('\____/ _  .___/\___//_/ /_//_/    /_/ |_| /_/  |_\____/  /_/ |_|')
disp('       /_/')
disp('==================================================================')
disp(filename)
disp('File read successfully')
disp('==================================================================')
disp("Name:          "+info.name)
disp("City:          "+info.city)
disp("Country:       "+info.country)
disp("Type:          "+info.type)
disp("Configuration: "+info.config)
disp("Direction:     "+info.direction)
disp("Mirror:        "+info.mirror)
disp("Date:          "+datestr(now,'dd/mm/yyyy'))
disp("Time:          "+datestr(now,'HH:MM:SS'))
disp('==================================================================')
disp('Track generation started.')
disp('==================================================================')
disp("Input format:   input_map geometry")
disp("Shape segments: "+num2str(length(segments)))
disp("Line segments:  "+num2str(sum([segments.type]=="LINE")))
disp("Arc segments:   "+num2str(sum([segments.type]=="ARC")))
disp("Mesh size:      "+num2str(mesh_size)+" [m]")
disp("Rotation:       "+num2str(rotation)+" [deg]")
disp('==================================================================')

%% Fine geometry generation

[x,X,Y,r,section] = build_input_map_geometry(segments,mesh_size) ;

if strcmp(info.mirror,'On')
    Y = -Y ;
    r = -r ;
end

if strcmp(info.direction,'Backward')
    x = x(end)-flipud(x) ;
    X = flipud(X) ;
    Y = flipud(Y) ;
    r = -flipud(r) ;
    section = flipud(section) ;
end

xyz = rotz(rotation)*[X';Y';zeros(1,length(X))] ;
X = xyz(1,:)' ;
Y = xyz(2,:)' ;

L = x(end) ;
n = length(x) ;
dx = diff(x) ;
dx = [dx;dx(end)] ;

disp('Pre-processing completed.')
disp("Track length:   "+num2str(L,'%0.3f')+" [m]")
disp("Mesh points:    "+num2str(n))
disp("Mean dx:        "+num2str(mean(dx),'%0.3f')+" [m]")
disp('Fine meshing completed.')

%% Track attached data

Z = section_lookup(section,section_el,0) ;
bank = section_lookup(section,section_bk,0) ;
factor_grip = section_lookup(section,section_gf,1) ;
sector = section_lookup(section,section_sc,1) ;
sector = round(sector) ;

if strcmp(info.direction,'Backward')
    bank = -bank ;
end

incl = -atand(diff(Z)./diff(x)) ;
incl = [incl;incl(end)] ;

disp('Track attached data completed.')
disp("Elevation min/max: "+num2str(min(Z),'%0.3f')+" / "+num2str(max(Z),'%0.3f')+" [m]")
disp("Banking min/max:   "+num2str(min(bank),'%0.3f')+" / "+num2str(max(bank),'%0.3f')+" [deg]")
disp("Grip min/max:      "+num2str(min(factor_grip),'%0.3f')+" / "+num2str(max(factor_grip),'%0.3f')+" [-]")
disp("Sectors:           "+num2str(max(sector)))

%% Closing correction

if strcmp(info.config,'Closed')
    disp('Closing fine mesh map.')
    DX = x/L*(X(1)-X(end)) ;
    DY = x/L*(Y(1)-Y(end)) ;
    DZ = x/L*(Z(1)-Z(end)) ;
    db = x/L*(bank(1)-bank(end)) ;

    X = X+DX ;
    Y = Y+DY ;
    Z = Z+DZ ;
    bank = bank+db ;

    incl = -atand(diff(Z)./diff(x)) ;
    incl = [incl;(incl(end-1)+incl(1))/2] ;
    disp('Fine mesh map closed.')
end

incl = smooth(incl) ;

%% Apexes

apex = find_input_map_apexes(section,r) ;
if isempty(apex)
    [~,apex] = max(abs(r)) ;
end
r_apex = r(apex) ;
disp('Apex calculation completed.')
disp("Apex count:     "+num2str(length(apex)))

%% Finish line arrow

factor_scale = 25 ;
half_angle = 40 ;
scale = max([max(X)-min(X);max(Y)-min(Y)])/factor_scale ;
if scale == 0
    scale = 1 ;
end
arrow_n = [X(1)-X(2);Y(1)-Y(2);Z(1)-Z(2)]/norm([X(1)-X(2);Y(1)-Y(2);Z(1)-Z(2)]) ;
arrow_1 = scale*rotz(half_angle)*arrow_n+[X(1);Y(1);Z(1)] ;
arrow_c = [X(1);Y(1);Z(1)] ;
arrow_2 = scale*rotz(-half_angle)*arrow_n+[X(1);Y(1);Z(1)] ;
arrow = [[arrow_1(1);arrow_c(1);arrow_2(1)], ...
         [arrow_1(2);arrow_c(2);arrow_2(2)], ...
         [arrow_1(3);arrow_c(3);arrow_2(3)]] ;

%% Plotting Results

set(0,'units','pixels') ;
SS = get(0,'screensize') ;
H = 900-90 ;
W = 900 ;
Xpos = floor((SS(3)-W)/2) ;
Ypos = floor((SS(4)-H)/2) ;
f = figure('Name',filename,'Position',[Xpos,Ypos,W,H]) ;
figtitle = ["OpenTRACK InputMap","Track Name: "+info.name,"Configuration: "+info.config,"Mirror: "+info.mirror,"Date & Time: "+datestr(now,'yyyy/mm/dd')+" "+datestr(now,'HH:MM:SS')] ;
figtitle = strrep(figtitle,"_"," ") ;
sgtitle(figtitle)

rows = 5 ;
cols = 2 ;

subplot(rows,cols,[1,3,5,7,9])
title('3D Map')
hold on
grid on
axis equal
axis tight
xlabel('x [m]')
ylabel('y [m]')
scatter3(X,Y,Z,20,sector,'.')
plot3(arrow(:,1),arrow(:,2),arrow(:,3),'k','LineWidth',2)

subplot(rows,cols,2)
title('Curvature')
hold on
grid on
xlabel('position [m]')
ylabel('curvature [m^-^1]')
plot(x,r)
scatter(x(apex),r_apex,'.')
xlim([x(1),x(end)])
legend({'curvature','apex'})

subplot(rows,cols,4)
title('Elevation')
hold on
grid on
xlabel('position [m]')
ylabel('elevation [m]')
plot(x,Z)
xlim([x(1),x(end)])

subplot(rows,cols,6)
title('Inclination')
hold on
grid on
xlabel('position [m]')
ylabel('inclination [deg]')
plot(x,incl)
xlim([x(1),x(end)])

subplot(rows,cols,8)
title('Banking')
hold on
grid on
xlabel('position [m]')
ylabel('banking [deg]')
plot(x,bank)
xlim([x(1),x(end)])

subplot(rows,cols,10)
title('Grip Factor')
hold on
grid on
xlabel('position [m]')
ylabel('grip factor [-]')
plot(x,factor_grip)
xlim([x(1),x(end)])

savefig(f,trackname+".fig")
disp('Plots created and saved.')

%% Saving circuit

save(trackname+".mat",'info','x','dx','n','r','bank','incl','factor_grip','sector','r_apex','apex','X','Y','Z','arrow')
disp('Track generated successfully.')
disp("Saved MAT:      "+trackname+".mat")
disp("Saved FIG:      "+trackname+".fig")
disp("Saved LOG:      "+trackname+".log")

%% ASCII map

disp_ascii_map(X,Y) ;

diary('off') ;

%% Functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function info = read_info_input_map(workbookFile,sheetName)
    if nargin < 2
        sheetName = 'Info' ;
    end

    defaults = ["InputMap";"Unknown";"Unknown";"Temporary";"Closed";"Forward";"Off"] ;

    try
        opts = spreadsheetImportOptions("NumVariables",2) ;
        opts.Sheet = sheetName ;
        opts.DataRange = "A1:B7" ;
        opts.VariableNames = ["info","data"] ;
        opts.VariableTypes = ["string","string"] ;
        tbl = readtable(workbookFile,opts,"UseExcel",false) ;
        data = string(tbl.data) ;
        data(ismissing(data) | data=="") = defaults(ismissing(data) | data=="") ;
    catch
        data = defaults ;
    end

    info.name = data(1) ;
    info.country = data(2) ;
    info.city = data(3) ;
    info.type = data(4) ;
    info.config = data(5) ;
    info.direction = data(6) ;
    info.mirror = data(7) ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function segments = read_input_map_shape(workbookFile,sheetName)
    T = readtable(workbookFile,"Sheet",sheetName,'VariableNamingRule','preserve') ;
    segments = struct([]) ;

    for i = 1:height(T)
        seg_type = strtrim(upper(string(T.Type{i}))) ;
        if ismissing(seg_type) || seg_type == ""
            continue
        end

        seg = struct() ;
        seg.section = read_table_value(T,i,"section",i) ;
        seg.type = seg_type ;
        seg.start = [read_table_value(T,i,"Start_X",0), read_table_value(T,i,"Start_Y",0)] ;
        seg.end = [read_table_value(T,i,"End_X",0), read_table_value(T,i,"End_Y",0)] ;
        seg.center = [read_table_value(T,i,"Center_X",NaN), read_table_value(T,i,"Center_Y",NaN)] ;
        seg.radius = read_table_value(T,i,"Radius",NaN) ;
        seg.length = read_table_value(T,i,"length",NaN) ;
        seg.direction = strtrim(string(T.Direction{i})) ;

        if isnan(seg.length)
            if seg.type == "LINE"
                seg.length = hypot(seg.end(1)-seg.start(1),seg.end(2)-seg.start(2)) ;
            else
                error("Missing length for ARC at row %d.",i)
            end
        end

        segments = [segments;seg] ; %#ok<AGROW>
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function value = read_table_value(T,row,name,defaultValue)
    if any(strcmp(string(T.Properties.VariableNames),name))
        value = double(T.(name)(row)) ;
        if isnan(value)
            value = defaultValue ;
        end
    else
        value = defaultValue ;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function section_data = read_section_data(workbookFile,sheetName,defaultValue)
    try
        T = readtable(workbookFile,"Sheet",sheetName,'VariableNamingRule','preserve') ;
        section_data = table2array(T(:,1:2)) ;
        section_data = section_data(~any(isnan(section_data),2),:) ;
    catch
        section_data = [1,defaultValue] ;
    end

    if isempty(section_data)
        section_data = [1,defaultValue] ;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_all,X_all,Y_all,r_all,section_all] = build_input_map_geometry(segments,mesh_size)
    x_all = 0 ;
    X_all = segments(1).start(1) ;
    Y_all = segments(1).start(2) ;
    r_all = 0 ;
    section_all = segments(1).section ;

    s = 0 ;

    for k = 1:length(segments)
        seg = segments(k) ;

        if seg.type == "LINE"
            L = hypot(seg.end(1)-seg.start(1),seg.end(2)-seg.start(2)) ;
            if L == 0
                continue
            end

            n_step = max(1,ceil(L/mesh_size)) ;
            for i = 1:n_step
                q = i/n_step ;
                X = seg.start(1)+q*(seg.end(1)-seg.start(1)) ;
                Y = seg.start(2)+q*(seg.end(2)-seg.start(2)) ;
                s = s+L/n_step ;

                x_all(end+1,1) = s ; %#ok<AGROW>
                X_all(end+1,1) = X ; %#ok<AGROW>
                Y_all(end+1,1) = Y ; %#ok<AGROW>
                r_all(end+1,1) = 0 ; %#ok<AGROW>
                section_all(end+1,1) = seg.section ; %#ok<AGROW>
            end

        elseif seg.type == "ARC"
            if isnan(seg.radius) || seg.radius <= 0
                error("Invalid radius for ARC section %d.",seg.section)
            end

            theta = atan2(seg.start(2)-seg.center(2),seg.start(1)-seg.center(1)) ;
            if startsWith(lower(seg.direction),"clock")
                sign_dir = -1 ;
            else
                sign_dir = 1 ;
            end

            n_step = max(1,ceil(seg.length/mesh_size)) ;
            dtheta = (seg.length/seg.radius)/n_step ;

            for i = 1:n_step
                theta = theta+sign_dir*dtheta ;
                X = seg.center(1)+seg.radius*cos(theta) ;
                Y = seg.center(2)+seg.radius*sin(theta) ;
                s = s+seg.length/n_step ;

                x_all(end+1,1) = s ; %#ok<AGROW>
                X_all(end+1,1) = X ; %#ok<AGROW>
                Y_all(end+1,1) = Y ; %#ok<AGROW>
                r_all(end+1,1) = sign_dir/seg.radius ; %#ok<AGROW>
                section_all(end+1,1) = seg.section ; %#ok<AGROW>
            end
        else
            warning("Skipping unknown segment type in section %d: %s",seg.section,seg.type)
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function values = section_lookup(section,section_data,defaultValue)
    values = defaultValue*ones(size(section)) ;

    for i = 1:length(section)
        idx = find(section_data(:,1)==section(i),1,'last') ;
        if isempty(idx)
            idx = find(section_data(:,1)<=section(i),1,'last') ;
        end
        if ~isempty(idx)
            values(i) = section_data(idx,2) ;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function apex = find_input_map_apexes(section,r)
    apex = [] ;
    sections = unique(section,'stable') ;

    for i = 1:length(sections)
        idx = find(section==sections(i)) ;
        if isempty(idx)
            continue
        end
        if max(abs(r(idx))) > 0
            apex(end+1,1) = idx(round((length(idx)+1)/2)) ; %#ok<AGROW>
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function disp_ascii_map(X,Y)
    charh = 15 ;
    charw = 8 ;
    linew = 66 ;
    mapw = max(X)-min(X) ;

    if mapw == 0
        disp('Map:')
        disp('o')
        return
    end

    YY = round(Y/(charh/charw)/mapw*linew) ;
    XX = round(X/mapw*linew) ;
    YY = -YY-min(-YY) ;
    XX = XX-min(XX) ;

    p = unique([XX,YY],'rows') ;
    XX = p(:,1)+1 ;
    YY = p(:,2)+1 ;

    maph = max(YY) ;
    mapw = max(XX) ;
    map = char(ones(maph,mapw)*' ') ;

    for i = 1:maph
        for j = 1:mapw
            check = [XX,YY]==[j,i] ;
            check = check(:,1).*check(:,2) ;
            if max(check)
                map(i,j) = 'o' ;
            end
        end
    end

    disp('Map:')
    disp(map)
end
