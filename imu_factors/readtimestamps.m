function [timestamps] = readtimestamps(filename)
    fileID = fopen(filename,'r');
    timestamps = [];
    d2s = 24*3600;
    while(~feof(fileID))
        day = fscanf(fileID,'%s',1);
        time = fscanf(fileID,'%s',1);
        t = d2s*datenum(time);
        timestamps = [timestamps;t];
    end
end