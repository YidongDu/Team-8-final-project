function [timestamps] = readtimestamps(filename)
    fileID = fopen(filename,'r');
    timestamps = [];
    while(~feof(fileID))
        day = fscanf(fileID,'%s',1);
        time = fscanf(fileID,'%s',1);
        t = datenum(time);
        timestamps = [timestamps;t];
    end
end