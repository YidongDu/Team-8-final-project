function [deltaPij, deltaVij, deltaRij] = extractfromprint(preint_string)
    p_index = strfind(preint_string,'deltaPij');
    V_index = strfind(preint_string,'deltaVij');
    R_index = strfind(preint_string,'deltaRij');
    C_index = strfind(preint_string,'measurementCovariance');
    %dissect p
    p_string = preint_string(p_index:(V_index-2));
    vec_string = strrep(p_string,'deltaPij [',' ');
    numstring = strrep(vec_string,']',' ');
    numstring = strrep(numstring,newline,' ');
    numchars = char(strsplit(numstring));
    deltaPij = reshape(str2num(numchars), 3, [])';
    %dissect V
    v_string = preint_string(V_index:(R_index-2));
    vec_string = strrep(v_string,'deltaVij [',' ');
    numstring = strrep(vec_string,']',' ');
    numstring = strrep(numstring,newline,' ');
    numchars = char(strsplit(numstring));
    deltaVij = reshape(str2num(numchars), 3, [])';
    %dissect R
    R_string = preint_string(R_index:(C_index-2));
    vec_string = strrep(R_string,'deltaRij [',' ');
    numstring = strrep(vec_string,']',' ');
    numstring = strrep(numstring,newline,' ');
    numchars = char(strsplit(numstring));
    deltaRij = reshape(str2num(numchars), 3, [])';
    
end