function make_tex_table(metricdata_cell, metric_cell, setts)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% WRITE PERFORMANCE METRIC DATA TO TEXT FILE IN LATEX SYNTAX FOR LATEX
% TABLE
%
% Brent Wallace  
%
% 2022-10-21
%
% This program, given an array of raw performance metric data, tabulates
% the data numerically and writes it to a text file so that the data can be
% copy/pasted into a LaTEX document for use in a table.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% setts             (Struct) Algorithm settings. Has the following fields:
%   relpath_data    (String) Relative path to write the LaTEX table to
%   filename_data   (String) File name to write the LaTEX table to
%   donewline       (Bool) Insert a LaTEX new line marker '\\' at the end
%                   of each row (=1) or don't insert (=0)
%   dorowpad        (Bool - OPTIONAL) Insert extra custom
%                   padding/formatting terms between each row of the LaTEX
%                   table (=1). If not desired to insert anything between
%                   between rows, leave this field blank. If it is desired,
%                   the following field must be filled out:
%   rowpad_cell     (N x 1 Cell Array - OPTIONAL) Each entry contains the
%                   string to insert in the respective line between each
%                   row of the table, where N is the number of lines of
%                   material desired to insert. Example: If one wanted to
%                   insert a horizontal line in between each row, this
%                   could be
%                       rowpad_cell = {'\hline'}
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% metricdata_cell   ('nummetric' x 'numpresets' Cell Array) Contains the
%                   formatted numerical performance data to be inserted in
%                   the LaTEX table. See 'make_aircraft_textable.m' for
%                   more info.
% metric_cell       ('nummetric' x 1 Cell Array) Each entry is a struct
%                   corresponding to the respective metric data in the row.
%                   Each entry is a struct with the following entries:
%   varname         (String) Programmatic variable name of the metric in
%                   the 'perf_metric_cell' struct.
%   format          (String) Format of how to print the numerical data to
%                   text
%   texname         (String) LaTEX table row name of this metric  
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN  
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
%
% INIT   
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% EXTRACT PROGRAM SETTINGS   
%
% *************************************************************************

% Relative path to performance metric data
relpath_data = setts.relpath_data;

% File name of performance metric data
filename_data = setts.filename_data;

% Insert a LaTEX new line marker '\\' at the end of each row (=1) or don't
% insert (=0)
donewline = setts.donewline;

% Check if latex format padding is to be added between successive rows of
% data
dorowpad = isfield(setts, 'rowpad_cell');

% If row padding is to be performed, extract the cell array of padding
% entries
if dorowpad
    rowpad_cell = setts.rowpad_cell;
    numpadlines = size(rowpad_cell,1);
end

% Get number of presets
numpresets = size(metricdata_cell,2);

% Get number of metrics
nummetric = size(metric_cell,1);


% *************************************************************************
%
% FORMATTING
%
% *************************************************************************

% One tab string
onetab = '    ';

% Column separator
colsep = '  &  ';

% LaTEX newline command
texnewline = '\\';

%.txt file newline
txtnewline = '\n';



%%
% *************************************************************************
% *************************************************************************
%
% WRITE PERFORMANCE METRIC DATA    
%
% *************************************************************************
% *************************************************************************

% Create directory to write data to
mkdir(relpath_data);

% Create text doc with write permissions
fileID = fopen([relpath_data filename_data],'w');

% ***********************
%       
% MAIN LOOP
%

for i = 1:nummetric

    % Get the current row of performance metric data
    currmetric_row = metricdata_cell(i,:);

    % Get the text name of the current metric
    currmetric_name = metric_cell{i}.texname;

    % Begin the current line to print
    currline = currmetric_name;

    % Write each preset's data to the current line
    for presetcount = 1:numpresets
        currmetricdatastr = currmetric_row{presetcount};
        currline = [currline colsep currmetricdatastr];
    end

    % Finish off current row with a LaTEX newline character if desired
    if donewline
        currline = [currline onetab texnewline];
    end

    % Make string .txt-compatible
    currline = maketxtcptble(currline);

    % Add a newline character for the text doc
    currline = [currline txtnewline];

    % Write the current row of data to the text doc
    fprintf(fileID,currline);

    % Write any padding rows to the text doc before starting the next row
    % of data
    if dorowpad && i < nummetric
        for j = 1:numpadlines
            currpadline = rowpad_cell{j};
            currpadline = maketxtcptble(currpadline);
            currpadline = [currpadline txtnewline];
            fprintf(fileID,currpadline);
        end
    end

end

% Close text doc
fclose(fileID);


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN    
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% FUNCTION: REPLACE ILLEGAL CHARACTERS IN fprintf.m
%
% See:
%
%   https://www.mathworks.com/help/matlab/ref/fprintf.html
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

function strout = maketxtcptble(strin)

% .txt backslash character '\'
txtbckslsh = '\\';

% .txt percent character '%'
txtpct = '%%';

% Replace any backslashes '\' with their .txt-compatible '\\' counterpart 
strout = strrep(strin,'\',txtbckslsh);

% Replace any percent signs '%' with their .txt-compatible '%%' counterpart 
strout = strrep(strout,'%',txtpct);

