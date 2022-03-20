function pontoseficienciahelice = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  PONTOSEFICIENCIAHELICE = IMPORTFILE(FILENAME) reads data from text
%  file FILENAME for the default selection.  Returns the data as a table.
%
%  PONTOSEFICIENCIAHELICE = IMPORTFILE(FILE, DATALINES) reads data for
%  the specified row interval(s) of text file FILENAME. Specify
%  DATALINES as a positive scalar integer or a N-by-2 array of positive
%  scalar integers for dis-contiguous row intervals.
%
%  Example:
%  pontoseficienciahelice = importfile("C:\Users\daanr\Documents\Faculdade\2022_1\Projetos 2\pontos_eficiencia_helice.csv", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 14-Feb-2022 14:20:48

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ";";

% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["VarName1", "VarName2"], "TrimNonNumeric", true);
opts = setvaropts(opts, ["VarName1", "VarName2"], "ThousandsSeparator", ",");

% Import the data
pontoseficienciahelice = readtable(filename, opts);

end