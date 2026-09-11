%% Reproduce Figures 4-9 of the manuscript
% MAIN ENTRY POINT
% Run this file from MATLAB. No other file needs to be run manually.
%
% If the precomputed MAT files are available in ./results, they are loaded
% directly. If either MAT file is missing, the corresponding simulation is
% executed first and the result file is recreated automatically.

clear; clc; close all;

rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));
cd(rootDir);

resultsDir = fullfile(rootDir, 'results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

fixedFile = fullfile(resultsDir, 'Result_MPC_Fixed_Disturbed.mat');
adaptiveFile = fullfile(resultsDir, 'Result_DDPG_MPC_Disturbed.mat');

% Recreate missing result files from the simulation code.
if ~isfile(fixedFile)
    result = run_simulation_case(false);
    save_case_result(result, fixedFile);
end

if ~isfile(adaptiveFile)
    result = run_simulation_case(true);
    save_case_result(result, adaptiveFile);
end

% Generate only Figures 4-9 used in the manuscript.
generate_paper_figures(fixedFile, adaptiveFile);
