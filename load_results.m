function [MPC, adaptive] = load_results(fixedFile, adaptiveFile)
%LOAD_RESULTS Load the two result structures used in Figures 4-9.

S = load(fixedFile, 'result');
MPC = S.result;
S = load(adaptiveFile, 'result');
adaptive = S.result;
end
