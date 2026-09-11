function save_case_result(result, fileName)
%SAVE_CASE_RESULT Save one simulation result using the paper-compatible format.

save(fileName, 'result');
end
