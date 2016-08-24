function l=prob_to_log_odds(p)
% Convert proability values p to the corresponding log odds l.
% p could be a scalar or a matrix.

% TODO: compute l.
for i=1:size(p,1)
    for j=1:size(p,2)
	l(i,j) = log(p(i,j)/(1-p(i,j)));
    endfor
endfor
end
