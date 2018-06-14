function ind = findind(ts,tl)

ind = zeros(length(ts),1);
for i = 1:length(ts)
    [~,ind(i)] = min(abs(tl-ts(i)));
end