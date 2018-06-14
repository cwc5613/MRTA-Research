function [neighbors]  = nodeNeighbors(A, robotIndex)
%function takes as inputs an adjacency matrix and robot index and
%outputs indices of robots that belong to the neighbors set
n = length(A);
%determine neighbors
neighbors = [];
for j = 1:n
    if A(robotIndex, j) == 1
        neighbors = [neighbors j];
    end
end
end