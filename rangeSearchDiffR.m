function [srcIdx] = rangeSearchDiffR(srcData,grid,specPoint,zoomVar)
%RANGESEARCHDIFFR 变化半径的rangesearch
%   此处显示详细说明


C=srcData;
dN=length(C);
id=(1:dN);
location =specPoint.Location;
distance_multi = location.^2*ones(size(C)) + ones(specPoint.N,2)*(C).^2-2*location*C;
full_indi = distance_multi < (grid*specPoint.ScaleRadius/zoomVar).^2;
for i=1:specPoint.N
    srcIdx{i}=id(full_indi(i,:));
end
srcIdx=srcIdx';

end

