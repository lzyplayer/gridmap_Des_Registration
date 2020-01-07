function [srcIdx] = rangeSearchDiffR(srcData,grid,specPoint,zoomVar)
%RANGESEARCHDIFFR �仯�뾶��rangesearch
%   �˴���ʾ��ϸ˵��


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

