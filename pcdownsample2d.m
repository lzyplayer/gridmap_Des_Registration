function [pointMap,pcToProcess] = pcdownsample2d(pointMap2d,gridStep)
%PCDOWNSAMPLE2D ��������ά��
%   �˴���ʾ��ϸ˵��
pNum = length(pointMap2d);
if(pNum > 2 && size(pointMap2d,1)==2)
    pointMap2d = pointMap2d';
end
pcToProcess = pcdownsample(pointCloud([pointMap2d,zeros(pNum,1)]),'gridAverage',gridStep);
pointMap = pcToProcess.Location(:,1:2);
end

