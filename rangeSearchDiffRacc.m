function [srcIdx] = rangeSearchDiffRacc(srcData,grid,specPoint,zoomVar)
%RANGESEARCHDIFFR �仯�뾶��rangesearch
%   �˴���ʾ��ϸ˵��
%  arrayfun(@()(sqrt),specPoint)

createns()


 C=srcData;
 dN=length(C);    
 id=(1:dN);
 srcIdx=cell(1,specPoint.N);
 for i=1:specPoint.N
    P=specPoint.Location(i,:);
    distance = sqrt(P.^2*ones(size(C))+[1,1]*(C).^2-2*P*C);
    srcIdx{i} = id(distance<(grid*specPoint.ScaleRadius(i)/zoomVar));
 end
srcIdx=srcIdx';
end

% sqrt(P.^2*ones(size(C'))+ones(size(P))*(C').^2-2*P*C')