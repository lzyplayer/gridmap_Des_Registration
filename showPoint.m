function   showPoint(line,map_flag,markersize)
%SHOWPOINT 输入2*n 描画各个点
%   此处显示详细说明
hold on
if(size(line,1)==2)
    line=line';
end
if(nargin==2)
    plot(line(:,1),line(:,2),'.','Color','k');
else
    if (nargin==3)
        plot(line(:,1),line(:,2),'.','MarkerSize',markersize,'Color',[0.9 0 0]);
    else
        plot(line(:,1),line(:,2),'.','MarkerSize',20,'Color','r');
    end
end

