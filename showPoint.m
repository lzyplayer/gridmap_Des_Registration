function   showPoint(line,map_flag,markersize)
%SHOWPOINT ����2*n �軭������
%   �˴���ʾ��ϸ˵��
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

