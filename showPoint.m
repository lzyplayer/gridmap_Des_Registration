function   showPoint(line)
%SHOWPOINT ����2*n �軭������
%   �˴���ʾ��ϸ˵��
hold on
if(size(line,1)==2)
    line=line';
end
plot(line(:,1),line(:,2),'.','MarkerSize',10,'Color','r')
end

