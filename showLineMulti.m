function  showLineMulti(start,ends)
%SHOWLINEMULTI �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
hold on;
for i=1:size(start,1)
    line([start(i,1);ends(i,1)],[start(i,2);ends(i,2)],'LineStyle','-','Marker','o');
end
end

