function draw_circle(center,radius,axes_curr)
%DRAW_CIRCLE �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if nargin == 2
    axes_curr = gca;
end
hold on;
for i=1:length(radius)
    pos = [center(i,:)-radius(i) 2*radius(i) 2*radius(i)];
    rectangle(axes_curr,'Position',pos,'Curvature',[1,1],'EdgeColor','g');
end
end

