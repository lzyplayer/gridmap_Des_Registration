function   showPoint(line)
%SHOWPOINT 输入2*n 描画各个点
%   此处显示详细说明
hold on
if(size(line,1)==2)
    line=line';
end
plot(line(:,1),line(:,2),'.','MarkerSize',10,'Color','r')
end

