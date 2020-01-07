function [MSE] = MSE_calcu(Model,Data)
%MSE_CALCU 此处显示有关此函数的摘要
%   此处显示详细说明
%% reshape
if(size(Model,1)==2)
    Model=Model';
end
if(size(Data,1)==2)
    Data=Data';
end
%%

Mns= createns(Model);
[corrB,TDB] = knnsearch(Mns,Data);
MSE = sum(TDB)/length(Data);
end

