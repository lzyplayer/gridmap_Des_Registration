function [MSE] = MSE_calcu(Model,Data)
%MSE_CALCU �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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

