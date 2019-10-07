function [range_result] = generate_analyse_data(M,D,detector_step)
%GENERATE_ANALYSE_DATA 此处显示有关此函数的摘要
%   此处显示详细说明
%% data format
if(size(M,1)<size(M,2))
    M=M';
end
if(size(D,2)~=size(M,2))
    D=D';
    if (size(D,2)~=size(M,2))
        error('D size not match M !')
    end
end
%% model searcher
kdMdl = KDTreeSearcher(M);
%%
i=1;
range_result=[];
% detector_step=0.01;
detector_max_size = 2;
for r=detector_step:detector_step:detector_max_size
    IDx = rangesearch(kdMdl,D,r);
    mid_search_result = cellfun(@length,IDx,'UniformOutput',true);%/(2*r)
    range_result=[range_result,mid_search_result];
    i=i+1;
end
end

