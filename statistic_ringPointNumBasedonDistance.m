function [statis_result] = statistic_ringPointNumBasedonDistance(distance,gridsize,max_gridscale)
%STATISTIC_RINGPOINTNUMBASEDONDISTANCE 此处显示有关此函数的摘要
%   output:
%       [1 x (max_gridscale/gridsize -1)]
%       n = max_gridscale / gridsize
%       size = n - 1
n = floor(max_gridscale / gridsize );
distance(end+1)=distance(end)+1; %add tail to fulfill distance
N = length(distance);
i = 1;
j = 1;
count=1;
statis_result=[];
while (i<=N && j<=n)
    if(distance(i)>j*gridsize)
        statis_result(count)=i-1;
        count=count+1;
        j=j+gridsize;
    else
        i=i+1;
    end
end
end

