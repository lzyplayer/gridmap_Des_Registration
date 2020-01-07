function [statis_result] = statistic_ringPointNumBasedonDistance_m2(distance,gridsize,max_gridscale)
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
statis_result=[];
while (i<=N && j<=n)
    if(distance(i)>j)
        statis_result(j)=i-1;
        j=j+1;
    else
        i=i+1;
    end
end

end



