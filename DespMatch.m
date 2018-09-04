function [pairInfoSorted] = DespMatch(srcDesp,tarDesp)%,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep
%DESPMATCH 此处显示有关此函数的摘要
%   此处显示详细说明
params.algorithm = 'kdtree';
params.trees = 8;
params.checks = 64;
% radii = (0.5:0.5:2)*gridStep;
[srcIdx,dist] = flann_search(srcDesp,tarDesp,1,params); % match with descriptors 特征值
pairInfo = [srcIdx',(1:size(tarDesp,2))',dist'];
pairInfoSorted = sortrows(pairInfo,3);

end

