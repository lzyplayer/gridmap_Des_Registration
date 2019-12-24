function [T,match_srcSeedset,match_tarSeedset] = eigMatch2D_noPropogation(srcDesp,tarDesp,srcScale,tarScale,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep,srcMap,tarMap,s)
%% parameter configuration for flann search
params.algorithm = 'kdtree';
params.trees = 8;
params.checks = 64;
radii = (0.5:0.5:3)*gridStep;
% srcSeed3d=[srcSeed;zeros(1,size(srcSeed,2))];
% tarSeed3d=[tarSeed;zeros(1,size(tarSeed,2))];
[srcIdx,dist] = flann_search(srcDesp,tarDesp,1,params); % match with descriptors ����ֵ,��src��tar����ĵ�?
[dist,id]= sort(dist);
%% aggregating each pair of correspondence for finding the best match
M = size(srcSeed,2);    %source���ӵ�����
N = size(tarSeed,2);    %target���ӵ�����
seedIdx = srcIdx; 
Err = inf(N,1);
tform = cell(1,N); 
ovNum = ceil(overlap*N);   %���ܹ��е���������Ŀ
distThr = 0.2/4*length(radii); 
thetaThr = 10; 
threshold = gridStep*gridStep;

matchpairs={};
%��ÿһ��ƥ�����һ��ѭ�������һ�����ű任
%% proceed with RANSAC directly
src_seed_select = srcSeed(:,srcIdx(:,1:floor(N/2)));
tar_seed_select = tarSeed(:,1:floor(N/2));
[T2d,match_srcSeedset,match_tarSeedset] = estimateGeometricTransform(src_seed_select',tar_seed_select','similarity');

T =T2d.T';

if(isempty(T))
    error(['match Failed with tarseed:' num2str(length(tarSeed)) ' srcSeed:' num2str(length(srcSeed)) ]);
end
end
