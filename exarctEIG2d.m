function [srcDesp,srcSeed,srcNorm] = exarctEIG2d(srcData,gridStep,specPoint)
%This code is the Matlab implimentation of the paper, 
%"Fast Descriptors and Correspondence Propagation for Robust Global Point Cloud Registration,"
%IEEE transactions on Image Processing, 2017.
%This code should be used only for academic research.
%any other useage of this code should not be allowed without Author agreement.
% If you have any problem or improvement idea about this code, please
% contact with Guang JIANG, Xidian University. gjiang@mail.xidian.edu.cn.


radii = (0.5:0.5:2)*gridStep;
srcSeed=specPoint.Location';
srcData=srcData';
%% compute descriptors for seed points in the source point cloud
K = length(radii);
srcIdx = rangesearch(srcData',srcSeed',radii(1));
idxSz = cellfun(@length,srcIdx,'uni',true);
srcIdx = srcIdx(idxSz>10);
srcSeed = srcSeed(:,idxSz>10);
M = sum(idxSz>10);
idx = num2cell((1:M)');
[s,n] = cellfun(@(x,y)svdCov(x,y,srcData,srcSeed),srcIdx,idx,'uni',false);
s = cell2mat(s); %ÿ��specialpoint ������ֵ 2*n   ��� 1*2n
n = cell2mat(n); %ÿ��specialpoint �ĵڶ�������ֵ���������� 2*n ��� 1*2n
for k = 2:K
    srcIdx = rangesearch(srcData',srcSeed',radii(k));
    [sk,nk] = cellfun(@(x,y)svdCov(x,y,srcData,srcSeed),srcIdx,idx,'uni',false);
    s = [s cell2mat(sk)];
    n = [n cell2mat(nk)];
end
s = s';
ds = diff(s);
srcDesp = reshape(ds,2*(K-1),[]);    %�õ�����ֵ��ֵ������ ��ÿ������3������ֵ��ֵ��ÿ���ֵ����������ֵ�� 6ά
n = mat2cell(n,2*ones(M,1),K);
srcNorm = cellfun(@(x)reshape(x,[],1),n','uni',false);
srcNorm = cell2mat(srcNorm);

end
