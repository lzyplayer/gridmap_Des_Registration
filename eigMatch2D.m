function T = eigMatch2D(srcDesp,tarDesp,srcScale,tarScale,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep,srcMap,tarMap,s)
%% parameter configuration for flann search
params.algorithm = 'kdtree';
params.trees = 8;
params.checks = 64;
radii = (0.5:0.5:3)*gridStep;
% srcSeed3d=[srcSeed;zeros(1,size(srcSeed,2))];
% tarSeed3d=[tarSeed;zeros(1,size(tarSeed,2))];
[srcIdx,dist] = flann_search(srcDesp,tarDesp,1,params); % match with descriptors ����ֵ,��src��tar����ĵ�
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
for i = 1:ceil(0.2*N) %��ÿһ�Զ�
    n= id(i);
%   for n = 1:N
    seed = srcSeed(:,seedIdx(n));
    seedNorm = srcNorm(:,seedIdx(n));
    seedScale = srcScale(seedIdx(n));
     %%  ��ǰ�������������������������ăȻ�
    % source point cloud
    d = bsxfun(@minus,srcSeed,seed);
    d = sqrt(sum(d.^2,1)); % distance of �����������뵱ǰ��������
    d = d./(seedScale);%�߶Ȼ�ԭ
    inProd = bsxfun(@times,srcNorm,seedNorm);    %��ǰ������ĸ����������������������ĸ����������ڻ�
    inProd = inProd(1:2:end,:) + inProd(2:2:end,:) ;
    theta = real(acosd(inProd));  % inner product

    % target point cloud
    r = bsxfun(@minus,tarSeed,tarSeed(:,n));
    r = sqrt(sum(r.^2,1)); % distance of �����������뵱ǰ��������
    r = r./(tarScale(n));
    inProd = bsxfun(@times,tarNorm,tarNorm(:,n));
    inProd = inProd(1:2:end,:) + inProd(2:2:end,:);
    alpha = real(acosd(inProd));  % inner product   
    
%% r,d �ֱ��ǵ�ǰ��������������������ŷ�Ͼ��룬IDX�����ӽ��Ŀ��ܵ���չ��Զ�
    %�������� rdΪ�Ѿ�ƽ����߶ȵľ�������
    IDX = rangesearch(r',d',gridStep/2,'distance','cityblock');    %cityblock�����پ��룬������һά���ݿ���ֻ��Ϊ�˼ӿ��ٶ�
    
    matches = [seedIdx(n) n];
    for m = [1:seedIdx(n)-1 seedIdx(n)+1:M]     %���˵�ǰ��������     MΪ˳�����еĵ���
        idx = IDX{m};%find(abs(r-d(m))<gridStep/2);%
        if(isempty(idx))
            continue;
        end
        dTheta = bsxfun(@minus,alpha(:,idx),theta(:,m));
        dTheta = abs(dTheta);
        Tab = dTheta<thetaThr;
        Tab = sum(Tab,1);
        if(all(Tab<size(theta,1)))%ֻ����ÿ��ά�����������϶��ӽ��Ĳ�����չ��ԣ�����һ���ӽ���Ծͼ���ִ��
            continue;
        end
        sim = mean(dTheta,1);
        sim(Tab<size(theta,1)) = inf; %��ȫ�ӽ��Ķ���Ϊ����
        [minSim,ol] = min(sim);       %ÿ��Ǳ�ڶ�Ӧ�㶼�в�ֵ
        R = norm(srcDesp(:,m)-tarDesp(:,idx(ol)));%�����Ӳ���С����
        if (minSim<thetaThr && R<distThr)
            matches = [matches; m idx(ol)];
        end
    end
    %% ����
    figure;
    mapPair = joinImage(srcMap,tarMap);
    imshow(mapPair);
    xdistance = size(srcMap,2);
    match_srcSeed = srcSeed(:,matches(:,1));
    match_tarSeed = tarSeed(:,matches(:,2));
    showPoint(match_srcSeed*s);
    showTarSeed = match_tarSeed*s;
    showTarSeed(1,:)=showTarSeed(1,:)+xdistance;
    showPoint(showTarSeed)
    close all
    %%
    if(size(matches,1)>10)
        match_srcSeed = srcSeed(:,matches(:,1));
        match_tarSeed = tarSeed(:,matches(:,2));
        match_srcSeed3d=[match_srcSeed;zeros(1,size(match_srcSeed,2))];
        match_tarSeed3d=[match_tarSeed;zeros(1,size(match_tarSeed,2))];
        CS = ransac(double(match_srcSeed3d),double(match_tarSeed3d),threshold);   
        %��̭���ɿ��ĶԶ���Ȼ���ÿɿ��Զ�ȥ�����˶�
        if(sum(CS)<3)%�ɿ��Զ�����3ʱ���ټ���
            continue;
        end
        
        match_srcSeed3d = match_srcSeed3d(:,CS);
        match_tarSeed3d = match_tarSeed3d(:,CS);
        [T, Eps] = estimateRigidTransform(match_tarSeed3d, match_srcSeed3d);
        T2d=[T(1:2,1:2),T(1:2,4); [0 0 1]];
        tarEst = T2d*[srcSeed;ones(1,M)];
        tarEst = tarEst(1:2,:);
        tform{n} = T2d;
        
        [index,dist] = flann_search(tarEst,tarSeed,1,params);
        [dist,ind] = sort(dist);        
        Err(n) = sum(sum((tarEst(:,index(ind(1:ovNum)))-tarSeed(:,ind(1:ovNum))).^2));
        
%         if(n==102)
%             save('dataBestMatch2d.mat');
%         end
        
    end
    if (size(matches,1)> 0.65*size(srcDesp,2))
        break;
    end
 end
[v,idx] = min(Err);

T = tform{idx};
% disp('final_select_match:');
% disp(final_select_match);
% disp(['size : ' num2str(final_select_match)]);
if(isempty(T))
    error(['match Failed with tarseed:' num2str(length(tarSeed)) ' srcSeed:' num2str(length(srcSeed)) ]);
end
final_select_match =   matchpairs_pointset{idx};
end
