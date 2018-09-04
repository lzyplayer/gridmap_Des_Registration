clc;clear;close all;
addpath('./flann/');
addpath('./estimateRigidTransform');
s=100;
gridStep=0.3;
overlap=0.2;
%% 转灰度
grayMap1=rgb2gray(imread('Fr2_5.png','png'));
grayMap2=rgb2gray(imread('Fr2_3.png','png'));
% proportionDetect(map1,map2,gridStep,overlap,s);


%灰度图为输入
%% 选取特殊点   
% 1.选取harris角点最强点200个 
% 2.均匀分布选取harris角点最强点200个    (current) 
cornersM1=detectHarrisFeatures(grayMap1);%plot(selectUniform(cornersM1,200,size(map1)));%cornersM1.selectStrongest(200)
seleCorM1=selectUniform(cornersM1,300,size(grayMap1));
cornersM2=detectHarrisFeatures(grayMap2);
seleCorM2=selectUniform(cornersM2,300,size(grayMap2));

%% 
zSeleCorM1=cornerPoints(seleCorM1.Location/s,'Metric',seleCorM1.Metric);
zSeleCorM2=cornerPoints(seleCorM2.Location/s,'Metric',seleCorM2.Metric);

%% 取为点云
[pcMap3d1,pointCMap1]=exarctPCfroImg(grayMap1);
[pcMap3d2,pointCMap2]=exarctPCfroImg(grayMap2);
pointCMap1=pointCMap1/s;
pointCMap2=pointCMap2/s;
pcMap3d1=pointCloud ( pcMap3d1.Location/s);
pcMap3d2=pointCloud ( pcMap3d2.Location/s);
fig=figure('Position',[1000 300 500 600]);
axes1=subplot(2,1,1);
title('Map 1')
plot(axes1,pointCMap1(:,1),pointCMap1(:,2),'.');hold on;
% plot(zSeleCorM1);
axis equal
axes2=subplot(2,1,2);
title('Map 2')
plot(axes2,pointCMap2(:,1),pointCMap2(:,2),'.');hold on;
% plot(zSeleCorM2);
axis equal
%%  获取描述子

[M1Desp,M1Seed,M1Norm]=exarctEIG2d(pointCMap1,gridStep,zSeleCorM1);
[M2Desp,M2Seed,M2Norm]=exarctEIG2d(pointCMap2,gridStep,zSeleCorM2);

%%  获取特征点匹配

pairInfo=DespMatch(M1Desp,M2Desp);
i=find(pairInfo(:,2)==197);

% for i=1:10
for r=[0.5 1 1.5 2]*gridStep
M1point=M1Seed(:,pairInfo(i,1))';
M2point=M2Seed(:,pairInfo(i,2))';
M2point=[ 3.21 5.5];
rectangle(axes1,'Position',[M1point(1)-r,M1point(2)-r,2*r,2*r],'Curvature',[1,1]);
rectangle(axes2,'Position',[M2point(1)-r,M2point(2)-r,2*r,2*r],'Curvature',[1,1]);
% plot(axesUp,M1point(1),M1point(2),'x');
% plot(axesUp,M1point(1),M1point(2)+0.3,'x');
% plot(axesDown,M2point(1),M2point(2),'x');
% plot(axesDown,M2point(1),M2point(2)+0.3,'x');
end
% if( input('agian','s')=='q' )
%     break
% end

% end
% tic;
% %% 选取特殊点   
% % 1.选取harris角点最强点200个 
% % 2.均匀分布选取harris角点最强点200个    (current) 
% cornersM1=detectHarrisFeatures(map1);
% seleCorM1=selectUniform(cornersM1,300,size(map1));
% 
% 
% % figure;
% % imshow(map2);hold on;
% cornersM2=detectHarrisFeatures(map2);
% seleCorM2=selectUniform(cornersM2,300,size(map2));
% 
% 
% zSeleCorM1=cornerPoints(seleCorM1.Location/s,'Metric',seleCorM1.Metric);
% zSeleCorM2=cornerPoints(seleCorM2.Location/s,'Metric',seleCorM2.Metric);
% %%  提取二维点云
% [pcMap3d1,pointCMap1]=exarctPCfroImg(map1);
% [pcMap3d2,pointCMap2]=exarctPCfroImg(map2);
% pointCMap1=pointCMap1/s;
% pointCMap2=pointCMap2/s;
% pcMap3d1=pointCloud ( pcMap3d1.Location/s);
% pcMap3d2=pointCloud ( pcMap3d2.Location/s);
% figure;
% plot(pointCMap1(:,1),pointCMap1(:,2),'.');hold on;
% plot(zSeleCorM1);
% axis equal
% figure;
% plot(pointCMap2(:,1),pointCMap2(:,2),'.');hold on;
% plot(zSeleCorM2);
% axis equal
% %%  获取描述子
% 
% [M1Desp,M1Seed,M1Norm]=exarctEIG2d(pointCMap1,gridStep,zSeleCorM1);
% [M2Desp,M2Seed,M2Norm]=exarctEIG2d(pointCMap2,gridStep,zSeleCorM2);
% %%  匹配
% Motion=eigMatch2D(M1Desp,M2Desp,M1Seed,M2Seed,M1Norm,M2Norm,overlap,gridStep);
% toc
% figure;
% obtain2d(pointCMap2,'.');hold on;
% transMap=Motion*[pointCMap1';ones(1,length(pointCMap1))];
% obtain2d(transMap,'.');

