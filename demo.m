clc;clear;close all;
addpath('./flann/');
addpath('./estimateRigidTransform');
s=100;
gridStep=0.3;
overlap=0.2;
%% 转灰度
map1=rgb2gray(imread('Fr1_5.png','png'));
map2=rgb2gray(imread('Fr2_5.png','png'));
% imshowpair(map1,map2);
tic;
%% 选取特殊点   
% 1.选取harris角点最强点200个 
% 2.均匀分布选取harris角点最强点200个    (current) 
% imshow(map1);hold on;
cornersM1=detectHarrisFeatures(map1);%plot(selectUniform(cornersM1,200,size(map1)));%cornersM1.selectStrongest(200)
seleCorM1=selectUniform(cornersM1,300,size(map1));
% plot(seleCorM1);

% figure;
% imshow(map2);hold on;
cornersM2=detectHarrisFeatures(map2);
seleCorM2=selectUniform(cornersM2,300,size(map2));
% plot(seleCorM2);

zSeleCorM1=cornerPoints(seleCorM1.Location/s,'Metric',seleCorM1.Metric);
zSeleCorM2=cornerPoints(seleCorM2.Location/s,'Metric',seleCorM2.Metric);
%%  提取二维点云
[pcMap3d1,pointCMap1]=exarctPCfroImg(map1);
[pcMap3d2,pointCMap2]=exarctPCfroImg(map2);
pointCMap1=pointCMap1/s;
pointCMap2=pointCMap2/s;
pcMap3d1=pointCloud ( pcMap3d1.Location/s);
pcMap3d2=pointCloud ( pcMap3d2.Location/s);
figure;
plot(pointCMap1(:,1),pointCMap1(:,2),'.');hold on;
plot(zSeleCorM1);
axis equal
figure;
plot(pointCMap2(:,1),pointCMap2(:,2),'.');hold on;
plot(zSeleCorM2);
axis equal
%%  获取描述子

[M1Desp,M1Seed,M1Norm]=exarctEIG2d(pointCMap1,gridStep,zSeleCorM1);
[M2Desp,M2Seed,M2Norm]=exarctEIG2d(pointCMap2,gridStep,zSeleCorM2);
%%  匹配
Motion=eigMatch2D(M1Desp,M2Desp,M1Seed,M2Seed,M1Norm,M2Norm,overlap,gridStep);
toc
figure;
obtain2d(pointCMap2,'.');hold on;
transMap=Motion*[pointCMap1';ones(1,length(pointCMap1))];
obtain2d(transMap,'.');
% pcshow(pcMap3d2);hold on;
% pcshow(pctransform(pcMap3d1,affine3d(Motion')));
% imshow(imrotate(map1,74.0));
% imshowpair(map1,map2);
% imshow(imresize(map1,[200,200]));

