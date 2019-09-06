clc;clear;close all;
addpath('./flann/');
addpath('./estimateRigidTransform');
s=100;
gridStep=0.2;
overlap=0.2;
icpSteps=100;
TrMin=0.2;
TrMax=1.0;
desNum=300;
%% 转灰度
% map1 = imread('../map_data/pair3/1.jpg');
% map2 = imread('../map_data/pair3/2.jpg');
map1  = imread('../map_data/fr/Fr1_3.png');
map2  = imread('../map_data/fr/Fr2_3.png');
if size(map1,3)~=1 
    map1 = rgb2gray(map1);
end
if size(map2,3)~=1 
    map2 = rgb2gray(map2);
end

% map1 = imresize(map1,1.0092);
% map2=imrotate(map2,5);
% imshowpair(map1,map2);
tic;
%% 选取特殊点   
% 1.选取harris角点最强点200个 
% 2.均匀分布选取harris角点最强点200个    (current) 
% imshow(map1);hold on;
cornersM1=detectHarrisFeatures(map1);%plot(selectUniform(cornersM1,200,size(map1)));%cornersM1.selectStrongest(200)
seleCorM1=selectUniform(cornersM1,desNum,size(map1));
% plot(seleCorM1);

% figure;
% imshow(map2);hold on;
cornersM2=detectHarrisFeatures(map2);
seleCorM2=selectUniform(cornersM2,desNum,size(map2));
% plot(seleCorM2);

zSeleCorM1=cornerPoints(seleCorM1.Location/s,'Metric',seleCorM1.Metric);
zSeleCorM2=cornerPoints(seleCorM2.Location/s,'Metric',seleCorM2.Metric);
%%  提取二维点云
[pcMap3d1,pointCMap1]=exarctPCfroImg(map1);
[pcMap3d2,pointCMap2]=exarctPCfroImg(map2);
pointCMap1=pointCMap1/s;
pointCMap2=pointCMap2/s;
% pcMap3d1=pointCloud ( pcMap3d1.Location/s);
% pcMap3d2=pointCloud ( pcMap3d2.Location/s);
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


%% testcode
% close all;
% plot(s*pointCMap1(:,1),s*pointCMap1(:,2),'.');
% hold on 
% plot(s*M1Seed(1,:)',s*M1Seed(2,:)','.','MarkerSize',10 ,'color','red');
%% 加ICP
R0=Motion(1:2,1:2);
t0=Motion(1:2,3);
%% originalsize
pcr2=pointCMap2*s;
pcr1=pointCMap1*s;
transmation_ori_in=t0*s;
% pcr2=pointCMap2;
% pcr1=pointCMap1;
[Rori,tori,trko,minPhio]=fastTrICP2D(pcr2',pcr1',R0,transmation_ori_in,TrMin,TrMax,icpSteps);
opMotion=[R0,transmation_ori_in;0 0 1];
figure;
obtain2d(pcr2,'.');hold on;
transMap=opMotion*[pcr1';ones(1,length(pcr1))];
obtain2d(transMap,'.k');
%%
[R,t,trk,minPhi]=fastTrICP2D(pointCMap2',pointCMap1',R0,t0,TrMin,TrMax,icpSteps);
opMotion=[R0,t0;0 0 1];
toc
figure;
obtain2d(pointCMap2,'.');hold on;
transMap=opMotion*[pointCMap1';ones(1,length(pointCMap1))];
obtain2d(transMap,'.k');

obtain2d(zSeleCorM2.Location,'+r');
transFeaPoint=opMotion*[zSeleCorM1.Location';ones(1,length(zSeleCorM1.Location))];
obtain2d(transFeaPoint,'xm');
%%

to=t*s;


% reset to original scale

% merging_fixed(map2,map1,pointCMap1,R,to);
%% inverse show merge
motion_inv = inv([[R,to];[0,0,1]]);
R1 = motion_inv(1:2,1:2);
t1 = motion_inv(1:2,3);
merging_fixed(map1,map2,pointCMap2,R1,t1);

