clc;clear;close all;
addpath('./flann/');
addpath('./estimateRigidTransform');
s=100;
gridStep=0.1;
overlap=0.2;
icpSteps=100;
TrMin=0.2;
TrMax=0.8;
desNum=400;
%% ת�Ҷ�

% map1=rgb2gray(imread('Fr1_5.png','png'));
% map2=rgb2gray(imread('Fr2_5.png','png'));
map1=rgb2gray(imread('..\map_data\pair1\Intel1_301_700.png','png'));
map2=rgb2gray(imread('..\map_data\pair1\lmap-02.png','png'));
% imshowpair(map1,map2);
tic;
%% ѡȡ�����   
% 1.ѡȡharris�ǵ���ǿ��200�� 
% 2.���ȷֲ�ѡȡharris�ǵ���ǿ��200��    (current) 
% imshow(map1);hold on;
cornersM1=detectHarrisFeatures(map1);%plot(selectUniform(cornersM1,200,size(map1)));%selectUniform(cornersM1,300,size(map1));
seleCorM1=cornersM1.selectStrongest(300);
% plot(seleCorM1);

% figure;
% imshow(map2);hold on;
cornersM2=detectHarrisFeatures(map2);
seleCorM2=cornersM2.selectStrongest(300);
% plot(seleCorM2);

zSeleCorM1=cornerPoints(seleCorM1.Location/s,'Metric',seleCorM1.Metric);
zSeleCorM2=cornerPoints(seleCorM2.Location/s,'Metric',seleCorM2.Metric);
%%  ��ȡ��ά����
[pcMap3d1,pointCMap1]=exarctPCfroImg(map1);
[pcMap3d2,pointCMap2]=exarctPCfroImg(map2);
%% temp change downsample
%     downPC = pcdownsample(pcMap3d2,'gridAverage',4);
%     pointCMap2 = downPC.Location(:,1:2);
%%


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
%%  ��ȡ������

[M1Desp,M1Seed,M1Norm]=exarctEIG2d(pointCMap1,gridStep,zSeleCorM1);
[M2Desp,M2Seed,M2Norm]=exarctEIG2d(pointCMap2,gridStep,zSeleCorM2);
%%  ƥ��
[Motion,match_pair]=eigMatch2D(M1Desp,M2Desp,M1Seed,M2Seed,M1Norm,M2Norm,overlap,gridStep);


%% testcode
% close all;
% plot(s*pointCMap1(:,1),s*pointCMap1(:,2),'.');
% hold on 
% plot(s*M1Seed(1,:)',s*M1Seed(2,:)','.','MarkerSize',10 ,'color','red');
%% ��ICP
R0=Motion(1:2,1:2);
t0=Motion(1:2,3);
%% originalsize
match_source = match_pair{1}*s;
match_target = match_pair{2}*s;
match_source2d = match_source(1:2,:)';
match_target2d = match_target(1:2,:)';
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
obtain2d(match_target2d,'x');
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

