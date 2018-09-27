clc;clear;close all;
load dataBestMatch2d.mat
map1=rgb2gray(imread('Fr1_3.png','png'));
map2=rgb2gray(imread('Fr2_3.png','png'));
[pcMap3d1,pointCMap1]=exarctPCfroImg(map1);
[pcMap3d2,pointCMap2]=exarctPCfroImg(map2);
s=100;
% match_srcSeed3d
% match_tarSeed3d
mapPair = joinImage(map1,map2);
figure('Position', [100 100 size(mapPair,2) size(mapPair,1)]);
imshow(mapPair);
axis on
hold on
%²î¾àmap2
xdistance = size(map1,2);
close all

figure;
imshow(map1);
hold on;
x1=s*match_srcSeed3d(1,:)';
y1=s*match_srcSeed3d(2,:)';
plot(x1,y1,'.','Marker','*','MarkerSize',10);
hold on;


showPoint(s*srcSeed)