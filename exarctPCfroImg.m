function [pCloud3d,points2d] = exarctPCfroImg(img)
%EXARCTPCFROIMG 此处显示有关此函数的摘要
%   此处显示详细说明
binImg=edge(img);
[Y,X] = find(binImg==1);
points2d=[X,Y];
pCloud3d=pointCloud([X Y zeros(length(X),1)]);

end

