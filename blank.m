[pcda1,pcda2] = exarctPCfroImg(cdata);
pcshow(pcda1)

[Y,X] = find(cdata==0);
points2d=[X,Y];
pCloud3d=pointCloud([X Y zeros(length(X),1)]);
pcshow(pCloud3d)

points_scale1 = detectScale(points2d,[120,126],1,30);
