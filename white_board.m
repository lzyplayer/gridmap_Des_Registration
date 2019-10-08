        figure;
        mapPair = joinImage(srcMap,tarMap);
        imshow(mapPair);
        xdistance = size(srcMap,2);
        showTarSeed = match_tarSeed*s;
        showTarSeed(1,:)=showTarSeed(1,:)+xdistance;
        showLineMulti(match_srcSeed'*s,showTarSeed');
        close 