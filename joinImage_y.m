function im = joinImage_y(image1, image2)

% Select the image with the fewest rows and fill in enough empty rows
%   to make it the same height as the other image.
rows1 = size(image1,1);
width1 = size(image1,2);
rows2 = size(image2,1);
width2 = size(image2,2);
if (width1 < width2)
%      image1(rows2,1) = 255;
     image1(1:rows1 ,width1+1:width2) = 255*ones(rows1,width2-width1);
else
     image2(1:rows2,width2+1:width1) = 255*ones(rows2,width1-width2);
%      image2(rows1,1) = 255;
end

% Now append both images side-by-side.
im = [image1;image2];   