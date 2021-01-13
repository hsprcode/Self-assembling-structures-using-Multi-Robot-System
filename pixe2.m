% pixe2.m
clear;
Image = rgb2gray(imread('img4.png'));
colormap(gray(256));
scale = 2;
newsize = scale*10;
reducedimage = imresize(Image, [newsize newsize]);
image(reducedimage);
reducedimage(ceil(newsize/2),ceil(newsize/2)); %for me


%% for finding the center for the structure
for i = 1:1:newsize
    for j = 1:1:newsize
        if reducedimage(i,j) >= 1
            A = i; 
            B = j;
            flag = 1;
            break;
        end  
    end
    if flag == 1
        break;
    end
end

%% for assigning potentials
for i = 1:1:newsize
    for j = 1:1:newsize
        if reducedimage(i,j) >= 1
            potentialimage(i,j) = ceil(sqrt((A-i)^2 + (B-j)^2));
        else
            potentialimage(i,j) = -10*ceil(sqrt((A-i)^2 + 2*(B-j)^2));
        end
    end
end
potentialimage = flip(potentialimage)
surf(potentialimage);
hold on
%% gradient
[px,py] = gradient(potentialimage)
quiver(px,py)
n = 7; m = 2; % x and y positions respectively
f_grad = [px(m,n) py(m,n)]
