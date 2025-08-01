%Name:        Alex Welke
%Date:        22/07/2025
%Description: Applies Gaussian blur to meshgrid

function result = gaussian_blur(SDF, sigma, n)
    tic
    r = floor(n/2);
    %Gets kernel

    kernel = zeros(1+r*2, 1+r*2, 1+r*2);
    for i = -r:r
        for j = -r:r
            for k = -r:r
                kernel(i+r+1, j+r+1) = exp(-0.5*(i^2+j^2+k^2)*sigma^(-2));
            end
        end
    end

    %normalise kernel
    %kernel = kernel/sum(kernel, "all");
    kernel = kernel/kernel(1, 1, 1);

    kernelSum = (sum(kernel, "all"));

    kernelRow = kernel(:, 1, 1);
    kernelCol= kernel(1, :, 1);
    kernelTube = kernel(1, 1, :)/kernelSum;

    result1 = SDF;
    result2 = SDF;
    result = SDF;

    for i = 1+r:(size(SDF, 1)-r)
        for j = 1:size(SDF, 2)
            for k = 1:size(SDF, 3)
                result1(i, j, k) = sum(kernelRow.*SDF(i-r:i+r, j, k), 1);
            end
        end
    end

    for i = 1+r:(size(SDF, 1)-r)
        for j = 1+r:(size(SDF, 2)-r)
            for k = 1:size(SDF, 3)
                result2(i, j, k) = sum(kernelCol.*result1(i, j-r:j+r, k), 2);
            end
        end
    end

    for i = 1+r:(size(SDF, 1)-r)
        for j = 1+r:(size(SDF, 2)-r)
            for k = 1+r:(size(SDF, 3)-r)
                result(i, j, k) = sum(kernelTube.*result2(i, j, k-r:k+r), 3);
            end
        end
    end

    toc

end