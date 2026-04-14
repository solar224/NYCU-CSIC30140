function results = sfm_main(datasetName, outputDir)
%SFM_MAIN Reconstruct sparse 3D points from a calibrated image pair.
%   results = sfm_main('Statue') or sfm_main('Mesona')
%
% Pipeline:
% 1) Feature detection + matching
% 2) Normalized 8-point with RANSAC for F
% 3) E = K2' * F * K1 and four [R|t] candidates
% 4) Triangulation and cheirality test to select pose
% 5) Export OBJ/MTL via provided obj_main.m

if nargin < 1 || isempty(datasetName)
    datasetName = 'Statue';
end
if isstring(datasetName)
    datasetName = char(datasetName);
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(pwd, 'output', datasetName);
end
if isstring(outputDir)
    outputDir = char(outputDir);
end

baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');
addpath(baseDir);

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

[imgPath1, imgPath2, calibPath] = getDatasetPaths(dataDir, datasetName);
I1 = imread(imgPath1);
I2 = imread(imgPath2);
rng(0, 'twister');

if size(I1, 3) == 3
    G1 = rgb2gray(I1);
else
    G1 = I1;
end
if size(I2, 3) == 3
    G2 = rgb2gray(I2);
else
    G2 = I2;
end

% Contrast normalization improves keypoint repeatability on low-texture areas.
if exist('adapthisteq', 'file') == 2
    G1 = adapthisteq(G1);
    G2 = adapthisteq(G2);
end

% 1) Feature detection + matching
[p1, p2, matchedPairsFigure, featureMethod] = detectAndMatchPoints(G1, G2);
fprintf('Feature method: %s\n', featureMethod);
fprintf('Matched points: %d\n', size(p1, 1));

saveas(matchedPairsFigure, fullfile(outputDir, '01_feature_matches.png'));
close(matchedPairsFigure);

% 2) Fundamental matrix with normalized 8-point + robust RANSAC
imgScale = max([size(G1, 1), size(G1, 2), size(G2, 1), size(G2, 2)]) / 1000;
ransacIters = 5000;
inlierThresh = max(0.8, 1.2 * imgScale);
[F, inlierMask] = estimateFundamentalRansac(p1, p2, ransacIters, inlierThresh);

p1In = p1(inlierMask, :);
p2In = p2(inlierMask, :);

fprintf('Inliers after RANSAC: %d / %d\n', size(p1In, 1), size(p1, 1));

inlierFig = figure('Name', 'Inlier matches');
showMatchedFeatures(I1, I2, p1In, p2In, 'montage');
title('Inlier Matches');
saveas(inlierFig, fullfile(outputDir, '02_inlier_matches.png'));
close(inlierFig);

% 3) Read K and compute E
[K1, K2] = readCalibration(calibPath);
E = K2' * F * K1;
E = enforceEssentialRank2(E);

% 4) Pose decomposition + triangulation + cheirality
[RSet, tSet] = decomposeEssential(E);
[P, p1Used, p2Used, R, t, poseIdx, positiveCount] = selectPoseByCheirality(K1, K2, p1In, p2In, RSet, tSet);

fprintf('Selected pose index: %d\n', poseIdx);
fprintf('Positive depth points: %d / %d\n', positiveCount, size(P, 1));

% Reprojection diagnostics
[P1cam, P2cam] = cameraMatrices(K1, K2, R, t);
[err1, err2] = reprojectionError(P, p1Used, p2Used, P1cam, P2cam);
fprintf('Mean reprojection error cam1: %.4f px\n', mean(err1));
fprintf('Mean reprojection error cam2: %.4f px\n', mean(err2));

% Remove high-error points to improve mesh stability and texture mapping.
[P, p1Used, p2Used, reprojThresh, removedCount] = filterReprojectionOutliers(P, p1Used, p2Used, err1, err2);
if removedCount > 0
    [err1, err2] = reprojectionError(P, p1Used, p2Used, P1cam, P2cam);
    fprintf('Reprojection filtering threshold: %.4f px\n', reprojThresh);
    fprintf('Removed high-error points: %d\n', removedCount);
    fprintf('Filtered mean reprojection cam1: %.4f px\n', mean(err1));
    fprintf('Filtered mean reprojection cam2: %.4f px\n', mean(err2));
end

save(fullfile(outputDir, 'sfm_results.mat'), 'F', 'E', 'K1', 'K2', 'R', 't', 'P', 'p1Used', 'p2Used', 'err1', 'err2');

% 3D scatter visualization
fig3d = figure('Name', 'Sparse 3D reconstruction');
scatter3(P(:, 1), P(:, 2), P(:, 3), 8, P(:, 3), 'filled');
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('%s sparse reconstruction', datasetName));
saveas(fig3d, fullfile(outputDir, '03_sparse_points.png'));
close(fig3d);

% 5) OBJ export (stage 2)
% Use image 2 as texture and camera 2 projection for visibility checks.
textureName = sprintf('%s_texture.png', datasetName);
texturePath = fullfile(outputDir, textureName);
imwrite(I2, texturePath);

if exist(texturePath, 'file') ~= 2
    error('Texture image was not written: %s', texturePath);
end

oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir));
cd(outputDir);
obj_main(P, p2Used, P2cam, textureName, 1, outputDir);
clear cleanupObj;

results = struct();
results.dataset = datasetName;
results.outputDir = outputDir;
results.featureMethod = featureMethod;
results.F = F;
results.E = E;
results.K1 = K1;
results.K2 = K2;
results.R = R;
results.t = t;
results.P = P;
results.p1 = p1Used;
results.p2 = p2Used;
results.meanReproj1 = mean(err1);
results.meanReproj2 = mean(err2);

fprintf('Done. Outputs written to: %s\n', outputDir);

end

function [imgPath1, imgPath2, calibPath] = getDatasetPaths(dataDir, datasetName)
name = lower(strtrim(datasetName));
switch name
    case 'mesona'
        imgPath1 = fullfile(dataDir, 'Mesona1.JPG');
        imgPath2 = fullfile(dataDir, 'Mesona2.JPG');
        calibPath = fullfile(dataDir, 'Mesona_calib.txt');
    case 'statue'
        imgPath1 = fullfile(dataDir, 'Statue1.bmp');
        imgPath2 = fullfile(dataDir, 'Statue2.bmp');
        calibPath = fullfile(dataDir, 'Statue_calib.txt');
    case 'my_dataset'
        imgPath1 = fullfile(dataDir, 'MyDataset1.jpg');
        imgPath2 = fullfile(dataDir, 'MyDataset2.jpg');
        calibPath = fullfile(dataDir, 'MyDataset_calib.txt');
    otherwise
        error('Unknown datasetName: %s (use ''Mesona'' or ''Statue'' or ''MyDataset'')', datasetName);
end

if ~exist(imgPath1, 'file') || ~exist(imgPath2, 'file') || ~exist(calibPath, 'file')
    error('Missing required data files for dataset: %s', datasetName);
end
end

function [p1, p2, figHandle, methodName] = detectAndMatchPoints(G1, G2)
% Try SIFT -> SURF -> ORB for compatibility.
maxFeatures = 5000;
if exist('detectSIFTFeatures', 'file') == 2
    methodName = 'SIFT';
    f1 = detectSIFTFeatures(G1);
    f2 = detectSIFTFeatures(G2);
elseif exist('detectSURFFeatures', 'file') == 2
    methodName = 'SURF';
    f1 = detectSURFFeatures(G1, 'MetricThreshold', 300, 'NumOctaves', 4, 'NumScaleLevels', 6);
    f2 = detectSURFFeatures(G2, 'MetricThreshold', 300, 'NumOctaves', 4, 'NumScaleLevels', 6);
elseif exist('detectORBFeatures', 'file') == 2
    methodName = 'ORB';
    f1 = detectORBFeatures(G1);
    f2 = detectORBFeatures(G2);
else
    error('No supported feature detector found (SIFT/SURF/ORB).');
end

if f1.Count > maxFeatures
    f1 = selectStrongest(f1, maxFeatures);
end
if f2.Count > maxFeatures
    f2 = selectStrongest(f2, maxFeatures);
end

[fDesc1, vp1] = extractFeatures(G1, f1);
[fDesc2, vp2] = extractFeatures(G2, f2);

idx12 = matchFeatures(fDesc1, fDesc2, ...
    'MaxRatio', 0.75, ...
    'MatchThreshold', 40, ...
    'Unique', true);

idx21 = matchFeatures(fDesc2, fDesc1, ...
    'MaxRatio', 0.75, ...
    'MatchThreshold', 40, ...
    'Unique', true);

idx21flip = [idx21(:, 2), idx21(:, 1)];
[~, ia] = intersect(idx12, idx21flip, 'rows', 'stable');
idxPairs = idx12(ia, :);

if size(idxPairs, 1) < 20
    % Fallback if mutual matching is too strict in low-texture scenes.
    idxPairs = idx12;
end

m1 = vp1(idxPairs(:, 1));
m2 = vp2(idxPairs(:, 2));

p1 = double(m1.Location);
p2 = double(m2.Location);

if size(p1, 1) < 8
    error('Too few matches (%d). Need at least 8.', size(p1, 1));
end

figHandle = figure('Name', 'Raw feature matches');
showMatchedFeatures(G1, G2, p1, p2, 'montage');
title(sprintf('Raw Matches (%s)', methodName));
end

function [Fbest, inlierMaskBest] = estimateFundamentalRansac(p1, p2, nIters, thresh)
N = size(p1, 1);
if N < 8
    error('Need at least 8 points for F estimation.');
end

bestCount = 0;
bestMedianErr = inf;
Fbest = [];
inlierMaskBest = false(N, 1);
conf = 0.999;
iter = 0;
maxIters = nIters;

while iter < maxIters
    iter = iter + 1;
    idx = randperm(N, 8);
    F = normalizedEightPoint(p1(idx, :), p2(idx, :));

    err = sampsonError(F, p1, p2);
    inlierMask = err < thresh;
    c = nnz(inlierMask);

    if c >= 8
        Flocal = normalizedEightPoint(p1(inlierMask, :), p2(inlierMask, :));
        errLocal = sampsonError(Flocal, p1, p2);
        inlierLocal = errLocal < thresh;
        cLocal = nnz(inlierLocal);
        medErr = median(errLocal(inlierLocal));

        if cLocal > bestCount || (cLocal == bestCount && medErr < bestMedianErr)
            bestCount = cLocal;
            bestMedianErr = medErr;
            inlierMaskBest = inlierLocal;
            Fbest = Flocal;

            inlierRatio = cLocal / N;
            pNoOutliers = 1 - inlierRatio ^ 8;
            pNoOutliers = min(max(pNoOutliers, eps), 1 - eps);
            maxIters = min(maxIters, ceil(log(1 - conf) / log(pNoOutliers)));
            maxIters = max(maxIters, iter + 50);
        end
    end
end

if bestCount < 8
    error('RANSAC failed: only %d inliers found.', bestCount);
end

% Re-estimate with all inliers
Fbest = normalizedEightPoint(p1(inlierMaskBest, :), p2(inlierMaskBest, :));
end

function F = normalizedEightPoint(p1, p2)
if size(p1, 1) ~= size(p2, 1)
    error('Point count mismatch.');
end
if size(p1, 1) < 8
    error('Need at least 8 correspondences.');
end

[p1n, T1] = normalizePoints2D(p1);
[p2n, T2] = normalizePoints2D(p2);

x1 = p1n(:, 1); y1 = p1n(:, 2);
x2 = p2n(:, 1); y2 = p2n(:, 2);

A = [x2 .* x1, x2 .* y1, x2, ...
     y2 .* x1, y2 .* y1, y2, ...
     x1,       y1,      ones(size(x1))];

[~, ~, V] = svd(A, 0);
Fh = reshape(V(:, end), 3, 3)';

% Enforce rank-2 on F
[U, S, V] = svd(Fh);
S(3, 3) = 0;
Fh = U * S * V';

% Denormalize
F = T2' * Fh * T1;

% Scale for numerical readability
if abs(F(3, 3)) > eps
    F = F / F(3, 3);
else
    F = F / norm(F, 'fro');
end
end

function [pn, T] = normalizePoints2D(p)
mu = mean(p, 1);
pShift = p - mu;
d = sqrt(sum(pShift .^ 2, 2));
meanD = mean(d);

if meanD < eps
    s = 1;
else
    s = sqrt(2) / meanD;
end

T = [s, 0, -s * mu(1); ...
     0, s, -s * mu(2); ...
     0, 0, 1];

ph = [p, ones(size(p, 1), 1)]';
pnh = T * ph;
pn = pnh(1:2, :)' ./ pnh(3, :)';
end

function e = sampsonError(F, p1, p2)
p1h = [p1, ones(size(p1, 1), 1)]';
p2h = [p2, ones(size(p2, 1), 1)]';

Fp1 = F * p1h;
Ftp2 = F' * p2h;
num = sum(p2h .* (F * p1h), 1) .^ 2;
den = Fp1(1, :) .^ 2 + Fp1(2, :) .^ 2 + Ftp2(1, :) .^ 2 + Ftp2(2, :) .^ 2;

e = (num ./ den)';
end

function [K1, K2] = readCalibration(calibPath)
textData = fileread(calibPath);
lines = splitlines(string(textData));
lines = strip(lines);

iK1 = find(lines == "K1:", 1, 'first');
iK2 = find(lines == "K2:", 1, 'first');

if isempty(iK1) || isempty(iK2) || (iK1 + 3 > numel(lines)) || (iK2 + 3 > numel(lines))
    error('Failed to parse K1/K2 blocks from calibration file: %s', calibPath);
end

k1Text = strjoin(lines(iK1 + (1:3)), ' ');
k2Text = strjoin(lines(iK2 + (1:3)), ' ');

k1Nums = regexp(char(k1Text), '[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', 'match');
k2Nums = regexp(char(k2Text), '[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', 'match');

if numel(k1Nums) ~= 9 || numel(k2Nums) ~= 9
    error('Calibration matrix parsing produced invalid size in: %s', calibPath);
end

K1 = reshape(str2double(k1Nums), [3, 3])';
K2 = reshape(str2double(k2Nums), [3, 3])';

% Normalize so K(3,3)=1 (important for Mesona where K(3,3)=0.001)
if abs(K1(3, 3)) < eps || abs(K2(3, 3)) < eps
    error('Invalid calibration matrix with near-zero K(3,3).');
end
K1 = K1 / K1(3, 3);
K2 = K2 / K2(3, 3);
end

function E2 = enforceEssentialRank2(E)
[U, S, V] = svd(E);
s = (S(1, 1) + S(2, 2)) / 2;
S = diag([s, s, 0]);
E2 = U * S * V';
end

function [RSet, tSet] = decomposeEssential(E)
[U, ~, V] = svd(E);

if det(U * V') < 0
    V = -V;
end

W = [0 -1 0; 1 0 0; 0 0 1];
R1 = U * W * V';
R2 = U * W' * V';
t = U(:, 3);

if det(R1) < 0
    R1 = -R1;
    t = -t;
end
if det(R2) < 0
    R2 = -R2;
end

RSet = cat(3, R1, R1, R2, R2);
tSet = [ t, -t, t, -t];
end

function [P, p1Sel, p2Sel, R, t, bestIdx, bestCount] = selectPoseByCheirality(K1, K2, p1, p2, RSet, tSet)
[P1cam, ~] = cameraMatrices(K1, K2, eye(3), [0; 0; 0]);

bestIdx = 1;
bestCount = -1;
Pbest = [];
p1Best = [];
p2Best = [];
R = RSet(:, :, 1);
t = tSet(:, 1);

for i = 1:4
    Ri = RSet(:, :, i);
    ti = tSet(:, i);
    [~, P2cam] = cameraMatrices(K1, K2, Ri, ti);

    Xi = triangulateDLT(P1cam, P2cam, p1, p2);

    depth1 = Xi(:, 3);
    X2 = (Ri * Xi' + ti);
    depth2 = X2(3, :)';

    valid = depth1 > 0 & depth2 > 0;
    c = nnz(valid);

    if c > bestCount
        bestCount = c;
        bestIdx = i;
        Pbest = Xi(valid, :);
        p1Best = p1(valid, :);
        p2Best = p2(valid, :);
        R = Ri;
        t = ti;
    end
end

if isempty(Pbest)
    error('No valid pose found by cheirality test.');
end

P = Pbest;
p1Sel = p1Best;
p2Sel = p2Best;
end

function [P1cam, P2cam] = cameraMatrices(K1, K2, R, t)
P1cam = K1 * [eye(3), zeros(3, 1)];
P2cam = K2 * [R, t];
end

function X = triangulateDLT(P1cam, P2cam, p1, p2)
N = size(p1, 1);
X = zeros(N, 3);

for i = 1:N
    x1 = p1(i, 1); y1 = p1(i, 2);
    x2 = p2(i, 1); y2 = p2(i, 2);

    A = [x1 * P1cam(3, :) - P1cam(1, :); ...
         y1 * P1cam(3, :) - P1cam(2, :); ...
         x2 * P2cam(3, :) - P2cam(1, :); ...
         y2 * P2cam(3, :) - P2cam(2, :)];

    [~, ~, V] = svd(A, 0);
    Xh = V(:, end);
    X(i, :) = (Xh(1:3) / Xh(4))';
end
end

function [err1, err2] = reprojectionError(P, p1, p2, P1cam, P2cam)
N = size(P, 1);
Ph = [P, ones(N, 1)]';

q1 = P1cam * Ph;
q1 = q1(1:2, :) ./ q1(3, :);
q2 = P2cam * Ph;
q2 = q2(1:2, :) ./ q2(3, :);

err1 = sqrt(sum((q1' - p1) .^ 2, 2));
err2 = sqrt(sum((q2' - p2) .^ 2, 2));
end

function [Pout, p1Out, p2Out, threshold, removedCount] = filterReprojectionOutliers(P, p1, p2, err1, err2)
maxErr = max(err1, err2);
threshold = percentileValue(maxErr, 75);

% Keep threshold adaptive but avoid overly aggressive trimming.
threshold = max(2.0, threshold);
keep = maxErr <= threshold;

minKeep = max(30, round(0.35 * numel(maxErr)));
if nnz(keep) < minKeep
    [~, order] = sort(maxErr, 'ascend');
    keep = false(size(maxErr));
    keep(order(1:minKeep)) = true;
    threshold = maxErr(order(minKeep));
end

Pout = P(keep, :);
p1Out = p1(keep, :);
p2Out = p2(keep, :);
removedCount = nnz(~keep);
end

function v = percentileValue(x, p)
xs = sort(x(:));
n = numel(xs);
if n == 0
    v = NaN;
    return;
end

idx = 1 + (n - 1) * (p / 100);
lo = floor(idx);
hi = ceil(idx);
if lo == hi
    v = xs(lo);
else
    alpha = idx - lo;
    v = (1 - alpha) * xs(lo) + alpha * xs(hi);
end
end
