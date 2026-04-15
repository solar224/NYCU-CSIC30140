function Klist = generate_calib(datasetName)
%GENERATE_CALIB Create calibration txt using one simple entrypoint.
%   Klist = generate_calib('Matcha') or generate_calib("Matcha") will create Matcha_calib.txt in the same directory as the dataset images.
%
% Strategy:
% 1) Find dataset images (my_data first, then data).
% 2) Try checkerboard-based calibration (best quality) if checkerboard images exist.
% 3) Fallback to FOV approximation when checkerboard data is unavailable.
%
% Output:
%   <dataset_dir>/<datasetName>_calib.txt

if nargin < 1 || isempty(datasetName)
    error('datasetName is required, e.g., ''Matcha''.');
end

datasetName = char(string(datasetName));
baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');
myDataDir = fullfile(baseDir, 'my_data');

[datasetRoot, files] = resolveDatasetImages(datasetName, myDataDir, dataDir);
if numel(files) < 2
    error('Need at least 2 images for %s, but only found %d in %s.', datasetName, numel(files), datasetRoot);
end

% HW2 is two-view SfM, so keep exactly 2 camera blocks.
numCameras = 2;
files = files(1:numCameras);
firstImagePath = fullfile(files(1).folder, files(1).name);

[K, methodUsed] = estimateBestIntrinsic(baseDir, datasetName, firstImagePath);
Klist = repmat(K, [1, 1, numCameras]);

calibPath = fullfile(datasetRoot, sprintf('%s_calib.txt', datasetName));
writeCalib(calibPath, Klist);

fprintf('Wrote calibration file: %s\n', calibPath);
fprintf('Dataset images: %d (using first %d for K blocks)\n', numel(files), numCameras);
fprintf('Calibration method: %s\n', methodUsed);
end

function [datasetRoot, files] = resolveDatasetImages(datasetName, myDataDir, dataDir)
candidates = {myDataDir, dataDir};
datasetRoot = '';
files = [];

for i = 1:numel(candidates)
    rootDir = candidates{i};
    if ~exist(rootDir, 'dir')
        continue;
    end

    filesLocal = collectDatasetImages(rootDir, datasetName);
    if ~isempty(filesLocal)
        datasetRoot = rootDir;
        files = filesLocal;
        return;
    end
end

error('No dataset images found for %s in my_data/ or data/.', datasetName);
end

function files = collectDatasetImages(rootDir, datasetName)
patterns = {
    sprintf('%s*.jpg', datasetName), ...
    sprintf('%s*.JPG', datasetName), ...
    sprintf('%s*.jpeg', datasetName), ...
    sprintf('%s*.JPEG', datasetName), ...
    sprintf('%s*.png', datasetName), ...
    sprintf('%s*.PNG', datasetName), ...
    sprintf('%s*.bmp', datasetName), ...
    sprintf('%s*.BMP', datasetName)
};

files = [];
for i = 1:numel(patterns)
    files = [files; dir(fullfile(rootDir, patterns{i}))]; %#ok<AGROW>
end

if isempty(files)
    return;
end

files = sortByNumericSuffix(files);
end

function [K, methodUsed] = estimateBestIntrinsic(baseDir, datasetName, firstImagePath)
if exist('detectCheckerboardPoints', 'file') == 2 && exist('estimateCameraParameters', 'file') == 2
    [cbDir, cbFiles] = resolveCheckerboardImages(baseDir, datasetName);
    if ~isempty(cbDir)
        [K, ok] = estimateIntrinsicFromCheckerboard(cbDir, cbFiles);
        if ok
            methodUsed = 'checkerboard';
            return;
        end
    end
end

K = estimateIntrinsicFromFov(firstImagePath, 69);
methodUsed = 'fov-approx (fallback)';
warning(['Checkerboard calibration was not available or failed. ', ...
    'Used FOV approximation fallback (69 deg).']);
end

function [cbDir, cbFiles] = resolveCheckerboardImages(baseDir, datasetName)
candidates = {
    fullfile(baseDir, 'my_data', 'checkerboard', datasetName), ...
    fullfile(baseDir, 'data', 'checkerboard', datasetName), ...
    fullfile(baseDir, 'my_data', [datasetName '_checkerboard']), ...
    fullfile(baseDir, 'data', [datasetName '_checkerboard'])
};

cbDir = '';
cbFiles = [];
for i = 1:numel(candidates)
    d = candidates{i};
    if ~exist(d, 'dir')
        continue;
    end

    files = listImagesInDir(d);
    if ~isempty(files)
        cbDir = d;
        cbFiles = files;
        return;
    end
end
end

function [K, ok] = estimateIntrinsicFromCheckerboard(cbDir, cbFiles)
ok = false;
K = eye(3);

imgPaths = fullfile(cbDir, {cbFiles.name});
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imgPaths);
if nnz(imagesUsed) < 3
    return;
end

imgPaths = imgPaths(imagesUsed);
imagePoints = imagePoints(:, :, imagesUsed);
worldPoints = generateCheckerboardPoints(boardSize, 10);

I0 = imread(imgPaths{1});
imageSize = [size(I0, 1), size(I0, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    'ImageSize', imageSize, 'WorldUnits', 'millimeters');

if isprop(cameraParams, 'Intrinsics')
    intr = cameraParams.Intrinsics;
    if isprop(intr, 'K')
        K = intr.K;
    else
        K = intr.IntrinsicMatrix';
    end
else
    K = cameraParams.IntrinsicMatrix';
end

K = K / K(3, 3);
ok = true;
end

function K = estimateIntrinsicFromFov(imgPath, fovDeg)
info = imfinfo(imgPath);
w = double(info.Width);
h = double(info.Height);

f = (w / 2) / tand(fovDeg / 2);
cx = (w - 1) / 2;
cy = (h - 1) / 2;
K = [f, 0, cx; 0, f, cy; 0, 0, 1];
end

function files = listImagesInDir(dirPath)
patterns = {'*.jpg', '*.JPG', '*.jpeg', '*.JPEG', '*.png', '*.PNG', '*.bmp', '*.BMP'};
files = [];
for i = 1:numel(patterns)
    files = [files; dir(fullfile(dirPath, patterns{i}))]; %#ok<AGROW>
end
if ~isempty(files)
    files = sortByNumericSuffix(files);
end
end

function files = sortByNumericSuffix(files)
names = {files.name};
keys = inf(numel(names), 1);
for i = 1:numel(names)
    tok = regexp(names{i}, '\\d+', 'match', 'once');
    if ~isempty(tok)
        keys(i) = str2double(tok);
    end
end

[~, idx] = sortrows([keys, (1:numel(names))']);
files = files(idx);
end

function writeCalib(calibPath, Klist)
fid = fopen(calibPath, 'wt');
if fid < 0
    error('Failed to open calibration file for writing: %s', calibPath);
end

cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'calibration parameters\n');

for i = 1:size(Klist, 3)
    fprintf(fid, 'Camera %s:\n', cameraLabel(i));
    fprintf(fid, 'K%d:\n', i);
    K = Klist(:, :, i);
    for r = 1:3
        fprintf(fid, '%.6f %.6f %.6f\n', K(r, 1), K(r, 2), K(r, 3));
    end
    fprintf(fid, '\n');
end
end

function label = cameraLabel(i)
if i <= 26
    label = char('A' + i - 1);
else
    label = sprintf('C%d', i);
end
end
