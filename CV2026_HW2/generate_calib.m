function Klist = generate_calib(datasetName, varargin)
%GENERATE_CALIB Create calibration txt from image sizes.
%   Klist = generate_calib('Matcha')
%   Klist = generate_calib('Matcha', 'NumCameras', 5, ...
%       'HorizontalFovDeg', 69, 'Overwrite', true)
%
% Output format follows Mesona/Statue style and is written to:
%   data/<datasetName>_calib.txt

if nargin < 1 || isempty(datasetName)
    error('datasetName is required, e.g., ''Matcha''.');
end

opts = parseOptions(varargin{:});
baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');

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
    files = [files; dir(fullfile(dataDir, patterns{i}))]; %#ok<AGROW>
end

if isempty(files)
    error('No images found for dataset %s in %s.', datasetName, dataDir);
end

names = naturalSort({files.name});
files = files(ismember({files.name}, names));

if opts.NumCameras > numel(files)
    error('Requested NumCameras=%d but only found %d images.', opts.NumCameras, numel(files));
end

files = files(1:opts.NumCameras);
Klist = zeros(3, 3, numel(files));

for i = 1:numel(files)
    imgPath = fullfile(files(i).folder, files(i).name);
    info = imfinfo(imgPath);
    w = double(info.Width);
    h = double(info.Height);

    % Approximate focal length from assumed horizontal FOV.
    f = (w / 2) / tand(opts.HorizontalFovDeg / 2);
    cx = (w - 1) / 2;
    cy = (h - 1) / 2;

    Klist(:, :, i) = [f, 0, cx; 0, f, cy; 0, 0, 1];
end

calibPath = fullfile(dataDir, sprintf('%s_calib.txt', datasetName));
if exist(calibPath, 'file') == 2 && ~opts.Overwrite
    error('Calibration file already exists: %s (set Overwrite=true).', calibPath);
end

writeCalib(calibPath, Klist);
fprintf('Wrote calibration file: %s\n', calibPath);
fprintf('Detected %d image(s), generated %d camera block(s).\n', numel(names), size(Klist, 3));
end

function opts = parseOptions(varargin)
p = inputParser;
p.addParameter('NumCameras', 2, @(x) isnumeric(x) && isscalar(x) && x >= 2);
p.addParameter('HorizontalFovDeg', 69, @(x) isnumeric(x) && isscalar(x) && x > 10 && x < 170);
p.addParameter('Overwrite', true, @(x) islogical(x) && isscalar(x));
p.parse(varargin{:});
opts = p.Results;
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

function sorted = naturalSort(names)
keys = zeros(numel(names), 1);
for i = 1:numel(names)
    token = regexp(names{i}, '\d+', 'match', 'once');
    if isempty(token)
        keys(i) = inf;
    else
        keys(i) = str2double(token);
    end
end

[~, idx] = sortrows([keys, (1:numel(names))']);
sorted = names(idx);
end