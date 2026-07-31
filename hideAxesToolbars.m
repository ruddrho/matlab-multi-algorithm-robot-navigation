function hideAxesToolbars(figureHandle)
%HIDEAXESTOOLBARS Hide MATLAB axes toolbars before capture or export.
%
% This function is compatibility-safe across MATLAB releases. It also
% disables default axes interactions so the toolbar does not reappear
% when the pointer passes over an axes during animation or image export.

if nargin < 1 || isempty(figureHandle)
    return;
end

try
    if ~ishandle(figureHandle) || ...
            ~strcmpi(get(figureHandle,'Type'),'figure')
        return;
    end
catch
    return;
end

axesHandles = findall(figureHandle,'Type','axes');

for axesIndex = 1:numel(axesHandles)
    axesHandle = axesHandles(axesIndex);

    try
        disableDefaultInteractivity(axesHandle);
    catch
        % Function unavailable in older MATLAB releases.
    end

    try
        toolbarHandle = axtoolbar(axesHandle);
        toolbarHandle.Visible = 'off';
    catch
        try
            axesHandle.Toolbar.Visible = 'off';
        catch
            % Toolbar property unavailable in older releases.
        end
    end
end

drawnow;
end
