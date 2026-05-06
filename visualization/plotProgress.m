function s = plotProgress(stage, region_idx, n_regions, rot_idx, n_rots, msg)
%PLOTPROGRESS  Format a progress event into a single-line status string.
%
%   s = plotProgress(stage, region_idx, n_regions, rot_idx, n_rots, msg)
%
%   Pure formatting helper - no figure or axes interaction.
%   Used by both the console default progress printer and the uihtml UI.
%
%   stage         "setup" | "region" | "rotation" | "done"
%   region_idx    1-based region index (0 during setup)
%   n_regions     total region count
%   rot_idx       1-based rotation index within the current region (0 if N/A)
%   n_rots        total rotations per region
%   msg           free-text message

    arguments
        stage      (1,1) string
        region_idx (1,1) double
        n_regions  (1,1) double
        rot_idx    (1,1) double
        n_rots     (1,1) double
        msg        (1,:) char
    end

    switch stage
        case "setup"
            s = sprintf('[setup]    %s', msg);
        case "region"
            s = sprintf('[region]   %d/%d - %s', region_idx, n_regions, msg);
        case "rotation"
            s = sprintf('[rotation] %d/%d  rot %d/%d - %s', ...
                region_idx, n_regions, rot_idx, n_rots, msg);
        case "done"
            s = sprintf('[done]     %s', msg);
        otherwise
            s = sprintf('[%s] %s', stage, msg);
    end
end
