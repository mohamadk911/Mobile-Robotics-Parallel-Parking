function localPoses = parkingWorldToLocal(worldPoses, framePoseWorld)
%parkingWorldToLocal Inverse rigid transform of world rear-axle pose columns.
% POSES are finite real numeric 3-by-N [x;y;theta], N>=1; FRAME is a finite
% 3-by-1 [ox;oy;phi] in m/rad. Output is double and the same size as POSES.
% pLocal=R(phi)'*(pWorld-origin); thetaLocal=thetaWorld-phi. No angle wrapping
% is applied, preserving the supplied heading branch. Equivalent headings
% must be compared modulo 2*pi. Inverse: parkingLocalToWorld. Very large
% origins can lose small offsets through floating-point cancellation.

    validateattributes(worldPoses,{'numeric'},{'real','finite','2d','nonempty','nrows',3}, ...
        mfilename,'worldPoses');
    validateattributes(framePoseWorld,{'numeric'},{'real','finite','size',[3,1]}, ...
        mfilename,'framePoseWorld');
    poses = double(worldPoses);
    frame = double(framePoseWorld);
    R = [cos(frame(3)),-sin(frame(3));sin(frame(3)),cos(frame(3))];
    localPoses = [R.'*(poses(1:2,:)-frame(1:2));poses(3,:)-frame(3)];
    if any(~isfinite(localPoses),'all')
        error('parkingWorldToLocal:NonfiniteResult','Local pose calculation overflowed.');
    end
end
