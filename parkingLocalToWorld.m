function worldPoses = parkingLocalToWorld(localPoses, framePoseWorld)
%parkingLocalToWorld Rigidly transform parking-local rear-axle poses to world.
% POSES are finite real numeric 3-by-N columns [x;y;theta], N>=1. FRAME is
% the finite 3-by-1 [ox;oy;phi] pose of the parking-local origin in world.
% Positions are metres, headings radians; output is double and the same size.
% Headings are thetaLocal+phi, deliberately NOT wrapped: continuous heading
% branches and full turns are retained. Compare angles modulo 2*pi using
% atan2(sin(dtheta),cos(dtheta)). Inverse: parkingWorldToLocal.

    validateattributes(localPoses,{'numeric'},{'real','finite','2d','nonempty','nrows',3}, ...
        mfilename,'localPoses');
    validateattributes(framePoseWorld,{'numeric'},{'real','finite','size',[3,1]}, ...
        mfilename,'framePoseWorld');
    poses = double(localPoses);
    frame = double(framePoseWorld);
    R = [cos(frame(3)),-sin(frame(3));sin(frame(3)),cos(frame(3))];
    worldPoses = [R*poses(1:2,:)+frame(1:2);poses(3,:)+frame(3)];
    if any(~isfinite(worldPoses),'all')
        error('parkingLocalToWorld:NonfiniteResult','World pose calculation overflowed.');
    end
end
