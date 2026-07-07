# Integration Notes

This repository starts with simulation backends so the mission flow can be
developed before the robot, D1 arm SDK, and camera pipeline are all available.

## Go2 motion backend

Replace `CreateSimRobotMotion()` with a Unitree SDK2 implementation that maps:

- `Initialize()` to `ChannelFactory::Instance()->Init(0, network_interface)` and `SportClient::Init()`.
- `FollowLine()` to `SportClient::Move(vx, 0, yaw_rate)`.
- `StopMove()` to `SportClient::StopMove()`.
- `JumpForward()` to `SportClient::FrontJump()` after bench testing.
- `EnableObstacleAvoidance()` to `SportClient::SwitchAvoidMode()` or the Go2 obstacles-avoid client, depending on firmware.
- `ClimbStairs()` to the gait you validate as safest for the three-step obstacle.

Keep the simulator available. It is useful for checking state-machine changes
without putting the robot at risk.

## Go2 video and OpenCV backend

The planned first production vision backend should:

- Use `VideoClient::GetImageSample()` to get JPEG bytes from Go2.
- Decode the bytes with `cv::imdecode`.
- Estimate black-line offset from a lower image ROI.
- Detect markers with ArUco or robust color/shape templates.
- Return `VisionResult` with confidence and a safe fallback when uncertain.

## D1 arm backend

Wrap the official D1 SDK behind `ArmController`. For the first competition
version, prefer scripted poses over online motion planning:

- `Home`
- `PickStartMaterial`
- `PlaceOnTransferPlatform`
- `PickFieldMaterial`
- `PlaceOnZone`
- `Release`

Every pose must have a measured success rate before it is used in the full run.

## Emergency stop

The runner stops when either condition is true:

- Ctrl+C is pressed.
- A file named `STOP` exists in the process working directory.

Hardware backends must make `EmergencyStop()` conservative: stop base movement,
release or hold the arm according to the safest tested behavior, and avoid
starting any new motion.

