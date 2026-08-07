# Robot Network Setup Notes

Stage one runs entirely with mock nodes. Real Go2, D1 arm, and D435i drivers
are intentionally not launched.

For later hardware integration:

- Prefer bridged networking for the Ubuntu VM when testing robot connectivity.
- Keep the robot, Jetson, and development VM on reachable IP ranges.
- Record robot IPs and Jetson IPs before field testing.
- Check DDS discovery settings before multi-machine ROS2 tests.
- Use `./scripts/check_network.sh <robot-ip>` for a quick host reachability test.

Do not change the mock launch into a hardware launch during stage one.
