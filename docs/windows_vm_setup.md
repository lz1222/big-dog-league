# Windows VM Development Setup

Use Ubuntu 22.04 inside VMware as the real ROS2 workspace host. Keep the
workspace on the Ubuntu ext4 filesystem, for example `~/rk_inspection_ws`.

Recommended workflow:

1. Install ROS2 Humble in Ubuntu 22.04.
2. Enable SSH in Ubuntu.
3. Connect from Windows VS Code through Remote SSH.
4. Open `~/rk_inspection_ws` in the remote VS Code window.
5. Edit only source files under `src/`, plus project docs and scripts.

Avoid placing the colcon workspace in a Windows shared folder. Shared folders
can break symlink installs, file permissions, and build performance.
