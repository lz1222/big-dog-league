# Go2 Inspection Runner

Starter C++17/CMake project for the Go2 multimodal inspection competition.

The current implementation is intentionally safe: it runs the complete mission
state machine with simulation backends for Go2 motion, Go2 vision, and the D1
arm. Hardware-specific code should be added behind the existing interfaces in
`include/inspection`.

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Run simulation

```bash
./build/go2_inspection_runner --config config/competition.conf --profile safe
```

On Windows with a multi-config generator, the executable may be under
`build/Debug/` or `build/Release/`.

Convenience scripts:

```bash
bash scripts/run_sim.sh safe
```

```powershell
.\scripts\run_sim.ps1 safe
```

## Safety

Press Ctrl+C or create a file named `STOP` in the working directory to trigger
emergency stop.

## Next hardware steps

1. Add a Unitree SDK2 `RobotMotion` backend.
2. Add a Go2 `VideoClient` + OpenCV `VisionPipeline` backend.
3. Add a D1 SDK `ArmController` backend.
4. Keep the simulator as the default for state-machine development.
