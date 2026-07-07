$ErrorActionPreference = "Stop"

$Profile = "safe"
if ($args.Count -ge 1) {
  $Profile = $args[0]
}

cmake -S . -B build
cmake --build build

$Candidates = @(
  ".\build\go2_inspection_runner.exe",
  ".\build\Debug\go2_inspection_runner.exe",
  ".\build\Release\go2_inspection_runner.exe"
)

foreach ($Candidate in $Candidates) {
  if (Test-Path $Candidate) {
    & $Candidate --config config/competition.conf --profile $Profile
    exit $LASTEXITCODE
  }
}

throw "go2_inspection_runner.exe was not found under build/"

