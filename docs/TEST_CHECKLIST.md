# Test Checklist

## Daily single-module tests

- Line following: 20 runs, record speed, turn gain, line loss count.
- Start jump: 20 runs, record landing stability and drift.
- Finish jump: 20 runs, record whether all feet stay in the target lane.
- Stairs: 20 runs, record slips, stuck events, and recovery behavior.
- Obstacle area: 20 runs, record board contact count.
- Each D1 arm action: 30 runs, record success rate and reset time.

## Vision dataset

- Capture at least 100 images per marker/material class.
- Include bright, dark, side-angle, and partially occluded cases.
- Do not promote a detector to full-run testing until it reaches 95% accuracy
  on the local validation set.

## Full-run acceptance

- Run the full mission 10 times in a row.
- Record final score estimate, runtime, failed state, and whether any board was touched.
- Safe profile is used for first official attempt.
- Attack profile is used only after a safe-profile run has scored reliably.

## Match-day drill

- Simulate inspection, draw input, first attempt, quick parameter adjustment,
  second attempt, and score confirmation inside 10 minutes.
- Keep a printed fallback parameter sheet and a known-good binary.

