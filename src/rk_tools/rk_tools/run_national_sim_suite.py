"""Execute all nominal and injected-fault national mission simulations."""

import argparse
import json
import os
import subprocess
import sys

from .national_route_simulator import (
    ScenarioResult,
    _write_suite_summary,
    default_output_dir,
    write_static_artifacts,
)
from .simulation.scenario_definitions import FAULT_SCENARIOS, NOMINAL_SCENARIOS


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='')
    args = parser.parse_args(argv)
    output_dir = args.output_dir or default_output_dir()
    write_static_artifacts(output_dir)
    results = []
    for scenario in NOMINAL_SCENARIOS + FAULT_SCENARIOS:
        print('[SIM_SUITE] running {}'.format(scenario.name), flush=True)
        # rclpy's Action/Future teardown is process-global.  A fresh process
        # per scenario prevents completed callbacks from one graph affecting
        # the timing or resources of the next scenario.
        completed = subprocess.run([
            sys.executable, '-m', 'rk_tools.national_route_simulator',
            '--scenario', scenario.name, '--output-dir', output_dir,
        ], check=False)
        result_path = os.path.join(
            output_dir, 'scenario_results', '{}.json'.format(scenario.name)
        )
        try:
            with open(result_path, encoding='utf-8') as stream:
                result = ScenarioResult(**json.load(stream))
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            result = ScenarioResult(
                scenario.name, 'PROCESS_ERROR', scenario.expected_terminal,
                False, 0.0, 'SIMULATION_PROCESS_ERROR', str(exc), 0, True,
            )
        if completed.returncode not in (0, 1):
            result.passed = False
            result.failure_code = 'SIMULATION_PROCESS_EXIT'
            result.failure_reason = 'exit code {}'.format(completed.returncode)
        results.append(result)
        print('[SIM_SUITE] {} terminal={} pass={}'.format(result.name, result.terminal_state, result.passed), flush=True)
    _write_suite_summary(output_dir, results)
    failures = [result for result in results if not result.passed or result.nonzero_authority_conflict]
    print('[SIM_SUITE] output={}'.format(output_dir), flush=True)
    print('[SIM_SUITE] total={} failures={}'.format(len(results), len(failures)), flush=True)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
