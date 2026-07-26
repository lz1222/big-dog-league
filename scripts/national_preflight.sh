#!/usr/bin/env bash

set -u

RK_PREFLIGHT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RK_PREFLIGHT_ROOT

python3 - <<'PY'
import ast
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(os.environ['RK_PREFLIGHT_ROOT']).resolve()

GENERATED_ROOTS = (
    'build',
    'install',
    'log',
    'third_party/unitree_ros2/cyclonedds_ws/build',
    'third_party/unitree_ros2/cyclonedds_ws/install',
    'third_party/unitree_ros2/cyclonedds_ws/log',
    'third_party/unitree_sdk2/build',
    'third_party/unitree_sdk2/install',
)


class Reporter:
    def __init__(self):
        self.pass_count = 0
        self.warn_count = 0
        self.fail_count = 0

    def pass_(self, label, detail=''):
        self.pass_count += 1
        self._print('PASS', label, detail)

    def warn(self, label, detail=''):
        self.warn_count += 1
        self._print('WARN', label, detail)

    def fail(self, label, detail=''):
        self.fail_count += 1
        self._print('FAIL', label, detail)

    @staticmethod
    def _print(level, label, detail):
        suffix = f' - {detail}' if detail else ''
        print(f'[{level}] {label}{suffix}')

    def finish(self):
        print()
        print(
            'National preflight summary: '
            f'PASS={self.pass_count} WARN={self.warn_count} '
            f'FAIL={self.fail_count}'
        )
        return 1 if self.fail_count else 0


REPORT = Reporter()


def run_git(*args):
    result = subprocess.run(
        ['git', '-C', str(ROOT), *args],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or 'git command failed')
    return [line for line in result.stdout.splitlines() if line]


def sample(paths, limit=4):
    values = sorted(str(path) for path in paths)
    shown = values[:limit]
    suffix = f' (+{len(values) - limit} more)' if len(values) > limit else ''
    return ', '.join(shown) + suffix


def is_under(path, root):
    return path == root or path.startswith(f'{root}/')


def is_generated_path(path):
    return any(is_under(path, root) for root in GENERATED_ROOTS)


def is_cache_or_venv(path):
    parts = Path(path).parts
    return (
        any(part in {
            '__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache',
            '.venv', 'venv', 'env',
        } for part in parts)
        or path.endswith(('.pyc', '.pyo', '.pyd'))
        or path in {'.coverage', 'coverage.xml'}
        or is_under(path, 'htmlcov')
    )


def check_git_artifacts():
    try:
        tracked_changes = set(run_git(
            'diff', '--diff-filter=ACMRTUXB', '--name-only', 'HEAD',
        ))
        untracked = set(run_git('ls-files', '--others', '--exclude-standard'))
        tracked = set(run_git('ls-files'))
    except RuntimeError as error:
        REPORT.fail('Git artifact policy', str(error))
        return

    candidates = tracked_changes | untracked
    generated = {path for path in candidates if is_generated_path(path)}
    cache_or_venv = {path for path in candidates if is_cache_or_venv(path)}
    media_extensions = {
        '.avi', '.bag', '.db3', '.mcap', '.mkv', '.mov', '.mp4', '.webm',
    }
    media = {
        path for path in candidates
        if Path(path).suffix.lower() in media_extensions
    }
    large_images = set()
    for path in candidates:
        candidate = ROOT / path
        if candidate.suffix.lower() not in {'.bmp', '.jpg', '.jpeg', '.png', '.tif', '.tiff'}:
            continue
        try:
            if candidate.stat().st_size > 5 * 1024 * 1024:
                large_images.add(path)
        except FileNotFoundError:
            continue

    prohibited = generated | cache_or_venv | media | large_images
    if prohibited:
        REPORT.fail(
            'No generated/cache/media files in this change',
            sample(prohibited),
        )
    else:
        REPORT.pass_('No generated/cache/media files in this change')

    historical_generated = {path for path in tracked if is_generated_path(path)}
    historical_cache = {path for path in tracked if is_cache_or_venv(path)}
    if historical_generated or historical_cache:
        REPORT.fail(
            'Historical generated/cache files are already tracked',
            f'generated={len(historical_generated)}, cache_or_venv={len(historical_cache)}; '
            'clean these in a separately reviewed change',
        )
    else:
        REPORT.pass_('No historical generated/cache files are tracked')

    large_tracked = set()
    for path in tracked:
        candidate = ROOT / path
        try:
            if candidate.is_symlink():
                continue
            if candidate.is_file() and candidate.stat().st_size > 5 * 1024 * 1024:
                large_tracked.add(path)
        except OSError:
            continue
    if large_tracked:
        REPORT.warn(
            'Large files are already tracked',
            f'count={len(large_tracked)}; {sample(large_tracked)}',
        )
    else:
        REPORT.pass_('No tracked file exceeds 5 MiB')


def check_gitignore():
    gitignore_path = ROOT / '.gitignore'
    if not gitignore_path.is_file():
        REPORT.fail('.gitignore coverage', '.gitignore is missing')
        return

    ignored_probes = {
        'build/.probe',
        'install/.probe',
        'log/.probe',
        'src/example/__pycache__/probe.pyc',
        'src/example/__pycache__/probe.txt',
        'src/example/module.pyd',
        '.pytest_cache/.probe',
        '.mypy_cache/.probe',
        '.ruff_cache/.probe',
        '.coverage',
        'coverage.xml',
        'htmlcov/index.html',
        '.venv/pyvenv.cfg',
        'venv/pyvenv.cfg',
        'env/pyvenv.cfg',
        'recording.bag',
        'recording.db3',
        'recording.mcap',
        'runtime.log',
        '.DS_Store',
        'Thumbs.db',
        '.vscode/settings.json',
        '.idea/workspace.xml',
        'third_party/unitree_ros2/cyclonedds_ws/build/.probe',
        'third_party/unitree_ros2/cyclonedds_ws/install/.probe',
        'third_party/unitree_ros2/cyclonedds_ws/log/.probe',
        'third_party/unitree_sdk2/build/.probe',
        'third_party/unitree_sdk2/install/.probe',
    }
    missing = []
    probe_errors = []
    for probe in sorted(ignored_probes):
        result = subprocess.run(
            ['git', '-C', str(ROOT), 'check-ignore', '--no-index', '-q', probe],
            check=False,
        )
        if result.returncode == 1:
            missing.append(probe)
        elif result.returncode != 0:
            probe_errors.append(f'{probe}: exit={result.returncode}')

    protected_probes = {
        'src/example/package.xml',
        'src/example/setup.py',
        'src/example/setup.cfg',
        'src/example/launch/example.launch.py',
        'src/example/config/example.yaml',
        'src/example/msg/Example.msg',
        'src/example/action/Example.action',
        'src/example/srv/Example.srv',
        'src/example/resource/example',
        'docs/example.md',
        'scripts/example.sh',
        'third_party/unitree_sdk2/include/unitree/common/log/log_buffer.hpp',
        'third_party/unitree_sdk2/lib/x86_64/libunitree_sdk2.a',
        'third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api/package.xml',
    }
    wrongly_ignored = []
    for probe in sorted(protected_probes):
        result = subprocess.run(
            ['git', '-C', str(ROOT), 'check-ignore', '--no-index', '-q', probe],
            check=False,
        )
        if result.returncode == 0:
            wrongly_ignored.append(probe)
        elif result.returncode != 1:
            probe_errors.append(f'{probe}: exit={result.returncode}')

    if missing or wrongly_ignored or probe_errors:
        detail_parts = []
        if missing:
            detail_parts.append(f'not ignored: {sample(missing)}')
        if wrongly_ignored:
            detail_parts.append(
                f'protected but ignored: {sample(wrongly_ignored)}'
            )
        if probe_errors:
            detail_parts.append(f'probe errors: {sample(probe_errors)}')
        REPORT.fail('.gitignore coverage', '; '.join(detail_parts))
    else:
        REPORT.pass_(
            '.gitignore coverage',
            f'{len(ignored_probes)} generated probes ignored; '
            f'{len(protected_probes)} source probes retained',
        )


def literal_assignment(tree, name):
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
            continue
        try:
            return ast.literal_eval(node.value)
        except (ValueError, TypeError):
            return None
    return None


def check_packages():
    errors = []
    manifests = sorted((ROOT / 'src').glob('*/package.xml'))
    if not manifests:
        REPORT.fail('package.xml/setup.py consistency', 'no package.xml found')
        return

    for manifest in manifests:
        package_dir = manifest.parent
        try:
            xml_root = ET.parse(manifest).getroot()
        except (ET.ParseError, OSError) as error:
            errors.append(f'{manifest.relative_to(ROOT)}: {error}')
            continue

        name = (xml_root.findtext('name') or '').strip()
        version = (xml_root.findtext('version') or '').strip()
        build_type_node = xml_root.find('./export/build_type')
        build_type = (build_type_node.text or '').strip() if build_type_node is not None else ''
        if not name:
            errors.append(f'{manifest.relative_to(ROOT)}: missing package name')
            continue
        if name != package_dir.name:
            errors.append(
                f'{manifest.relative_to(ROOT)}: name={name} does not match directory={package_dir.name}'
            )

        setup_path = package_dir / 'setup.py'
        if build_type == 'ament_python':
            if not setup_path.is_file():
                errors.append(f'{package_dir.relative_to(ROOT)}: ament_python package lacks setup.py')
                continue
            if not (package_dir / 'resource' / name).is_file():
                errors.append(f'{package_dir.relative_to(ROOT)}: missing resource/{name}')
            if not (package_dir / name / '__init__.py').is_file():
                errors.append(f'{package_dir.relative_to(ROOT)}: missing Python package {name}/__init__.py')

            try:
                tree = ast.parse(setup_path.read_text(encoding='utf-8'), str(setup_path))
            except (OSError, SyntaxError) as error:
                errors.append(f'{setup_path.relative_to(ROOT)}: {error}')
                continue
            setup_name = literal_assignment(tree, 'package_name')
            if setup_name != name:
                errors.append(
                    f'{setup_path.relative_to(ROOT)}: package_name={setup_name!r}, manifest={name!r}'
                )
            setup_version = None
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                function_name = getattr(node.func, 'id', '')
                if function_name != 'setup':
                    continue
                for keyword in node.keywords:
                    if keyword.arg == 'version' and isinstance(keyword.value, ast.Constant):
                        setup_version = str(keyword.value.value)
            if setup_version != version:
                errors.append(
                    f'{setup_path.relative_to(ROOT)}: version={setup_version!r}, manifest={version!r}'
                )
        elif build_type == 'ament_cmake':
            cmake_path = package_dir / 'CMakeLists.txt'
            if not cmake_path.is_file():
                errors.append(f'{package_dir.relative_to(ROOT)}: ament_cmake package lacks CMakeLists.txt')
                continue
            cmake_text = cmake_path.read_text(encoding='utf-8')
            project_match = re.search(r'\bproject\s*\(\s*([A-Za-z0-9_]+)', cmake_text)
            if not project_match or project_match.group(1) != name:
                project_name = project_match.group(1) if project_match else None
                errors.append(
                    f'{cmake_path.relative_to(ROOT)}: project={project_name!r}, manifest={name!r}'
                )
        else:
            errors.append(f'{manifest.relative_to(ROOT)}: unsupported/missing build_type={build_type!r}')

    if errors:
        REPORT.fail('package.xml/setup.py consistency', '; '.join(errors[:8]))
    else:
        REPORT.pass_(
            'package.xml/setup.py consistency',
            f'{len(manifests)} packages checked',
        )


def eval_static(node, environment):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        return environment.get(node.id)
    if isinstance(node, ast.List):
        values = [eval_static(value, environment) for value in node.elts]
        return values if all(value is not None for value in values) else None
    if isinstance(node, ast.Tuple):
        values = [eval_static(value, environment) for value in node.elts]
        return tuple(values) if all(value is not None for value in values) else None
    if isinstance(node, ast.Dict):
        keys = [eval_static(key, environment) for key in node.keys]
        values = [eval_static(value, environment) for value in node.values]
        if any(key is None for key in keys) or any(value is None for value in values):
            return None
        return dict(zip(keys, values))
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = eval_static(node.left, environment)
        right = eval_static(node.right, environment)
        if left is not None and right is not None:
            try:
                return left + right
            except TypeError:
                return None
    return None


def python_console_entries(setup_path):
    tree = ast.parse(setup_path.read_text(encoding='utf-8'), str(setup_path))
    environment = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        value = eval_static(node.value, environment)
        for target in node.targets:
            if isinstance(target, ast.Name):
                environment[target.id] = value

    scripts = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or getattr(node.func, 'id', '') != 'setup':
            continue
        for keyword in node.keywords:
            if keyword.arg != 'entry_points':
                continue
            entry_points = eval_static(keyword.value, environment)
            if not isinstance(entry_points, dict):
                continue
            for entry in entry_points.get('console_scripts', []):
                if isinstance(entry, str) and '=' in entry:
                    executable, target = entry.split('=', 1)
                    scripts[executable.strip()] = target.strip()
    return scripts


def python_console_scripts(setup_path):
    return set(python_console_entries(setup_path))


def check_console_targets():
    errors = []
    count = 0
    for setup_path in sorted((ROOT / 'src').glob('*/setup.py')):
        try:
            entries = python_console_entries(setup_path)
        except (OSError, SyntaxError) as error:
            errors.append(f'{setup_path.relative_to(ROOT)}: {error}')
            continue
        for executable, target in entries.items():
            count += 1
            if ':' not in target:
                errors.append(f'{setup_path.relative_to(ROOT)}: {executable} -> {target}')
                continue
            module_name, function_name = target.split(':', 1)
            module_path = setup_path.parent / (module_name.replace('.', '/') + '.py')
            if not module_path.is_file():
                errors.append(
                    f'{setup_path.relative_to(ROOT)}: {executable} module missing: {module_name}'
                )
                continue
            try:
                tree = ast.parse(module_path.read_text(encoding='utf-8'), str(module_path))
            except (OSError, SyntaxError) as error:
                errors.append(f'{module_path.relative_to(ROOT)}: {error}')
                continue
            functions = {
                node.name for node in tree.body
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
            if function_name not in functions:
                errors.append(
                    f'{setup_path.relative_to(ROOT)}: {executable} function missing: {target}'
                )
    if errors:
        REPORT.fail('Console entry point targets', sample(errors, limit=8))
    else:
        REPORT.pass_('Console entry point targets', f'{count} targets checked')


def internal_executables():
    packages = {}
    conditional_packages = {}
    for manifest in sorted((ROOT / 'src').glob('*/package.xml')):
        name = (ET.parse(manifest).getroot().findtext('name') or '').strip()
        executables = set()
        conditional = set()
        setup_path = manifest.parent / 'setup.py'
        if setup_path.is_file():
            executables.update(python_console_scripts(setup_path))
        cmake_path = manifest.parent / 'CMakeLists.txt'
        if cmake_path.is_file():
            text = cmake_path.read_text(encoding='utf-8')
            executables.update(re.findall(
                r'\badd_executable\s*\(\s*([A-Za-z0-9_.+-]+)',
                text,
            ))
            for block in re.findall(
                r'\bif\s*\(\s*unitree_sdk2_FOUND\s*\)(.*?)\belse\s*\(\s*\)',
                text,
                flags=re.DOTALL,
            ):
                conditional.update(re.findall(
                    r'\badd_executable\s*\(\s*([A-Za-z0-9_.+-]+)',
                    block,
                ))
            for block in re.findall(
                r'\binstall\s*\(\s*PROGRAMS(.*?)DESTINATION',
                text,
                flags=re.DOTALL,
            ):
                for script in re.findall(r'([A-Za-z0-9_./+-]+\.(?:py|sh))', block):
                    executables.add(Path(script).name)
        packages[name] = executables
        conditional_packages[name] = conditional
    return packages, conditional_packages


def call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return ''


def keyword_map(call):
    return {keyword.arg: keyword.value for keyword in call.keywords if keyword.arg}


def check_launch_executables():
    try:
        registry, conditional_registry = internal_executables()
    except (OSError, SyntaxError, ET.ParseError) as error:
        REPORT.fail('Launch executable references', f'cannot build executable registry: {error}')
        return

    missing = []
    external = set()
    conditional = set()
    dynamic = []
    absolute_runtime_paths = set()
    checked = 0
    for launch_path in sorted((ROOT / 'src').glob('*/launch/*.py')):
        try:
            tree = ast.parse(launch_path.read_text(encoding='utf-8'), str(launch_path))
        except (OSError, SyntaxError) as error:
            missing.append(f'{launch_path.relative_to(ROOT)}: {error}')
            continue
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Constant)
                and isinstance(node.value, str)
                and node.value.startswith('/home/')
            ):
                absolute_runtime_paths.add(node.value)
            if not isinstance(node, ast.Call):
                continue
            name = call_name(node)
            keywords = keyword_map(node)
            package = None
            executable = None
            if name == 'Node':
                package = eval_static(keywords.get('package'), {})
                executable = eval_static(keywords.get('executable'), {})
                if not isinstance(package, str) or not isinstance(executable, str):
                    dynamic.append(str(launch_path.relative_to(ROOT)))
                    continue
            elif name == 'ExecuteProcess':
                cmd_node = keywords.get('cmd')
                if isinstance(cmd_node, (ast.List, ast.Tuple)) and len(cmd_node.elts) >= 4:
                    prefix = [eval_static(item, {}) for item in cmd_node.elts[:4]]
                    if prefix[:2] == ['ros2', 'run'] and all(isinstance(item, str) for item in prefix[2:]):
                        package, executable = prefix[2], prefix[3]
                    else:
                        continue
                else:
                    continue
            else:
                continue

            checked += 1
            if package in registry:
                if executable not in registry[package]:
                    missing.append(
                        f'{launch_path.relative_to(ROOT)}: {package}/{executable}'
                    )
                elif executable in conditional_registry.get(package, set()):
                    conditional.add(f'{package}/{executable}')
            else:
                external.add(f'{package}/{executable}')

    if missing:
        REPORT.fail('Launch executable references', sample(missing, limit=8))
    else:
        REPORT.pass_(
            'Launch executable references',
            f'{checked} static Node/ros2-run references checked',
        )
    if external:
        REPORT.warn(
            'External launch executables require an installed ROS environment',
            sample(external, limit=8),
        )
    if conditional:
        REPORT.warn(
            'Conditional launch executables require SDK-enabled builds',
            sample(conditional, limit=8),
        )
    if dynamic:
        REPORT.warn(
            'Dynamic launch executable references need runtime validation',
            sample(set(dynamic), limit=8),
        )
    if absolute_runtime_paths:
        REPORT.warn(
            'Absolute robot-side launch paths require runtime validation',
            sample(absolute_runtime_paths, limit=6),
        )


def static_launch_entries(tree):
    entries = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = call_name(node)
        keywords = keyword_map(node)
        package = None
        executable = None
        if name == 'Node':
            package = eval_static(keywords.get('package'), {})
            executable = eval_static(keywords.get('executable'), {})
        elif name == 'ExecuteProcess':
            cmd_node = keywords.get('cmd')
            if isinstance(cmd_node, (ast.List, ast.Tuple)) and len(cmd_node.elts) >= 4:
                prefix = [eval_static(item, {}) for item in cmd_node.elts[:4]]
                if (
                    prefix[:2] == ['ros2', 'run']
                    and all(isinstance(item, str) for item in prefix[2:])
                ):
                    package, executable = prefix[2], prefix[3]
        if isinstance(package, str) and isinstance(executable, str):
            entries.append((package, executable, node))
    return entries


def static_parameter_values(call):
    parameters = keyword_map(call).get('parameters')
    values = {}
    if parameters is None:
        return values
    for node in ast.walk(parameters):
        if not isinstance(node, ast.Dict):
            continue
        for key_node, value_node in zip(node.keys, node.values):
            key = eval_static(key_node, {})
            value = eval_static(value_node, {})
            if isinstance(key, str) and value is not None:
                values.setdefault(key, []).append(value)
    return values


def check_competition_control_authority():
    required_paths = [
        ROOT / 'src' / 'rk_safety' / 'rk_safety' / 'command_mux_core.py',
        ROOT / 'src' / 'rk_safety' / 'rk_safety' / 'command_mux_node.py',
        ROOT / 'src' / 'rk_safety' / 'test' / 'test_command_mux_core.py',
    ]
    missing = [
        path.relative_to(ROOT)
        for path in required_paths
        if not path.is_file()
    ]
    if missing:
        REPORT.fail(
            'Competition control authority files',
            f'missing: {sample(missing)}',
        )
    else:
        REPORT.pass_(
            'Competition control authority files',
            'command mux core, ROS node, and pure-core test are present',
        )

    launch_path = (
        ROOT / 'src' / 'rk_bringup' / 'launch'
        / 'competition_line_nav.launch.py'
    )
    try:
        tree = ast.parse(
            launch_path.read_text(encoding='utf-8'),
            str(launch_path),
        )
    except (OSError, SyntaxError) as error:
        REPORT.fail('Competition control authority launch wiring', str(error))
        REPORT.fail(
            'Competition launch excludes alternate cmd_vel publishers',
            'competition launch could not be inspected',
        )
        return

    entries = static_launch_entries(tree)

    def matching_calls(package, executable):
        return [
            call for entry_package, entry_executable, call in entries
            if entry_package == package and entry_executable == executable
        ]

    wiring_errors = []
    mux_calls = matching_calls('rk_safety', 'command_mux_node')
    mission_calls = matching_calls(
        'rk_mission',
        'line_course_mission_node',
    )
    forwarder_calls = matching_calls(
        'rk_go2_sdk_bridge',
        'cmd_vel_udp_forwarder.py',
    )
    expected_counts = (
        ('rk_safety/command_mux_node', mux_calls),
        ('rk_mission/line_course_mission_node', mission_calls),
        ('rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py', forwarder_calls),
    )
    for label, calls in expected_counts:
        if len(calls) != 1:
            wiring_errors.append(f'{label} launch count={len(calls)}')

    parameter_checks = (
        (
            'line_course_mission cmd_vel_topic',
            mission_calls,
            'cmd_vel_topic',
            '/control/mission_cmd',
        ),
        (
            'command_mux output_cmd_topic',
            mux_calls,
            'output_cmd_topic',
            '/navigation/cmd_vel',
        ),
        (
            'command_mux enable_estop_service',
            mux_calls,
            'enable_estop_service',
            True,
        ),
        (
            'command_mux estop_service_name',
            mux_calls,
            'estop_service_name',
            '/safety/estop',
        ),
        (
            'UDP forwarder cmd_vel_topic',
            forwarder_calls,
            'cmd_vel_topic',
            '/navigation/cmd_vel',
        ),
    )
    for label, calls, parameter, expected in parameter_checks:
        if len(calls) != 1:
            continue
        values = static_parameter_values(calls[0]).get(parameter, [])
        if expected not in values:
            wiring_errors.append(
                f'{label} must be statically set to {expected}'
            )

    if wiring_errors:
        REPORT.fail(
            'Competition control authority launch wiring',
            '; '.join(wiring_errors),
        )
    else:
        REPORT.pass_(
            'Competition control authority launch wiring',
            'mission -> /control/mission_cmd -> command_mux -> '
            '/navigation/cmd_vel -> UDP forwarder',
        )

    background_script = (
        ROOT / 'src' / 'rk_bringup' / 'scripts' / 'start_line_system.sh'
    )
    background_requirements = {
        'ros2 run rk_safety command_mux_node':
            'command mux process',
        'cmd_vel_topic:=/control/mission_cmd':
            'line-course mission candidate output',
        'mission_cmd_topic:=/control/mission_cmd':
            'command mux mission input',
        'enable_estop_service:=true':
            'command mux SetBool estop service',
        'estop_service_name:=/safety/estop':
            'command mux estop service name',
        'output_cmd_topic:=/navigation/cmd_vel':
            'command mux final output',
    }
    try:
        background_text = background_script.read_text(encoding='utf-8')
    except OSError as error:
        REPORT.fail('Background line-system control wiring', str(error))
    else:
        missing_background = [
            label
            for snippet, label in background_requirements.items()
            if snippet not in background_text
        ]
        if missing_background:
            REPORT.fail(
                'Background line-system control wiring',
                'missing: {}'.format(', '.join(missing_background)),
            )
        else:
            REPORT.pass_(
                'Background line-system control wiring',
                'start_line_system uses the same mission -> mux -> final path',
            )

    forbidden_exact = {
        'cmd_vel_speed_sweep_node',
        'gait_basic_test_node',
        'gait_control_node',
        'keyboard_route_node',
        'mission_state_machine_node',
        'obstacle_direct_route_node',
        'safety_node',
        'two_step_walk_test_node',
    }
    forbidden = []
    unexpected_final_topic = []
    allowed_final_topic = {
        ('rk_safety', 'command_mux_node'),
        ('rk_go2_sdk_bridge', 'cmd_vel_udp_forwarder.py'),
    }
    for package, executable, call in entries:
        lowered = executable.lower()
        if (
            executable in forbidden_exact
            or 'keyboard' in lowered
            or 'mock' in lowered
            or 'obstacle_direct' in lowered
        ):
            forbidden.append(f'{package}/{executable}')
        parameter_values = static_parameter_values(call)
        final_topic_parameters = [
            name
            for name, values in parameter_values.items()
            if (
                name in {'cmd_vel_topic', 'output_cmd_topic'}
                and '/navigation/cmd_vel' in values
            )
        ]
        if (
            final_topic_parameters
            and (package, executable) not in allowed_final_topic
        ):
            unexpected_final_topic.append(
                f'{package}/{executable} '
                f'({", ".join(final_topic_parameters)})'
            )

    if forbidden or unexpected_final_topic:
        details = []
        if forbidden:
            details.append(
                f'forbidden nodes: {sample(set(forbidden), limit=8)}'
            )
        if unexpected_final_topic:
            details.append(
                'other final-topic wiring: '
                f'{sample(set(unexpected_final_topic), limit=8)}'
            )
        REPORT.fail(
            'Competition launch excludes alternate cmd_vel publishers',
            '; '.join(details),
        )
    else:
        REPORT.pass_(
            'Competition launch excludes alternate cmd_vel publishers',
            'no legacy safety/keyboard/mock/obstacle-direct/gait/test '
            'publisher is started; final topic appears only on mux output '
            'and UDP input',
        )

    test_path = (
        ROOT / 'src' / 'rk_safety' / 'test'
        / 'test_command_mux_core.py'
    )
    if not test_path.is_file():
        REPORT.fail(
            'Command mux pure-core tests',
            f'missing: {test_path.relative_to(ROOT)}',
        )
        return

    environment = os.environ.copy()
    safety_source = str(ROOT / 'src' / 'rk_safety')
    existing_pythonpath = environment.get('PYTHONPATH')
    environment['PYTHONPATH'] = (
        safety_source
        if not existing_pythonpath
        else safety_source + os.pathsep + existing_pythonpath
    )
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
    try:
        with tempfile.TemporaryDirectory(
            prefix='rk-command-mux-preflight-',
        ) as temporary_directory:
            result = subprocess.run(
                [
                    sys.executable,
                    '-m',
                    'pytest',
                    str(test_path),
                    '-q',
                    '-p',
                    'no:cacheprovider',
                ],
                check=False,
                cwd=temporary_directory,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=120,
            )
    except (OSError, subprocess.TimeoutExpired) as error:
        REPORT.fail('Command mux pure-core tests', str(error))
        return

    output_lines = [
        line.strip()
        for line in (result.stdout + '\n' + result.stderr).splitlines()
        if line.strip()
    ]
    detail = ' | '.join(output_lines[-4:]) or f'exit={result.returncode}'
    if result.returncode == 0:
        REPORT.pass_('Command mux pure-core tests', detail)
    else:
        REPORT.fail('Command mux pure-core tests', detail)


def check_competition_estop_contract():
    errors = []
    package_path = ROOT / 'src' / 'rk_safety' / 'package.xml'
    try:
        package_root = ET.parse(package_path).getroot()
    except (ET.ParseError, OSError) as error:
        errors.append(f'{package_path.relative_to(ROOT)}: {error}')
    else:
        exec_dependencies = {
            (node.text or '').strip()
            for node in package_root.findall('exec_depend')
        }
        if 'std_srvs' not in exec_dependencies:
            errors.append('rk_safety/package.xml lacks exec_depend std_srvs')

    node_path = (
        ROOT / 'src' / 'rk_safety' / 'rk_safety'
        / 'command_mux_node.py'
    )
    try:
        node_tree = ast.parse(
            node_path.read_text(encoding='utf-8'),
            str(node_path),
        )
    except (OSError, SyntaxError) as error:
        errors.append(f'{node_path.relative_to(ROOT)}: {error}')
        node_tree = None

    if node_tree is not None:
        imports_set_bool = any(
            isinstance(node, ast.ImportFrom)
            and node.module == 'std_srvs.srv'
            and any(alias.name == 'SetBool' for alias in node.names)
            for node in ast.walk(node_tree)
        )
        if not imports_set_bool:
            errors.append('command_mux_node does not import std_srvs/SetBool')

        command_mux_class = next(
            (
                node for node in node_tree.body
                if isinstance(node, ast.ClassDef)
                and node.name == 'CommandMuxNode'
            ),
            None,
        )
        if command_mux_class is None:
            errors.append('CommandMuxNode class is missing')
        else:
            methods = {
                node.name: node
                for node in command_mux_class.body
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            }

            parameter_defaults = {}
            for call in (
                node for node in ast.walk(command_mux_class)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and len(node.args) >= 2
            ):
                name = eval_static(call.args[0], {})
                value = eval_static(call.args[1], {})
                if isinstance(name, str):
                    parameter_defaults[name] = value
            expected_defaults = {
                'enable_estop_service': True,
                'estop_service_name': '/safety/estop',
            }
            for name, expected in expected_defaults.items():
                if parameter_defaults.get(name) != expected:
                    errors.append(
                        'command_mux_node default {} must be {!r}'.format(
                            name,
                            expected,
                        )
                    )

            service_calls = [
                call for call in ast.walk(command_mux_class)
                if isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'create_service'
            ]
            has_set_bool_service = any(
                len(call.args) >= 3
                and isinstance(call.args[0], ast.Name)
                and call.args[0].id == 'SetBool'
                and isinstance(call.args[2], ast.Attribute)
                and call.args[2].attr == '_on_estop_service'
                for call in service_calls
            )
            if not has_set_bool_service:
                errors.append(
                    'command_mux_node lacks SetBool/_on_estop_service'
                )

            subscription_calls = [
                call for call in ast.walk(command_mux_class)
                if isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'create_subscription'
            ]
            has_estop_subscription = any(
                len(call.args) >= 3
                and isinstance(call.args[0], ast.Name)
                and call.args[0].id == 'Bool'
                and isinstance(call.args[2], ast.Attribute)
                and call.args[2].attr == '_on_estop'
                for call in subscription_calls
            )
            if not has_estop_subscription:
                errors.append(
                    'command_mux_node lacks Bool/_on_estop subscription'
                )

            def method_calls(method_name, called_name):
                method = methods.get(method_name)
                if method is None:
                    return False
                return any(
                    isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == called_name
                    for call in ast.walk(method)
                )

            for callback in ('_on_estop', '_on_estop_service'):
                if not method_calls(callback, '_transition_estop'):
                    errors.append(
                        '{} must call shared _transition_estop'.format(
                            callback
                        )
                    )
            if not method_calls('_transition_estop', 'set_estop'):
                errors.append(
                    '_transition_estop must update CommandMuxCore.set_estop'
                )

    if errors:
        REPORT.fail(
            'Competition SetBool estop contract',
            '; '.join(errors[:10]),
        )
    else:
        REPORT.pass_(
            'Competition SetBool estop contract',
            'std_srvs dependency and shared topic/service transition checked',
        )

    stop_path = (
        ROOT / 'src' / 'rk_bringup' / 'scripts'
        / 'stop_line_system.sh'
    )
    try:
        stop_text = stop_path.read_text(encoding='utf-8')
    except OSError as error:
        REPORT.fail('Line-system shutdown authority', str(error))
    else:
        shutdown_errors = []
        fallback_match = re.search(
            r'(?ms)^emergency_cmd_vel_fallback\(\)\s*\{\s*\n'
            r'(?P<body>.*?)^\}',
            stop_text,
        )
        direct_publish = re.compile(
            r'\bros2[ \t]+topic[ \t]+pub(?:[ \t]+--once)?'
            r'[ \t]+/navigation/cmd_vel\b'
        )
        all_direct_count = len(direct_publish.findall(stop_text))
        normal_path_text = stop_text
        if fallback_match is None:
            shutdown_errors.append(
                'emergency_cmd_vel_fallback function is missing'
            )
        else:
            fallback_body = fallback_match.group('body')
            outside_fallback = (
                stop_text[:fallback_match.start()]
                + stop_text[fallback_match.end():]
            )
            normal_path_text = outside_fallback
            fallback_direct_count = len(
                direct_publish.findall(fallback_body)
            )
            outside_direct_count = len(
                direct_publish.findall(outside_fallback)
            )
            if 'EMERGENCY FALLBACK' not in fallback_body:
                shutdown_errors.append(
                    'direct fallback lacks EMERGENCY FALLBACK warning'
                )
            if fallback_direct_count != 1:
                shutdown_errors.append(
                    'fallback direct cmd_vel publish count={}'.format(
                        fallback_direct_count
                    )
                )
            if outside_direct_count:
                shutdown_errors.append(
                    'direct cmd_vel publish exists outside emergency fallback'
                )

        normal_requirements = {
            '/safety/estop': 'SetBool service name',
            'std_srvs/srv/SetBool': 'SetBool service type',
            'ros2 service call': 'normal estop service call',
            '{data: true}': 'estop=true request',
        }
        missing_normal = [
            label
            for snippet, label in normal_requirements.items()
            if snippet not in normal_path_text
        ]
        if missing_normal:
            shutdown_errors.append(
                'missing normal path: {}'.format(
                    ', '.join(missing_normal)
                )
            )
        if all_direct_count != 1:
            shutdown_errors.append(
                'shutdown direct cmd_vel publish total={}'.format(
                    all_direct_count
                )
            )

        if shutdown_errors:
            REPORT.fail(
                'Line-system shutdown authority',
                '; '.join(shutdown_errors),
            )
        else:
            REPORT.pass_(
                'Line-system shutdown authority',
                'normal path calls SetBool; one direct zero publish is '
                'isolated in the marked EMERGENCY FALLBACK',
            )

    authority_docs = [
        ROOT / 'docs' / 'CONTROL_AUTHORITY.md',
        ROOT / 'src' / 'rk_bringup' / 'README_line_system.md',
    ]
    documentation_errors = []
    unique_publisher = re.compile(
        r'(唯一.{0,80}发布者|only.{0,80}publisher|'
        r'unique.{0,80}(?:publisher|output))',
        flags=re.IGNORECASE | re.DOTALL,
    )
    for path in authority_docs:
        try:
            text = path.read_text(encoding='utf-8')
        except OSError as error:
            documentation_errors.append(
                f'{path.relative_to(ROOT)}: {error}'
            )
            continue
        relative = path.relative_to(ROOT)
        required = {
            'command_mux_node': 'normal mux owner',
            '/navigation/cmd_vel': 'final velocity topic',
            'std_srvs/srv/SetBool': 'normal SetBool stop path',
            'stop_line_system.sh': 'shutdown entry point',
            'EMERGENCY FALLBACK': 'explicit fallback marker',
        }
        missing = [
            label for snippet, label in required.items()
            if snippet not in text
        ]
        if not unique_publisher.search(text):
            missing.append('normal unique-publisher statement')
        lowered = text.lower()
        if '正常' not in text and 'normal' not in lowered:
            missing.append('normal-path label')
        if '直接' not in text and 'direct' not in lowered:
            missing.append('direct-fallback explanation')
        if missing:
            documentation_errors.append(
                '{} missing: {}'.format(relative, ', '.join(missing))
            )

    if documentation_errors:
        REPORT.fail(
            'Control authority fallback documentation',
            '; '.join(documentation_errors),
        )
    else:
        REPORT.pass_(
            'Control authority fallback documentation',
            'authoritative docs distinguish normal mux ownership from the '
            'marked direct-publish emergency fallback',
        )


def check_yaml():
    yaml_paths = sorted((ROOT / 'src').glob('**/*.yaml'))
    yaml_paths.extend(sorted((ROOT / 'src').glob('**/*.yml')))
    try:
        import yaml
    except ImportError:
        REPORT.fail('YAML parse', 'PyYAML is not installed')
        return

    errors = []
    empty = []
    for path in yaml_paths:
        try:
            with path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
            if data is None:
                empty.append(str(path.relative_to(ROOT)))
        except (OSError, yaml.YAMLError) as error:
            errors.append(f'{path.relative_to(ROOT)}: {error}')
    if errors:
        REPORT.fail('YAML parse', sample(errors, limit=6))
    else:
        REPORT.pass_('YAML parse', f'{len(yaml_paths)} files checked')
    if empty:
        REPORT.warn('Empty YAML configuration', sample(empty, limit=8))


def check_python_syntax():
    paths = sorted((ROOT / 'src').glob('**/*.py'))
    paths.extend(sorted((ROOT / 'scripts').glob('**/*.py')))
    errors = []
    for path in paths:
        try:
            source = path.read_text(encoding='utf-8')
            compile(source, str(path), 'exec')
        except (OSError, SyntaxError, UnicodeError) as error:
            errors.append(f'{path.relative_to(ROOT)}: {error}')
    if errors:
        REPORT.fail('Python syntax', sample(errors, limit=6))
    else:
        REPORT.pass_('Python syntax', f'{len(paths)} files checked without writing .pyc')


def check_shell_syntax():
    try:
        shell_paths = set(run_git('ls-files', '--', '*.sh'))
        shell_paths.update(run_git(
            'ls-files', '--others', '--exclude-standard', '--', '*.sh',
        ))
    except RuntimeError as error:
        REPORT.fail('Shell syntax', str(error))
        return

    paths = [ROOT / path for path in sorted(shell_paths)]
    errors = []
    for path in paths:
        try:
            result = subprocess.run(
                ['bash', '-n', str(path)],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as error:
            errors.append(f'bash unavailable: {error}')
            break
        if result.returncode != 0:
            detail = result.stderr.strip() or f'exit={result.returncode}'
            errors.append(f'{path.relative_to(ROOT)}: {detail}')
    if errors:
        REPORT.fail('Shell syntax', sample(errors, limit=6))
    else:
        REPORT.pass_(
            'Shell syntax',
            f'{len(paths)} tracked/current files checked with bash -n',
        )


def check_portable_paths():
    candidates = []
    readme_path = ROOT / 'README.md'
    if readme_path.is_file():
        candidates.append(readme_path)
    docs_root = ROOT / 'docs'
    if docs_root.exists():
        candidates.extend(path for path in docs_root.rglob('*.md') if path.is_file())
    scripts_root = ROOT / 'scripts'
    if scripts_root.exists():
        candidates.extend(path for path in scripts_root.rglob('*') if path.is_file())

    file_uri = 'file:' + '///'
    unix_home = re.compile(r'/(?:home|Users)/([^/\s`\"\'<>]+)')
    windows_home = re.compile(r'[A-Za-z]:[\\/]+Users[\\/]+[^\\/\s`\"\'<>]+')
    personal = []
    robot_runtime = []
    checked = 0
    for path in sorted(set(candidates)):
        try:
            if path.stat().st_size > 2 * 1024 * 1024:
                continue
            text = path.read_text(encoding='utf-8')
        except (OSError, UnicodeError):
            continue
        checked += 1
        for number, line in enumerate(text.splitlines(), 1):
            location = f'{path.relative_to(ROOT)}:{number}'
            if file_uri in line or windows_home.search(line):
                personal.append(location)
            for match in unix_home.finditer(line):
                username = match.group(1)
                if username == 'unitree':
                    robot_runtime.append(location)
                else:
                    personal.append(location)

    if personal:
        REPORT.fail(
            'Portable documentation and scripts',
            f'personal path or local file URI: {sample(set(personal), limit=8)}',
        )
    else:
        REPORT.pass_(
            'Portable documentation and scripts',
            f'{checked} files checked',
        )
    if robot_runtime:
        REPORT.warn(
            'Robot-side absolute paths require deployment validation',
            sample(set(robot_runtime), limit=8),
        )

    source_runtime = []
    source_root = ROOT / 'src'
    source_suffixes = {'.py', '.sh', '.yaml', '.yml'}
    if source_root.exists():
        for path in source_root.rglob('*'):
            if not path.is_file() or path.suffix.lower() not in source_suffixes:
                continue
            try:
                text = path.read_text(encoding='utf-8')
            except (OSError, UnicodeError):
                continue
            for number, line in enumerate(text.splitlines(), 1):
                if unix_home.search(line) or windows_home.search(line):
                    source_runtime.append(f'{path.relative_to(ROOT)}:{number}')
    if source_runtime:
        REPORT.warn(
            'Source/config absolute paths require environment validation',
            sample(set(source_runtime), limit=8),
        )


def check_interfaces():
    expected = [
        'msg/LineTrack.msg',
        'msg/SpecialTargetDetection.msg',
        'msg/SignDetection.msg',
        'msg/SignDetectionArray.msg',
        'msg/ItemTag.msg',
        'msg/ItemTagArray.msg',
        'action/ExecuteMotion.action',
        'action/ExecuteArmTask.action',
        'action/RunMission.action',
    ]
    interface_root = ROOT / 'src' / 'rk_interfaces'
    cmake_path = interface_root / 'CMakeLists.txt'
    cmake_text = cmake_path.read_text(encoding='utf-8') if cmake_path.is_file() else ''
    missing = []
    for relative in expected:
        if not (interface_root / relative).is_file():
            missing.append(relative)
        elif f'"{relative}"' not in cmake_text:
            missing.append(f'{relative} (not registered in CMakeLists.txt)')
    if missing:
        REPORT.fail('Critical ROS interfaces', sample(missing, limit=9))
    else:
        REPORT.pass_('Critical ROS interfaces', f'{len(expected)} files present and registered')


def check_conflict_markers():
    matches = []
    pattern = re.compile(r'^(<<<<<<<|=======|>>>>>>>)')
    for root_name in ('src', 'scripts', 'docs'):
        base = ROOT / root_name
        if not base.exists():
            continue
        for path in base.rglob('*'):
            if not path.is_file() or path.suffix.lower() in {'.png', '.jpg', '.jpeg'}:
                continue
            try:
                for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
                    if pattern.match(line):
                        matches.append(f'{path.relative_to(ROOT)}:{number}')
            except (OSError, UnicodeError):
                continue
    if matches:
        REPORT.fail('Merge conflict markers', sample(matches, limit=8))
    else:
        REPORT.pass_('Merge conflict markers')


print(f'National Competition preflight: {ROOT}')
try:
    branch = run_git('branch', '--show-current')
    branch_name = branch[0] if branch else '(detached HEAD)'
    if branch_name == 'master':
        REPORT.warn('Audit branch', 'currently on master; do not commit audit work here')
    else:
        REPORT.pass_('Audit branch', branch_name)
except RuntimeError as error:
    REPORT.fail('Audit branch', str(error))

check_git_artifacts()
check_gitignore()
check_packages()
check_console_targets()
check_launch_executables()
check_competition_control_authority()
check_competition_estop_contract()
check_yaml()
check_python_syntax()
check_shell_syntax()
check_portable_paths()
check_interfaces()
check_conflict_markers()
sys.exit(REPORT.finish())
PY
