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
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(os.environ['RK_PREFLIGHT_ROOT']).resolve()


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


def has_generated_segment(path):
    return any(part in {'build', 'install', 'log'} for part in Path(path).parts)


def check_git_artifacts():
    try:
        tracked_changes = set(run_git('diff', '--name-only', 'HEAD'))
        untracked = set(run_git('ls-files', '--others', '--exclude-standard'))
        tracked = set(run_git('ls-files'))
    except RuntimeError as error:
        REPORT.fail('Git artifact policy', str(error))
        return

    candidates = tracked_changes | untracked
    generated = {path for path in candidates if has_generated_segment(path)}
    cache_or_venv = {
        path for path in candidates
        if '__pycache__' in Path(path).parts
        or '.pytest_cache' in Path(path).parts
        or Path(path).parts[:1] == ('.venv',)
        or path.endswith(('.pyc', '.pyo'))
    }
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

    historical_generated = {path for path in tracked if has_generated_segment(path)}
    historical_cache = {
        path for path in tracked
        if '__pycache__' in Path(path).parts
        or '.pytest_cache' in Path(path).parts
        or Path(path).parts[:1] == ('.venv',)
        or path.endswith(('.pyc', '.pyo'))
    }
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
    for manifest in sorted((ROOT / 'src').glob('*/package.xml')):
        name = (ET.parse(manifest).getroot().findtext('name') or '').strip()
        executables = set()
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
                r'\binstall\s*\(\s*PROGRAMS(.*?)DESTINATION',
                text,
                flags=re.DOTALL,
            ):
                for script in re.findall(r'([A-Za-z0-9_./+-]+\.(?:py|sh))', block):
                    executables.add(Path(script).name)
        packages[name] = executables
    return packages


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
        registry = internal_executables()
    except (OSError, SyntaxError, ET.ParseError) as error:
        REPORT.fail('Launch executable references', f'cannot build executable registry: {error}')
        return

    missing = []
    external = set()
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
    sdk_bridge_cmake = ROOT / 'src' / 'rk_go2_sdk_bridge' / 'CMakeLists.txt'
    if sdk_bridge_cmake.is_file() and 'if(unitree_sdk2_FOUND)' in sdk_bridge_cmake.read_text(encoding='utf-8'):
        REPORT.warn(
            'Conditional SDK executables',
            'rk_go2_sdk_bridge SDK2 targets exist only when unitree_sdk2 is found',
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
check_packages()
check_console_targets()
check_launch_executables()
check_yaml()
check_python_syntax()
check_interfaces()
check_conflict_markers()
sys.exit(REPORT.finish())
PY
