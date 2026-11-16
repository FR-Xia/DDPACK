"""Package or verify derivative audit data; does not run models or solvers.

Copyright (c) 2026 Feiran Xia
Beijing Institute of Technology, China
SPDX-License-Identifier: BSD-3-Clause
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import shutil

ROOT = Path(__file__).resolve().parents[1]
CASES = {'vibration', 'planar', 'terminal_defect', 'obstacle_2d', 'obstacle_3d',
         'reactor', 'pendulum', 'thermal', 'storage'}
WIDTHS = {1e-4, 1e-5, 1e-6}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def audit_pair(matlab, cpp):
    counts = {}
    for language, directory in [('matlab', matlab), ('cpp', cpp)]:
        summary = read_json(directory / 'summary.json')
        rows = read_csv(directory / 'derivative_checks.csv')
        assert summary['success'] and summary['failed_checks'] == 0, language
        assert summary['reports'] == 81 and summary['formal_cases'] == 9, language
        assert len(rows) == summary['checks'], (language, 'row count')
        unique = set()
        groups = set()
        for row in rows:
            case, sample, width = row['case'], int(row['sample']), float(row['difference_step'])
            assert case in CASES and sample in {1, 2, 3} and width in WIDTHS
            key = case, sample, width, row['check']
            assert key not in unique, (language, 'duplicate check', key)
            unique.add(key)
            groups.add(key[:3])
            assert row['passed'].lower() in {'1', 'true'}, (language, key)
            assert row['category'] in {'derivative', 'value_contract'}
            for field in ['max_absolute_error', 'max_scaled_error', 'max_tolerance_ratio']:
                value = float(row[field])
                assert math.isfinite(value) and value >= 0, (language, key, field)
            assert float(row['max_tolerance_ratio']) <= 1, (language, key)
        assert len(groups) == 81
        counts[language] = len(rows)
    native_points = {(r['case'], r['sample'], r['difference_step']): r
                     for r in read_json(cpp / 'points.json')}
    matlab_points = read_json(matlab / 'reports.json')
    assert len(native_points) == len(matlab_points) == 81
    vector = lambda value: value if isinstance(value, list) else [value]
    for report in matlab_points:
        key = report['case'], report['sample'], report['options']['DifferenceStep']
        other = native_points[key]
        for field in ['state', 'control']:
            a, b = vector(report[field]), vector(other[field])
            assert len(a) == len(b)
            assert all(math.isfinite(x) and math.isfinite(y) and abs(x-y) <= 1e-12
                       for x, y in zip(a, b)), (key, field)
        assert report['success'] and report['failed_count'] == 0
    return counts


def source_digest(file):
    path = (ROOT / file).resolve()
    assert path.is_relative_to(ROOT) and path.is_file(), ('source path', file)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, default=ROOT/'experiments/derivative_results')
    parser.add_argument('--matlab', type=Path)
    parser.add_argument('--cpp', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if any(value is not None for value in [args.matlab, args.cpp, args.output]):
        assert all(value is not None for value in [args.matlab, args.cpp, args.output]), 'Supply all three packaging arguments'
        assert not args.output.exists(), 'Choose a fresh output directory'
        audit_pair(args.matlab, args.cpp)
        # Bind current sources only after validating the supplied raw reports.
        # Re-run both audit programs before packaging after source changes.
        sources = read_csv(ROOT/'docs/source_licenses/source_license_audit.csv')
        files = {r['file']: r['sha256'] for r in sources}
        files['.github/workflows/ci.yml'] = source_digest('.github/workflows/ci.yml')
        for file, digest in files.items():
            assert source_digest(file) == digest, ('Stale source inventory', file)
        args.output.mkdir(parents=True)
        for language, directory, filenames in [
            ('matlab', args.matlab, ['summary.json', 'derivative_checks.csv', 'reports.json']),
            ('cpp', args.cpp, ['summary.json', 'derivative_checks.csv', 'points.json'])]:
            target = args.output/language
            target.mkdir()
            for filename in filenames:
                shutil.copyfile(directory/filename, target/filename)
        with (args.output/'source_sha256.csv').open('w', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['file', 'sha256'])
            writer.writerows(sorted(files.items()))
        args.results = args.output
    counts = audit_pair(args.results/'matlab', args.results/'cpp')
    sources = read_csv(args.results/'source_sha256.csv')
    assert sources and len({r['file'] for r in sources}) == len(sources)
    for row in sources:
        assert source_digest(row['file']) == row['sha256'], ('Stale derivative evidence', row['file'])
    print(f"Derivative evidence PASS: {counts['matlab']} MATLAB checks, {counts['cpp']} C++ checks, "
          f"81 matched probes per language, {len(sources)} source files.")


if __name__ == '__main__':
    main()
