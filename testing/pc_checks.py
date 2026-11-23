import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
APP = os.path.join(ROOT, 'app_program')
MARK = re.compile(r'^/\* ======== from (\S+) ======== \*/$')

STRICT = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-Wconversion', '-Wshadow', '-pedantic']

CHECKS = {
    's3': {
        'headers': [('subsystem3/subsystem3.h', ['line_cfg.h', 's3_ir.h', 's3_line.h', 's3_pulse.h', 's3_state.h', 's3_check.h'])],
        'sources': [('subsystem3/subsystem3.c', ['s3_ir.c', 's3_line.c', 's3_pulse.c', 's3_state.c']),
                    ('subsystem3/subsystem3_test.c', ['s3_check.c'])],
        'files': ['testing/subsystem3/s3_main.c', 'testing/subsystem3/s3_main.h'],
        'cflags': STRICT + ['-fsanitize=address,undefined', '-DS3_HOST_TEST'],
        'build': ['s3_ir.c', 's3_line.c', 's3_pulse.c', 's3_state.c', 's3_check.c', 's3_main.c'],
    },
    'line': {
        'headers': [('subsystem3/subsystem3.h', ['line_cfg.h', 's3_ir.h', 's3_line.h', 'line_follow.h', 'line_check.h'])],
        'sources': [('subsystem3/subsystem3.c', ['s3_ir.c', 's3_line.c', 'line_follow.c']),
                    ('subsystem3/subsystem3_test.c', ['line_check.c'])],
        'cflags': STRICT + ['-DLF_HOST_TEST'],
        'build': ['s3_ir.c', 's3_line.c', 'line_follow.c', 'line_check.c'],
    },
    'barcode': {
        'headers': [('subsystem3/subsystem3.h', ['line_cfg.h', 'barcode_map.h', 's3_pulse.h', 's3_code39.h', 'bc_reader.h', 'bc_rec.h', 'bc_check.h'])],
        'sources': [('subsystem3/subsystem3.c', ['s3_code39.c', 's3_pulse.c', 'bc_reader.c', 'bc_rec.c']),
                    ('subsystem3/subsystem3_test.c', ['bc_check.c'])],
        'cflags': STRICT + ['-DBC_HOST_TEST'],
        'build': ['s3_code39.c', 's3_pulse.c', 'bc_reader.c', 'bc_rec.c', 'bc_check.c'],
    },
    'lift': {
        'headers': [('subsystem4/imu.h', ['app_fields.h', 'app_types.h', 'imu_cfg.h', 'imu_logic.h'])],
        'sources': [('subsystem4/subsystem4.c', ['imu_logic.c'])],
        'files': ['testing/subsystem4/lift_check.c'],
        'cflags': ['-std=c11', '-Wall', '-Wextra', '-DLIFT_HOST_TEST'],
        'libs': ['-lm'],
        'build': ['lift_check.c', 'imu_logic.c'],
    },
    'hump': {
        'headers': [('subsystem4/imu.h', ['app_fields.h', 'app_types.h', 'imu_cfg.h', 'imu_logic.h'])],
        'sources': [('subsystem4/subsystem4.c', ['imu_logic.c'])],
        'files': ['testing/subsystem4/hump_check.c'],
        'cflags': ['-std=c11', '-Wall', '-Wextra', '-DLIFT_HOST_TEST'],
        'libs': ['-lm'],
        'build': ['hump_check.c', 'imu_logic.c'],
    },
    'bypass': {
        'headers': [('vehicle_ctrl.h', ['vc_bypass.h'])],
        'sources': [('vehicle_ctrl.c', ['vc_bypass.c'])],
        'files': ['testing/vehicle_ctrl/vc_bypass_check.c'],
        'cflags': STRICT + ['-DVC_BYP_HOST_TEST'],
        'libs': ['-lm'],
        'build': ['vc_bypass.c', 'vc_bypass_check.c'],
        'args': ['-n', '8'],
    },
    'telemetry': {
        'headers': [('subsystem1/subsystem1.h', ['bitpack.h', 'telem_codec.h'])],
        'sources': [('subsystem1/subsystem1.c', ['bitpack.c', 'telem_codec.c'])],
        'incdirs': ['app_program/subsystem1'],
        'files': ['testing/subsystem1/host/test_bitpack.c',
                  'testing/subsystem1/host/test_codec.c',
                  'testing/subsystem1/host/test_format.c'],
        'cflags': ['-Wall', '-Wextra'],
        'programs': {
            'test_bitpack': ['test_bitpack.c', 'bitpack.c'],
            'test_codec': ['test_codec.c', 'bitpack.c', 'telem_codec.c'],
            'test_format': ['test_format.c'],
        },
    },
}

EXTRACT = {
    'bringup': {
        'headers': [('subsystem4/imu.h', ['app_fields.h', 'app_types.h', 'imu_cfg.h', 'imu.h', 'imu_logic.h', 'imu_port.h'])],
        'sources': [('subsystem4/subsystem4.c', ['imu.c', 'imu_logic.c'])],
        'files': ['app_program/subsystem4/imu.h'],
        'to': os.path.join(HERE, 'subsystem4'),
    },
}


def sections(path):
    lines = open(path, newline='').read().replace('\r\n', '\n').split('\n')
    starts = [(i, MARK.match(l).group(1)) for i, l in enumerate(lines) if MARK.match(l)]
    out = {}
    for n, (i, name) in enumerate(starts):
        end = starts[n + 1][0] if n + 1 < len(starts) else len(lines)
        if n + 1 == len(starts) and path.endswith('.h'):
            guard_end = None
            depth = 0
            for k in range(i + 1, end):
                if re.match(r'\s*#\s*if', lines[k]):
                    depth += 1
                elif re.match(r'\s*#\s*endif', lines[k]):
                    depth -= 1
                    if depth == 0:
                        guard_end = k
            if guard_end is not None:
                end = guard_end + 1
        out[name] = '\n'.join(lines[i + 1:end]).rstrip('\n') + '\n'
    return out


def write_set(spec, dest):
    order = []
    for hfile, names in spec['headers']:
        secs = sections(os.path.join(APP, hfile))
        for n in names:
            open(os.path.join(dest, n), 'w').write(secs[n])
            order.append(n)
    with open(os.path.join(dest, 'pc_all.h'), 'w') as f:
        for n in order:
            f.write('#include "%s"\n' % n)
    for cfile, names in spec['sources']:
        secs = sections(os.path.join(APP, cfile))
        for n in names:
            open(os.path.join(dest, n), 'w').write(secs[n])
    for rel in spec.get('files', []):
        shutil.copy(os.path.join(ROOT, rel), dest)
    for f in os.listdir(dest):
        if not f.endswith(('.c', '.h')):
            continue
        for inc in re.findall(r'#\s*include\s+"([^"/]+)"', open(os.path.join(dest, f)).read()):
            if not os.path.exists(os.path.join(dest, inc)) and re.match(r'(subsystem\d|vehicle_ctrl|imu)\.h$', inc):
                open(os.path.join(dest, inc), 'w').close()


def run(name, spec, keep):
    dest = tempfile.mkdtemp(prefix='pc_%s_' % name)
    write_set(spec, dest)
    progs = spec.get('programs') or {name: spec['build']}
    incs = sum((['-I', os.path.join(ROOT, d)] for d in spec.get('incdirs', [])), [])
    ok = True
    for prog, srcs in progs.items():
        exe = os.path.join(dest, prog)
        cmd = ['gcc'] + spec['cflags'] + ['-include', 'pc_all.h', '-I', dest] + incs + srcs + ['-o', exe] + spec.get('libs', [])
        r = subprocess.run(cmd, cwd=dest, capture_output=True, text=True)
        if r.returncode != 0:
            print('[%s] build failed:\n%s' % (prog, r.stderr))
            ok = False
            continue
        r = subprocess.run([exe] + spec.get('args', []), cwd=dest, capture_output=True, text=True)
        sys.stdout.write(r.stdout)
        print('[%s] exit code %d' % (prog, r.returncode))
        ok = ok and r.returncode == 0
    if keep:
        print('sources kept in', dest)
    else:
        shutil.rmtree(dest)
    return ok


def main():
    args = sys.argv[1:]
    keep = '--keep' in args
    args = [a for a in args if a != '--keep']
    if args and args[0] == 'extract':
        for name in args[1:]:
            spec = EXTRACT[name]
            os.makedirs(spec['to'], exist_ok=True)
            write_set(spec, spec['to'])
            print('wrote the %s sources to %s' % (name, spec['to']))
        return
    names = args or list(CHECKS)
    bad = [n for n in names if not run(n, CHECKS[n], keep)]
    print('PC checks failed: ' + ', '.join(bad) if bad else 'PC checks passed: ' + ', '.join(names))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
