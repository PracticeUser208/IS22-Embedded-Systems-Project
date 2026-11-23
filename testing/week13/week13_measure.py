"""
week13_measure.py -- Week 13 evidence for the minimum-bit data representation.

Measures, with the firmware's own compiler (arm-none-eabi-gcc, Cortex-M0+,
-O2) and its own source code:
  1. the size of each data item before / after the change (ram_compare.c);
  2. the RAM of the message buffers (kernel formula: 4-byte header + message
     rounded up to 4 bytes, per slot);
  3. the firmware's RAM (arm-none-eabi-size, and per app_program module from
     the link map) when a build exists in build_make/;
  4. the size of each MQTT message, binary and as text (JSON);
  5. the CPU cost of one telemetry frame: instructions executed on an
     emulated Cortex-M0+ (unicorn), and cycles counted per instruction
     with the Cortex-M0+ timings (Arm DDI 0484C, table 3-1).

Run from the repository root:
    python3 testing/week13/week13_measure.py [build_make/<firmware>.elf]
Needs: arm-none-eabi-gcc, gcc, and  pip install unicorn capstone pyelftools
Writes testing/week13/results.md and prints it.
"""

import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
import pc_checks  # noqa: E402

CC = 'arm-none-eabi-gcc'
ARM = ['-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft', '-O2', '-ffunction-sections', '-fdata-sections']
S1 = os.path.join(ROOT, 'app_program', 'subsystem1')
CLOCK_MHZ = 125

out = []


def emit(line=''):
    out.append(line)


def run(cmd, **kw):
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        sys.exit('failed: ' + ' '.join(cmd) + '\n' + r.stderr)
    return r.stdout


def symbol_sizes(obj, prefix=''):
    sizes = {}
    for line in run(['arm-none-eabi-nm', '-S', obj]).splitlines():
        p = line.split()
        if len(p) == 4 and p[3].startswith(prefix):
            sizes[p[3]] = int(p[1], 16)
    return sizes


def item_sizes(tmp):
    obj = os.path.join(tmp, 'ram_compare.o')
    run([CC] + ARM + ['-I', S1, '-c', os.path.join(HERE, 'ram_compare.c'), '-o', obj])
    s = symbol_sizes(obj)
    return {k[7:]: (s['BEFORE_' + k[7:]], s['AFTER_' + k[7:]]) for k in s if k.startswith('BEFORE_')}


ITEMS = [
    ('mc_snap', '1. Motion control', 'Motion Control -> IMU snapshot (speed, distance, sequence)'),
    ('nav_msg', '2. Line and barcode', '`nav_q` message (event, barcode action)'),
    ('dead_mask', '2. Line and barcode', 'barcode recording: drop-out flag per sample (3072 samples)'),
    ('obst_msg', '3. Obstacle detection', '`obst_q` message (type, side, flag, distance, width, clearance)'),
    ('s5_status', '3. Obstacle detection', 'Obstacle Scan status (state, pinging, 8 fault bits, distance, servo, retries)'),
    ('scan_table', '3. Obstacle detection', 'scan table, 25 bearings'),
    ('echo_result', '6. ISR -> task', 'echo result written by the GPIO interrupt (status, distance)'),
    ('scan_cmd', '7. Inter-task messages', '`scan_q` message (command, argument)'),
    ('imu_evt', '4. Hump estimation', '`imu_evt_q` message (type, hump height / class / lift cause, time)'),
    ('states', '5. Vehicle and controller states', 'six state values (VC, line, obstacle, motion, barcode, plan)'),
    ('faults', '6. Shared fault bits', 'system faults (8 conditions)'),
    ('estops', '6. Shared stop bits', 'stop requests (3 sources)'),
    ('telem_snapshot', '7. Inter-task messages', '`telem_q` message (telemetry snapshot, 31 fields)'),
    ('remote_cmd', '7. Inter-task messages', '`remote_cmd_q` message (command code, sequence)'),
]

# (queue, depth, item key or fixed (before, after) message bytes)
QUEUES = [
    ('telem_q (VC -> Telemetry)', 4, 'telem_snapshot'),
    ('remote_cmd_q (Telemetry -> VC)', 4, 'remote_cmd'),
    ('obst_q (Obstacle Scan -> VC)', 8, 'obst_msg'),
    ('imu_evt_q (IMU -> VC)', 8, 'imu_evt'),
    ('nav_q (Line -> VC)', 8, 'nav_msg'),
    ('scan_q (VC -> Obstacle Scan)', 8, 'scan_cmd'),
]


def slot(n):
    return 4 + ((n + 3) & ~3)


def host_consts(tmp):
    src = os.path.join(tmp, 'k.c')
    open(src, 'w').write('#include <stdio.h>\n#include "telem_format.h"\nint main(void){printf("%u %u %u %u %u %u %u %u %u\\n",'
                         'TF_TM_BITS,TF_TM_LEN,TF_HB_BITS,TF_HB_LEN,TF_CMD_BITS,TF_CMD_LEN,TF_ACK_BITS,TF_ACK_LEN,TF_BC_LEN_MAX);return 0;}\n')
    exe = os.path.join(tmp, 'k')
    run(['gcc', '-I', S1, src, '-o', exe])
    return [int(x) for x in run([exe]).split()]


LD = """
MEMORY { FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 2M
         RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 256K }
SECTIONS {
  .text : { *(.text*) *(.rodata*) } > FLASH
  .data : { *(.data*) } > RAM
  .bss  : { *(.bss*) *(COMMON) } > RAM
  end = .; _end = .;
}
"""

# Cortex-M0+ cycles (Arm DDI 0484C table 3-1): 1 per instruction except
# loads/stores 2, multiple load/store 1+N, taken branch 2, BL 3, BX/BLX 2,
# POP with PC 3+N.
def m0_cycles(insn, taken):
    m = insn.mnemonic.split('.')[0]
    ops = insn.op_str
    if m in ('ldr', 'ldrb', 'ldrh', 'ldrsb', 'ldrsh', 'str', 'strb', 'strh'):
        return 2
    if m in ('push', 'pop', 'ldm', 'stm', 'ldmia', 'stmia'):
        n = ops.count(',') + 1 if '{' in ops else 1
        if m == 'pop' and 'pc' in ops:
            return 3 + n
        return 1 + n
    if m == 'bl':
        return 3
    if m in ('bx', 'blx'):
        return 2 if m == 'bx' else 3
    if m.startswith('b') and m not in ('bic', 'bics'):
        return 2 if taken else 1
    return 1


def bench(tmp):
    from elftools.elf.elffile import ELFFile
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR
    import capstone

    d = os.path.join(tmp, 'bench')
    os.makedirs(d)
    pc_checks.write_set(pc_checks.CHECKS['telemetry'], d)
    ld = os.path.join(d, 'bench.ld')
    open(ld, 'w').write(LD)
    elf = os.path.join(d, 'bench.elf')
    run([CC] + ARM + ['-include', 'pc_all.h', '-I', d, '-I', S1, os.path.join(HERE, 'bench.c'),
                      os.path.join(d, 'bitpack.c'), os.path.join(d, 'telem_codec.c'),
                      '--specs=nano.specs', '--specs=nosys.specs', '-nostartfiles', '-Wl,--gc-sections', '-Wl,-u,bench_pack', '-Wl,-u,bench_unpack', '-Wl,-u,bench_text', '-Wl,-u,bench_text_buf',
                      '-Wl,-e,bench_init', '-T', ld, '-o', elf])
    f = open(elf, 'rb')
    e = ELFFile(f)
    syms = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
    sizes = {s.name: s['st_size'] for s in e.get_section_by_name('.symtab').iter_symbols()}
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0x10000000, 2 * 1024 * 1024)
    uc.mem_map(0x20000000, 256 * 1024)
    uc.mem_map(0x0FFFF000, 0x1000)
    for seg in e.iter_segments():
        if seg['p_type'] == 'PT_LOAD':
            uc.mem_write(seg['p_vaddr'], seg.data())
    md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
    stats = {}

    def hook(u, addr, size, user):
        st = stats['cur']
        if st['prev'] is not None:
            pa, pinsn = st['prev']
            st['cycles'] += m0_cycles(pinsn, addr != pa + pinsn.size)
        insn = next(md.disasm(bytes(u.mem_read(addr, size)), addr))
        st['prev'] = (addr, insn)
        st['n'] += 1

    uc.hook_add(UC_HOOK_CODE, hook)

    def call(name):
        stats['cur'] = {'n': 0, 'cycles': 0, 'prev': None}
        uc.reg_write(UC_ARM_REG_SP, 0x20000000 + 256 * 1024 - 16)
        uc.reg_write(UC_ARM_REG_LR, 0x0FFFF001)
        uc.emu_start(syms[name] | 1, 0x0FFFF000)
        st = stats['cur']
        if st['prev'] is not None:
            st['cycles'] += m0_cycles(st['prev'][1], True)
        return st['n'], st['cycles']

    call('bench_init')
    res = {}
    for name in ('bench_pack', 'bench_unpack', 'bench_text'):
        res[name] = call(name)
        res[name + '_result'] = int.from_bytes(uc.mem_read(syms['result'], 4), 'little', signed=True)
    text_addr = syms['text']
    raw = bytes(uc.mem_read(text_addr, 512))
    res['text'] = raw[:raw.index(b'\0')].decode()
    res['frame'] = bytes(uc.mem_read(syms['frame'], sizes['frame'])).hex(' ').upper()
    code = {n: sizes.get(n, 0) for n in ('tc_encode_tm', 'tc_decode_tm', 'bs_put', 'bs_get', '_svfprintf_r', 'snprintf', '_vfprintf_r')}
    res['code'] = code
    return res


def firmware_ram(elf):
    o = run(['arm-none-eabi-size', '-A', elf])
    sec = {}
    for line in o.splitlines():
        p = line.split()
        if len(p) == 3 and p[1].isdigit():
            sec[p[0]] = (int(p[1]), int(p[2], 0))
    ram = sum(sz for n, (sz, addr) in sec.items() if 0x20000000 <= addr < 0x20042000 and not n.startswith('.heap')
              and not n.startswith('.stack'))
    mod = {}
    mp = elf[:-4] + '.map'
    if os.path.exists(mp):
        txt = open(mp, errors='replace').read()
        txt = txt[txt.find('Linker script and memory map'):]
        for m in re.finditer(r'^ (\.(?:bss|data|sdata|sbss)[^\s]*|COMMON)\s*\n?\s+0x(2[0-9a-f]{7})\s+0x([0-9a-f]+)\s+(\S+)', txt, re.M):
            path = m.group(4)
            if '/app_program/' not in path:
                continue
            mod_name = path.split('/app_program/')[1].replace('.o', '.c')
            mod[mod_name] = mod.get(mod_name, 0) + int(m.group(3), 16)
    return ram, sec, mod


def json_len(fields):
    return len('{' + ','.join('"%s":%s' % (k, v) for k, v in fields) + '}')


def main():
    elfs = sys.argv[1:] or [p for p in [os.path.join(ROOT, 'build_make', 'mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf')]
                            if os.path.exists(p)]
    tmp = tempfile.mkdtemp(prefix='week13_')
    items = item_sizes(tmp)
    k = host_consts(tmp)
    b = bench(tmp)

    emit('# Week 13 measurements (generated by `testing/week13/week13_measure.py`)')
    emit()
    emit('Compiler: `%s` (%s), flags `%s`.' % (CC, run([CC, '-dumpversion']).strip(), ' '.join(ARM)))
    emit()
    emit('## 1. Size of each data item (bytes, measured from the object file)')
    emit()
    emit('| Requirement area | Data item | Before | After | Saved |')
    emit('|---|---|---:|---:|---:|')
    tb = ta = 0
    for key, area, what in ITEMS:
        bb, aa = items[key]
        tb += bb
        ta += aa
        emit('| %s | %s | %d | %d | %d |' % (area, what, bb, aa, bb - aa))
    emit('| | **Total** | **%d** | **%d** | **%d (%.0f%%)** |' % (tb, ta, tb - ta, 100.0 * (tb - ta) / tb))
    emit()
    emit('## 2. Message buffer RAM (bytes; per slot: 4-byte header + message rounded up to 4)')
    emit()
    emit('| Queue | Slots | Message before | Message after | RAM before | RAM after |')
    emit('|---|---:|---:|---:|---:|---:|')
    qb = qa = 0
    for name, depth, key in QUEUES:
        mb, ma = items[key]
        rb, ra = depth * slot(mb), depth * slot(ma)
        qb += rb
        qa += ra
        emit('| %s | %d | %d | %d | %d | %d |' % (name, depth, mb, ma, rb, ra))
    text_tm = len(b['text'])
    tx_b, tx_a = 8 * slot(1 + max(text_tm, k[8])), 8 * slot(1 + max(k[1], k[8]))
    qb += tx_b
    qa += tx_a
    emit('| S1 TX queue (frames waiting for MQTT; largest = telemetry as text / barcode record) | 8 | %d | %d | %d | %d |'
         % (1 + max(text_tm, k[8]), 1 + max(k[1], k[8]), tx_b, tx_a))
    emit('| **Total** | | | | **%d** | **%d** |' % (qb, qa))
    emit()
    emit('## 3. MQTT messages (bytes on the wire)')
    emit()
    hb_text = json_len([('v', 3), ('seq', 7), ('state', 2), ('fault', 16), ('uptime', 125), ('drops', 3), ('reconnects', 1)])
    cmd_text = json_len([('cmd', '"START"'), ('seq', 5)])
    ack_text = json_len([('cmd', '"START"'), ('seq', 5), ('result', '"FORWARDED"')])
    emit('| Message | Bits | Binary (after) | Text / JSON (before) | Smaller by |')
    emit('|---|---:|---:|---:|---:|')
    for name, bits, n, t in (('`is22/tm` telemetry', k[0], k[1], text_tm), ('`is22/hb` heartbeat', k[2], k[3], hb_text),
                             ('`is22/cmd` command', k[4], k[5], cmd_text), ('`is22/ack` command result', k[6], k[7], ack_text)):
        emit('| %s | %d | %d | %d | %.0fx |' % (name, bits, n, t, t / n))
    emit()
    emit('Telemetry test frame, binary: `%s`' % b['frame'])
    emit()
    emit('The same values as JSON text (%d bytes): `%s`' % (text_tm, b['text']))
    emit()
    emit('## 4. CPU time for one telemetry frame (emulated Cortex-M0+, %d MHz)' % CLOCK_MHZ)
    emit()
    emit('| Step | Instructions | Cycles | Time |')
    emit('|---|---:|---:|---:|')
    for name, label in (('bench_pack', 'pack into the 22-byte frame (`tc_encode_tm`)'),
                        ('bench_text', 'write the same values as JSON (`snprintf`)'),
                        ('bench_unpack', 'unpack the frame (`tc_decode_tm`, PC side / tests)')):
        n, c = b[name]
        emit('| %s | %d | %d | %.1f us |' % (label, n, c, c / CLOCK_MHZ))
    emit()
    emit('Code size: `tc_encode_tm` %d B, `bs_put` %d B; the printf engine (`_svfprintf_r`) %d B.'
         % (b['code']['tc_encode_tm'], b['code']['bs_put'], b['code']['_svfprintf_r']))
    for elf in elfs:
        ram, sec, mod = firmware_ram(elf)
        emit()
        emit('## 5. Firmware RAM: `%s`' % os.path.basename(elf))
        emit()
        emit('| Section | Bytes |')
        emit('|---|---:|')
        for n, (sz, addr) in sec.items():
            if 0x20000000 <= addr < 0x20042000 and sz:
                emit('| `%s` | %d |' % (n, sz))
        emit('| **static RAM total** | **%d** |' % ram)
        if mod:
            emit()
            emit('Static RAM of the application modules (`app_program`, from the link map):')
            emit()
            emit('| Module | Bytes |')
            emit('|---|---:|')
            for n in sorted(mod):
                emit('| `%s` | %d |' % (n, mod[n]))
            emit('| **application total** | **%d** |' % sum(mod.values()))
    text = '\n'.join(out) + '\n'
    open(os.path.join(HERE, 'results.md'), 'w').write(text)
    print(text)


if __name__ == '__main__':
    main()
