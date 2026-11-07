"""Transactional wrappers for the easyeda commands that edit the board in place and can leave it worse than before.

The CLI is a closed binary, so its bugs (issues #4 beautify, #5 route-short, #6 auto-place) cannot be fixed here.
These wrappers make sure a bad result never stays on the board: snapshot -> run -> read back -> keep only what is no
worse than before -> roll back the rest. Snapshots and a report go to guard_runs/<board>/<time>_<command>/.

usage:
  python kit/pcb/guard.py place <auto-place|arrange|place-constrained|...> [flags]
      layout-lint before/after; parts that cause new shorts / overlaps / off-board are moved back (all moved parts
      if that is not enough); a result with more ratsnest crossings is undone too unless --accept-worse.
  python kit/pcb/guard.py route-short [route-short flags] [--dry-run]
      takes route-short's own dry-run plan, drops every hop that would come closer than the clearance rule to
      other-net copper or the board edge, draws the rest itself, reads back, runs DRC and deletes any new track
      that is still in a clearance / physical violation. Forces --no-multilayer (the plan gives no via positions).
  python kit/pcb/guard.py beautify [--net N ...] [--max-tracks 60] [beautify flags] [--dry-run]
      beautifies a few nets per call (one call per chunk of <= --max-tracks tracks, so no call hits the connector
      deadline), checks each chunk's connectivity and puts a broken net back from the snapshot; after the last
      chunk it re-pours, runs DRC and restores every net that has a violation it did not have before.
  python kit/pcb/guard.py restore-copper <snapshot.json> [--net N ...]
      put the tracks and vias of the given nets (default: every net that differs) back as they are in a
      `pcb dump --include-copper` snapshot - the recovery for a half-finished beautify / route-short.
"""
import collections, json, math, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'route'))
import common as K
from conn import islands

B = K.board()
HARD = lambda v: v.get('rule') != 'Connection Error'      # clearance / physical / short: never acceptable


def pop_opt(args, name, multi=False, default=None):
    vals = []
    while name in args:
        i = args.index(name)
        if i + 1 >= len(args): raise SystemExit(f'{name} needs a value')
        vals.append(args[i + 1]); del args[i:i + 2]
    return vals if multi else (vals[-1] if vals else default)


def pop_flag(args, name):
    if name in args:
        args.remove(name); return True
    return False


def rundir(cmd):
    d = B.path(os.path.join('guard_runs', B.name, time.strftime('%Y%m%d-%H%M%S') + '_' + cmd))
    os.makedirs(d, exist_ok=True)
    return d


def reply_ok(r):
    return isinstance(r, dict) and r.get('ok') is True


def err(r):
    return json.dumps((r or {}).get('error') or '', ensure_ascii=False)[:200]


def refresh(repour):
    """Re-pour (when the board has pours) and save, so the next DRC sees pours flowed around the new copper.
    No `doc reload` here, on purpose (measured on EasyEDA 4.1.60 / easyeda-agent 1.8.1):
      - a pour rebuilt before save + reload came back stale after the reload (DRC: 0 mil pour-to-track);
      - DRC straight after pour-rebuild, without a reload, was correct;
      - `doc reload` is save + close + reopen, and with the connector under load the reopen timed out and left
        no PCB open, so every later command failed until the tab was reopened by hand."""
    if repour:
        K.run(['pcb', 'pour-rebuild'])
    K.run(['pcb', 'save'])


def has_pours():
    return bool((K.run(['pcb', 'pour-list']).get('result') or {}).get('pours'))


def drc(path):
    d = K.run(['pcb', 'drc', '--json', '--timeout', '300'], timeout=420)
    d = d if 'violations' in d else (d.get('result') or {})
    if 'violations' not in d:
        raise SystemExit('DRC did not run (is the PCB the foreground document?) - nothing changed after this point')
    json.dump(d, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    return d


def drc_counts(d):
    return dict(collections.Counter(v.get('rule') for v in d.get('violations', [])))


# ---------------------------------------------------------------- copper snapshots and restore
def copper_items(d, nets=None):
    """[(kind, item)] for lines, arcs and vias of `nets` (all nets when None) in a dump."""
    cu = d.get('copper') or {}
    out = []
    for kind in ('lines', 'arcs', 'vias'):
        out += [(kind, it) for it in cu.get(kind) or [] if nets is None or it.get('net') in nets]
    return out


def geo_key(kind, it):
    r = lambda v: round(float(v), 1)
    if kind == 'vias':
        return ('via', it.get('net'), r(it['x']), r(it['y']))
    ends = tuple(sorted([(r(it['startX']), r(it['startY'])), (r(it['endX']), r(it['endY']))]))
    return (kind, it.get('net'), it.get('layer'), ends, r(it.get('lineWidth', 0)), r(it.get('arcAngle', 0) or 0))


def changed_nets(a, b):
    """Nets whose tracks / arcs / vias differ between two dumps."""
    ka, kb = collections.defaultdict(collections.Counter), collections.defaultdict(collections.Counter)
    for kind, it in canon(copper_items(a)): ka[it.get('net')][geo_key(kind, it)] += 1
    for kind, it in canon(copper_items(b)): kb[it.get('net')][geo_key(kind, it)] += 1
    return sorted(n for n in set(ka) | set(kb) if n and ka[n] != kb[n])


def canon(items):
    """Copper in a canonical form: EasyEDA merges collinear, touching tracks of one net / layer / width into a single
    track when they are created close together (two creates 0.3 s apart come back as one line), so straight runs
    are compared merged. Each returned item carries '_ids', the primitive ids it stands for."""
    out, groups = [], collections.defaultdict(list)
    for kind, it in items:
        if kind == 'lines' and math.dist((it['startX'], it['startY']), (it['endX'], it['endY'])) < 0.5:
            # near-zero-length track: never let it join the merge - as a "point" it lies on every line through
            # it and glued a horizontal and a vertical run into one line, so restore-copper ripped the vertical
            # run up, never redrew it and still reported the net as restored (5V_MAIN, 2026-11-07)
            out.append((kind, dict(it, _ids=[it.get('primitiveId')])))
        elif kind == 'lines':
            groups[(it.get('net'), it.get('layer'), round(float(it.get('lineWidth', 0)), 1))].append(it)
        else:
            out.append((kind, dict(it, _ids=[it.get('primitiveId')])))
    for (net, layer, w), segs in groups.items():
        par = list(range(len(segs)))
        def f(i):
            while par[i] != i: par[i] = par[par[i]]; i = par[i]
            return i
        P = [((s['startX'], s['startY']), (s['endX'], s['endY'])) for s in segs]
        def on_line(p, a, b):                   # p within 0.05 mil of the infinite line a-b
            L = math.dist(a, b)
            return L > 1e-6 and abs((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / L < 0.05
        def on_seg(p, a, b):
            return on_line(p, a, b) and min(a[0], b[0]) - 0.05 <= p[0] <= max(a[0], b[0]) + 0.05 \
                and min(a[1], b[1]) - 0.05 <= p[1] <= max(a[1], b[1]) + 0.05
        by_end = collections.defaultdict(list)
        for i, (a, b) in enumerate(P):
            for p in (a, b): by_end[(round(p[0]), round(p[1]))].append(i)
        for i, (a, b) in enumerate(P):
            cand = {j for p in (a, b) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                    for j in by_end[(round(p[0]) + dx, round(p[1]) + dy)] if j > i}
            for j in cand:
                c, d = P[j]
                if on_line(c, a, b) and on_line(d, a, b) and (on_seg(c, a, b) or on_seg(d, a, b) or on_seg(a, c, d)):
                    par[f(j)] = f(i)
        comp = collections.defaultdict(list)
        for i in range(len(segs)): comp[f(i)].append(i)
        for idx in comp.values():
            a, b = P[idx[0]]
            L = math.dist(a, b) or 1
            ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            pts = [p for i in idx for p in P[i]]
            t = [(p[0] - a[0]) * ux + (p[1] - a[1]) * uy for p in pts]
            p0, p1 = pts[t.index(min(t))], pts[t.index(max(t))]
            out.append(('lines', dict(segs[idx[0]], startX=p0[0], startY=p0[1], endX=p1[0], endY=p1[1],
                                      _ids=[segs[i].get('primitiveId') for i in idx])))
    return out


def missing(want_items, have_items):
    """Canonical items of `want_items` with no geometric twin among the canonical `have_items`, duplicates counted."""
    want_items, have_items = canon(want_items), canon(have_items)
    left = collections.Counter(geo_key(*t) for t in want_items) - collections.Counter(geo_key(*t) for t in have_items)
    out = []
    for kind, it in want_items:
        k = geo_key(kind, it)
        if left[k] > 0: out.append((kind, it)); left[k] -= 1
    return out


def settle(path, tries=12, wait=5):
    """Dump until two reads in a row agree: a command that timed out on the CLI side can still be editing."""
    prev = K.dump(path)
    sig = lambda d: collections.Counter(geo_key(*t) for t in canon(copper_items(d)))
    for _ in range(tries):
        time.sleep(wait)
        cur = K.dump(path)
        if sig(cur) == sig(prev):
            return cur
        prev = cur
    print('!! board still changing after', tries * wait, 's - using the last read')
    return prev


def draw(kind, it):
    if kind == 'vias':
        return K.run(['pcb', 'via', '--x', str(it['x']), '--y', str(it['y']), '--hole', str(it.get('holeDiameter', 12)),
                      '--diameter', str(it.get('diameter', 24)), '--net', it['net']], tries=1)
    return K.run(['pcb', 'track', '--x1', str(it['startX']), '--y1', str(it['startY']), '--x2', str(it['endX']),
                  '--y2', str(it['endY']), '--layer', str(it['layer']), '--width', str(it.get('lineWidth', 6)),
                  '--net', it['net']], tries=1)


def delete_ids(kind_ids):
    for kind, ids in kind_ids.items():
        cmd = 'via-delete' if kind == 'vias' else 'track-delete'
        for k in range(0, len(ids), 40):
            K.run(['pcb', cmd, '--ids', ','.join(ids[k:k + 40])])


def restore_nets(snap, nets, tmp):
    """Make the copper of `nets` equal to `snap`: rip up, delete leftovers, redraw from the snapshot, read back.
    Returns the snapshot items still missing afterwards (empty = restored)."""
    nets = sorted(set(nets))
    if not nets: return []
    for k in range(0, len(nets), 10):
        K.run(['pcb', 'rip-up', '--net', ','.join(nets[k:k + 10])])
    S = set(nets)
    want = copper_items(snap, S)
    cur = K.dump(tmp)
    # whatever rip-up left (locked copper, arcs it does not delete) stays only if the snapshot has it
    extra = collections.defaultdict(list)
    for kind, it in missing(copper_items(cur, S), want):
        extra[kind] += it['_ids']
    if extra:
        delete_ids(extra); cur = K.dump(tmp)
    todo = missing(want, copper_items(cur, S))
    for attempt in range(2):                     # draw, read back, redraw only what is still missing (once)
        for kind, it in todo: draw(kind, it)
        cur = K.dump(tmp)
        todo = missing(want, copper_items(cur, S))
        if not todo: break
    rm = collections.defaultdict(list)           # a "failed" create that landed twice
    for kind, it in missing(copper_items(cur, S), want):
        if not it.get('locked'): rm[kind] += it['_ids']
    if rm: delete_ids(rm)
    return todo


def cmd_restore(args):
    nets = pop_opt(args, '--net', multi=True)
    if not args: raise SystemExit('usage: guard.py restore-copper <snapshot.json> [--net N ...]')
    snap = K.load_dump(args[0])
    rd = rundir('restore')
    cur = K.dump(os.path.join(rd, 'before.json'))
    nets = nets or changed_nets(snap, cur)
    print('nets to restore:', len(nets), nets[:20])
    left = restore_nets(snap, nets, os.path.join(rd, 'tmp.json'))
    refresh(has_pours())
    after = K.dump(os.path.join(rd, 'after.json'))
    still = [n for n in changed_nets(snap, after) if n in nets]
    print('restored' if not still else '!! still different from the snapshot: ' + ', '.join(still),
          '| items still missing:', len(left))


# ---------------------------------------------------------------- place
def lint():
    r = K.run(['pcb', 'layout-lint', '--json'])
    rep = r.get('report') or (r.get('result') or {}).get('report') or r.get('result') or {}
    if 'summary' not in rep:
        raise SystemExit('layout-lint gave no report: ' + json.dumps(r, ensure_ascii=False)[:300])
    return rep


def strings(o):
    if isinstance(o, str): yield o
    elif isinstance(o, dict):
        for v in o.values(): yield from strings(v)
    elif isinstance(o, list):
        for v in o: yield from strings(v)


def problems(rep, names):
    """{'short'|'overlap'|'off-board': (count, {designators involved})}. Counts come from the summary line (the one
    format the CLI documents), designators from whatever lists the report carries."""
    s = rep.get('summary', '')
    out = {}
    for label, keys in (('short', ('shorts',)), ('overlap', ('overlaps',)), ('off-board', ('outsideOutline',))):
        m = re.search(r'(\d+) ' + label, s)
        items = [it for k in keys for it in (rep.get(k) or [])]
        out[label] = (int(m.group(1)) if m else len(items), {t for t in strings(items) if t in names})
    return out


def worse(p1, p0):
    return [k for k in p0 if p1[k][0] > p0[k][0]]


def components():
    return {c['designator']: c for c in K.run(['pcb', 'list'])['result']['components']}


def moved(ref, live):
    return sorted(d for d, r in ref.items() if d in live and (
        abs(live[d]['x'] - r['x']) > 0.5 or abs(live[d]['y'] - r['y']) > 0.5
        or round((live[d]['rotation'] - r['rotation']) % 360, 3) not in (0, 360) or live[d]['layer'] != r['layer']))


def move_back(ref, names):
    live = components()
    for d in names:
        c, r = live[d], ref[d]
        if c.get('locked'): K.run(['pcb', 'lock', '--ids', c['primitiveId'], '--unlock'])
        if c['layer'] != r['layer']:          # a patch with layer + x/y does not land: flip on its own first
            K.run(['pcb', 'modify', '--id', c['primitiveId'], '--patch', json.dumps({'layer': r['layer']})])
        K.run(['pcb', 'modify', '--id', c['primitiveId'], '--patch',
               json.dumps({'x': r['x'], 'y': r['y'], 'rotation': r['rotation']})])
        if r.get('locked'): K.run(['pcb', 'lock', '--ids', c['primitiveId']])
    K.run(['pcb', 'save'])
    still = [d for d in moved(ref, components()) if d in names]
    if still: print('!! not back in place after readback:', still)


def cmd_place(args):
    accept_worse = pop_flag(args, '--accept-worse')
    if not args: raise SystemExit('usage: guard.py place <auto-place|arrange|...> [flags]')
    rd = rundir('place')
    ref = {c['designator']: c for c in K.dump(os.path.join(rd, 'before.json'))['components']}
    names = set(ref)
    L0 = lint(); P0 = problems(L0, names)
    print('before:', L0['summary'])
    if '--dry-run' in args:
        r = K.run(['pcb'] + args, tries=1, timeout=900)
        print(json.dumps(r.get('result', r), ensure_ascii=False, indent=1)[:3000]); return
    r = K.run(['pcb'] + args, tries=1, timeout=900)
    print('pcb', args[0], '-> ok', r.get('ok'), err(r) if not reply_ok(r) else '')
    K.run(['pcb', 'save'])
    L1 = lint(); P1 = problems(L1, names)
    print('after: ', L1['summary'])
    reverted = set()
    for rnd in range(5):                         # move back the parts named in the new problems
        bad = worse(P1, P0)
        if not bad: break
        live = components()
        culprits = sorted({d for k in bad for d in P1[k][1] - P0[k][1]} & set(moved(ref, live)) - reverted) or \
                   sorted({d for k in bad for d in P1[k][1]} & set(moved(ref, live)) - reverted)
        if not culprits: break
        print(f'round {rnd + 1}: {", ".join(bad)} worse -> moving back {culprits}')
        move_back(ref, culprits); reverted |= set(culprits)
        L1 = lint(); P1 = problems(L1, names); print('       ', L1['summary'])
    why = None
    if worse(P1, P0):
        why = 'still ' + ', '.join(worse(P1, P0)) + ' worse than before'
    elif L1.get('crossingCount', 0) > L0.get('crossingCount', 0) and not accept_worse:
        why = f"ratsnest crossings {L0.get('crossingCount')} -> {L1.get('crossingCount')} (pass --accept-worse to keep)"
    if why:
        back = moved(ref, components())
        print('undo:', why, '-> moving back all', len(back), 'moved parts')
        move_back(ref, back)
        L1 = lint(); print('final: ', L1['summary'])
    else:
        print('kept:', len(moved(ref, components())), 'parts moved,', len(reverted), 'moved back')
    json.dump({'before': L0, 'after': L1, 'reverted': sorted(reverted), 'undone': why},
              open(os.path.join(rd, 'report.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('snapshot:', os.path.join(rd, 'before.json'), '(python kit/pcb/place_from_dump.py <it> puts everything back)')


# ---------------------------------------------------------------- route-short
def obstacles(d):
    """[(geometry, layers, net, name)] for pads, tracks, arcs and vias in a dump (mil)."""
    from shapely.geometry import LineString, Point, box
    from shapely import affinity
    out = []
    for c in d['components']:
        for p in c.get('pads', []):
            w, h = p.get('width', 0), p.get('height', 0)
            shape = p.get('shape') or ['RECT']
            if shape[0] in ('ELLIPSE', 'CIRCLE'):
                g = affinity.scale(Point(p['x'], p['y']).buffer(1, 32), w / 2, h / 2)
            else:
                g = box(p['x'] - w / 2, p['y'] - h / 2, p['x'] + w / 2, p['y'] + h / 2)
            layers = {1, 2} if p.get('layer') in (12, None) else {p['layer']}
            out.append((g, layers, p.get('net') or '', f"{c['designator']}.{p.get('padNumber')}"))
    for kind, it in copper_items(d):
        if kind == 'vias':
            out.append((Point(it['x'], it['y']).buffer(it.get('diameter', 24) / 2, 16), {1, 2}, it.get('net') or '', 'via'))
        else:
            g = LineString([(it['startX'], it['startY']), (it['endX'], it['endY'])]).buffer(it.get('lineWidth', 6) / 2, 8)
            out.append((g, {it['layer']}, it.get('net') or '', 'track'))
    return out


def legal_hops(d, routes, clr, edge, min_width=None):
    """Split route-short's plan into pad-to-pad hops and keep the ones that stay `clr` mil from other-net copper
    (pads, existing copper, hops already kept) and `edge` mil inside the outline. Tracks thinner than their net's
    rule minimum (`min_width` {net: mil}) are widened first. -> (kept segs, [(hop, reason)])."""
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree
    from shapely.ops import unary_union
    min_width = min_width or {}
    routes = [dict(s, width=max(s.get('width', 6), min_width.get(s['net'], 0))) for s in routes]
    obs = obstacles(d)
    pts = (d.get('outline') or {}).get('points') or []
    board = Polygon(pts).buffer(0).buffer(-edge) if len(pts) >= 3 else None
    pads = collections.defaultdict(list)
    for g, layers, net, name in obs:
        if name not in ('via', 'track') and net: pads[net].append((g, layers))
    by_net = collections.defaultdict(list)
    for i, s in enumerate(routes): by_net[s['net']].append(i)
    kept, rejected = [], []
    tree, geoms = STRtree([o[0] for o in obs]), list(obs)
    added = []                                   # hops kept so far (other nets must clear them too)
    key = lambda x, y: (round(x, 2), round(y, 2))
    for net, idx in by_net.items():
        def terminal(x, y, L):
            return any(L in ls and g.buffer(0.5).contains(Point(x, y)) for g, ls in pads[net])
        par = {i: i for i in idx}
        def f(i):
            while par[i] != i: par[i] = par[par[i]]; i = par[i]
            return i
        ends = collections.defaultdict(list)
        for i in idx:
            s = routes[i]
            for x, y in ((s['x1'], s['y1']), (s['x2'], s['y2'])):
                ends[key(x, y) + (s['layer'],)].append(i)
        for (x, y, L), ii in ends.items():
            if not terminal(x, y, L):
                for j in ii[1:]: par[f(j)] = f(ii[0])
        hops = collections.defaultdict(list)
        for i in idx: hops[f(i)].append(i)
        for hop in hops.values():
            segs = [routes[i] for i in hop]
            layers = {s['layer'] for s in segs}
            free = [(x, y, L) for (x, y, L), ii in ends.items() if len([i for i in ii if i in hop]) == 1]
            reason = None
            if len(layers) != 1:
                reason = 'changes layer'
            elif any(not terminal(x, y, L) for x, y, L in free):
                reason = 'does not end on a pad of its net'
            L = segs[0]['layer']
            geom = unary_union([LineString([(s['x1'], s['y1']), (s['x2'], s['y2'])]).buffer(s.get('width', 6) / 2, 8) for s in segs])
            if not reason and board is not None and not board.contains(geom):
                reason = f'closer than {edge:.1f} mil to the board edge'
            if not reason:
                near = [geoms[k] for k in tree.query(geom.buffer(clr))] + added
                for g, ls, n, name in near:
                    if L in ls and n != net and geom.distance(g) < clr - 0.01:
                        reason = f'{geom.distance(g):.1f} mil from {n or "no-net"} {name} (rule {clr:.1f})'; break
            if reason:
                rejected.append(({'net': net, 'segments': segs}, reason))
            else:
                kept += segs; added.append((geom, {L}, net, 'new hop'))
    return kept, rejected


def cmd_route_short(args):
    dry = pop_flag(args, '--dry-run')
    if '--no-multilayer' not in args: args.append('--no-multilayer')
    rd = rundir('route-short')
    before = K.dump(os.path.join(rd, 'before.json'))
    rules = before.get('rules') or {}
    clr = rules.get('clearanceMil') or B.rules.get('copper_mm', 0.15) * K.MIL
    edge = rules.get('copperToEdgeMil') or B.rules.get('edge_mm', 0.25) * K.MIL
    plan = K.run(['pcb', 'route-short', '--dry-run'] + args, tries=1, timeout=600)
    routes = plan.get('routes', (plan.get('result') or {}).get('routes'))
    if not isinstance(routes, list):
        raise SystemExit('route-short dry-run gave no plan: ' + json.dumps(plan, ensure_ascii=False)[:300])
    minw = {n: w + 0.05 for n, w in B.class_min_mil().items()}     # a hair over the rule: DRC rounds widths
    kept, rejected = legal_hops(before, routes, clr, edge, minw)
    print(f'plan: {len(routes)} segments | kept {len(kept)} | dropped {len(rejected)} hops (left for Freerouting)')
    for hop, why in rejected[:30]: print('  drop', hop['net'], len(hop['segments']), 'seg:', why)
    report = {'plan': routes, 'kept': kept, 'rejected': [{'net': h['net'], 'segments': h['segments'], 'why': w} for h, w in rejected]}
    json.dump(report, open(os.path.join(rd, 'report.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    if dry or not kept: return
    d0 = drc(os.path.join(rd, 'drc_before.json'))
    want = [{'startX': s['x1'], 'startY': s['y1'], 'endX': s['x2'], 'endY': s['y2'], 'layer': s['layer'],
             'lineWidth': s.get('width', 6), 'net': s['net']} for s in kept]
    old = {it['primitiveId'] for _, it in copper_items(before)}
    want = [('lines', w) for w in want]
    todo = want
    for attempt in range(2):                     # draw, read back, redraw only the missing ones (once)
        for kind, w in todo: draw(kind, w)
        cur = K.dump(os.path.join(rd, 'after.json'))
        todo = missing(want, [(k, it) for k, it in copper_items(cur) if it['primitiveId'] not in old])
        if not todo: break
    print('drawn; missing after readback:', len(todo))
    repour = has_pours()                         # pours still flow around the old copper until re-poured
    refresh(repour)
    for rnd in range(3):
        cur = K.dump(os.path.join(rd, 'after.json'))
        new = {it['primitiveId']: it for _, it in copper_items(cur) if it['primitiveId'] not in old}
        d1 = drc(os.path.join(rd, f'drc_after{rnd}.json'))
        bad_nets = {new[o]['net'] for v in d1['violations'] if HARD(v) for o in v.get('objs', []) if o in new}
        print(f'DRC round {rnd}: {drc_counts(d1)} (before {drc_counts(d0)}) | nets with new violations: {sorted(bad_nets)}')
        if not bad_nets: break
        delete_ids({'lines': [i for i, it in new.items() if it['net'] in bad_nets]})
        refresh(repour)
    hard0, hard1 = sum(map(HARD, d0['violations'])), sum(map(HARD, d1['violations']))
    print('result:', 'OK' if hard1 <= hard0 else '!! still worse', f'| hard violations {hard0} -> {hard1}',
          '| snapshot', os.path.join(rd, 'before.json'))


# ---------------------------------------------------------------- beautify
def cmd_beautify(args):
    nets = pop_opt(args, '--net', multi=True)
    max_tracks = int(pop_opt(args, '--max-tracks', default=60))
    if pop_flag(args, '--dry-run'):
        r = K.run(['pcb', 'beautify', '--dry-run'] + sum([['--net', n] for n in nets], []) + args, tries=1)
        print(json.dumps(r.get('result', r), ensure_ascii=False, indent=1)); return
    if '--selected' in args: raise SystemExit('--selected cannot be chunked or checked: pass --net instead')
    repour = not pop_flag(args, '--no-pour-rebuild')
    rd = rundir('beautify')
    snap = K.dump(os.path.join(rd, 'before.json'))
    d0 = drc(os.path.join(rd, 'drc_before.json'))
    base = islands(snap)
    per_net = collections.Counter(it.get('net') for k, it in copper_items(snap) if k == 'lines' and it.get('net'))
    todo = [n for n in (nets or sorted(per_net)) if per_net.get(n)]
    chunks, cur_chunk, size = [], [], 0
    for n in todo:
        if cur_chunk and size + per_net[n] > max_tracks: chunks.append(cur_chunk); cur_chunk, size = [], 0
        cur_chunk.append(n); size += per_net[n]
    if cur_chunk: chunks.append(cur_chunk)
    print(f'{len(todo)} nets, {sum(per_net[n] for n in todo)} tracks -> {len(chunks)} calls (<= {max_tracks} tracks each)')
    restored, tmp = set(), os.path.join(rd, 'tmp.json')
    for i, chunk in enumerate(chunks):
        r = K.run(['pcb', 'beautify'] + sum([['--net', n] for n in chunk], []) + args + ['--no-pour-rebuild'],
                  tries=1, timeout=900)
        cur = settle(tmp)
        broke = sorted(n for n in chunk if len(islands(cur, {n}).get(n, [0])) > len(base.get(n, [0])))
        # anything outside the chunk that changed was not asked for
        stray = sorted(set(changed_nets(snap, cur)) - set(todo))
        print(f'[{i + 1}/{len(chunks)}] {",".join(chunk)[:60]} ok={r.get("ok")} broken={broke} stray={stray}'
              + ('' if reply_ok(r) else ' ' + err(r)))
        if broke or stray:
            left = restore_nets(snap, broke + stray, tmp)
            restored |= set(broke + stray)
            if left: print('!! could not restore', len(left), 'items - stop; recover with: guard.py restore-copper', os.path.join(rd, 'before.json')); break
    repour = repour and has_pours()
    refresh(repour)
    for rnd in range(2):
        cur = K.dump(tmp)
        d1 = drc(os.path.join(rd, f'drc_after{rnd}.json'))
        old_ids = {it['primitiveId'] for _, it in copper_items(snap)}
        new_ids = {it['primitiveId'] for _, it in copper_items(cur)} - old_ids
        conn0 = collections.Counter(v.get('net') for v in d0['violations'] if not HARD(v))
        conn1 = collections.Counter(v.get('net') for v in d1['violations'] if not HARD(v))
        bad = {v.get('net') for v in d1['violations'] if HARD(v) and set(v.get('objs', [])) & new_ids}
        bad |= {n for n in conn1 if conn1[n] > conn0.get(n, 0)}
        bad = sorted(n for n in bad if n and n in changed_nets(snap, cur))
        print(f'DRC: {drc_counts(d1)} (before {drc_counts(d0)}) | nets to restore: {bad}')
        if not bad: break
        restore_nets(snap, bad, tmp); restored |= set(bad)
        refresh(repour)
    cur = K.dump(os.path.join(rd, 'after.json'))
    kept = sorted(set(changed_nets(snap, cur)) - restored)
    json.dump({'nets': todo, 'beautified': kept, 'restored': sorted(restored), 'drc_before': drc_counts(d0),
               'drc_after': drc_counts(d1)}, open(os.path.join(rd, 'report.json'), 'w'), indent=1)
    print(f'beautified {len(kept)} nets, restored {len(restored)}: {sorted(restored)} | snapshot {os.path.join(rd, "before.json")}')


if __name__ == '__main__':
    if len(sys.argv) < 2: raise SystemExit(__doc__)
    cmd, rest = sys.argv[1], sys.argv[2:]
    {'place': cmd_place, 'route-short': cmd_route_short, 'beautify': cmd_beautify,
     'restore-copper': cmd_restore}.get(cmd, lambda a: (_ for _ in ()).throw(SystemExit(__doc__)))(rest)
