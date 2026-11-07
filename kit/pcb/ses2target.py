"""Build a `guard.py restore-copper` target from a routed SES: the live dump with the copper of the nets that
differ between <base.ses> and <cand.ses> replaced by <cand.ses> geometry; everything else stays as it is.

usage: python ses2target.py <live_dump.json> <base.ses> <cand.ses> <out_target.json>
       python guard.py restore-copper out_target.json [--net N ...]     (rips up and redraws only those nets,
       reads back, re-pours, saves), then python cover_check.py out_target.json <new dump>
base.ses = the SES the live board was made from (same frame as the dump; for a full re-route any SES of the
board works - every net then differs). Prints the changed nets.
"""
import json, os, re, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'route'))
from dsnses import Ses

LAYER = {'TopLayer': 1, 'BottomLayer': 2}


def parse(path):
    """{net: (sorted segments, sorted vias)} in dump units (mil)."""
    t = open(path, encoding='utf-8').read()
    s = Ses(path)
    ps = {m.group(1): int(m.group(2)) * s.unit_mm for m in re.finditer(r'\(padstack (\S+)\s*\(shape\s*\(circle \S+ (\d+)', t)}
    mil = lambda v: round(v / 0.0254, 3)
    nets = {}
    for w in s.wires:
        segs = nets.setdefault(w['net'], ([], []))[0]
        pts = [(mil(x), mil(y)) for x, y in w['pts']]
        segs += [(LAYER[w['layer']], mil(w['width']), a, b) for a, b in zip(pts, pts[1:])]
    for v in s.vias:
        nets.setdefault(v['net'], ([], []))[1].append((mil(ps.get(v['padstack'], s.via_diam)), mil(v['x']), mil(v['y'])))
    return {n: (sorted(a), sorted(b)) for n, (a, b) in nets.items()}


if __name__ == '__main__':
    dump_p, base_p, cand_p, out_p = sys.argv[1:5]
    base, cand = parse(base_p), parse(cand_p)
    changed = sorted(n for n in set(base) | set(cand) if base.get(n) != cand.get(n))
    d = json.load(open(dump_p, encoding='utf-8'))
    cu = d['copper']
    S = set(changed)
    cu['lines'] = [l for l in cu['lines'] if l.get('net') not in S]
    cu['vias'] = [v for v in cu['vias'] if v.get('net') not in S]
    for n in changed:
        segs, vias = cand.get(n, ([], []))
        cu['lines'] += [dict(net=n, layer=L, lineWidth=round(w, 3), startX=a[0], startY=a[1], endX=b[0], endY=b[1],
                             locked=False, primitiveId=None) for L, w, a, b in segs]
        cu['vias'] += [dict(net=n, x=x, y=y, diameter=dia, holeDiameter=dia / 2, locked=False, primitiveId=None)
                       for dia, x, y in vias]
    json.dump(d, open(out_p, 'w', encoding='utf-8'), ensure_ascii=False)
    print('changed nets', len(changed), ' '.join(changed))
