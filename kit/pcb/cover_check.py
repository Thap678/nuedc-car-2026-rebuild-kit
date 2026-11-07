"""Did every track of a restore target really land on the board? Independent of guard.py's own comparison.

Each non-degenerate target track must be covered by live tracks of the same net / layer / width (its start, middle
and end within 0.1 mil of a live track - EasyEDA merges collinear pieces, so 1:1 matching does not work), and every
target via must exist. guard.py restore-copper's read-back once reported success while a vertical 5V run was
missing (fixed in its canon(), 2026-11-07); run this after every restore.

usage: python cover_check.py <target.json> <live_dump.json> [missing_out.json]
missing_out.json ({"lines": [...], "vias": [...]}) can be redrawn with guard.draw(), e.g.
  python -c "import guard, json; m = json.load(open('missing_out.json')); [guard.draw(k, t) for k in m for t in m[k]]"
"""
import json, math, sys
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree

t, l = (json.load(open(f, encoding='utf-8')) for f in sys.argv[1:3])
live = l['copper']['lines']
geo = [LineString([(x['startX'], x['startY']), (x['endX'], x['endY'])]) if (x['startX'], x['startY']) != (x['endX'], x['endY'])
       else Point(x['startX'], x['startY']) for x in live]
tree = STRtree(geo)
miss = []
for s in t['copper']['lines']:
    a, b = (s['startX'], s['startY']), (s['endX'], s['endY'])
    if math.dist(a, b) < 0.05:
        continue
    for p in (a, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), b):
        P = Point(p)
        if not any(live[j]['net'] == s['net'] and live[j]['layer'] == s['layer'] and abs(live[j]['lineWidth'] - s['lineWidth']) < 0.2
                   and geo[j].distance(P) < 0.1 for j in tree.query(P.buffer(0.2))):
            miss.append(s)
            break
vm = [v for v in t['copper']['vias'] if not any(w['net'] == v['net'] and math.dist((w['x'], w['y']), (v['x'], v['y'])) < 0.2
                                                 for w in l['copper']['vias'])]
print('target tracks not on the board:', len(miss), sorted({m['net'] for m in miss}), '| vias missing:', len(vm))
if len(sys.argv) > 3:
    json.dump(dict(lines=miss, vias=vm), open(sys.argv[3], 'w'))
sys.exit(1 if miss or vm else 0)
