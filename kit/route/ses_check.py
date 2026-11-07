"""Exact offline check of a routed SES before it goes to EasyEDA: clearance, board edge, connectivity.

clearance  every track / via vs every other-net pad, track and via on its layer (shapely), >= rules.copper_mm
edge       copper to the board outline >= rules.edge_mm
connected  per net, with EasyEDA's rule: copper must really overlap (every shape shrunk by 0.01 mm) - a track that
           only grazes a pad corner is NOT connected (EasyEDA DRC agrees; conn.py's endpoint rule does not, and it
           also reports a track that ends in the middle of another track - route_bundle.py's T-junctions - as open).
           The ground net is skipped by default: the pours join it (check that after pouring, gmetrics.py / DRC).

usage: python ses_check.py <prepared.dsn> <a.ses> [b.ses ...] [--with-gnd]
"""
import collections
import os
import sys

from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import common as K  # noqa: E402
from dsnses import Dsn, Ses  # noqa: E402

B = K.try_board()
RULE = B.rules.get('copper_mm', 0.15) if B else 0.15
EDGE = B.rules.get('edge_mm', 0.3) if B else 0.3
GND = B.gnd if B else 'GND'
LAYERS = ('TopLayer', 'BottomLayer')


def pad_geom(p, shrink=0.0):
    return (Point(p['x'], p['y']).buffer(p['r'] - shrink, 24) if p['kind'] == 'circle'
            else Polygon(p['pts']).buffer(0).buffer(-shrink))


def clearance(dsn, s):
    bad = []
    edge = Polygon(dsn.outline).buffer(0).exterior
    for L in LAYERS:
        items = [(q['net'] or '-' + q['pin'], pad_geom(q), 'pad') for q in dsn.pads if q['layer'] == L]
        items += [(v['net'], Point(v['x'], v['y']).buffer(s.via_diam / 2, 24), 'via') for v in s.vias]
        for w in s.wires:
            if w['layer'] == L:
                items += [(w['net'], LineString([a, b]).buffer(w['width'] / 2, 16), 'trk') for a, b in zip(w['pts'], w['pts'][1:])]
        tree = STRtree([g for _, g, _ in items])
        for i, (n, g, k) in enumerate(items):
            if k == 'pad':
                continue
            if g.distance(edge) < EDGE - 1e-4:
                bad.append((L, n, 'board edge', round(g.distance(edge), 3), tuple(round(c, 1) for c in g.centroid.coords[0])))
            for j in tree.query(g.buffer(RULE)):
                m, h, kk = items[j]
                if m != n and (kk == 'pad' or j > i) and g.distance(h) < RULE - 1e-4:
                    bad.append((L, n, m, round(g.distance(h), 3), tuple(round(c, 1) for c in g.centroid.coords[0])))
    return bad


def open_nets(dsn, s, eps=0.01, skip=(GND,)):
    """{net: number of separate copper groups holding pins} for nets that are not in one piece."""
    items = collections.defaultdict(list)
    for p in dsn.pads:
        if p['net']:
            items[p['net']].append((p['layer'], pad_geom(p, eps), ('pin', p['pin'])))
    for w in s.wires:
        for a, b in zip(w['pts'], w['pts'][1:]):
            items[w['net']].append((w['layer'], LineString([a, b]).buffer(w['width'] / 2 - eps, 12), None))
    for k, v in enumerate(s.vias):
        for L in LAYERS:
            items[v['net']].append((L, Point(v['x'], v['y']).buffer(s.via_diam / 2 - eps, 12), ('via', k)))
    bad = {}
    for net, its in items.items():
        if net in skip:
            continue
        par = list(range(len(its)))

        def f(i):
            while par[i] != i:
                par[i] = par[par[i]]
                i = par[i]
            return i
        first = {}
        for i, (L, g, tag) in enumerate(its):
            if tag is not None:
                if tag in first:
                    par[f(i)] = f(first[tag])          # the same pin / via on the other layer
                else:
                    first[tag] = i
        for L in LAYERS:
            idx = [i for i, it in enumerate(its) if it[0] == L]
            tree = STRtree([its[i][1] for i in idx])
            for i in idx:
                for b in tree.query(its[i][1], predicate='intersects'):
                    par[f(i)] = f(idx[b])
        groups = {f(i) for i, (L, g, tag) in enumerate(its) if tag and tag[0] == 'pin'}
        if len(groups) > 1:
            bad[net] = len(groups)
    return bad


if __name__ == '__main__':
    args = [x for x in sys.argv[1:] if not x.startswith('--')]
    skip = () if '--with-gnd' in sys.argv else (GND,)
    dsn = Dsn(args[0])
    rc = 0
    for p in args[1:]:
        s = Ses(p)
        bad, op = clearance(dsn, s), open_nets(dsn, s, skip=skip)
        print(f'{os.path.basename(p)}: clearance/edge violations {len(bad)} {bad[:6]} | open nets {len(op)} {op}')
        rc |= bool(bad or op)
    sys.exit(rc)
