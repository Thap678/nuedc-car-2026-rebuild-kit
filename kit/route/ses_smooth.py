"""Straighten the grid staircases of a routed SES (route_bundle.py output) without moving the routing.

1. merge collinear same-direction vertices (float tolerant: grid points are not exactly collinear in floats)
2. octilinear string pulling: per wire, from vertex i take the farthest vertex j (look-ahead --look) that one
   straight 0/45/90-degree segment or two of them (one 45-degree bend) can reach while keeping >= rules.copper_mm
   + 0.01 to every other-net copper on the layer, the edge clearance to the board edge, and staying within --dev mm
   of the routed path (so bundles stay bundles). Wire ends and every point where another track / via of the same
   net joins the wire (T-junctions) are kept, so connectivity cannot change.
On the nuedc-car-2026 board: 20433 -> 2616 segments with --dev 0.15, pour metrics unchanged; with no deviation
limit the shortcuts leave the bundles and the largest bottom ground piece shrank by 7 %.

usage: python ses_smooth.py <prepared.dsn> <in.ses> <out.ses> [--dev 0.15] [--look 80]
"""
import argparse
import math
import os
import sys

from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import common as K  # noqa: E402
from dsnses import Dsn, Ses, write_ses  # noqa: E402

B = K.try_board()
RULE = (B.rules.get('copper_mm', 0.15) if B else 0.15) + 0.01
EDGE = ((B.route.get('edge_clearance_um', 450) if B else 450) / 1000.0) - 0.03


def merge_collinear(pts):
    q = [pts[0]]
    for b in pts[1:]:
        if math.dist(q[-1], b) < 1e-6:
            continue
        if len(q) >= 2:
            a, m = q[-2], q[-1]
            u, v = (m[0] - a[0], m[1] - a[1]), (b[0] - m[0], b[1] - m[1])
            if abs(u[0] * v[1] - u[1] * v[0]) < 1e-6 * math.hypot(*u) * math.hypot(*v) and u[0] * v[0] + u[1] * v[1] > 0:
                q[-1] = b
                continue
        q.append(b)
    return q


def octi(p, q):
    """1- or 2-segment octilinear paths p -> q."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    ax, ay = abs(dx), abs(dy)
    if ax < 1e-6 or ay < 1e-6 or abs(ax - ay) < 1e-6:
        return [[p, q]]
    sx, sy, m = math.copysign(1, dx), math.copysign(1, dy), min(ax, ay)
    return [[p, (p[0] + sx * m, p[1] + sy * m), q], [p, (q[0] - sx * m, q[1] - sy * m), q]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dsn')
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--dev', type=float, default=0.15)
    ap.add_argument('--look', type=int, default=80)
    a = ap.parse_args()
    dsn, s = Dsn(a.dsn), Ses(a.inp)
    edge = Polygon(dsn.outline).buffer(0).exterior
    vr = s.via_diam / 2
    wires = [dict(net=w['net'], layer=w['layer'], width=w['width'], pts=merge_collinear([tuple(p) for p in w['pts']]))
             for w in s.wires]
    wires = [w for w in wires if len(w['pts']) >= 2]
    vias = [dict(net=v['net'], x=v['x'], y=v['y'], padstack=v['padstack']) for v in s.vias]
    pads = {L: [(q['net'] or '-', Point(q['x'], q['y']).buffer(q['r'], 24) if q['kind'] == 'circle'
                 else Polygon(q['pts']).buffer(0)) for q in dsn.pads if q['layer'] == L] for L in ('TopLayer', 'BottomLayer')}
    before = sum(len(w['pts']) - 1 for w in s.wires)
    for k, w in enumerate(wires):
        if len(w['pts']) < 3:
            continue
        L, net, hw = w['layer'], w['net'], w['width'] / 2
        obs = [g for nn, g in pads[L] if nn != net]
        obs += [Point(v['x'], v['y']).buffer(vr, 16) for v in vias if v['net'] != net]
        obs += [LineString(o['pts']).buffer(o['width'] / 2, 8) for j, o in enumerate(wires)
                if j != k and o['layer'] == L and o['net'] != net]
        tree = STRtree(obs)
        pts = list(w['pts'])
        joins = [o['pts'][e] for j, o in enumerate(wires) if j != k and o['net'] == net and o['layer'] == L for e in (0, -1)]
        joins += [(v['x'], v['y']) for v in vias if v['net'] == net]
        anchor = {0, len(pts) - 1}
        for jp in joins:
            best = None
            for i in range(len(pts) - 1):
                dd = LineString([pts[i], pts[i + 1]]).distance(Point(jp))
                if dd < hw and (best is None or dd < best[0]):
                    best = (dd, i)
            if best is None:
                continue
            i = best[1]
            if math.dist(jp, pts[i]) < 1e-6:
                anchor.add(i)
            elif math.dist(jp, pts[i + 1]) < 1e-6:
                anchor.add(i + 1)
            else:                                   # the joint is inside a segment: make it a vertex
                seg = LineString([pts[i], pts[i + 1]])
                pp = seg.interpolate(seg.project(Point(jp)))
                pts.insert(i + 1, (pp.x, pp.y))
                anchor = {x + (x > i) for x in anchor} | {i + 1}

        def ok(path):
            g = LineString(path).buffer(hw, 8)
            return g.distance(edge) >= EDGE and all(g.distance(obs[j]) >= RULE for j in tree.query(g.buffer(RULE)))

        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            nxt = min(x for x in anchor if x > i)
            took = (i + 1, [pts[i], pts[i + 1]])
            for j in range(min(nxt, i + a.look), i + 1, -1):
                orig = LineString(pts[i:j + 1])
                cand = next((c for c in octi(pts[i], pts[j])
                             if LineString(c).hausdorff_distance(orig) <= a.dev and ok(c)), None)
                if cand:
                    took = (j, cand)
                    break
            out += took[1][1:]
            i = took[0]
        w['pts'] = merge_collinear(out)
    write_ses(a.inp, a.out, wires, vias)
    print('segments', before, '->', sum(len(w['pts']) - 1 for w in wires), '| wrote', a.out)


if __name__ == '__main__':
    main()
