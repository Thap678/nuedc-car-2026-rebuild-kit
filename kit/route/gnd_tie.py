"""Tie every ground island of the poured board to the main ground with routed copper.

Reads a dump taken AFTER pouring (gndconn_lib.islands: group 0 = main ground). Every other group - a ground pad
the pour could not reach, a pour piece enclosed by track bundles, a stub - gets a connection on either layer
(vias allowed) from its copper to the main ground's pour, found with route_bundle's search and checked exactly.
Most ties are a single via: the island lies over main ground on the other layer. Vias keep hole-to-hole from
through-hole pads. Islands of less than --min-area mm2 with no pad are left (crumbs).
--drop x,y (mm, repeatable) removes the nearest via first, with ground stubs ending on it (e.g. a via DRC flags
for hole-to-hole).

Note: set the pours to connect pads directly first (connectMode 1 in Plane > Copper Zone, see docs/05) - with
thermal spokes a pad boxed in by tracks stays unconnected even where the pour reaches it.

usage: python gnd_tie.py <prepared.dsn> <dump_after_pour.json> <in.ses> <out.ses> [--drop x,y] [--min-area 1]
then import only the ground net: ses2target.py live.json in.ses out.ses target.json; guard.py restore-copper
target.json --net GND
"""
import argparse
import math
import os
import sys

import numpy as np
from shapely.geometry import LineString, Point

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import route_bundle as RB  # noqa: E402
import gndconn_lib as G  # noqa: E402
from dsnses import Dsn, Ses, write_ses  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dsn')
    ap.add_argument('dump')
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--drop', action='append', default=[])
    ap.add_argument('--min-area', type=float, default=1.0)
    a = ap.parse_args()
    s = Ses(a.inp)
    wires = [dict(net=w['net'], layer=w['layer'], width=w['width'], pts=[tuple(p) for p in w['pts']]) for w in s.wires]
    vias = [dict(net=v['net'], x=v['x'], y=v['y'], padstack=v['padstack']) for v in s.vias]
    for d in a.drop:
        x, y = map(float, d.split(','))
        k = min(range(len(vias)), key=lambda i: math.dist((vias[i]['x'], vias[i]['y']), (x, y)))
        p = (vias[k]['x'], vias[k]['y'])
        print('drop via', vias[k]['net'], tuple(round(c, 2) for c in p))
        vias.pop(k)
        wires = [w for w in wires if not (w['net'] == RB.GND and min(math.dist(w['pts'][0], p), math.dist(w['pts'][-1], p)) < 0.05)]
    tmp = a.out + '.tmp.ses'
    write_ses(a.inp, tmp, wires, vias)
    args = argparse.Namespace(open=0.0, d=1.0, bot=1.0, bot_hug=False, via=2.0, pen=30.0, padpen=0.0, padzone=0.6,
                              maxw=0.3, width=RB.B.rules.get('track_default_mm', 0.25) if RB.B else 0.25,
                              keep_gnd=True, fix_all=True)
    R = RB.Router(Dsn(a.dsn), tmp, args)
    n, nx, ny, hw, vr = R.n, R.nx, R.ny, R.hw, R.vr
    items, gs = G.islands(a.dump, RB.GND)
    tgt_m = np.zeros((2, ny, nx), bool)
    for kind, lays, geo, lab in gs[0]:
        if kind == 'pour':
            g2 = geo.buffer(-(hw + 0.05))
            for L in lays:
                if not g2.is_empty:
                    RB.paint(tgt_m[L - 1], R.g.geom(g2))
    own_plain, own_inf, _ = R.own_masks(RB.GND)
    fixed_t = (R.fix_inf[hw] - own_inf[hw]) > 0
    fixed_v = (R.fix_inf[vr] - own_inf[vr]) > 0
    via_ok = ~fixed_v[0] & ~fixed_v[1] & R.edge[vr] & ~R.tht_keep & ~own_plain[0] & ~own_plain[1]
    for v in vias:                                   # keep new vias off the existing ones (hole-to-hole)
        j0, i0, m = R.g.geom(Point(v['x'], v['y']).buffer(R.vd + 0.1))
        via_ok[j0:j0 + m.shape[0], i0:i0 + m.shape[1]] &= ~m
    legal = (~fixed_t & R.edge[hw][None]) | own_plain
    cost = np.ones(2 * n, np.float32)
    tied = 0
    for gi, grp in enumerate(gs[1:], 1):
        area = sum(x[2].area for x in grp if x[0] == 'pour')
        pads = [x[3] for x in grp if x[0] == 'pad']
        if not pads and area < a.min_area:
            continue
        src = np.zeros((2, ny, nx), bool)
        for kind, lays, geo, lab in grp:
            g2 = geo if kind != 'pour' else geo.buffer(-(hw + 0.05))
            for L in lays:
                if not g2.is_empty:
                    RB.paint(src[L - 1], R.g.geom(g2))
        srcs = np.flatnonzero((src & legal).reshape(-1)).astype(np.int64)
        tgt = np.zeros(2 * n, np.int32)
        tgt[tgt_m.reshape(-1)] = 1
        R.cur += 1
        st = RB.astar(cost, legal.reshape(-1), via_ok.reshape(-1), srcs, tgt, np.zeros(n, np.float32), nx, n,
                      RB.T45, RB.T90, RB.RES, 2.0, R.dist, R.par, R.stamp, R.cur) if len(srcs) else -1
        if st < 0:
            print(f'island {gi} {pads} {area:.0f} mm2: no path (boxed in on both layers - re-route a neighbour)')
            continue
        w2, v2 = R._split(R._path(st))
        r = dict(wires=w2, vias=v2)
        bad, _ = R.check(f'{RB.GND}#tie{gi}', r)
        if bad:
            print(f'island {gi} {pads}: clearance fails at {bad[:2]}')
            continue
        R.commit(f'{RB.GND}#tie{gi}', r)
        for li, pts in w2:
            RB.paint(tgt_m[li], R.g.geom(LineString(pts).buffer(hw * 0.5)))
        L = sum(math.dist(p, q) for _, pts in w2 for p, q in zip(pts, pts[1:]))
        print(f'island {gi} {pads} {area:.0f} mm2: tied with {L:.1f} mm track, {len(v2)} via(s)')
        tied += 1
    for key, r in R.routes.items():
        wires += [dict(net=RB.GND, layer=RB.LAY[li], width=2 * hw, pts=pts) for li, pts in r['wires']]
        vias += [dict(net=RB.GND, x=x, y=y, padstack=R.padstack) for x, y in r['vias']]
    write_ses(a.inp, a.out, wires, vias)
    os.remove(tmp)
    print(f'tied {tied} of {len(gs) - 1} islands | wrote {a.out}')


if __name__ == '__main__':
    main()
