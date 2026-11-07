"""Bundling router: route the signal nets from the pins so tracks run in even-pitch bundles and the ground
pours stay in large pieces (two layers, it places its own vias).

Why: Freerouting minimises length and vias, so a low-speed board ends up with lonely tracks spread over both
layers that cut the pours into pieces. Pushing those tracks around afterwards (balloon / shove) cannot fix it:
which corridor a net takes is decided by where its vias are. This router decides that itself.

usage: python route_bundle.py <prepared.dsn> <fixed.ses> <out.ses> [options]
  fixed.ses   a routed SES whose wide nets (track width > --maxw, i.e. power) are kept as they are; every other
              net except ground is routed again from its pads. Ground is not routed: every SMD ground pad gets a
              short stub + via first (the pours join them), unless --keep-gnd keeps the ground copper of fixed.ses.
Cost per mm of track:
  top      1 + OPEN * min(d - dmin, D) / D   (d = distance to the nearest other track; dmin = clearance + half
           width): a track right at clearance from another costs 1, one in open ground 1 + OPEN, so tracks follow
           each other into bundles. Pads are not attractors; within --padzone of a pad costs --padpen more, so pins
           routed later can still get out.
  bottom   BOT (x the same hug factor unless --no-bot-hug: bottom tracks bundle too)
  via      VIA each; turns 45 deg T45, 90 deg T90, sharper not allowed (45-degree routing)
Defaults (bot 1.5, via 25, polish 1) were picked on the nuedc-car-2026 upper board (80 x 87 mm, 81 signal nets):
signal vias about Freerouting's count, largest bottom ground piece +40 %, see docs/05. Bottom 10x / via 3-10
gives a near-solid bottom plane but 2-3x the vias.

Search: A* on a GRID mm grid (state = layer, cell, heading), numba. Clearance is hard: a cell is legal when its
centre is outside every other-net copper grown by (router clearance) + own half width; vias also keep
hole-to-hole from every through-hole pad. Every finished net is checked exactly with shapely (>= rules.copper_mm
+ 0.01) and retried with the violating spot forbidden. A net that cannot be routed is searched again with
routed signal copper passable at a penalty; the nets on that path are ripped up and queued again (their history
cost grows each time). Polish: with everything in place each net is routed once more with dearer bottom / vias
and kept if it has no more vias.

Output tracks are staircases on the grid: run ses_smooth.py next (straightens them, 20k -> 3k segments), then
ses_check.py, then import (ses2target.py + guard.py restore-copper, or apply_ses.py).
Needs numba (requirements.txt). Runtime: ~15 min for 81 nets on 80 x 87 mm, ~1.5 GB RAM.
"""
import argparse
import collections
import heapq
import math
import os
import sys
import time

import numba as nb
import numpy as np
import shapely
from scipy import ndimage
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import common as K  # noqa: E402
from dsnses import Dsn, Ses, write_ses  # noqa: E402

B = K.try_board()
RES = 0.05                                                              # grid, mm
SLACK = 0.02                                                            # grid inflation beyond the clearance
CLR = ((B.route.get('router_clearance_mm', 0.2) if B else 0.2) - SLACK)
RULE = (B.rules.get('copper_mm', 0.15) if B else 0.15) + 0.01           # exact check
EDGE = ((B.route.get('edge_clearance_um', 450) if B else 450) / 1000.0)
TRACK = None                                                            # set from config / --width in main
GND = B.gnd if B else 'GND'
HOLE_KEEP = (B.rules.get('hole_to_hole_mm', 0.4) if B else 0.4) + 0.1   # via centre beyond a through-hole pad edge
T45, T90 = 0.25, 0.8
LAY = ('TopLayer', 'BottomLayer')
DX = np.array([1, 1, 0, -1, -1, -1, 0, 1], np.int64)
DY = np.array([0, 1, 1, 1, 0, -1, -1, -1], np.int64)


# ------------------------------------------------------------------ grid rasters
class Grid:
    def __init__(self, W, H):
        self.nx, self.ny = int(math.ceil(W / RES)) + 1, int(math.ceil(H / RES)) + 1
        self.xc = np.arange(self.nx) * RES
        self.yc = np.arange(self.ny) * RES

    def box(self, x0, y0, x1, y1):
        i0, i1 = max(0, int(math.floor(x0 / RES))), min(self.nx, int(math.ceil(x1 / RES)) + 1)
        j0, j1 = max(0, int(math.floor(y0 / RES))), min(self.ny, int(math.ceil(y1 / RES)) + 1)
        return i0, i1, j0, j1

    def geom(self, g):
        """(j0, i0, mask): cells whose centre lies in shapely geometry g."""
        i0, i1, j0, j1 = self.box(*g.bounds)
        if i0 >= i1 or j0 >= j1:
            return (0, 0, np.zeros((0, 0), bool))
        X, Y = np.meshgrid(self.xc[i0:i1], self.yc[j0:j1])
        return (j0, i0, shapely.contains_xy(g, X, Y))

    def track(self, pts, hw):
        """[(j0, i0, mask)] per segment: cells within hw of the centre line."""
        out = []
        for a, b in zip(pts, pts[1:]):
            i0, i1, j0, j1 = self.box(min(a[0], b[0]) - hw, min(a[1], b[1]) - hw, max(a[0], b[0]) + hw, max(a[1], b[1]) + hw)
            X, Y = np.meshgrid(self.xc[i0:i1], self.yc[j0:j1])
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy
            t = np.clip(((X - a[0]) * vx + (Y - a[1]) * vy) / L2, 0, 1) if L2 > 0 else np.zeros_like(X)
            out.append((j0, i0, np.hypot(X - a[0] - t * vx, Y - a[1] - t * vy) <= hw))
        return out


def add(arr, item, v):
    j0, i0, m = item
    arr[j0:j0 + m.shape[0], i0:i0 + m.shape[1]] += v * m


def paint(arr, item):
    j0, i0, m = item
    arr[j0:j0 + m.shape[0], i0:i0 + m.shape[1]] |= m


# ------------------------------------------------------------------ search
@nb.njit(cache=False)
def astar(cost, legal, via_ok, srcs, tgt, h, nx, n, t45, t90, res, via_cost, dist, par, stamp, cur):
    """A* over (layer, cell, heading); returns the reached state (or -1). par/dist/stamp are reused buffers."""
    heap = [(np.float32(0.0), np.int64(0))]
    heap.pop()
    for node in srcs:
        c = node % n
        for d in range(8):
            s = node * 8 + d
            stamp[s] = cur
            dist[s] = 0.0
            par[s] = -1
            heap.append((np.float32(h[c]), np.int64(s)))
    heapq.heapify(heap)
    sq2 = math.sqrt(2.0)
    ny = n // nx
    while len(heap):
        f, s = heapq.heappop(heap)
        g = dist[s]
        node = s // 8
        c = node % n
        if f > g + h[c] + 1e-4:
            continue
        if tgt[node] > 0:
            return s
        l = node // n
        d = s % 8
        cx = c % nx
        cy = c // nx
        for nd in range(8):
            t = abs(nd - d)
            if t > 4:
                t = 8 - t
            if t > 2:
                continue
            pen = 0.0 if t == 0 else (t45 if t == 1 else t90)
            x = cx + DX[nd]
            y = cy + DY[nd]
            if x < 0 or y < 0 or x >= nx or y >= ny:
                continue
            m = y * nx + x
            mn = l * n + m
            if not legal[mn]:
                continue
            L = res * (sq2 if nd % 2 else 1.0)
            v = g + L * 0.5 * (cost[node] + cost[mn]) + pen
            ns = mn * 8 + nd
            if stamp[ns] != cur or v < dist[ns]:
                stamp[ns] = cur
                dist[ns] = v
                par[ns] = s
                heapq.heappush(heap, (np.float32(v + h[m]), np.int64(ns)))
        if via_ok[c]:
            on = (1 - l) * n + c
            if legal[on]:
                v = g + via_cost
                ns = on * 8 + d
                if stamp[ns] != cur or v < dist[ns]:
                    stamp[ns] = cur
                    dist[ns] = v
                    par[ns] = s
                    heapq.heappush(heap, (np.float32(v + h[c]), np.int64(ns)))
    return -1


# ------------------------------------------------------------------ router
class Router:
    def __init__(self, dsn, fixed_ses, args):
        self.a = args
        self.dsn = dsn
        self.g = g = Grid(*dsn.board)
        self.nx, self.ny = g.nx, g.ny
        self.n = g.nx * g.ny
        ses = Ses(fixed_ses)
        self.vd = ses.via_diam
        self.vr = self.vd / 2
        self.padstack = ses.vias[0]['padstack'] if ses.vias else 'via0'
        wc = collections.defaultdict(collections.Counter)
        for w in ses.wires:
            wc[w['net']][round(w['width'], 3)] += w['length']
        self.fixed = {n for n, c in wc.items() if c.most_common(1)[0][0] > args.maxw}
        if args.keep_gnd:
            self.fixed.add(GND)
        if getattr(args, 'fix_all', False):
            self.fixed = set(wc)
        self.fixed_wires = [w for w in ses.wires if w['net'] in self.fixed]
        self.fixed_vias = [v for v in ses.vias if v['net'] in self.fixed]
        self.hw = args.width / 2
        self.H = (self.hw, self.vr)                     # inflations: track half width, via radius
        shp = (2, self.ny, self.nx)
        self.fix_plain = np.zeros(shp, np.int16)
        self.fix_inf = {h: np.zeros(shp, np.int16) for h in self.H}
        self.sig_plain = np.zeros(shp, np.int16)
        self.sig_inf = {h: np.zeros(shp, np.int16) for h in self.H}
        self.fixed_geoms = {L: [] for L in LAY}         # (net, geom) for the exact check
        self.pad_geo = {}                               # pin -> {layer index: (geom, net)}
        self.pins = collections.defaultdict(list)       # net -> pins
        self.tht_keep = np.zeros((self.ny, self.nx), bool)
        pz = np.zeros((self.ny, self.nx), bool)
        for p in dsn.pads:
            geo = Point(p['x'], p['y']).buffer(p['r'], 24) if p['kind'] == 'circle' else Polygon(p['pts']).buffer(0)
            li = LAY.index(p['layer'])
            self.pad_geo.setdefault(p['pin'], {})[li] = (geo, p['net'] or '')
            self._stamp_fixed(li, geo)
            self.fixed_geoms[p['layer']].append((p['net'] or '-' + p['pin'], geo))
            if p['net'] and p['pin'] not in self.pins[p['net']]:
                self.pins[p['net']].append(p['pin'])
            if li == 0:
                paint(pz, g.geom(geo.buffer(args.padzone, 12)))
            if li == 1 and p['kind'] == 'circle':    # through-hole pad (on both layers): hole-to-hole keep for vias
                paint(self.tht_keep, g.geom(geo.buffer(HOLE_KEEP + self.vr, 16)))
        self.pad_zone = pz
        self.trk = np.zeros(shp, np.int16)               # fixed tracks: attractors for the hug cost
        for w in self.fixed_wires:
            li = LAY.index(w['layer'])
            geo = LineString(w['pts']).buffer(w['width'] / 2, 12)
            self._stamp_fixed(li, geo)
            self.fixed_geoms[w['layer']].append((w['net'], geo))
            for it in g.track(w['pts'], w['width'] / 2):
                add(self.trk[li], it, 1)
        for v in self.fixed_vias:
            geo = Point(v['x'], v['y']).buffer(self.vr, 16)
            for li in (0, 1):
                self._stamp_fixed(li, geo)
                self.fixed_geoms[LAY[li]].append((v['net'], geo))
        self.outline = Polygon(dsn.outline).buffer(0)
        self.edge = {}
        for h in self.H:
            m = np.zeros((self.ny, self.nx), bool)
            paint(m, g.geom(self.outline.buffer(-(EDGE + h))))
            self.edge[h] = m
        self.routes = {}                                 # key (net, or GND#pin for a fanout) -> route
        S = 2 * self.n * 8
        self.dist = np.zeros(S, np.float32)
        self.par = np.zeros(S, np.int64)
        self.stamp = np.zeros(S, np.int32)
        self.cur = 0
        self.hist = collections.Counter()

    # -- rasters of fixed / routed copper
    def _stamp_fixed(self, li, geo):
        add(self.fix_plain[li], self.g.geom(geo), 1)
        for h in self.H:
            add(self.fix_inf[h][li], self.g.geom(geo.buffer(CLR + SLACK + h, 12)), 1)

    def _items(self, r):
        out = []
        for li, pts in r['wires']:
            plain = self.g.track(pts, self.hw)
            inf = {h: self.g.track(pts, self.hw + CLR + SLACK + h) for h in self.H}
            for k in range(len(plain)):
                out.append((li, plain[k], {h: inf[h][k] for h in self.H}))
        for x, y in r['vias']:
            geo = Point(x, y).buffer(self.vr, 16)
            for li in (0, 1):
                out.append((li, self.g.geom(geo), {h: self.g.geom(geo.buffer(CLR + SLACK + h, 12)) for h in self.H}))
        return out

    def _geoms(self, r):
        gs = {L: [] for L in LAY}
        for li, pts in r['wires']:
            gs[LAY[li]].append(LineString(pts).buffer(self.hw, 12))
        for x, y in r['vias']:
            for L in LAY:
                gs[L].append(Point(x, y).buffer(self.vr, 16))
        return gs

    def _put(self, key, sgn):
        for li, plain, inf in self.routes[key]['items']:
            add(self.sig_plain[li], plain, sgn)
            for h in self.H:
                add(self.sig_inf[h][li], inf[h], sgn)

    def rip(self, key):
        if key in self.routes:
            self._put(key, -1)
            del self.routes[key]

    def commit(self, key, r):
        r = dict(wires=r['wires'], vias=r['vias'])
        r['items'] = self._items(r)
        r['geoms'] = self._geoms(r)
        self.routes[key] = r
        self._put(key, 1)

    def own_masks(self, net):
        plain = np.zeros((2, self.ny, self.nx), bool)
        inf = {h: np.zeros((2, self.ny, self.nx), np.int16) for h in self.H}
        pin_cells = {}
        for pin in self.pins[net]:
            cells = []
            for li, (geo, _) in self.pad_geo[pin].items():
                m = np.zeros((self.ny, self.nx), bool)
                paint(m, self.g.geom(geo))
                plain[li] |= m
                cells.append((li, m))
                for h in self.H:
                    add(inf[h][li], self.g.geom(geo.buffer(CLR + SLACK + h, 12)), 1)
            pin_cells[pin] = cells
        return plain, inf, pin_cells

    def _hug(self, obst, dmin):
        d = ndimage.distance_transform_edt(~obst) * RES
        return 1.0 + self.a.open * np.clip(d - dmin, 0, self.a.d) / self.a.d

    def _path(self, st):
        nodes = []
        while st >= 0:
            nodes.append(st // 8)
            st = self.par[st]
        return nodes[::-1]

    def _split(self, nodes):
        """Grid path -> (wires [(layer, pts)], vias [(x, y)])."""
        n, nx = self.n, self.nx
        wires, vias = [], []
        run = [nodes[0]]
        for p, q in zip(nodes, nodes[1:]):
            if p // n != q // n:
                c = p % n
                vias.append(((c % nx) * RES, (c // nx) * RES))
                if len(run) > 1:
                    wires.append(self._poly(run))
                run = [q]
            else:
                run.append(q)
        if len(run) > 1:
            wires.append(self._poly(run))
        return wires, vias

    def _poly(self, run):
        n, nx = self.n, self.nx
        li = run[0] // n
        pts = [((c % nx) * RES, (c // nx) * RES) for c in (x % n for x in run)]
        poly = [pts[0]]
        for a, b, c in zip(pts, pts[1:], pts[2:]):
            if abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) > 1e-9:
                poly.append(b)
        poly.append(pts[-1])
        return (li, poly)

    # -- one signal net
    def route_net(self, net, soft=False, forbid=None):
        a = self.a
        n, nx, ny = self.n, self.nx, self.ny
        hw, vr = self.hw, self.vr
        own_plain, own_inf, pin_cells = self.own_masks(net)
        fixed_t = (self.fix_inf[hw] - own_inf[hw]) > 0
        fixed_v = (self.fix_inf[vr] - own_inf[vr]) > 0
        legal = ~fixed_t & self.edge[hw][None]
        if not soft:
            legal &= ~(self.sig_inf[hw] > 0)
        legal |= own_plain
        if forbid is not None:
            legal &= ~forbid
        via_ok = ~fixed_v[0] & ~fixed_v[1] & self.edge[vr] & ~own_plain[0] & ~own_plain[1] & ~self.tht_keep
        if not soft:
            via_ok &= ~(self.sig_inf[vr][0] > 0) & ~(self.sig_inf[vr][1] > 0)
        dmin = CLR + hw
        cost = np.empty((2, ny, nx), np.float32)
        cost[0] = self._hug((self.trk[0] + self.sig_plain[0]) > 0, dmin)
        if a.padpen > 0:
            cost[0] += a.padpen * (self.pad_zone & ~own_plain[0])
        cost[1] = a.bot * (self._hug((self.trk[1] + self.sig_plain[1]) > 0, dmin) if a.bot_hug else 1.0)
        if soft:
            pen = np.zeros((2, ny, nx), np.float32)
            for other, r in self.routes.items():
                p = a.pen * (1 + self.hist[other])
                for li, plain, inf in r['items']:
                    j0, i0, m = inf[hw]
                    sub = pen[li, j0:j0 + m.shape[0], i0:i0 + m.shape[1]]
                    np.maximum(sub, p * m, out=sub)
            cost += pen
        legal_f, cost_f, via_f = legal.reshape(-1), cost.reshape(-1), via_ok.reshape(-1)
        pins = list(self.pins[net])
        tree = np.zeros((2, ny, nx), bool)
        for li, m in pin_cells[pins[0]]:
            tree[li] |= m
        todo = set(pins[1:])
        wires, vias = [], []
        while todo:
            tgt = np.zeros(2 * n, np.int32)
            tmask = np.zeros((ny, nx), bool)
            for k, pin in enumerate(pins):
                if pin in todo:
                    for li, m in pin_cells[pin]:
                        tgt[li * n:(li + 1) * n][m.reshape(-1)] = k + 1
                        tmask |= m
            h = (ndimage.distance_transform_edt(~tmask) * RES).astype(np.float32).reshape(-1)
            srcs = np.flatnonzero((tree & legal).reshape(-1)).astype(np.int64)
            if not len(srcs):
                return None
            self.cur += 1
            st = astar(cost_f, legal_f, via_f, srcs, tgt, h, nx, n, T45, T90, RES, a.via,
                       self.dist, self.par, self.stamp, self.cur)
            if st < 0:
                return None
            nodes = self._path(st)
            hit = tgt[nodes[-1]] - 1
            w2, v2 = self._split(nodes)
            wires += w2
            vias += v2
            for node in nodes:
                li, c = divmod(node, n)
                tree[li].reshape(-1)[c] = True
            for x, y in v2:
                c = int(round(y / RES)) * nx + int(round(x / RES))
                tree[0].reshape(-1)[c] = tree[1].reshape(-1)[c] = True
            for k, pin in enumerate(pins):
                if pin in todo and (k == hit or any((m & tree[li]).any() for li, m in pin_cells[pin])):
                    todo.discard(pin)
                    for li, m in pin_cells[pin]:
                        tree[li] |= m
        return dict(wires=wires, vias=sorted(set(vias)))

    def check(self, key, r):
        """Exact clearance of a candidate route: (violation spots [(layer, x, y)], keys of the copper hit)."""
        bad, crossed = [], set()
        gs = self._geoms(r)
        base = key.split('#')[0]
        for L in LAY:
            others = [(nn, g) for nn, g in self.fixed_geoms[L] if nn != base]
            others += [(k, g) for k, rr in self.routes.items() if k.split('#')[0] != base for g in rr['geoms'][L]]
            if not others or not gs[L]:
                continue
            tree = STRtree([g for _, g in others])
            for g in gs[L]:
                for j in tree.query(g.buffer(RULE)):
                    nn, og = others[j]
                    if g.distance(og) < RULE - 1e-6:
                        p = g.intersection(og.buffer(RULE)).centroid
                        bad.append((LAY.index(L), p.x, p.y))
                        crossed.add(nn)
                if g.distance(self.outline.exterior) < EDGE - 0.05:
                    bad.append((LAY.index(L), g.centroid.x, g.centroid.y))
        return bad, crossed

    def route_checked(self, net):
        forbid = None
        for _ in range(4):
            r = self.route_net(net, forbid=forbid)
            if r is None:
                return None
            bad, _ = self.check(net, r)
            if not bad:
                return r
            if forbid is None:
                forbid = np.zeros((2, self.ny, self.nx), bool)
            for li, x, y in bad:
                paint(forbid[li], self.g.geom(Point(x, y).buffer(0.25)))
        return None

    # -- ground fanout: stub from an SMD ground pad to the nearest legal via spot
    def fanout(self, pin):
        n, nx, ny = self.n, self.nx, self.ny
        hw, vr = self.hw, self.vr
        own_plain, own_inf, pin_cells = self.own_masks(GND)
        fixed_t = (self.fix_inf[hw] - own_inf[hw]) > 0
        fixed_v = (self.fix_inf[vr] - own_inf[vr]) > 0
        legal = np.zeros((2, ny, nx), bool)
        legal[0] = (~fixed_t[0] & ~(self.sig_inf[hw][0] > 0) & self.edge[hw]) | own_plain[0]
        spot = (~fixed_v[0] & ~fixed_v[1] & ~(self.sig_inf[vr][0] > 0) & ~(self.sig_inf[vr][1] > 0) & self.edge[vr]
                & ~own_plain[0] & ~own_plain[1] & ~self.tht_keep)
        tgt = np.zeros(2 * n, np.int32)
        tgt[:n][spot.reshape(-1)] = 1
        pad = np.zeros((ny, nx), bool)
        for li, m in pin_cells[pin]:
            if li == 0:
                pad |= m
        srcs = np.flatnonzero(pad.reshape(-1) & legal[0].reshape(-1)).astype(np.int64)
        if not len(srcs):
            return None
        self.cur += 1
        st = astar(np.ones(2 * n, np.float32), legal.reshape(-1), np.zeros(n, np.bool_), srcs, tgt,
                   np.zeros(n, np.float32), nx, n, T45, T90, RES, 1.0, self.dist, self.par, self.stamp, self.cur)
        if st < 0:
            return None
        nodes = self._path(st)
        c = nodes[-1] % n
        r = dict(wires=[self._poly(nodes)] if len(nodes) > 1 else [], vias=[((c % nx) * RES, (c // nx) * RES)])
        bad, _ = self.check(GND + '#' + pin, r)
        return None if bad else r

    def write(self, template, out):
        wires = [dict(net=w['net'], layer=w['layer'], width=w['width'], pts=[tuple(p) for p in w['pts']]) for w in self.fixed_wires]
        vias = [dict(net=v['net'], x=v['x'], y=v['y'], padstack=v['padstack']) for v in self.fixed_vias]
        for key, r in self.routes.items():
            net = key.split('#')[0]
            wires += [dict(net=net, layer=LAY[li], width=2 * self.hw, pts=pts) for li, pts in r['wires']]
            vias += [dict(net=net, x=x, y=y, padstack=self.padstack) for x, y in r['vias']]
        write_ses(template, out, wires, vias)


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dsn')
    ap.add_argument('fixed')
    ap.add_argument('out')
    ap.add_argument('--open', type=float, default=4.0, help='extra cost per mm in open ground (hug strength)')
    ap.add_argument('--d', type=float, default=0.3, help='mm beyond clearance at which a cell counts as open')
    ap.add_argument('--bot', type=float, default=1.5, help='bottom-layer cost factor')
    ap.add_argument('--no-bot-hug', dest='bot_hug', action='store_false', help='flat bottom cost (no bundling there)')
    ap.add_argument('--via', type=float, default=25.0, help='cost of one via (mm of hugging top track)')
    ap.add_argument('--padpen', type=float, default=2.0)
    ap.add_argument('--padzone', type=float, default=0.6)
    ap.add_argument('--pen', type=float, default=30.0, help='rip-up search: cost per mm through routed copper')
    ap.add_argument('--iters', type=int, default=400)
    ap.add_argument('--polish', type=int, default=1)
    ap.add_argument('--polish-mul', type=float, default=1.5)
    ap.add_argument('--maxw', type=float, default=0.3, help='nets with wider tracks in fixed.ses are kept')
    ap.add_argument('--width', type=float, default=None, help='signal track width (default rules.track_default_mm)')
    ap.add_argument('--keep-gnd', action='store_true', help='keep the ground copper of fixed.ses, no new fanout')
    return ap


def main():
    a = parser().parse_args()
    if a.width is None:
        a.width = B.rules.get('track_default_mm', 0.25) if B else 0.25
    dsn = Dsn(a.dsn)
    R = Router(dsn, a.fixed, a)
    nets = [n for n in R.pins if n != GND and n not in R.fixed and len(R.pins[n]) > 1]

    def span(n):
        c = [geo.centroid for p in R.pins[n] for geo, _ in R.pad_geo[p].values()]
        return (max(p.x for p in c) - min(p.x for p in c)) + (max(p.y for p in c) - min(p.y for p in c))
    queue = collections.deque(sorted(nets, key=lambda n: -span(n)))   # long nets first: they lay the trunks
    t0 = time.time()
    if not a.keep_gnd:
        smd = [p['pin'] for p in dsn.pads if p['net'] == GND and p['layer'] == LAY[0] and p['kind'] != 'circle']
        ok = 0
        for pin in smd:
            r = R.fanout(pin)
            if r is not None:
                R.commit(GND + '#' + pin, r)
                ok += 1
            else:
                print('  no ground fanout for pad', pin, flush=True)
        print(f'ground fanout {ok}/{len(smd)}', flush=True)
    print('signal nets', len(nets), flush=True)
    it = 0
    while queue and it < a.iters:
        it += 1
        net = queue.popleft()
        r = R.route_checked(net)
        if r is not None:
            R.commit(net, r)
            print(f'[{it}] {net}: {len(r["vias"])} vias ({time.time() - t0:.0f}s)', flush=True)
            continue
        soft = R.route_net(net, soft=True)
        if soft is None:
            print(f'[{it}] {net}: no path even through other nets', flush=True)
            continue
        _, crossed = R.check(net, soft)
        victims = [c for c in crossed if c in R.routes and '#' not in c]
        for v in victims:
            R.rip(v)
            R.hist[v] += 1
            queue.append(v)
        r = R.route_checked(net)
        if r is not None:
            R.commit(net, r)
            print(f'[{it}] {net}: ok after ripping {victims}', flush=True)
        else:
            queue.append(net)
            print(f'[{it}] {net}: still blocked after ripping {victims}', flush=True)
    for ps in range(a.polish):
        bot0, via0 = a.bot, a.via
        a.bot, a.via = bot0 * a.polish_mul, via0 * a.polish_mul
        fewer = 0
        for net in sorted([k for k in R.routes if k in nets], key=lambda k: -len(R.routes[k]['vias'])):
            old = R.routes[net]
            R.rip(net)
            r = R.route_checked(net)
            if r is not None and len(r['vias']) <= len(old['vias']):
                fewer += len(r['vias']) < len(old['vias'])
                R.commit(net, r)
            else:
                R.commit(net, old)
        a.bot, a.via = bot0, via0
        print(f'polish {ps + 1}: fewer vias on {fewer} nets ({time.time() - t0:.0f}s)', flush=True)
    missing = [n for n in nets if n not in R.routes]
    R.write(a.fixed, a.out)
    print('routed', len(nets) - len(missing), 'of', len(nets), 'missing', missing,
          '| signal vias', sum(len(r['vias']) for k, r in R.routes.items() if '#' not in k), '| wrote', a.out)


if __name__ == '__main__':
    main()
