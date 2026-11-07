"""Freerouting DSN / SES readers shared by the route scripts (ses_conn, ses_polish, render, batch_score, style_metrics).

  DSN  geometry only: pad shapes from `library/image/pin` + `padstack`, net members from `network`, the board outline
       from `boundary`.  NOTE: EasyEDA writes DSN structure/library coordinates in MIL even though it declares
       `(resolution mil 1000)`; the scale is auto-detected from the boundary bbox.
  SES  routed copper: `(path <layer> <width> x1 y1 ...)`, `(via <padstack> x y)`. Coordinates/widths are in resolution
       units (1/1000 mil -> /1000*0.0254 mm).
Everything comes out in mm.
"""
import math


# ----------------------------------------------------------------------------- S-expression parser
def parse_sexpr(text):
    stack, i, n = [[]], 0, len(text)
    while i < n:
        c = text[i]
        if c == '(':
            stack.append([])
            i += 1
        elif c == ')':
            node = stack.pop()
            if stack:
                stack[-1].append(node)
            i += 1
        elif c.isspace():
            i += 1
        elif c == '"':
            j = text.find('"', i + 1)
            stack[-1].append(text[i + 1:j])
            i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()"':
                j += 1
            stack[-1].append(text[i:j])
            i = j
    return stack[0][0] if stack and stack[0] else []


def child(node, head):
    for c in node[1:] if node and isinstance(node[0], str) else node:
        if isinstance(c, list) and c and c[0] == head:
            return c
    return None


def children(node, head):
    out = []
    for c in (node[1:] if node and isinstance(node[0], str) else node):
        if isinstance(c, list) and c and c[0] == head:
            out.append(c)
    return out


def fnum(x):
    return float(x)


# ----------------------------------------------------------------------------- DSN
class Dsn:
    """Pads (per layer) + net membership + board size from a Freerouting DSN."""

    def __init__(self, path):
        self.path = path
        text = open(path, encoding='utf-8', errors='replace').read()
        root = parse_sexpr(text)
        structure = child(root, 'structure')
        if structure is None:
            raise ValueError(f'{path}: no (structure ...)')
        # coordinate scale: EasyEDA writes DSN coords in mil regardless of (resolution mil 1000)
        bnd = child(structure, 'boundary')
        pts = [float(v) for v in child(bnd, 'path')[3:]]          # (path <layer> <width> x1 y1 x2 y2 ...)
        xs, ys = pts[0::2], pts[1::2]
        bbox = (min(xs), max(xs), min(ys), max(ys))
        self.unit_mm = 0.0254 if bbox[1] < 1e4 else 0.0254 / 1000       # mil vs 1/1000 mil
        # board size: the boundary starts at the origin and any inset (prep_dsn.py) is the same on both sides
        self.board = ((bbox[0] + bbox[1]) * self.unit_mm, (bbox[2] + bbox[3]) * self.unit_mm)
        self.outline = [(x * self.unit_mm, y * self.unit_mm) for x, y in zip(xs, ys)]   # routing boundary, mm
        self.inset = bbox[0] * self.unit_mm     # boundary inset from the board edge (0 raw, 0.15 prepared)

        lib = child(root, 'library')
        padstacks = {}
        for ps in children(lib, 'padstack'):
            shapes = []
            for sh in children(ps, 'shape'):
                if not (isinstance(sh[1], list) and sh[1]):
                    continue
                kind, layer = sh[1][0], sh[1][1]
                nums = [float(v) for v in sh[1][2:]]
                if kind == 'circle':
                    d = nums[0]
                    dx = nums[1] if len(nums) > 1 else 0.0
                    dy = nums[2] if len(nums) > 2 else 0.0
                    shapes.append((layer, 'circle', (dx, dy, d / 2)))
                elif kind == 'polygon':
                    xy = nums[1:]
                    shapes.append((layer, 'polygon', list(zip(xy[0::2], xy[1::2]))))
                elif kind == 'rect':
                    # (rect layer x1 y1 x2 y2)
                    x1, y1, x2, y2 = nums[:4]
                    shapes.append((layer, 'polygon', [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]))
            padstacks[ps[1]] = shapes

        net_of = {}
        net = child(root, 'network')
        self.nets = {}
        for nb in children(net, 'net'):
            name = nb[1]
            pins = child(nb, 'pins') or []
            refs = [p for p in pins[1:] if isinstance(p, str)]
            self.nets[name] = refs
            for r in refs:
                net_of[r.split('-', 1)[1] if '-' in r else r] = name
        self.pad_nets = net_of

        self.pads = []       # dict(x, y, layer, kind, geom, net)  coords already in mm
        image = child(lib, 'image')
        for pin in children(image, 'pin'):
            ps, name, x, y = pin[1], pin[2], float(pin[3]), float(pin[4])
            netname = net_of.get(name)
            for layer, kind, geom in padstacks.get(ps, []):
                if kind == 'circle':
                    dx, dy, r = geom
                    self.pads.append(dict(x=(x + dx) * self.unit_mm, y=(y + dy) * self.unit_mm,
                                          layer=layer, kind='circle', r=r * self.unit_mm,
                                          net=netname, pin=name))
                else:
                    self.pads.append(dict(x=x * self.unit_mm, y=y * self.unit_mm, layer=layer,
                                          kind='polygon',
                                          pts=[((x + px) * self.unit_mm, (y + py) * self.unit_mm)
                                               for px, py in geom],
                                          net=netname, pin=name))


# ----------------------------------------------------------------------------- SES
class Ses:
    def __init__(self, path):
        self.path = path
        root = parse_sexpr(open(path, encoding='utf-8', errors='replace').read())
        routes = child(root, 'routes')
        if routes is None:
            raise ValueError(f'{path}: no (routes ...)')
        res = child(routes, 'resolution')
        self.unit_mm = 0.0254 / int(res[2]) if res and res[1] == 'mil' else 0.0254

        lib = child(routes, 'library_out')
        self.via_diam = 0.6
        for ps in children(lib, 'padstack'):
            for sh in children(ps, 'shape'):
                if isinstance(sh[1], list) and sh[1][0] == 'circle' and sh[1][1] == 'TopLayer':
                    self.via_diam = float(sh[1][2]) * self.unit_mm
                    break
        self.wires = []      # dict(net, layer, width_mm, pts_mm, length_mm)
        self.vias = []       # dict(net, x, y, r_mm, padstack)
        self.degenerate = 0  # paths with 0/1 points (Freerouting occasionally emits these)
        netout = child(routes, 'network_out')
        for nb in children(netout, 'net'):
            netname = nb[1]
            for w in children(nb, 'wire'):
                p = child(w, 'path')
                if p is None:
                    continue
                layer, width = p[1], float(p[2])
                xy = [float(v) for v in p[3:]]
                pts = [(xy[k] * self.unit_mm, xy[k + 1] * self.unit_mm) for k in range(0, len(xy) - 1, 2)]
                if len(pts) < 2:
                    self.degenerate += 1
                    continue
                L = sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                        for k in range(len(pts) - 1))
                self.wires.append(dict(net=netname, layer=layer, width=width * self.unit_mm, pts=pts,
                                       length=L))
            for v in children(nb, 'via'):
                self.vias.append(dict(net=netname, x=float(v[2]) * self.unit_mm,
                                      y=float(v[3]) * self.unit_mm, padstack=v[1]))


def write_ses(template, out_path, wires, vias):
    """Write an SES: `template`'s header / library with its whole (network_out ...) replaced by wires / vias.

    wires: dicts net, layer, width (mm), pts [(x, y) mm]; vias: dicts net, x, y (mm), padstack. Units follow the
    template's resolution, so the result imports wherever the template did."""
    import collections
    text = open(template, encoding='utf-8').read()
    unit = Ses(template).unit_mm
    i0 = text.index('(network_out')
    depth, j = 0, i0
    while True:
        depth += {'(': 1, ')': -1}.get(text[j], 0)
        if depth == 0:
            break
        j += 1
    nets = collections.OrderedDict()
    for w in wires:
        nets.setdefault(w['net'], ([], []))[0].append(w)
    for v in vias:
        nets.setdefault(v['net'], ([], []))[1].append(v)
    out = ['(network_out']
    for name, (wl, vl) in nets.items():
        out.append('  (net %s' % (name if name.replace('_', '').isalnum() else '"%s"' % name))
        for w in wl:
            out.append('    (wire (path %s %d %s))' % (w['layer'], int(round(w['width'] / unit)), ' '.join(
                '%d %d' % (int(round(x / unit)), int(round(y / unit))) for x, y in w['pts'])))
        for v in vl:
            out.append('    (via %s %d %d)' % (v.get('padstack', 'via0'), int(round(v['x'] / unit)), int(round(v['y'] / unit))))
        out.append('  )')
    out.append(')')
    open(out_path, 'w', encoding='utf-8', newline='\n').write(text[:i0] + '\n'.join(out) + text[j + 1:])
