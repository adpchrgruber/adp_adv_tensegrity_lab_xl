#!/usr/bin/env python3
"""Generate the example scene files in examples/*.json from the FBX geometry.

Attachment points are computed from the same meshes make_examples.py writes, using
the app's import convention: an FBX is re-centred on its bounding box and scaled so
its largest dimension is `size` world units; points are stored in that local frame.

    python3 tools/make_scenes.py
"""
import json
import math
import os

import make_examples as mx

OUT = mx.OUT
FBX_SIZE = 3.0


def frame(mesh, size=FBX_SIZE):
    """(center, scale) of the app's import normalisation."""
    xs, ys, zs = zip(*mesh.verts)
    lo = (min(xs), min(ys), min(zs)); hi = (max(xs), max(ys), max(zs))
    center = tuple((a + b) / 2 for a, b in zip(lo, hi))
    return center, size / max(b - a for a, b in zip(lo, hi))


def local(mesh, p):
    c, s = frame(mesh)
    return [round((p[i] - c[i]) * s, 5) for i in range(3)]


# --- small vector / rotation helpers -------------------------------------------------

def sub(a, b): return [a[i] - b[i] for i in range(3)]
def add(a, b): return [a[i] + b[i] for i in range(3)]
def mul(a, k): return [x * k for x in a]
def dot(a, b): return sum(a[i] * b[i] for i in range(3))
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return mul(a, 1 / norm(a))


def quat_from_basis(x, y, z):
    """Quaternion [x,y,z,w] of the rotation whose columns are the given axes."""
    m00, m01, m02 = x[0], y[0], z[0]
    m10, m11, m12 = x[1], y[1], z[1]
    m20, m21, m22 = x[2], y[2], z[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1); w = 0.25 / s
        qx, qy, qz = (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2 * math.sqrt(1 + m00 - m11 - m22)
        w, qx, qy, qz = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = 2 * math.sqrt(1 + m11 - m00 - m22)
        w, qx, qy, qz = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = 2 * math.sqrt(1 + m22 - m00 - m11)
        w, qx, qy, qz = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    return [round(v, 6) for v in (qx, qy, qz, w)]


def rotate(q, v):
    x, y, z, w = q
    u = [x, y, z]
    t = mul(cross(u, v), 2)
    return add(add(v, mul(t, w)), cross(u, t))


def prism_nodes(n, r, h, y0, phase=0.0):
    twist = math.pi / 2 + math.pi / n
    bot = [[r * math.cos(phase + i * 2*math.pi/n), y0, r * math.sin(phase + i * 2*math.pi/n)] for i in range(n)]
    top = [[r * math.cos(phase + i * 2*math.pi/n + twist), y0 + h, r * math.sin(phase + i * 2*math.pi/n + twist)] for i in range(n)]
    cables = []
    for i in range(n):
        j = (i + 1) % n
        cables += [(('b', i), ('b', j)), (('t', i), ('t', j)), (('b', i), ('t', (i - 1) % n))]
    return bot, top, cables


def obj(kind, name, position, quaternion=(0, 0, 0, 1), scale=(1, 1, 1), points=(), fixed=False, **extra):
    o = {'kind': kind, 'name': name, 'position': [round(v, 5) for v in position],
         'quaternion': list(quaternion), 'scale': [round(v, 5) for v in scale], 'fixed': fixed}
    o.update(extra)
    o['points'] = [list(p) for p in points]
    return o


def scene(objects, cables, params=None, camera=None):
    p = {'gravity': 9.8, 'stiffness': 1000, 'damping': 8, 'prestress': 0.92, 'density': 1, 'collisions': False}
    p.update(params or {})
    s = {'format': 'tensegrity-lab', 'version': 1, 'params': p, 'objects': objects, 'cables': cables}
    if camera:
        s['camera'] = camera
    return s


# --- examples --------------------------------------------------------------------------

def floating_table():
    m = mx.table_half()
    c, s = frame(m)
    tip = local(m, (0, 66, 3))                 # underside of the arm, just short of its end
    corners = [local(m, (x, 4, z)) for x, z in ((-45, -45), (45, -45), (45, 45), (-45, 45))]
    lower_y = (c[1] - 0) * s                   # base plate on the ground
    upper_y = lower_y + 2 * tip[1] - 0.4       # upper arm tip 0.4 below the lower one
    flip = [1, 0, 0, 0]                        # 180° about x: y -> -y, z -> -z
    lower = obj('fbx', 'Table — lower', (0, lower_y + 0.01, 0), src='table-half.fbx', size=FBX_SIZE,
                points=[tip] + corners)
    upper = obj('fbx', 'Table — upper', (0, upper_y, 0), flip, src='table-half.fbx', size=FBX_SIZE,
                points=[tip] + corners)
    cables = [[0, 0, 1, 0]]                    # the "kiss" cable between the arm tips
    # upper corner k sits over lower corner with (x, -z): corners 0<->3, 1<->2
    for a, b in ((0, 3), (1, 2), (2, 1), (3, 0)):
        cables.append([0, 1 + a, 1, 1 + b])
    return scene([lower, upper], cables, camera={'position': [6.5, 4.2, 6.5], 'target': [0, 1.7, 0]})


def bow_prism():
    m = mx.curved_strut()
    c, s = frame(m)
    end_a, end_b = local(m, (-150, 0, 0)), local(m, (150, 0, 0))
    chord_local = norm(sub(end_b, end_a))
    mid_local = mul(add(end_a, end_b), 0.5)
    bot, top, cab = prism_nodes(3, 1.6, 2.6, 0.12)
    objs = []
    for i in range(3):
        p, q = bot[i], top[i]
        d = sub(q, p)
        L = norm(d)
        x = unit(d)
        mid = mul(add(p, q), 0.5)
        outward = [mid[0], 0, mid[2]]
        y = unit(sub(outward, mul(x, dot(outward, x))))    # bow outward, square to the chord
        z = cross(x, y)
        quat = quat_from_basis(x, y, z)
        k = L / chord_local
        pos = sub(mid, rotate(quat, mul(mid_local, k)))
        objs.append(obj('fbx', f'Bow {i+1}', pos, quat, (k, k, k), src='curved-strut.fbx', size=FBX_SIZE,
                        points=[end_a, end_b]))
    ref = lambda n: [n[1], 0 if n[0] == 'b' else 1]
    return scene(objs, [ref(a) + ref(b) for a, b in cab])


def hourglass_box():
    m = mx.hourglass_strut()
    c, s = frame(m)
    end_a, end_b = local(m, (0, -140, 0)), local(m, (0, 140, 0))
    length_local = norm(sub(end_b, end_a))
    bot, top, cab = prism_nodes(4, 1.7, 2.4, 0.15, phase=math.pi / 4)
    objs = []
    for i in range(4):
        p, q = bot[i], top[i]
        d = sub(q, p)
        y = unit(d)
        x = unit(cross(y, [0, 0, 1]) if abs(y[2]) < 0.9 else cross(y, [1, 0, 0]))
        z = cross(x, y)
        quat = quat_from_basis(x, y, z)
        k = norm(d) / length_local
        mid_local = mul(add(end_a, end_b), 0.5)
        pos = sub(mul(add(p, q), 0.5), rotate(quat, mul(mid_local, k)))
        objs.append(obj('fbx', f'Hourglass {i+1}', pos, quat, (k, k, k), src='hourglass-strut.fbx', size=FBX_SIZE,
                        points=[end_a, end_b]))
    ref = lambda n: [n[1], 0 if n[0] == 'b' else 1]
    return scene(objs, [ref(a) + ref(b) for a, b in cab])


def plate_mobile():
    plate = mx.triangle_plate()
    bow = mx.curved_strut()
    R = 120 / math.sqrt(3) - 6                     # corners, slightly inset
    ang = (math.pi/2, math.pi/2 + 2*math.pi/3, math.pi/2 + 4*math.pi/3)
    top_c = [local(plate, (R*math.cos(a), 3, -R*math.sin(a))) for a in ang]
    bot_c = [local(plate, (R*math.cos(a), -3, -R*math.sin(a))) for a in ang]
    centre_bot = local(plate, (0, -3, 0))
    pk = FBX_SIZE / 120 * 0.8                     # plates a bit smaller than 3 units
    yaw60 = [0, math.sin(math.pi/6), 0, math.cos(math.pi/6)]
    bow_end_a, bow_end_b = local(bow, (-150, 0, 0)), local(bow, (150, 0, 0))
    bow_k = 0.8

    anchors = [obj('anchor', f'Anchor {i+1}', (2.2*math.cos(a), 6.2, -2.2*math.sin(a)), fixed=True, points=[[0, 0, 0]])
               for i, a in enumerate(ang)]
    centroid = mul(local(plate, (0, 0, 0)), 0.8)    # bbox centre isn't the triangle's centre
    p1 = obj('fbx', 'Plate — upper', sub((0, 4.6, 0), centroid), src='triangle-plate.fbx', size=FBX_SIZE,
             scale=(0.8, 0.8, 0.8), points=top_c + bot_c)
    p2 = obj('fbx', 'Plate — lower', sub((0, 3.2, 0), rotate(yaw60, centroid)), yaw60, src='triangle-plate.fbx', size=FBX_SIZE,
             scale=(0.8, 0.8, 0.8), points=top_c + [centre_bot])
    bowo = obj('fbx', 'Bow', (0, 1.6, 0), (1, 0, 0, 0), src='curved-strut.fbx', size=FBX_SIZE,
               scale=(bow_k, bow_k, bow_k), points=[bow_end_a, bow_end_b])
    objs = anchors + [p1, p2, bowo]
    A, P1, P2, B = 0, 3, 4, 5
    cables = []
    for i in range(3):
        cables.append([A + i, 0, P1, i])                   # anchors -> upper plate corners
        cables.append([P1, 3 + i, P2, i])                  # upper plate underside -> lower plate
        cables.append([P1, 3 + i, P2, (i + 1) % 3])        # crossed, so the lower plate can't spin freely
    cables.append([P2, 3, B, 0])
    cables.append([P2, 3, B, 1])
    return scene(objs, cables, params={'damping': 12, 'prestress': 1.0},
                 camera={'position': [7.5, 5.5, 8.5], 'target': [0, 3.4, 0]})


if __name__ == '__main__':
    for name, fn in [('floating-table', floating_table), ('bow-prism', bow_prism),
                     ('hourglass-box', hourglass_box), ('plate-mobile', plate_mobile)]:
        with open(os.path.join(OUT, name + '.json'), 'w') as f:
            json.dump(fn(), f, indent=1)
        print('wrote', name + '.json')
