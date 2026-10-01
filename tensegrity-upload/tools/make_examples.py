#!/usr/bin/env python3
"""Generate the example FBX models in examples/.

Writes minimal ASCII FBX 7.4 files (one mesh, one material) that three.js'
FBXLoader reads. Geometry is authored in centimetres, like most DCC exports,
so the app's import normalisation gets exercised.

    python3 tools/make_examples.py
"""
import math
import os

OUT = os.path.join(os.path.dirname(__file__), '..', 'examples')


class Mesh:
    def __init__(self):
        self.verts = []   # [x, y, z]
        self.polys = []   # [(vertex indices, per-corner normals)]

    def v(self, p):
        self.verts.append(p)
        return len(self.verts) - 1

    def face(self, idx, normals):
        self.polys.append((idx, normals))

    def flat(self, pts):
        """Planar polygon from points (counter-clockwise seen from outside)."""
        a, b, c = pts[0], pts[1], pts[2]
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        n = [u[1]*w[2] - u[2]*w[1], u[2]*w[0] - u[0]*w[2], u[0]*w[1] - u[1]*w[0]]
        ln = math.sqrt(sum(x*x for x in n)) or 1
        n = [x / ln for x in n]
        self.face([self.v(p) for p in pts], [n] * len(pts))

    def box(self, cx, cy, cz, sx, sy, sz):
        x0, x1 = cx - sx/2, cx + sx/2
        y0, y1 = cy - sy/2, cy + sy/2
        z0, z1 = cz - sz/2, cz + sz/2
        self.flat([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)])  # +z
        self.flat([(x1, y0, z0), (x0, y0, z0), (x0, y1, z0), (x1, y1, z0)])  # -z
        self.flat([(x1, y0, z1), (x1, y0, z0), (x1, y1, z0), (x1, y1, z1)])  # +x
        self.flat([(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)])  # -x
        self.flat([(x0, y1, z1), (x1, y1, z1), (x1, y1, z0), (x0, y1, z0)])  # +y
        self.flat([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)])  # -y

    def sweep(self, rings, normals, cap=True):
        """Smooth tube through rings of equal vertex count."""
        n = len(rings[0])
        for r in range(len(rings) - 1):
            for i in range(n):
                j = (i + 1) % n
                pts = [rings[r][i], rings[r+1][i], rings[r+1][j], rings[r][j]]
                nrm = [normals[r][i], normals[r+1][i], normals[r+1][j], normals[r][j]]
                self.face([self.v(p) for p in pts], nrm)
        if cap:
            self.flat(list(reversed(rings[0])))
            self.flat(list(rings[-1]))

    def fbx(self, name, color):
        verts = ','.join(f'{c:.4f}' for p in self.verts for c in p)
        idx, nrm = [], []
        for vi, ns in self.polys:
            idx += vi[:-1] + [-vi[-1] - 1]
            nrm += [f'{c:.5f}' for n in ns for c in n]
        r, g, b = color
        return f'''; FBX 7.4.0 project file
; ----------------------------------------------------
FBXHeaderExtension:  {{
	FBXHeaderVersion: 1003
	FBXVersion: 7400
	Creator: "tensegrity-lab tools/make_examples.py"
}}
GlobalSettings:  {{
	Version: 1000
	Properties70:  {{
		P: "UpAxis", "int", "Integer", "",1
		P: "UnitScaleFactor", "double", "Number", "",1
	}}
}}
Objects:  {{
	Geometry: 1000, "Geometry::{name}", "Mesh" {{
		Vertices: *{len(self.verts)*3} {{
			a: {verts}
		}}
		PolygonVertexIndex: *{len(idx)} {{
			a: {','.join(map(str, idx))}
		}}
		GeometryVersion: 124
		LayerElementNormal: 0 {{
			Version: 101
			Name: ""
			MappingInformationType: "ByPolygonVertex"
			ReferenceInformationType: "Direct"
			Normals: *{len(nrm)} {{
				a: {','.join(nrm)}
			}}
		}}
		Layer: 0 {{
			Version: 100
			LayerElement:  {{
				Type: "LayerElementNormal"
				TypedIndex: 0
			}}
		}}
	}}
	Model: 2000, "Model::{name}", "Mesh" {{
		Version: 232
		Properties70:  {{
		}}
	}}
	Material: 3000, "Material::{name}_mat", "" {{
		Version: 102
		ShadingModel: "phong"
		Properties70:  {{
			P: "DiffuseColor", "Color", "", "A",{r},{g},{b}
			P: "Shininess", "double", "Number", "",12
		}}
	}}
}}
Connections:  {{
	C: "OO",2000,0
	C: "OO",1000,2000
	C: "OO",3000,2000
}}
'''


def ring(center, u, w, radius, n):
    pts, nrm = [], []
    for i in range(n):
        a = 2 * math.pi * i / n
        d = [math.cos(a)*u[k] + math.sin(a)*w[k] for k in range(3)]
        pts.append(tuple(center[k] + radius*d[k] for k in range(3)))
        nrm.append(d)
    return pts, nrm


def curved_strut():
    """A bow: 300 cm chord, 40 cm rise, 4 cm radius tube."""
    m = Mesh()
    chord, rise, rad, seg, sides = 300.0, 40.0, 4.0, 24, 14
    R = (chord**2 / 4 + rise**2) / (2 * rise)
    half = math.asin(chord / 2 / R)
    rings, norms = [], []
    for s in range(seg + 1):
        t = -half + 2 * half * s / seg
        c = (R * math.sin(t), R * math.cos(t) - (R - rise), 0.0)
        tan = (math.cos(t), -math.sin(t), 0.0)
        u = (math.sin(t), math.cos(t), 0.0)  # in-plane normal
        w = (0.0, 0.0, 1.0)
        assert abs(sum(tan[i]*u[i] for i in range(3))) < 1e-9
        p, n = ring(c, u, w, rad, sides)
        rings.append(p); norms.append(n)
    m.sweep(rings, norms)
    return m


def hourglass_strut():
    """Lathe strut, 280 cm long, waisted in the middle, bulbous ends."""
    m = Mesh()
    L, seg, sides = 280.0, 30, 16
    rings, norms = [], []
    for s in range(seg + 1):
        t = s / seg
        y = (t - 0.5) * L
        r = 3.0 + 4.0 * (2 * t - 1)**4 + 1.2 * math.cos(math.pi * (2 * t - 1))
        p, n = ring((0, y, 0), (1, 0, 0), (0, 0, 1), r, sides)
        rings.append(p); norms.append(n)
    m.sweep(rings, norms)
    return m


def triangle_plate():
    """Equilateral triangular plate, 120 cm side, 6 cm thick."""
    m = Mesh()
    s, t = 120.0, 6.0
    R = s / math.sqrt(3)
    tri = [(R * math.cos(a), R * math.sin(a)) for a in (math.pi/2, math.pi/2 + 2*math.pi/3, math.pi/2 + 4*math.pi/3)]
    top = [(x, t/2, -z) for x, z in tri]
    bot = [(x, -t/2, -z) for x, z in tri]
    m.flat(top)
    m.flat(list(reversed(bot)))
    for i in range(3):
        j = (i + 1) % 3
        m.flat([bot[i], bot[j], top[j], top[i]])
    return m


def table_half():
    """One half of a tensegrity table: base plate, post at the back, arm reaching
    forward over the centre. Two of these, one flipped, make the classic
    'floating table'. 100 x 100 cm base, 70 cm post, arm ends 50 cm in."""
    m = Mesh()
    m.box(0, 2, 0, 100, 4, 100)            # base plate, top at y=4
    m.box(0, 39, -45, 10, 70, 10)          # post at the back edge
    m.box(0, 70, -22.5, 10, 8, 55)         # arm to the centre
    return m


def looks_ascii_to_three(text):
    """Mirror of FBXLoader's isFbxFormatASCII: it samples characters at the triangular
    numbers 0, 1, 3, 6, … and calls the file binary if any matches 'Kaydara\\FBX\\Binary\\\\'."""
    magic = 'Kaydara\\FBX\\Binary\\\\'
    pos = 0
    for i, ch in enumerate(magic):
        if pos < len(text) and text[pos] == ch:
            return False
        pos += i + 1
    return True


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for fname, mesh, col in [
        ('curved-strut.fbx', curved_strut(), (0.09, 0.09, 0.09)),
        ('hourglass-strut.fbx', hourglass_strut(), (0.55, 0.38, 0.22)),
        ('triangle-plate.fbx', triangle_plate(), (0.85, 0.85, 0.82)),
        ('table-half.fbx', table_half(), (0.62, 0.45, 0.28)),
    ]:
        text = mesh.fbx(fname[:-4].replace('-', '_'), col)
        assert looks_ascii_to_three(text), fname
        with open(os.path.join(OUT, fname), 'w') as f:
            f.write(text)
        print('wrote', fname, len(mesh.verts), 'verts')
