#!/usr/bin/env python3
"""
Hunter SE Custom Frame Generator - FBX ASCII 7.4
Coordinates in mm, UnitScaleFactor=0.1
Components:
  1. Aluminium T-slot Support Columns x4 (40x40x190mm, silver)
  2. Main Electronics Enclosure (400x280x750mm, white/black)
  3. Sensor Mount Bracket (200x310x150mm, black)
"""
import math, os

# ── Mesh ─────────────────────────────────────────────────────────────────────

class Mesh:
    def __init__(self):
        self.verts = []
        self.faces = []
        self.normals = []   # per face-vertex
        self.uvs = []       # per face-vertex
        self.face_mat = []  # material index per face

    def _fn(self, v0, v1, v2):
        ax,ay,az = v1[0]-v0[0],v1[1]-v0[1],v1[2]-v0[2]
        bx,by,bz = v2[0]-v0[0],v2[1]-v0[1],v2[2]-v0[2]
        nx=ay*bz-az*by; ny=az*bx-ax*bz; nz=ax*by-ay*bx
        l=math.sqrt(nx*nx+ny*ny+nz*nz)
        return (nx/l,ny/l,nz/l) if l>1e-8 else (0,1,0)

    def add_face(self, vs, mat=0):
        base = len(self.verts)
        self.verts.extend(vs)
        self.faces.append(list(range(base, base+len(vs))))
        n = self._fn(vs[0],vs[1],vs[2])
        self.normals.extend([n]*len(vs))
        self.uvs.extend([(0,0)]*len(vs))
        self.face_mat.append(mat)

    def merge(self, other):
        off = len(self.verts)
        self.verts.extend(other.verts)
        self.faces.extend([[v+off for v in f] for f in other.faces])
        self.normals.extend(other.normals)
        self.uvs.extend(other.uvs)
        self.face_mat.extend(other.face_mat)


def add_box(m, cx, cy, cz, sx, sy, sz, mat=0):
    x0,x1 = cx-sx/2, cx+sx/2
    y0,y1 = cy-sy/2, cy+sy/2
    z0,z1 = cz-sz/2, cz+sz/2
    m.add_face([(x0,y1,z1),(x1,y1,z1),(x1,y1,z0),(x0,y1,z0)], mat)  # +Y
    m.add_face([(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)], mat)  # -Y
    m.add_face([(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)], mat)  # +Z
    m.add_face([(x1,y0,z0),(x0,y0,z0),(x0,y1,z0),(x1,y1,z0)], mat)  # -Z
    m.add_face([(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)], mat)  # +X
    m.add_face([(x0,y0,z1),(x0,y1,z1),(x0,y1,z0),(x0,y0,z0)], mat)  # -X


def add_box_open_top(m, cx, cy, cz, sx, sy, sz, mat=0):
    """Box without top face"""
    x0,x1 = cx-sx/2, cx+sx/2
    y0,y1 = cy-sy/2, cy+sy/2
    z0,z1 = cz-sz/2, cz+sz/2
    m.add_face([(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)], mat)  # -Y
    m.add_face([(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)], mat)  # +Z
    m.add_face([(x1,y0,z0),(x0,y0,z0),(x0,y1,z0),(x1,y1,z0)], mat)  # -Z
    m.add_face([(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)], mat)  # +X
    m.add_face([(x0,y0,z1),(x0,y1,z1),(x0,y1,z0),(x0,y0,z0)], mat)  # -X


# ── Component 1: T-slot Column ────────────────────────────────────────────────
# 40x40x190mm aluminium extrusion, T-slot on all 4 faces
# Profile points in XZ plane (clockwise from above)

def tslot_profile():
    """Returns 20 XZ points for 40x40 T-slot cross-section"""
    # half=12.5 (25x25mm), slot_w=2.5 (half), slot_d=5
    h=12.5; sw=2.5; sd=5
    return [
        (-h,   h),          # P0  top-left outer
        (-sw,  h),          # P1  top slot left outer
        (-sw,  h-sd),       # P2  top slot left inner
        ( sw,  h-sd),       # P3  top slot right inner
        ( sw,  h),          # P4  top slot right outer
        ( h,   h),          # P5  top-right outer
        ( h,   sw),         # P6  right slot top outer
        ( h-sd,sw),         # P7  right slot top inner
        ( h-sd,-sw),        # P8  right slot bottom inner
        ( h,  -sw),         # P9  right slot bottom outer
        ( h,  -h),          # P10 bottom-right outer
        ( sw, -h),          # P11 bottom slot right outer
        ( sw, -(h-sd)),     # P12 bottom slot right inner
        (-sw, -(h-sd)),     # P13 bottom slot left inner
        (-sw, -h),          # P14 bottom slot left outer
        (-h,  -h),          # P15 bottom-left outer
        (-h,  -sw),         # P16 left slot bottom outer
        (-(h-sd),-sw),      # P17 left slot bottom inner
        (-(h-sd), sw),      # P18 left slot top inner
        (-h,   sw),         # P19 left slot top outer
    ]

def build_tslot_column(y_bottom, height):
    """Build one T-slot column mesh"""
    m = Mesh()
    prof = tslot_profile()
    n = len(prof)
    y0, y1 = y_bottom, y_bottom + height
    cx, cz = 0.0, 0.0  # fan center

    # Top face (Y=y1, viewed from +Y, need CCW = reverse CW profile)
    top_verts = [(p[0], y1, p[1]) for p in prof]
    center_top = (cx, y1, cz)
    for i in range(n):
        j = (i+1) % n
        m.add_face([center_top, top_verts[j], top_verts[i]], 0)  # CCW from above

    # Bottom face (Y=y0, CW when viewed from above = CCW from below)
    bot_verts = [(p[0], y0, p[1]) for p in prof]
    center_bot = (cx, y0, cz)
    for i in range(n):
        j = (i+1) % n
        m.add_face([center_bot, bot_verts[i], bot_verts[j]], 0)  # CCW from below

    # Side walls
    for i in range(n):
        j = (i+1) % n
        # Quad: bot_i, bot_j, top_j, top_i  (check winding = outward normal)
        b0 = bot_verts[i]; b1 = bot_verts[j]
        t0 = top_verts[i]; t1 = top_verts[j]
        m.add_face([b0, b1, t1, t0], 0)

    return m


def build_all_columns(y_bottom=0, height=190):
    """4 columns at corner positions"""
    # Positions based on robot frame footprint (~272x642mm)
    # inset from Hunter SE frame edges to fit structurally
    positions = [(-96, -271), (96, -271), (96, 271), (-96, 271)]
    m = Mesh()
    for (cx, cz) in positions:
        col = build_tslot_column(y_bottom, height)
        # Translate column
        col.verts = [(v[0]+cx, v[1], v[2]+cz) for v in col.verts]
        m.merge(col)
    return m


# ── Component 2: Main Electronics Enclosure ───────────────────────────────────
# 400(X) x 280(Y) x 750(Z) mm, white body, black top plates
# Beveled front face (top-front diagonal), fan grille, side vents

def build_enclosure(y_base=190):
    """Main electronics box"""
    m = Mesh()
    W = 400   # X full width
    H = 150   # Y full height (reduced)
    L = 750   # Z full length
    cx, cz = 0.0, 0.0
    x0,x1 = -W/2, W/2
    y0,y1 = y_base, y_base+H
    z0,z1 = -L/2, L/2

    bevel_h = 35   # bevel height at top-front
    bevel_d = 50   # bevel depth into Z

    # Front face Z = z1 (positive Z = front)
    # Bevel: top-front edge cut diagonally
    # Below bevel: full rect; bevel: diagonal; above bevel: nothing (top face fills it)

    # --- Bottom face ---
    m.add_face([(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)], 1)

    # --- Back face (Z = z0) ---
    m.add_face([(x1,y0,z0),(x0,y0,z0),(x0,y1,z0),(x1,y1,z0)], 1)

    # --- Right face (X = x1) ---
    # Has 6 vertical vent slots: slot 12mm wide x 120mm tall, spaced along Z
    # Simplified: full face + inset vent panels
    # Right face with bevel cutout at top-front corner
    # Bevel cuts off triangle at top-front: from (x1,y1,z1) → (x1,y1,z1-bevel_d) and (x1,y1-bevel_h,z1)
    right_verts = [
        (x1, y0, z0),
        (x1, y0, z1),
        (x1, y1-bevel_h, z1),
        (x1, y1, z1-bevel_d),
        (x1, y1, z0),
    ]
    m.add_face(right_verts, 1)

    # Vent slots on right face (6 slots, each 12x120mm, white inset panels slightly recessed)
    vent_w = 12; vent_h = 70
    for zi in [-280, -168, -56, 56, 168, 280]:
        # vent slot as dark inset quad
        m.add_face([
            (x1-1, y_base+30,          zi-vent_w/2),
            (x1-1, y_base+30+vent_h,   zi-vent_w/2),
            (x1-1, y_base+30+vent_h,   zi+vent_w/2),
            (x1-1, y_base+30,          zi+vent_w/2),
        ], 3)  # dark material

    # --- Left face (X = x0) ---
    left_verts = [
        (x0, y0, z1),
        (x0, y0, z0),
        (x0, y1, z0),
        (x0, y1, z1-bevel_d),
        (x0, y1-bevel_h, z1),
    ]
    m.add_face(left_verts, 1)

    # Vent slots on left face
    for zi in [-280, -168, -56, 56, 168, 280]:
        m.add_face([
            (x0+1, y_base+30,          zi+vent_w/2),
            (x0+1, y_base+30+vent_h,   zi+vent_w/2),
            (x0+1, y_base+30+vent_h,   zi-vent_w/2),
            (x0+1, y_base+30,          zi-vent_w/2),
        ], 3)

    # --- Front face (Z = z1) - below bevel only ---
    front_h = H - bevel_h
    m.add_face([
        (x0, y0,              z1),
        (x1, y0,              z1),
        (x1, y1-bevel_h,      z1),
        (x0, y1-bevel_h,      z1),
    ], 1)

    # --- Bevel face (diagonal, front-top) ---
    m.add_face([
        (x0, y1-bevel_h, z1),
        (x1, y1-bevel_h, z1),
        (x1, y1,         z1-bevel_d),
        (x0, y1,         z1-bevel_d),
    ], 1)

    # --- Depth camera cutout on bevel face (dark inset) ---
    bevel_nx = 0
    bevel_ny = bevel_d / math.sqrt(bevel_h**2 + bevel_d**2)
    bevel_nz = bevel_h / math.sqrt(bevel_h**2 + bevel_d**2)
    # Camera window: 80x60mm centered on bevel face
    # Approximate position in bevel coordinates
    cam_cx = 0.0
    # Parametric point on bevel: t=0.5 → y = y1-bevel_h + 0.5*bevel_h = y1-bevel_h/2
    # z = z1 - 0.5*bevel_d
    cam_y = y1 - bevel_h * 0.5
    cam_z = z1 - bevel_d * 0.5
    cw, ch_b = 80, 50
    # Inset by 3mm along bevel normal
    inset = 3
    in_ny = inset * bevel_ny
    in_nz = inset * bevel_nz
    m.add_face([
        (cam_cx-cw/2, cam_y-ch_b/2, cam_z - in_nz),
        (cam_cx+cw/2, cam_y-ch_b/2, cam_z - in_nz),
        (cam_cx+cw/2, cam_y+ch_b/2, cam_z - in_nz),
        (cam_cx-cw/2, cam_y+ch_b/2, cam_z - in_nz),
    ], 3)  # dark = camera lens

    # --- Top face (Z from z0 to z1-bevel_d) ---
    m.add_face([
        (x0, y1, z0),
        (x1, y1, z0),
        (x1, y1, z1-bevel_d),
        (x0, y1, z1-bevel_d),
    ], 1)

    # --- Black top mounting plates (2x, 230mm wide, 50mm deep, on top edges) ---
    plate_w = 230; plate_d = 50; plate_t = 8
    # Front black plate
    add_box(m, 0, y1+plate_t/2, z1-bevel_d-plate_d/2, plate_w, plate_t, plate_d, 2)
    # Back black plate
    add_box(m, 0, y1+plate_t/2, z0+plate_d/2, plate_w, plate_t, plate_d, 2)

    # --- Fan grille on top (center, 130mm radius octagon with radial fins) ---
    fan_cx, fan_cy, fan_cz = 0.0, y1+plate_t+1, 0.0
    fan_r = 115
    # Octagon base plate
    n_seg = 8
    oct_pts = [(fan_r*math.cos(math.pi/n_seg + i*2*math.pi/n_seg),
                fan_r*math.sin(math.pi/n_seg + i*2*math.pi/n_seg)) for i in range(n_seg)]
    # Octagon top face
    for i in range(n_seg):
        j=(i+1)%n_seg
        m.add_face([
            (fan_cx, fan_cy, fan_cz),
            (fan_cx+oct_pts[j][0], fan_cy, fan_cz+oct_pts[j][1]),
            (fan_cx+oct_pts[i][0], fan_cy, fan_cz+oct_pts[i][1]),
        ], 2)  # black grille

    # Radial fins (8 fins, thin boxes)
    for i in range(8):
        angle = i * math.pi/4
        fin_len = fan_r - 15
        fx = fan_cx + math.cos(angle) * fin_len/2
        fz = fan_cz + math.sin(angle) * fin_len/2
        # thin radial box, rotated along angle
        dx = math.cos(angle); dz = math.sin(angle)
        # Create fin as a thin box along direction
        fw = 4; fh = 6
        p0 = (fx - dz*fw/2 - dx*fin_len/2, fan_cy,     fz + dx*fw/2 - dz*fin_len/2)
        p1 = (fx + dz*fw/2 - dx*fin_len/2, fan_cy,     fz - dx*fw/2 - dz*fin_len/2)
        p2 = (fx + dz*fw/2 + dx*fin_len/2, fan_cy,     fz - dx*fw/2 + dz*fin_len/2)
        p3 = (fx - dz*fw/2 + dx*fin_len/2, fan_cy,     fz + dx*fw/2 + dz*fin_len/2)
        p4 = (fx - dz*fw/2 - dx*fin_len/2, fan_cy+fh,  fz + dx*fw/2 - dz*fin_len/2)
        p5 = (fx + dz*fw/2 - dx*fin_len/2, fan_cy+fh,  fz - dx*fw/2 - dz*fin_len/2)
        p6 = (fx + dz*fw/2 + dx*fin_len/2, fan_cy+fh,  fz - dx*fw/2 + dz*fin_len/2)
        p7 = (fx - dz*fw/2 + dx*fin_len/2, fan_cy+fh,  fz + dx*fw/2 + dz*fin_len/2)
        m.add_face([p0,p1,p2,p3], 2)  # bottom
        m.add_face([p4,p7,p6,p5], 2)  # top
        m.add_face([p0,p4,p5,p1], 2)  # side
        m.add_face([p2,p6,p7,p3], 2)  # other side

    return m


# ── Component 3: Sensor Mount Bracket ─────────────────────────────────────────
# Z/L-shaped black bracket: base plate + vertical arm + top LiDAR platform
# W=200mm, total H=310mm, D=150mm

def build_sensor_bracket(y_base=340):
    """Z-shaped sensor bracket"""
    m = Mesh()
    W = 200   # X width
    T = 6     # sheet thickness
    D = 150   # Z depth
    arm_h = 200  # vertical arm height
    top_d = 120  # top platform depth

    # --- Bottom mounting plate (horizontal, sits on top of enclosure) ---
    # 200 x 6 x 150 mm
    add_box(m, 0, y_base + T/2, 0, W, T, D, 2)

    # --- Vertical arm (rises from back edge of base plate) ---
    # Back edge at Z = -D/2 = -75
    arm_z_center = -D/2 + T/2
    add_box(m, 0, y_base + T + arm_h/2, arm_z_center, W, arm_h, T, 2)

    # --- Top platform (extends forward from top of arm) ---
    # At Y = y_base + T + arm_h
    top_y = y_base + T + arm_h
    add_box(m, 0, top_y + T/2, -D/2 + T + top_d/2, W, T, top_d, 2)

    # --- Cable routing slots on vertical arm (5 oblong cutouts inset 1mm) ---
    for zi_off, hi_off in [(-50, 60), (-20, 120), (10, 80), (-65, 160), (20, 40)]:
        slot_w = 20; slot_h = 10
        slot_z = arm_z_center + zi_off * 0.0 - 1  # inset side
        slot_y = y_base + T + arm_h/2 + hi_off * 0.5
        if slot_y < y_base+T+10 or slot_y > top_y-10: continue
        m.add_face([
            (-slot_w/2 + slot_z*0, slot_y - slot_h/2, arm_z_center-T/2-1),
            ( slot_w/2 + slot_z*0, slot_y - slot_h/2, arm_z_center-T/2-1),
            ( slot_w/2 + slot_z*0, slot_y + slot_h/2, arm_z_center-T/2-1),
            (-slot_w/2 + slot_z*0, slot_y + slot_h/2, arm_z_center-T/2-1),
        ], 3)

    # --- LiDAR mount ring on top platform ---
    lidar_cx = 0.0
    lidar_cz = -D/2 + T + top_d/2
    lidar_cy = top_y + T + 1
    lidar_r_outer = 50
    lidar_r_inner = 42
    n_seg = 16
    for i in range(n_seg):
        a0 = i * 2*math.pi/n_seg
        a1 = (i+1) * 2*math.pi/n_seg
        ox0,oz0 = lidar_r_outer*math.cos(a0), lidar_r_outer*math.sin(a0)
        ox1,oz1 = lidar_r_outer*math.cos(a1), lidar_r_outer*math.sin(a1)
        ix0,iz0 = lidar_r_inner*math.cos(a0), lidar_r_inner*math.sin(a0)
        ix1,iz1 = lidar_r_inner*math.cos(a1), lidar_r_inner*math.sin(a1)
        # Ring quad (top face, annular segment)
        m.add_face([
            (lidar_cx+ix0, lidar_cy, lidar_cz+iz0),
            (lidar_cx+ox0, lidar_cy, lidar_cz+oz0),
            (lidar_cx+ox1, lidar_cy, lidar_cz+oz1),
            (lidar_cx+ix1, lidar_cy, lidar_cz+iz1),
        ], 2)

    # --- Camera mount window on vertical arm (front face, lower section) ---
    cam_w = 80; cam_h_c = 55
    cam_y_c = y_base + T + 70
    m.add_face([
        (-cam_w/2, cam_y_c,        arm_z_center+T/2+1),
        ( cam_w/2, cam_y_c,        arm_z_center+T/2+1),
        ( cam_w/2, cam_y_c+cam_h_c, arm_z_center+T/2+1),
        (-cam_w/2, cam_y_c+cam_h_c, arm_z_center+T/2+1),
    ], 3)  # dark = camera cutout

    # --- Mounting screw holes (4x M5, represented as small dark circles) ---
    for sx_off, sy_off in [(-85,-30),(85,-30),(-85,30),(85,30)]:
        hole_cx = sx_off
        hole_cy = top_y + T/2
        hole_cz = lidar_cz + sy_off * 0.5
        hole_r = 3.5
        hn = 8
        hole_pts = [(hole_cx + hole_r*math.cos(i*2*math.pi/hn),
                     hole_cy + 1,
                     hole_cz + hole_r*math.sin(i*2*math.pi/hn)) for i in range(hn)]
        m.add_face(list(reversed(hole_pts)), 3)

    return m


# ── FBX ASCII Writer ──────────────────────────────────────────────────────────

def fmt_list(vals, per_line=12):
    chunks = [vals[i:i+per_line] for i in range(0, len(vals), per_line)]
    return ','.join(','.join(f'{v:.6g}' for v in c) for c in chunks)

def fmt_int_list(vals, per_line=15):
    chunks = [vals[i:i+per_line] for i in range(0, len(vals), per_line)]
    return ','.join(','.join(str(v) for v in c) for c in chunks)

def write_geometry(f, geom_id, name, mesh):
    verts_flat = []
    for v in mesh.verts:
        verts_flat.extend(v)

    poly_idx = []
    for face in mesh.faces:
        for i, vi in enumerate(face):
            if i == len(face)-1:
                poly_idx.append(-(vi+1))
            else:
                poly_idx.append(vi)

    normals_flat = []
    for n in mesh.normals:
        normals_flat.extend(n)

    uvs_flat = []
    for uv in mesh.uvs:
        uvs_flat.extend(uv)

    mat_indices = mesh.face_mat if mesh.face_mat else [0]*len(mesh.faces)

    nv = len(mesh.verts)
    np_ = len(poly_idx)
    nn = len(mesh.normals)
    nu = len(mesh.uvs)
    nm = len(mat_indices)

    f.write(f'\tGeometry: {geom_id}, "Geometry::{name}", "Mesh" {{\n')
    f.write(f'\t\tVertices: *{nv*3} {{\n')
    f.write(f'\t\t\ta: {fmt_list(verts_flat)}\n')
    f.write(f'\t\t}}\n')
    f.write(f'\t\tPolygonVertexIndex: *{np_} {{\n')
    f.write(f'\t\t\ta: {fmt_int_list(poly_idx)}\n')
    f.write(f'\t\t}}\n')
    f.write(f'\t\tGeometryVersion: 124\n')

    f.write(f'\t\tLayerElementNormal: 0 {{\n')
    f.write(f'\t\t\tVersion: 102\n')
    f.write(f'\t\t\tName: ""\n')
    f.write(f'\t\t\tMappingInformationType: "ByPolygonVertex"\n')
    f.write(f'\t\t\tReferenceInformationType: "Direct"\n')
    f.write(f'\t\t\tNormals: *{nn*3} {{\n')
    f.write(f'\t\t\t\ta: {fmt_list(normals_flat)}\n')
    f.write(f'\t\t\t}}\n')
    f.write(f'\t\t}}\n')

    f.write(f'\t\tLayerElementUV: 0 {{\n')
    f.write(f'\t\t\tVersion: 101\n')
    f.write(f'\t\t\tName: "UVMap"\n')
    f.write(f'\t\t\tMappingInformationType: "ByPolygonVertex"\n')
    f.write(f'\t\t\tReferenceInformationType: "Direct"\n')
    f.write(f'\t\t\tUV: *{nu*2} {{\n')
    f.write(f'\t\t\t\ta: {fmt_list(uvs_flat)}\n')
    f.write(f'\t\t\t}}\n')
    f.write(f'\t\t}}\n')

    f.write(f'\t\tLayerElementMaterial: 0 {{\n')
    f.write(f'\t\t\tVersion: 101\n')
    f.write(f'\t\t\tName: ""\n')
    f.write(f'\t\t\tMappingInformationType: "ByPolygon"\n')
    f.write(f'\t\t\tReferenceInformationType: "IndexToDirect"\n')
    f.write(f'\t\t\tMaterials: *{nm} {{\n')
    f.write(f'\t\t\t\ta: {fmt_int_list(mat_indices)}\n')
    f.write(f'\t\t\t}}\n')
    f.write(f'\t\t}}\n')

    f.write(f'\t\tLayer: 0 {{\n')
    f.write(f'\t\t\tVersion: 100\n')
    f.write(f'\t\t\tLayerElement: {{\n')
    f.write(f'\t\t\t\tType: "LayerElementNormal"\n')
    f.write(f'\t\t\t\tTypedIndex: 0\n')
    f.write(f'\t\t\t}}\n')
    f.write(f'\t\t\tLayerElement: {{\n')
    f.write(f'\t\t\t\tType: "LayerElementMaterial"\n')
    f.write(f'\t\t\t\tTypedIndex: 0\n')
    f.write(f'\t\t\t}}\n')
    f.write(f'\t\t\tLayerElement: {{\n')
    f.write(f'\t\t\t\tType: "LayerElementUV"\n')
    f.write(f'\t\t\t\tTypedIndex: 0\n')
    f.write(f'\t\t\t}}\n')
    f.write(f'\t\t}}\n')

    f.write(f'\t}}\n')


def write_model(f, model_id, name, tx=0, ty=0, tz=0):
    f.write(f'\tModel: {model_id}, "Model::{name}", "Mesh" {{\n')
    f.write(f'\t\tVersion: 232\n')
    f.write(f'\t\tProperties70:  {{\n')
    f.write(f'\t\t\tP: "RotationActive", "bool", "", "",1\n')
    f.write(f'\t\t\tP: "InheritType", "enum", "", "",1\n')
    f.write(f'\t\t\tP: "ScalingMax", "Vector3D", "Vector", "",0,0,0\n')
    f.write(f'\t\t\tP: "DefaultAttributeIndex", "int", "Integer", "",0\n')
    f.write(f'\t\t\tP: "Lcl Translation", "Lcl Translation", "", "A",{tx},{ty},{tz}\n')
    f.write(f'\t\t\tP: "Lcl Rotation", "Lcl Rotation", "", "A",0,0,0\n')
    f.write(f'\t\t\tP: "Lcl Scaling", "Lcl Scaling", "", "A",1,1,1\n')
    f.write(f'\t\t}}\n')
    f.write(f'\t\tShading: Y\n')
    f.write(f'\t\tCulling: "CullingOff"\n')
    f.write(f'\t}}\n')


def write_material(f, mat_id, name, r, g, b):
    f.write(f'\tMaterial: {mat_id}, "Material::{name}", "" {{\n')
    f.write(f'\t\tVersion: 102\n')
    f.write(f'\t\tShadingModel: "Lambert"\n')
    f.write(f'\t\tMultiLayer: 0\n')
    f.write(f'\t\tProperties70:  {{\n')
    f.write(f'\t\t\tP: "AmbientColor", "Color", "", "A",{r*0.3:.3f},{g*0.3:.3f},{b*0.3:.3f}\n')
    f.write(f'\t\t\tP: "DiffuseColor", "Color", "", "A",{r:.3f},{g:.3f},{b:.3f}\n')
    f.write(f'\t\t\tP: "DiffuseFactor", "Number", "", "A",1\n')
    f.write(f'\t\t\tP: "ShadingModel", "KString", "", "", "Lambert"\n')
    f.write(f'\t\t}}\n')
    f.write(f'\t}}\n')


def write_fbx(output_path, meshes_info):
    """
    meshes_info: list of (geom_id, model_id, name, mesh, mat_ids_list)
    Materials: 0=silver(columns), 1=white(enclosure), 2=black, 3=dark_gray(vents/grooves)
    """
    # IDs
    root_model_id = 900000000
    mat_ids = {0: 800000001, 1: 800000002, 2: 800000003, 3: 800000004}

    with open(output_path, 'w', encoding='utf-8') as f:
        # Header
        f.write('; FBX 7.4.0 project file\n')
        f.write('; Generated by Hunter SE Custom Frame Generator\n\n')
        f.write('FBXHeaderExtension:  {\n')
        f.write('\tFBXHeaderVersion: 1003\n')
        f.write('\tFBXVersion: 7400\n')
        f.write('\tEncryptionType: 0\n')
        f.write('\tCreationTimeStamp:  {\n')
        f.write('\t\tVersion: 1000\n\t\tYear: 2024\n\t\tMonth: 1\n\t\tDay: 1\n')
        f.write('\t\tHour: 0\n\t\tMinute: 0\n\t\tSecond: 0\n\t\tMillisecond: 0\n')
        f.write('\t}\n')
        f.write('\tCreator: "HunterSEFrameGen"\n')
        f.write('}\n\n')

        # GlobalSettings
        f.write('GlobalSettings:  {\n')
        f.write('\tVersion: 1000\n')
        f.write('\tProperties70:  {\n')
        f.write('\t\tP: "UpAxis", "int", "Integer", "",1\n')
        f.write('\t\tP: "UpAxisSign", "int", "Integer", "",1\n')
        f.write('\t\tP: "FrontAxis", "int", "Integer", "",2\n')
        f.write('\t\tP: "FrontAxisSign", "int", "Integer", "",1\n')
        f.write('\t\tP: "CoordAxis", "int", "Integer", "",0\n')
        f.write('\t\tP: "CoordAxisSign", "int", "Integer", "",1\n')
        f.write('\t\tP: "OriginalUpAxis", "int", "Integer", "",-1\n')
        f.write('\t\tP: "OriginalUpAxisSign", "int", "Integer", "",1\n')
        f.write('\t\tP: "UnitScaleFactor", "double", "Number", "",0.1\n')
        f.write('\t\tP: "OriginalUnitScaleFactor", "double", "Number", "",0.1\n')
        f.write('\t\tP: "AmbientColor", "ColorRGB", "Color", "",0,0,0\n')
        f.write('\t\tP: "DefaultCamera", "KString", "", "", "Producer Perspective"\n')
        f.write('\t\tP: "TimeMode", "enum", "", "",11\n')
        f.write('\t\tP: "TimeSpanStart", "KTime", "Time", "",0\n')
        f.write('\t\tP: "TimeSpanStop", "KTime", "Time", "",46186158000\n')
        f.write('\t\tP: "CustomFrameRate", "double", "Number", "",-1\n')
        f.write('\t}\n')
        f.write('}\n\n')

        n_models = len(meshes_info) + 1  # +1 for root
        n_geoms = len(meshes_info)
        n_mats = 4

        # Definitions
        f.write('Definitions:  {\n')
        f.write('\tVersion: 100\n')
        f.write(f'\tCount: {1 + n_models + n_geoms + n_mats}\n')
        f.write('\tObjectType: "GlobalSettings" {\n\t\tCount: 1\n\t}\n')
        f.write(f'\tObjectType: "Model" {{\n\t\tCount: {n_models}\n\t}}\n')
        f.write(f'\tObjectType: "Geometry" {{\n\t\tCount: {n_geoms}\n\t}}\n')
        f.write(f'\tObjectType: "Material" {{\n\t\tCount: {n_mats}\n\t}}\n')
        f.write('}\n\n')

        # Objects
        f.write('Objects:  {\n')

        # Root model
        write_model(f, root_model_id, 'Frame')

        # Component models and geometries
        for (geom_id, model_id, name, mesh, _) in meshes_info:
            write_geometry(f, geom_id, name, mesh)
            write_model(f, model_id, name)

        # Materials
        # 0=silver anodized aluminium
        write_material(f, mat_ids[0], 'AluminiumSilver', 0.78, 0.80, 0.82)
        # 1=matte white
        write_material(f, mat_ids[1], 'MatteWhite', 0.92, 0.92, 0.90)
        # 2=matte black
        write_material(f, mat_ids[2], 'MatteBlack', 0.08, 0.08, 0.08)
        # 3=dark gray (vents/cutouts)
        write_material(f, mat_ids[3], 'DarkGray', 0.18, 0.18, 0.18)

        f.write('}\n\n')

        # Connections
        f.write('Connections:  {\n')
        for (geom_id, model_id, name, mesh, comp_mat_ids) in meshes_info:
            # geometry → model
            f.write(f'\tC: "OO",{geom_id},{model_id}\n')
            # model → root
            f.write(f'\tC: "OO",{model_id},{root_model_id}\n')
            # materials → model
            for mid in comp_mat_ids:
                f.write(f'\tC: "OO",{mat_ids[mid]},{model_id}\n')
        # root model → scene
        f.write(f'\tC: "OO",{root_model_id},0\n')
        f.write('}\n')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("Building T-slot columns...")
    col_mesh = build_all_columns(y_bottom=0, height=190)

    print("Building main enclosure...")
    enc_mesh = build_enclosure(y_base=190)

    print("Building sensor bracket...")
    brk_mesh = build_sensor_bracket(y_base=340)  # 190 + 150

    meshes_info = [
        (100000001, 200000001, 'AluminiumColumns', col_mesh, [0]),
        (100000002, 200000002, 'MainEnclosure',    enc_mesh, [1, 2, 3]),
        (100000003, 200000003, 'SensorBracket',    brk_mesh, [2, 3]),
    ]

    out = '/home/sangukbae/autodrive/AutoDRIVE-Simulator/Assets/Models/Vehicle/Hunter SE/CustomFrame.fbx'
    print(f"Writing FBX to {out} ...")
    write_fbx(out, meshes_info)

    print("Done!")
    print(f"Stats:")
    for (_, _, name, mesh, _) in meshes_info:
        print(f"  {name}: {len(mesh.verts)} verts, {len(mesh.faces)} faces")

if __name__ == '__main__':
    main()
