#!/usr/bin/env python3
"""turtlebot3_house の model.sdf から 2D 占有格子 (map.pgm/map.yaml) を作る。

house 用の SLAM 地図は turtlebot3 に同梱されていないので、壁の box collision を
LDS の高さで水平スライスして地図にする。ドア上部の垂れ壁 (z 1.7-2.5) は
スライスに掛からないので開口部として正しく抜ける。家具は mesh collision なので
入らない — scan には映るが地図には無い障害物になる (vi_planner 側の
local_penalty が拾う)。

  python3 scripts/gen_house_map.py [<model.sdf>] [<out_dir>]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

SDF = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/.pixi/envs/ros/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "assets", "tb3_house")

RES = 0.05          # [m/cell] tb3_world の地図と同じ
LIDAR_Z = 0.18      # [m] burger の base_scan (base_link+0.171, spawn z=0.01)
MARGIN = 0.5        # [m] bbox の外側余白
SPAWN = (-2.0, -0.5)  # turtlebot3_house.launch.py の既定 spawn (flood fill の種)


def parse_pose(text):
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    return v[0], v[1], v[2], v[5]  # x, y, z, yaw (roll/pitch は house では 0)


def compose(parent, child):
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw)


def walk(elem, pose, out):
    """model/link を再帰的に辿り、LiDAR 高さに掛かる box を (cx,cy,yaw,sx,sy) で集める。"""
    for m in elem.findall("model"):
        walk(m, compose(pose, parse_pose(m.findtext("pose"))), out)
    for ln in elem.findall("link"):
        lp = compose(pose, parse_pose(ln.findtext("pose")))
        for col in ln.findall("collision"):
            box = col.find("geometry/box/size")
            if box is None:
                continue  # mesh (家具) は無視
            sx, sy, sz = [float(x) for x in box.text.split()]
            cp = compose(lp, parse_pose(col.findtext("pose")))
            if not (cp[2] - sz / 2 <= LIDAR_Z <= cp[2] + sz / 2):
                continue  # 垂れ壁など、LDS の高さに掛からない
            out.append((cp[0], cp[1], cp[3], sx, sy))


root = ET.parse(SDF).getroot()
boxes = []
walk(root, (0.0, 0.0, 0.0, 0.0), boxes)
if not boxes:
    sys.exit("no box collision found in %s" % SDF)

xs, ys = [], []
for cx, cy, yaw, sx, sy in boxes:
    r = math.hypot(sx, sy) / 2
    xs += [cx - r, cx + r]
    ys += [cy - r, cy + r]
ox = math.floor((min(xs) - MARGIN) / RES) * RES
oy = math.floor((min(ys) - MARGIN) / RES) * RES
w = int(math.ceil((max(xs) + MARGIN - ox) / RES))
h = int(math.ceil((max(ys) + MARGIN - oy) / RES))

OCC, FREE, UNK = 0, 254, 205
grid = [[UNK] * w for _ in range(h)]  # 行 j は y = oy + (j+0.5)*RES

for cx, cy, yaw, sx, sy in boxes:
    r = math.hypot(sx, sy) / 2
    c, s = math.cos(-yaw), math.sin(-yaw)
    for j in range(max(0, int((cy - r - oy) / RES)),
                   min(h, int((cy + r - oy) / RES) + 2)):
        y = oy + (j + 0.5) * RES - cy
        for i in range(max(0, int((cx - r - ox) / RES)),
                       min(w, int((cx + r - ox) / RES) + 2)):
            x = ox + (i + 0.5) * RES - cx
            if abs(c * x - s * y) <= sx / 2 and abs(s * x + c * y) <= sy / 2:
                grid[j][i] = OCC

# spawn から 4 近傍 flood fill: 届く所だけ free、家の外は unknown のまま
# (vi_planner の unknown_as_obstacle:=true が外周を壁として扱う)
si, sj = int((SPAWN[0] - ox) / RES), int((SPAWN[1] - oy) / RES)
assert grid[sj][si] != OCC, "spawn is inside a wall"
stack, grid[sj][si] = [(si, sj)], FREE
while stack:
    i, j = stack.pop()
    for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
        if 0 <= ni < w and 0 <= nj < h and grid[nj][ni] == UNK:
            grid[nj][ni] = FREE
            stack.append((ni, nj))

# 玄関 (x≈1.3, y=-0.175 の開口) が抜けていれば家の中まで fill が届かない。
# LIDAR_Z のスライス位置を間違えると垂れ壁で塞がるので、その回帰チェックも兼ねる。
li, lj = int((-2.0 - ox) / RES), int((3.0 - oy) / RES)
assert grid[lj][li] == FREE, "living room (-2, 3) unreachable from spawn"

os.makedirs(OUT, exist_ok=True)
pgm = os.path.join(OUT, "map.pgm")
with open(pgm, "wb") as f:  # PGM の行は上が y 最大 — grid を逆順に書く
    f.write(b"P5\n%d %d\n255\n" % (w, h))
    for j in range(h - 1, -1, -1):
        f.write(bytes(grid[j]))
with open(os.path.join(OUT, "map.yaml"), "w") as f:
    f.write("image: map.pgm\nresolution: %f\norigin: [%f, %f, 0.000000]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n" % (RES, ox, oy))

free = sum(row.count(FREE) for row in grid)
print("%s: %dx%d @ %.2f m, origin (%.2f, %.2f), free %d cells (%.1f m^2), walls %d"
      % (pgm, w, h, RES, ox, oy, free, free * RES * RES,
         sum(row.count(OCC) for row in grid)))
