"""정차 자세 쌍마다 최단 경로(로봇 반경 여유)가 west_aisle을 지나는지, 막으면 얼마나 돌아가는지."""
import heapq
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

repo = sys.argv[1]
img = np.array(Image.open(f'{repo}/src/localization/amr_mapping/maps/restaurant_gt.pgm'))
res, ox, oy = 0.05, -12.5, -8.5
H, W = img.shape
occ = img < 128
clear = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
dist = ndimage.distance_transform_edt(~occ) * res
free = dist > clear
st = yaml.safe_load(open(f'{repo}/src/fleet/amr_fleet_manager/config/stations_restaurant.yaml'))['stations']
zones = {z['id']: z['area'] for z in yaml.safe_load(open(
    f'{repo}/src/fleet/amr_fleet_manager/config/congestion_zones_restaurant.yaml'))['zones']}


def cell(x, y):
    return H - 1 - int((y - oy) / res), int((x - ox) / res)


def xy(r, c):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res


def in_zone(r, c, a):
    x, y = xy(r, c)
    return a[0] <= x <= a[1] and a[2] <= y <= a[3]


wa = zones['west_aisle']
zone_mask = np.zeros_like(free)
for r in range(H):
    for c in range(W):
        zone_mask[r, c] = in_zone(r, c, wa)

N8 = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


def dijkstra(src, ok):
    d = np.full(free.shape, np.inf)
    prev = {}
    d[src] = 0
    q = [(0, src)]
    while q:
        dd, u = heapq.heappop(q)
        if dd > d[u]:
            continue
        for dr, dc, w in N8:
            v = (u[0] + dr, u[1] + dc)
            if 0 <= v[0] < H and 0 <= v[1] < W and ok[v] and dd + w < d[v]:
                d[v] = dd + w
                prev[v] = u
                heapq.heappush(q, (d[v], v))
    return d, prev


def path(prev, src, dst):
    p = [dst]
    while p[-1] != src:
        p.append(prev[p[-1]])
    return p[::-1]


names = list(st)
cells = {n: cell(*st[n][:2]) for n in names}
bad = [n for n in names if not free[cells[n]]]
print('stations not free at clearance', clear, bad)
ok_free = free.copy()
ok_avoid = free & ~zone_mask
rows = []
for a in names:
    if not free[cells[a]] or zone_mask[cells[a]]:
        continue
    d1, p1 = dijkstra(cells[a], ok_free)
    d2, _ = dijkstra(cells[a], ok_avoid)
    for b in names:
        if b == a or not free[cells[b]] or zone_mask[cells[b]] or not np.isfinite(d1[cells[b]]):
            continue
        pth = path(p1, cells[a], cells[b])
        inside = sum(zone_mask[c] for c in pth) * res
        if inside < 1.0:
            continue
        l1, l2 = d1[cells[b]] * res, d2[cells[b]] * res
        rows.append((a, b, l1, l2, inside))
rows.sort(key=lambda r: (r[3] - r[2]))
for a, b, l1, l2, inside in rows[:60]:
    print(f'{a:>10} -> {b:<10} short {l1:5.1f} m  in west_aisle {inside:4.1f} m  detour {l2:5.1f} m (+{l2 - l1:4.1f})')
