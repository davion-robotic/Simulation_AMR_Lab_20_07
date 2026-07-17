#!/usr/bin/env python3
"""
Regenerate nav_graphs/0.yaml from L1.building.yaml AND convert its coordinates
from traffic-editor pixels into Nav2 world meters.

Why: our traffic-editor map has no 'measurement', so building_map_generator emits
pixel coordinates. But the floorplan IS the Nav2 map (my_map.png == my_map.pgm),
which already defines the scale. We use that scale instead of a measurement:

    world_x = origin_x + px * resolution
    world_y = origin_y + (img_height - py) * resolution      (y flipped)

Run after every traffic-editor save:
    python3 gen_navgraph_meters.py
"""
import os, subprocess, tempfile, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
BUILDING = os.path.join(HERE, 'L1.building.yaml')
MAP_YAML = os.path.join(HERE, 'my_map.yaml')          # Nav2 map: resolution + origin
OUT      = os.path.join(HERE, 'nav_graphs', '0.yaml')


def read_map_scale():
    with open(MAP_YAML) as f:
        m = yaml.safe_load(f)
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    pgm = os.path.join(HERE, m['image'])
    with open(pgm, 'rb') as f:                         # read PGM header for height
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
    return res, ox, oy, h


def main():
    res, ox, oy, H = read_map_scale()
    print(f"[gen] Nav2 scale: res={res} m/px, origin=({ox},{oy}), img_height={H}px")

    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            ['ros2', 'run', 'rmf_building_map_tools', 'building_map_generator',
             'nav', BUILDING, tmp], check=True)
        with open(os.path.join(tmp, '0.yaml')) as f:
            g = yaml.safe_load(f)

    # generator y is already negated (gy = -py_building), so:
    #   world_y = oy + (H + gy) * res
    for lvl in g.get('levels', {}).values():
        for v in lvl.get('vertices', []):
            gx, gy = float(v[0]), float(v[1])
            v[0] = round(ox + gx * res, 3)
            v[1] = round(oy + (H + gy) * res, 3)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        yaml.safe_dump(g, f, default_flow_style=False, sort_keys=False)

    print(f"[gen] wrote {OUT} (meters). Vertices:")
    for lvl in g.get('levels', {}).values():
        for i, v in enumerate(lvl.get('vertices', [])):
            p = v[2] if len(v) > 2 and isinstance(v[2], dict) else {}
            print(f"   v{i}: ({v[0]:.3f},{v[1]:.3f}) name={p.get('name','')!r} "
                  f"charger={p.get('is_charger', False)}")


if __name__ == '__main__':
    main()
