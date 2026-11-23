# Terrain for hive_base

`terrain.dae` (the ground, visual), `terrain_collision.dae` (only where the ground rises
above 5 cm; the flat ground plane in the world carries the rest) and `terrain.jpg` (the
painted ground texture: forest floor, meadows, fields, heath and rock) are **generated** by
`worlds/generate_hive_base.py` together with `worlds/hive_base.sdf`. Do not edit them by
hand: change the generator and run `python3 worlds/generate_hive_base.py` from the repo root
(needs numpy and Pillow).

The world references them as `/workspace/worlds/terrain/...`, the path where the start
scripts mount the repo inside the container, like the trees.
