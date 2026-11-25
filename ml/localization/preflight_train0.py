"""Static candidate-route geometry only. Does not launch Webots or authorize movement."""
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"ml/perception"))
from generate_layout import build_layout, generate, layout_seed, load_splits, top_level_nodes

MARGIN=.272+.35
RASTER_STEP=.002


def clearance(point, footprints, half):
    x,y=point
    return min(half-max(abs(x),abs(y)),
               *(math.hypot(max(a-x,x-c,0),max(b-y,y-d,0)) for a,b,c,d in footprints))


def route():
    start=(-8.,0.)
    points=[start,(-7.4,0.),(-7.4+.3*math.cos(.42),.3*math.sin(.42))]
    for a,b in zip(points,points[1:]):
        steps=math.ceil(math.dist(a,b)/RASTER_STEP)
        yield from ((a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps)
                    for i in range(steps+1))


def main():
    template=(ROOT/"simulation/worlds/warehouse.wbt").read_text()
    _,nodes=top_level_nodes(template)
    arena=dict(nodes)["RectangleArena"]
    sx,sy=map(float,re.search(r"floorSize ([\d.]+) ([\d.]+)",arena).groups())
    wall=float(re.search(r"wallThickness ([\d.]+)",arena).group(1))
    if sx!=sy: raise SystemExit("expected square class warehouse")
    half=sx/2-wall/2-.05
    out={}
    for name in ("train_0","train_1","train_2"):
        seed=layout_seed(name,load_splits())
        shape=build_layout(seed,half,half)
        world=generate(template,name,seed,"../protos")
        if any(t not in world for t in ('translation -8 0 0.095','supervisor TRUE','"capstone lidar"','Astra {')):
            raise SystemExit("required TIAGo class commissioning features missing")
        margin=min(clearance(p,shape.rects,half) for p in route())-.005
        out[name]=dict(seed=seed,planned_world_sha256=hashlib.sha256(world.encode()).hexdigest(),
                       footprint_count=len(shape.rects),centerline_clearance_lower_bound_m=margin,
                       conservative_margin_m=MARGIN,candidate_route_geometry_clear=margin>=MARGIN)
    print(json.dumps(dict(scope="static candidate commissioning route; not a mapping route, smoke or permission to move",
                          results=out),indent=2))
    if not out["train_0"]["candidate_route_geometry_clear"]: return 2
    return 0


if __name__=="__main__":
    raise SystemExit(main())
