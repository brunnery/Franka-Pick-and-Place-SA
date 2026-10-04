# Run from a working folder that contains:
#   franka_description/  (git clone https://github.com/frankarobotics/franka_description)
#   cad/Setup.glb, cad/Iris_wheel.glb  (made with step_to_glb.py)
#   viz/assets/  (output)
import trimesh, numpy as np, fast_simplification, sys
sys.path.insert(0,'cad')
SP='.'
def dec(g, ratio):
    if len(g.faces) < 2500: return g
    v,f = fast_simplification.simplify(g.vertices.astype(np.float32), g.faces.astype(np.int32), target_reduction=1-ratio)
    m=trimesh.Trimesh(v,f,process=False)
    m.visual=trimesh.visual.TextureVisuals(material=g.visual.material) if hasattr(g.visual,'material') else g.visual
    return m
def colorize(m, rgba, metal=0.1, rough=0.55):
    mat=trimesh.visual.material.PBRMaterial(baseColorFactor=rgba, metallicFactor=metal, roughnessFactor=rough)
    m=trimesh.Trimesh(m.vertices,m.faces,process=True)
    m=trimesh.graph.smooth_shade(m, angle=np.radians(35))
    m.visual=trimesh.visual.TextureVisuals(material=mat); return m

# ---------- Franka FR3 ----------
sc=trimesh.Scene()
for i in range(8):
    s=trimesh.load(f'franka_description/meshes/robots/fr3/visual/link{i}.dae')
    sc.graph.update(frame_from=sc.graph.base_frame, frame_to=f'link{i}')
    for j,(n,g) in enumerate(s.geometry.items()):
        # bake scene-graph transform of the dae
        nodes=s.graph.geometry_nodes[n]
        for k,node in enumerate(nodes):
            Tm,_=s.graph[node]; gg=g.copy(); gg.apply_transform(Tm)
            c=np.array(g.visual.material.baseColorFactor if hasattr(g.visual.material,'baseColorFactor') and g.visual.material.baseColorFactor is not None else [200,200,200,255])/255
            gg=dec(gg,0.45)
            lum=c[:3].mean()
            gg=colorize(gg, list(c), metal=0.0, rough=0.35 if lum>0.8 else 0.6)
            sc.add_geometry(gg, node_name=f'link{i}_m{j}_{k}', geom_name=f'link{i}_m{j}_{k}', parent_node_name=f'link{i}')
print('franka faces', sum(len(g.faces) for g in sc.geometry.values()))
sc.export('viz/assets/franka_fr3.glb', include_normals=True)

# ---------- Setup ----------
s=trimesh.load('cad/Setup.glb')
names={0:'base_plate',1:'ph_pedestal',2:'ph_column',3:'fixture_nut',4:'fixture_screw',5:'nut',6:'screw',7:'ph_tool',8:'ph_arm'}
out=trimesh.Scene()
for n,g in s.geometry.items():
    i=int(n.split('_')[0]); g=g.copy(); g.apply_scale(0.001)
    if names[i]=='ph_arm': g=dec(g,0.35)
    out.add_geometry(colorize(g,[0.7,0.7,0.7,1]), node_name=names[i], geom_name=names[i])
out.export('viz/assets/setup.glb', include_normals=True)

# ---------- Iris wheel gripper (Iris_wheel_gripper.step) ----------
ROLE={0:'iris_ring',1:'post_l',2:'post_r',3:'bottom_plate',4:'top_box',5:'flange_adapter',6:'shaft',7:'gear_mid',
      8:'frame',9:'socket',10:'gear_top',11:'housing_l',12:'housing_r',13:'motor_mount',14:'clip_a',15:'clip_b',16:'camera',
      17:'wheel_l',18:'wheel_r',19:'blade_0',20:'blade_1',21:'blade_2',22:'blade_3',23:'blade_4',24:'blade_5'}
s=trimesh.load('cad/Iris_wheel.glb')
out=trimesh.Scene()
for n,g in s.geometry.items():
    i=int(n.split('_')[0]); g=g.copy(); g.apply_scale(0.001); g=dec(g,0.6)
    out.add_geometry(colorize(g,[0.7,0.7,0.7,1]), node_name=ROLE[i], geom_name=ROLE[i])
out.export('viz/assets/gripper_iris.glb', include_normals=True)
