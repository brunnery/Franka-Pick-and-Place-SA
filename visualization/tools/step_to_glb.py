import sys, json, numpy as np, trimesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.IFSelect import IFSelect_RetDone

def name_of(lab):
    n = TDataStd_Name()
    if lab.FindAttribute(TDataStd_Name.GetID_s(), n):
        return n.Get().ToExtString()
    return '?'

def mesh_shape(shape, defl=0.1):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.3, True)
    V=[];F=[];off=0
    ex=TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face=TopoDS.Face(ex.Current()); loc=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            tr=loc.Transformation()
            for i in range(1,tri.NbNodes()+1):
                p=tri.Node(i).Transformed(tr); V.append((p.X(),p.Y(),p.Z()))
            rev = face.Orientation()==1
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get()
                F.append((off+a-1,off+c-1,off+b-1) if rev else (off+a-1,off+b-1,off+c-1))
            off+=tri.NbNodes()
        ex.Next()
    return np.array(V), np.array(F)

def walk(st, lab, depth, out, loc=None):
    from OCP.TopLoc import TopLoc_Location
    nm=name_of(lab)
    shape=st.GetShape_s(lab)
    if st.IsAssembly_s(lab):
        seq=TDF_LabelSequence(); st.GetComponents_s(lab, seq)
        print('  '*depth+f'[A] {nm} ({seq.Length()})')
        for i in range(1,seq.Length()+1):
            walk(st, seq.Value(i), depth+1, out)
    elif st.IsReference_s(lab):
        ref=TDF_Label(); st.GetReferredShape_s(lab, ref)
        if st.IsAssembly_s(ref):
            print('  '*depth+f'[R->A] {nm}')
            seq=TDF_LabelSequence(); st.GetComponents_s(ref, seq)
            for i in range(1,seq.Length()+1): walk(st, seq.Value(i), depth+1, out)
        else:
            # mesh the located shape of the component
            out.append((nm, name_of(ref), shape))
            print('  '*depth+f'[P] {nm} -> {name_of(ref)}')
    else:
        out.append((nm, nm, shape)); print('  '*depth+f'[S] {nm}')

f=sys.argv[1]
doc=TDocStd_Document(TCollection_ExtendedString("doc"))
r=STEPCAFControl_Reader(); r.SetNameMode(True)
assert r.ReadFile(f)==IFSelect_RetDone; r.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots=TDF_LabelSequence(); st.GetFreeShapes(roots)
out=[]
for i in range(1,roots.Length()+1): walk(st, roots.Value(i), 0, out)
scene=trimesh.Scene(); info=[]
solids=[]
for inst,part,shape in out:
    ex=TopExp_Explorer(shape, TopAbs_SOLID); j=0
    while ex.More():
        solids.append((f'{inst}#{j}',part,ex.Current())); j+=1; ex.Next()
for k,(inst,part,shape) in enumerate(solids):
    V,Fc=mesh_shape(shape)
    if len(V)==0: continue
    m=trimesh.Trimesh(V,Fc,process=True)
    scene.add_geometry(m, node_name=f'{k}_{inst}', geom_name=f'{k}_{inst}')
    info.append(dict(i=k,inst=inst,part=part,tris=len(m.faces),min=m.bounds[0].round(1).tolist(),max=m.bounds[1].round(1).tolist()))
for x in info: print(x)
print('TOTAL bounds', scene.bounds.round(1).tolist())
scene.export(sys.argv[2])
