import base64, pathlib, sys
here = pathlib.Path(__file__).resolve().parent.parent
src = (here/'src/viewer.src.html').read_text(encoding='utf-8')
kin = (here/'src/kin.js').read_text(encoding='utf-8')
a = {k: base64.b64encode((here/'assets'/f).read_bytes()).decode() for k, f in
     [('franka','franka_fr3.glb'),('setup','setup.glb'),('gripper','gripper_iris.glb')]}
assets = 'const ASSETS = {' + ','.join(f'{k}:"{v}"' for k, v in a.items()) + '};'
out = src.replace('/*KIN*/', kin).replace('/*ASSETS*/', assets)
dst = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else here/'index.html'
dst.write_text(out, encoding='utf-8'); print(dst, len(out)//1024, 'KB')
