"""Build the small UI preview mesh from the installed robot description."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from ament_index_python.packages import get_package_share_directory


def rotation(rpy):
    r,p,y=rpy; cr,sr=np.cos(r),np.sin(r); cp,sp=np.cos(p),np.sin(p); cy,sy=np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def origin(element):
    m=np.eye(4)
    if element is not None:
        m[:3,:3]=rotation(np.fromstring(element.get('rpy','0 0 0'),sep=' '))
        m[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
    return m


def mesh_points(path):
    root=ET.parse(path).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    meshes={}
    for g in root.findall('.//c:geometry',ns):
        pos=g.find('.//c:vertices/c:input[@semantic="POSITION"]',ns).get('source')[1:]
        meshes[g.get('id')]=np.fromstring(g.find(f'.//c:source[@id="{pos}"]/c:float_array',ns).text,sep=' ').reshape(-1,3)
    points=[]
    def visit(node,parent):
        m=parent.copy()
        for t in node:
            tag=t.tag.rsplit('}',1)[-1]
            if tag=='matrix':m=m@np.fromstring(t.text,sep=' ').reshape(4,4)
            elif tag=='instance_geometry':
                v=meshes[t.get('url')[1:]];points.append(v@m[:3,:3].T+m[:3,3])
        for child in node.findall('c:node',ns):visit(child,m)
    for node in root.findall('c:library_visual_scenes/c:visual_scene/c:node',ns):visit(node,np.eye(4))
    unit=root.find('c:asset/c:unit',ns)
    return np.concatenate(points)*float(unit.get('meter','1'))


def hull(points, samples):
    k=np.arange(samples);z=1-2*(k+.5)/samples;a=k*np.pi*(3-np.sqrt(5));r=np.sqrt(1-z*z)
    directions=np.stack([r*np.cos(a),r*np.sin(a),z],axis=1)
    v=np.unique(points[np.argmax(points@directions.T,axis=0)],axis=0)
    h=ConvexHull(v);f=h.simplices.copy()
    for i,face in enumerate(f):
        normal=np.cross(v[face[1]]-v[face[0]],v[face[2]]-v[face[0]])
        if normal@h.equations[i,:3]<0:f[i,[1,2]]=f[i,[2,1]]
    return v,f


def build(output):
    share=Path(get_package_share_directory('go2_mid360_description'))
    robot=ET.fromstring(subprocess.check_output(['xacro',str(share/'xacro/robot_mid360.xacro')],text=True))
    joints={j.find('child').get('link'):j for j in robot.findall('joint')}
    parts=[]
    for link in robot.findall('link'):
        name=link.get('name');joint=joints.get(name)
        part={'name':name,'parent':joint.find('parent').get('link') if joint is not None else None,
              'origin':origin(joint.find('origin') if joint is not None else None).tolist(),
              'joint':joint.get('name') if joint is not None and joint.get('type')=='revolute' else '',
              'axis':np.fromstring(joint.find('axis').get('xyz'),sep=' ').tolist() if joint is not None and joint.find('axis') is not None else [0,0,1], 'meshes':[]}
        for index,visual in enumerate(link.findall('visual')):
            # Omit tiny placeholders and the detailed open protective cage.
            if name in ('base_link','imu_link') or (name=='mid360_mount_link' and index>0):continue
            g=visual.find('geometry')[0];kind=g.tag
            if kind=='mesh':
                uri=g.get('filename')
                if uri.startswith('file://'):path=uri[7:]
                elif uri.startswith('package://'):
                    package,relative=uri[10:].split('/',1);path=str(Path(get_package_share_directory(package))/relative)
                else:raise ValueError(uri)
                points=mesh_points(path)*np.fromstring(g.get('scale','1 1 1'),sep=' ')
            elif kind=='sphere':
                t=np.linspace(0,np.pi,7);a=np.linspace(0,2*np.pi,10,endpoint=False)
                points=np.array([[np.sin(p)*np.cos(y),np.sin(p)*np.sin(y),np.cos(p)] for p in t for y in a])*float(g.get('radius'))
            else:continue
            v,f=hull(points,48 if name=='trunk' else 20)
            m=origin(visual.find('origin'));v=v@m[:3,:3].T+m[:3,3]
            color=[183,203,217]
            if 'foot' in name:color=[42,60,73]
            if 'mid360' in name:color=[58,165,184] if name=='mid360_link' else [61,81,98]
            part['meshes'].append({'vertices':np.round(v,6).tolist(),'faces':f.tolist(),'color':color})
        parts.append(part)
    output.write_text(json.dumps(parts,separators=(',',':')))
    print(len(parts),'links,',sum(len(m['faces']) for p in parts for m in p['meshes']),'triangles,',output.stat().st_size,'bytes')


if __name__=='__main__':
    build(Path(__file__).resolve().parents[1]/'assets/go2-preview.json')
