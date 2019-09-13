import sys
import os
import numpy as np
import math

DATA_INPUT = sys.argv[1]#"/home/milos/LineMatching/output_data/office_lines.ply"
RESULTS_FOLDER= sys.argv[2]#"/home/milos/LineMatching/output_data"

MARK_SIZE=0.03

#Calculate marks faces
vertices=[np.array([1,1,1]),np.array([1,1,-1]),np.array([1,-1,1]),np.array([1,-1,-1]),np.array([-1,1,1]),np.array([-1,1,-1]),np.array([-1,-1,1]),np.array([-1,-1,-1])]
vertices=list(map(lambda x: MARK_SIZE * x, vertices))


#Makes cube around xyz
def make_cube(xyz):

    #Output: [6x[list of vertices]]

    verts = list(map(lambda x: xyz + x, vertices))

    faces = np.array([[verts[0], verts[4], verts[1],verts[5]],
                     [verts[2], verts[6], verts[3], verts[7]],
                     [verts[1], verts[4], verts[3], verts[7]],
                     [verts[0], verts[5], verts[2], verts[6]],
                     [verts[0], verts[2], verts[1], verts[3]],
                     [verts[4], verts[6], verts[5], verts[7]]])   
    
    #Return cross constructed in naive way
    return list(map(lambda x: x/2.0, np.array([faces[0]+faces[1],faces[2]+faces[3],faces[4]+faces[5]])))

data=np.loadtxt(DATA_INPUT)

pointcounts=data[:,0]
data=data[:,1:]

mask=np.where(pointcounts > 1)[0]
data=data[mask,:]
pointcounts=pointcounts[mask]

length=1
multipl=150

d=np.empty((data.shape[0]*multipl,3), dtype=np.float32)
p=np.empty(data.shape[0]*multipl, dtype=np.float32)

for i in range(multipl):
    d[data.shape[0]*i:data.shape[0]*(i+1),:] = data[:,:3] + length*i/float(multipl)*data[:,3:]
    p[data.shape[0]*i:data.shape[0]*(i+1)] = pointcounts
data=d

descr=DATA_INPUT.split("/")[-1][:-5]

finalFilePath = os.path.join(RESULTS_FOLDER, "{:s}_lines.ply".format(descr))

with open(finalFilePath,'w') as f:
    f.write('ply\n')
    if sys.byteorder == 'little':
        f.write('format binary_little_endian 1.0\n')
    else:
        f.write('format binary_big_endian 1.0\n')
    f.write('element vertex {:d}\n'.format(data.shape[0]*12))
    f.write('property float x\n')
    f.write('property float y\n')
    f.write('property float z\n')
    
    f.write('property uchar red\n')
    f.write('property uchar green\n')         
    f.write('property uchar blue\n')
    
    f.write('element face {:d}\n'.format(data.shape[0]))
    f.write('property list uchar int vertex_index\n')
    f.write('end_header\n')

marks = np.apply_along_axis( make_cube, axis=1, arr=data[:,0:3] ).astype(np.float32)

marks = np.reshape(marks, (-1,3))

faces = np.reshape(np.arange(data.shape[0]*12, dtype=np.int32), [-1, 4])

tmp2Path=os.path.join(RESULTS_FOLDER, "_marks.tmp")
tmp3Path=os.path.join(RESULTS_FOLDER, "_faces.tmp")

#marks.tofile(tmp2Path)
#marks_bytes=marks.tobytes()

max_count = math.log(float(np.max(pointcounts)))

with open(tmp2Path, 'w+b') as f:

    i=0

    for byte in [marks[x:x+1] for x in range(len(marks))]:

        f.write(byte)
        green = int( math.log(p[int(i/12)]) / max_count * 255 )
        blue = 255 - green

        g=bytes([green])
        b=bytes([blue])

        f.write(b'\00')
        f.write(g)
        f.write(b)

        i+=1

#np.savetxt(tmp2Path, marks, fmt="%f")#.tofile(tmp2Path) 
#np.savetxt(tmp3Path, np.concatenate((4*np.ones((faces.shape[0],1)),faces), axis=1), fmt="%d")
#faces_bytes=faces.tobytes()

four=b'\x04'

with open(tmp3Path, 'w+b') as f:

    for byte in [faces[x:x+1] for x in range(len(faces))]:

        f.write(four)
        f.write(byte)

os.system("cat {:s} {:s} >> {:s}".format(tmp2Path, tmp3Path, finalFilePath))
os.system("rm {:s} {:s}".format(tmp2Path, tmp3Path))

print("{:s}_lines.ply created\n".format(descr))
