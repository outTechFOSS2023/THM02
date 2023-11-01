# -*- coding: utf-8 -*-
from yade import pack,export
from yade import pack, plot 
from yade import ymport

O.load('/tmp/a.yade.bz2')
# maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
# print ("maxZ:",maxZ)
# for b in O.bodies:
# 	if isinstance(b.shape,Sphere):
# 		if b.state.pos[2]-b.shape.radius>=15.0:
# 			O.bodies.erase(b.id)
minX=min([b.state.pos[0]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
maxX=max([b.state.pos[0]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])

minY=min([b.state.pos[1]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
maxY=max([b.state.pos[1]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])

minZ=min([b.state.pos[2]-b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])
maxZ=max([b.state.pos[2]+b.shape.radius for b in O.bodies if isinstance(b.shape,Sphere)])

print ("minX:",minX,"maxX:",maxX,"minY:",minY,"maxY:",maxY,"minZ:",minZ,"maxZ:",maxZ)
export.text('/home/ngoudarzi/Desktop/VST_SA7(YADE_Periodic_Jiang)/Sample Preparation/Samples/layer_SA5_Regular_Small_Periodic_Real_SI_Uncut.txt')

 

