#!/usr/bin/env python
import pcl

pc = pcl.load("./sample/lobby.pcd") #"pc.from_file" Deprecated
#cloud = pcl.load_XYZRGBA("tabletop.pcd")
print(pc)
