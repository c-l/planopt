import openravepy
import numpy as np
from operator import truediv

def create_dest(env, destname, t, vert=False, dim=[0.1,0.1]):
  THICKNESS = 0.0001

  surface = openravepy.KinBody.GeometryInfo()
  surface._type = openravepy.GeometryType.Box
  if vert:
    surface._vGeomData = [THICKNESS/2, float(dim[0])/2, float(dim[1])/2]
  else:
    surface._vGeomData = [float(dim[0])/2, float(dim[1])/2, THICKNESS/2]
  surface._vDiffuseColor = [1, 0, 0]

  dest = openravepy.RaveCreateKinBody(env, '')
  dest.InitFromGeometries([surface])
  dest.SetName(destname)

  dest.SetTransform(t)

  return dest

def on_table(obj, table, offset=0):
  T = obj.GetTransform()

  table_ab = table.ComputeAABB()
  table_top_z = table_ab.pos()[2] + table_ab.extents()[2]

  if "cloth" in obj.GetName():
    T[2, 3] = table_top_z + 0.03
    obj.SetTransform(T)
  else:
    obj_ab = obj.ComputeAABB()
    obj_min_z = obj_ab.pos()[2] - obj_ab.extents()[2]

    diff_z = obj_min_z - table_top_z
    T[2, 3] -= diff_z
    T[2, 3] += offset
    obj.SetTransform(T)

def create_cylinder(env, body_name, t, dims, color=[0,1,1]):
  infocylinder = openravepy.KinBody.GeometryInfo()
  infocylinder._type = openravepy.GeometryType.Cylinder
  infocylinder._vGeomData = dims
  infocylinder._bVisible = True
  infocylinder._vDiffuseColor = color
  infocylinder._t[2, 3] = dims[1] / 2.0

  # cylinder = openravepy.RaveCreateKinBody(env, '')
  cylinder = openravepy.RaveCreateRobot(env, '')
  cylinder.InitFromGeometries([infocylinder])
  cylinder.SetName(body_name)
  cylinder.SetTransform(t)
  cylinder.SetActiveDOFs(np.ndarray(0), openravepy.DOFAffine.Transform)

  return cylinder

def cylinder_collisionfree_model(env, body_name, pos, dims):
  iv_str = '#Inventor V2.1 ascii\nSeparator {\nCylinder {\nparts ALL\nradius %f\nheight %f\n}\n}'%(dims[0], dims[1]) # hardcode collision-free disk dimensions
  with open("../models/cylinder.iv", "w+") as f:
    f.write(iv_str)
  xml_str = '<KinBody name="%s"> <Body name="surface" type="dynamic"> <Geom type="sphere"> <Render>cylinder.iv</Render> <Radius>0.0001</Radius> \
<Translation>0 0 %f</Translation> <RotationAxis>1 0 0 90></RotationAxis> </Geom> </Body> </KinBody>'%(body_name, dims[1] / 2)
  with open("../models/temp_cylinder.xml", "w+") as f:
    f.write(xml_str)
  env.Load("../models/temp_cylinder.xml")
  cylinder = env.GetKinBody(body_name)
  x, y, z = pos
  cylinder_t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
  cylinder.SetTransform(cylinder_t)

  return cylinder

def create_table(env, table_name, dim1, dim2, thickness, legdim1, legdim2, legheight, back = False):
  tabletop = openravepy.KinBody.GeometryInfo()
  tabletop._type = openravepy.GeometryType.Box
  tabletop._vGeomData = [dim1/2, dim2/2, thickness/2]
  tabletop._vDiffuseColor = [0.5, 0.2, 0.1]

  leg1 = openravepy.KinBody.GeometryInfo()
  leg1._type = openravepy.GeometryType.Box
  leg1._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg1._t[0, 3] = dim1/2 - legdim1/2
  leg1._t[1, 3] = dim2/2 - legdim2/2
  leg1._t[2, 3] = -legheight/2 - thickness/2
  leg1._vDiffuseColor = [0.5, 0.2, 0.1]

  leg2 = openravepy.KinBody.GeometryInfo()
  leg2._type = openravepy.GeometryType.Box
  leg2._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg2._t[0, 3] = dim1/2 - legdim1/2
  leg2._t[1, 3] = -dim2/2 + legdim2/2
  leg2._t[2, 3] = -legheight/2 - thickness/2
  leg2._vDiffuseColor = [0.5, 0.2, 0.1]

  leg3 = openravepy.KinBody.GeometryInfo()
  leg3._type = openravepy.GeometryType.Box
  leg3._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg3._t[0, 3] = -dim1/2 + legdim1/2
  leg3._t[1, 3] = dim2/2 - legdim2/2
  leg3._t[2, 3] = -legheight/2 - thickness/2
  leg3._vDiffuseColor = [0.5, 0.2, 0.1]

  leg4 = openravepy.KinBody.GeometryInfo()
  leg4._type = openravepy.GeometryType.Box
  leg4._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg4._t[0, 3] = -dim1/2 + legdim1/2
  leg4._t[1, 3] = -dim2/2 + legdim2/2
  leg4._t[2, 3] = -legheight/2 - thickness/2
  leg4._vDiffuseColor = [0.5, 0.2, 0.1]

  if back:
    back_plate = openravepy.KinBody.GeometryInfo()
    back_plate._type = openravepy.GeometryType.Box
    back_plate._vGeomData = [legdim1/10, dim2/2, legheight-thickness/2]
    back_plate._t[0, 3] = dim1/2 - legdim1/10
    back_plate._t[1, 3] = 0
    back_plate._t[2, 3] = -legheight/2 - thickness/4
    back_plate._vDiffuseColor = [0.5, 0.2, 0.1]
    # import IPython
    # IPython.embed()

  # table = openravepy.RaveCreateKinBody(env, '')
  table = openravepy.RaveCreateRobot(env, '')
  if not back:
    table.InitFromGeometries([tabletop, leg1, leg2, leg3, leg4])
  else:
    table.InitFromGeometries([tabletop, leg1, leg2, leg3, leg4, back_plate])
  table.SetName(table_name)

  return table
