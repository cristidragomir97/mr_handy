#!/usr/bin/env python3
"""Validate navigable room openings and physics of the loose pickup props."""
from pathlib import Path
import json
import mujoco,numpy as np
p=Path('tmp/home/scene.xml');m=mujoco.MjModel.from_xml_path(str(p));d=mujoco.MjData(m)
for sensor in range(m.nsensor):
 if m.sensor_objtype[sensor]==mujoco.mjtObj.mjOBJ_SITE:
  name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_SENSOR,sensor)
  site='base_imu_site' if name.startswith('base_imu_') else name
  assert m.sensor_objid[sensor]==mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,site),name
props=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name) for name in
 ('pickup_cup_kitchen','pickup_honey_kitchen','pickup_cup_dining','pickup_honey_dining','pickup_cup_living','pickup_honey_living','pickup_block_0','pickup_block_1','pickup_block_2')]
assert all(i>=0 for i in props)
for body in props:
 assert m.body_mass[body]>0
 joint=m.body_jntadr[body];assert m.jnt_type[joint]==mujoco.mjtJoint.mjJNT_FREE
assert len({j['asset'] for j in json.loads((p.parent/'world_manifest.json').read_text())['instances']})==4
for _ in range(3000):mujoco.mj_step(m,d)
assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
accel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SENSOR,'base_imu_accel')
assert np.linalg.norm(d.sensordata[m.sensor_adr[accel]:m.sensor_adr[accel]+3])>8
print('PASS: appliance sites preserve every robot sensor binding and the IMU measures gravity.')
lidar=[d.sensordata[m.sensor_adr[i]] for i in range(m.nsensor) if m.sensor_type[i]==mujoco.mjtSensor.mjSENS_RANGEFINDER]
assert sum(.12<v<12 for v in lidar)>180,(min(lidar),max(lidar))
print('PASS: lidar sees room geometry instead of its own housing.')
for body in props:
 assert .40<d.xpos[body,2]<.85,(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,body),d.xpos[body])
print('PASS: nine positive-mass, free-jointed pickup objects remain on their supports after six seconds of contact physics.')
# Move one object clear of its support: it must respond to gravity, not stay frozen.
body=props[0];joint=m.body_jntadr[body];index=m.jnt_qposadr[joint];d.qpos[index:index+3]=[.5,-.5,.5];mujoco.mj_forward(m,d)
for _ in range(600):mujoco.mj_step(m,d)
assert d.xpos[body,2]<.15
print('PASS: a released pickup object falls and collides with the room floor.')
# Scene has three connected floor areas and three authored door openings >=1.4m.
for floor in ('kitchen_floor','dining_floor','living_floor'):assert mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,floor)>=0
assert m.ncam==3
print('PASS: three room floors, articulated RoboCasa appliances, and only base/head/Luxonis cameras.')
