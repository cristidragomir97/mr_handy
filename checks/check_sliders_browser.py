import json,time
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True,args=['--no-sandbox']);page=browser.new_page(viewport={'width':1500,'height':1000});errors=[];sent=[];received=[]
 page.on('pageerror',lambda e:errors.append(str(e)))
 def connected(ws):
  ws.on('framesent',lambda f:sent.append(f));ws.on('framereceived',lambda f:received.append(f))
 page.on('websocket',connected)
 page.goto('http://127.0.0.1:8888/',wait_until='networkidle');page.locator('.mdl-layout__drawer-button').click();page.get_by_text('Joint sliders',exact=True).first.click();page.wait_for_timeout(2500)
 sliders=page.locator('input[type=range][data-joint]');print('Sliders',sliders.count());assert sliders.count()==16
 assert page.locator('[data-joint=left_arm_joint_wrist_yaw]').count()==1
 assert page.locator('[data-joint=right_arm_joint_wrist_yaw]').count()==1
 assert page.locator('[data-joint=left_lift_joint]').get_attribute('max')=='0.45'
 for name,goal in [('left_lift_joint',.03),('head_camera_tilt_joint',.2),('left_arm_joint_wrist_yaw',.15),('right_arm_6',.5)]:
  page.locator(f'[data-joint={name}]').evaluate('(e,value)=>{e.value=value;e.dispatchEvent(new Event("input",{bubbles:true}))}',goal)
  page.wait_for_timeout(600)
 page.wait_for_timeout(3500);page.screenshot(path='tmp/joint-sliders.png')
 commands=[]
 for frame in sent:
  try:msg=json.loads(frame)
  except:continue
  if isinstance(msg,list) and len(msg)>1 and isinstance(msg[1],dict) and msg[1].get('topicType')=='trajectory_msgs/msg/JointTrajectory':commands.append(msg[1])
 print('Trajectory publishes',len(commands));assert len(commands)>=4,commands
 states=[]
 for frame in received:
  try:message=json.loads(frame)
  except:continue
  if isinstance(message,list) and len(message)>1 and isinstance(message[1],dict) and message[1].get('_topic_name')=='/joint_states':states.append(message[1])
 assert states,'No live joint feedback in browser'
 state=dict(zip(states[-1]['name'],states[-1]['position']))
 for name,target in [('left_lift_joint',.03),('head_camera_tilt_joint',.2),('left_arm_joint_wrist_yaw',.15),('right_arm_6',.5)]:
  assert abs(state[name]-target)<.015,(name,state[name],target)
 print('PASS: browser slider goals reached measured robot feedback.',state['left_lift_joint'],state['head_camera_tilt_joint'],state['left_arm_joint_wrist_yaw'],state['right_arm_6'])
 assert not errors,errors
 print('PASS: all 16 sliders, both wrist-yaw axes, lift metre limits and real trajectory commands.')
 browser.close()
