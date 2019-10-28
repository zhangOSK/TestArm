from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph import plug

taskRH    = MetaTaskKine6d('rh',robot.dynamic,'rh',robot.OperationalPointsMap['right-wrist'])
taskRH.feature.frame('desired')

taskLH    = MetaTaskKine6d('lh',robot.dynamic,'lh',robot.OperationalPointsMap['left-wrist'])
taskLH.feature.frame('desired')

taskC    = MetaTaskKine6d('ch',robot.dynamic,'ch',robot.OperationalPointsMap['chest'])
taskC.feature.frame('desired')

# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10
taskCom.feature.selec.value = '011'

# --- POSTURE 
robot.taskUpperBody = Task ('task_upper_body')
robot.taskUpperBody.feature = FeaturePosture('feature_upper_body')

q = list(robot.dynamic.position.value)
robot.taskUpperBody.feature.state.value = q
robot.taskUpperBody.feature.posture.value = q

robot.taskUpperBody.feature.selectDof(27,True)
robot.taskUpperBody.feature.selectDof(35,True)

robot.taskUpperBody.controlGain.value = 100.0
robot.taskUpperBody.add(robot.taskUpperBody.feature.name)
plug(robot.dynamic.position, robot.taskUpperBody.feature.state)

# --- CONTACTS (define contactLF and contactRF)
contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
contactLF.feature.frame('desired')
contactLF.gain.setConstant(10)
contactLF.keep()
locals()['contactLF'] = contactLF

contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
contactRF.feature.frame('desired')
contactRF.gain.setConstant(10)
contactRF.keep()
locals()['contactRF'] = contactRF

from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control,robot.device.control)

from dynamic_graph.ros import RosPublish
ros_publish_state = RosPublish ("ros_publish_state")
ros_publish_state.add ("vector", "state", "/sot_control/state")
from dynamic_graph import plug
plug (robot.device.state, ros_publish_state.state)
robot.device.after.addDownsampledSignal ("ros_publish_state.trigger", 100)

sot.push(robot.taskUpperBody.name)
sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
robot.device.control.recompute(0)
