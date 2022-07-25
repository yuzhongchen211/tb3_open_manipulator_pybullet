# tb3_open_manipulator_pybullet
Turtlebot3 with open manipulator in pybullet

## Control
turtlebot3 with open manipulator (tb3om) 一共有18个Joint，控制机器人其实就是控制Joint的参数。
但是并不是所有Joint都可以被控制，tb3om能控制的Joint有：`[b'wheel_left_joint', b'wheel_right_joint', b'joint1', b'joint2', b'joint3', b'joint4', b'gripper', b'gripper_sub']` <br>
他们的index为:`[1, 2, 11, 12, 13, 14, 15, 16]` <br>
```python
p.setJointMotorControlArray(
        bodyUniqueId=robot_id, #机器人的ID
        jointIndices=[1], # Joint index list 比如1就是wheel_left_joint的index
        controlMode=p.VELOCITY_CONTROL, #控制速度
        targetVelocities=[target_v], # 每个Joint的目标速度的list
        forces=[max_force] # 为达到每个Joint的目标速度的驱动力的list
    )
```

| Joint name | index | 功能描述 | 使用方法 |
|  ----  | ----  | ----  | ----  |
| wheel_left_joint | 1 | 左边轮子的关节 | 建议将targetVelocities和forces设置为10，左轮会转动起来|
| wheel_right_joint | 2 | 右边轮子的关节 | 建议将targetVelocities和forces设置为10，右轮会转动起来 |
| joint1 | 11 | 机械臂的基座，可以控制机械臂左右转动 | 将targetVelocities和forces设置为5左右，机械臂会向左转，右转则建议将targetVelocities设置为-5左右 |
| joint2 | 12 | 机械臂和基座连接第一个关节，可以控制机械臂上下移动| 将targetVelocities和forces设置为5左右，机械臂会向下移动， 将targetVelocities设置为-5左右，机械臂向上移动|
| joint3 | 13 | 机械臂第一个手臂和第二个手臂的连接关节，可以控制机械臂上下移动| 同上 |
| joint4 | 14 | 机械臂第二个手臂和抓手的连接关节，可以控制抓手上下移动| 同上 |
| gripper | 15 | 机械臂左抓手| 将targetVelocities和forces设置为5左右就会张开，将targetVelocities设置成-5则会闭合|
| gripper_sub | 16 | 机械臂左抓手| 同上|

## Example
这是一个控制机械臂抓手开合的例子，基于此可以设计机器人可以做的动作进行强化学习实验。
```python
import pybullet as p
import pybullet_data  # pybullet自带的一些模型
import time
server_id = p.connect(p.GUI)  # 连接到仿真环境，p.DIREACT是不显示仿真界面,p.GUI则为显示仿真界面
p.setGravity(0, 0, -10)  # 设定重力
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加pybullet的额外数据地址，使程序可以直接调用到内部的一些模型
planeId = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("./urdf/model.urdf")
p.resetBasePositionAndOrientation(robot_id, startPos, startOrientation)
joint_num = p.getNumJoints(robot_id)
print("turtlebot3节点数量为：", joint_num)
available_joints_indexes = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
print([p.getJointInfo(robot_id, i)[1] for i in available_joints_indexes])
print(available_joints_indexes)
wheel_joints_indexes = [i for i in available_joints_indexes if "wheel" in str(p.getJointInfo(robot_id, i)[1])]
print(wheel_joints_indexes)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setRealTimeSimulation(0)
target_v =  -5             # 电机达到的预定角速度（rad/s）
max_force = 5
for i in range(100000):
    if i % 100 == 0:
        target_v = -5 if i % 200 == 0 else 5
    p.stepSimulation()
    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=[15, 16], #控制机械臂钳子
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[target_v, target_v],
        forces=[max_force, max_force]
    )
    time.sleep(1 / 240)         # 模拟器一秒模拟迭代240步
p.disconnect(server_id)
```
## Keyboard_Control_Robot
我写了一个简单的控制机器人动作的程序，目前可以使用的是：
- 上下左右箭头来控制机器人移动，目前也支持左上右上，左下右下
- a和d控制机器人Joint1机械臂左转右转
- f和h控制机器人Joint2机械臂关节上下移动
- j和k控制机器人Joint3机械臂关节上下移动
- m和n控制机器人Joint4机械臂关节上下移动
- o和c控制机械臂抓手的开关，o是open就是打开抓手，c是close就是关闭抓手