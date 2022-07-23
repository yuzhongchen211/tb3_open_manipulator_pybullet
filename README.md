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
| joint4 | 14 | 机械臂第二个手臂和钳子的连接关节，可以控制钳子上下移动| 同上 |
| gripper | 15 | 机械臂左钳子| 将targetVelocities和forces设置为5左右就会张开，将targetVelocities设置成-5则会闭合|
| gripper_sub | 16 | 机械臂左钳子| 同上|