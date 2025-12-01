# Alliance 自瞄库 ROS2 绑定

通过Github Aciton在每天两点自动拉取最新的自瞄版本

## 使用教程
```
git clone -b <ros-your-version> https://github.com/AlrayQiu/alliance_ros_auto_aim.git
```

```
colcon build
```

在需要使用的Package.xml中添加
```
  <depend>alliance_ros_auto_aim</depend>
```

在需要使用的CMakeLists.txt中添加
```
  ament_target_dependencies(<your project>
  alliance_ros_auto_aim
  )
```
