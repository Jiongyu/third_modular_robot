## QT 界面设计文件

|文件名|解释|
|----|-----|
|*modular_robot_control.ui*| *机器人主控制界面*|
|*gripper_control.ui*|*夹持器控制界面*|
|*robot_choice.ui*| *机器人选择界面*|
|*zero_point_set.ui*| *零点设置界面*|
|*path_point_recorder.ui*| *离线示教界面*|

##### 使用方法：
>1. 使用qtcreator编辑界面文件，添加或删除控件。
>

>2. 将ui文件转为python文件。
>命令:
>``` sudo pyuic5 -o 输出python文件  ui源文件 ```
>示例：
>``` sudo pyuic5 -o modular_robot_control.py modular_robot_control.ui ```
>

>3. 生成文件保存在 ui/script文件夹内。
