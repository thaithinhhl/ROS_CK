
# Robot Gmapping

PDF: 
[ROS_GK-1.pdf](https://github.com/user-attachments/files/19553201/ROS_GK-1.pdf)





https://github.com/user-attachments/assets/f2fef3fe-7e6d-499a-9c30-d6d76342fc66






## 🚀 Cài đặt

### 🔧 Bước 1: Tạo ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace   
```

### 📥 Bước 2: Clone Repository

```bash
git clone https://github.com/thaithinhhl/ROS_CK.git
mv ROS xerobotvisai2 
cd ~/catkin_ws
catkin_make
```

### 🧠 Bước 3: Source workspace

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 🛰️ Chạy Gmapping 

### 🎯 Bước 4: Khởi chạy mô phỏng trong Gazebo 

``` bash
roslaunch xerobotvisai2 gazebo.launch
```

### ⚙️ Bước 5: Chạy rviz
``` bash
roslaunch xerobotvisai2 gmapping.launch 
```

### 🌐 Bước 6: Điểu khiển xe để quét map
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```


### 🦾 Bước 7: Điều khiển tay máy (Arm Controller)
Điều khiển bẳng 4 phím mũi tên trên bàn phím
```bash
rosrun Assem2 arm_teleop_keyboard.py
```

### 🎮 Bước 8: Điều khiển robot di chuyển

2 cách để điều khiển robot di chuyển:

---

#### 🧭 Cách 1: Gửi lệnh trực tiếp qua topic `/cmd_vel`

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 3.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.5" -r 10
```
#### 🕹️ Cách 2: Chạy script điều khiển bằng bàn phím

```bash
rosrun Assem2 teleop_keyboard.py
```

### 🧾 Bước 9: Đọc giá trị encoder từ bánh xe

Có thể kiểm tra vị trí và vận tốc của các joint (bánh xe & tay máy) bằng cách đọc topic:

```bash
rostopic echo /joint_states
```
📌 Topic này cung cấp thông tin về:

  - name: tên các joint (VD: joint_L, joint_R)

  - position: vị trí hiện tại của joint (theo radian)

  - velocity: tốc độ góc hiện tại của joint (rad/s)

