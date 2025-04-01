# MIDTERM- ROS PROJECT: BUILD AND CONTROL DIFFERENTIAL DRIVE ROBOT 

## Hướng dẫn mô phỏng và điều khiển Robot
### 1. Sau khi git clone về máy tính, build lại *workspace* và *source* không gian làm việc

```bash
cd ~/catkin_ws/xerobotvisai2
```
```bash
catkin_make
```
```bash
source devel/setup.bash
```
### 2. Chạy file launch để khởi động GAZEBO và RVIZ

```bash
roslaunch xerobotvisai2 gazebo.launch
```
Nếu có vấn đề hay log bị lỗi gì, anh tắt đi rồi launch lại file nhé a

### GAZEBO
![gazebo](https://github.com/user-attachments/assets/14a39cef-c9ea-4daa-aa5d-4e541e89e717)

### RVIZ
![Screenshot from 2025-03-31 12-11-25](https://github.com/user-attachments/assets/d222ed8f-c159-4ce3-8ad4-782151fcf6b9)

### 3. Thêm vật cản vào trong gazebo
![Screenshot from 2025-03-31 12-37-38](https://github.com/user-attachments/assets/206224f7-5520-42c6-a7be-3e34e2956ca5)


### 4. Điểu khiển Robot Model và Tay Máy bằng bàn phím

Cài đặt gói Teleop Twist Keyboard 
```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

Cấp quyền thực thi cho file arm.py
```bash
chmod +x arm.py
```

Điều khiển xe di chuyển bằng bàn phím
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
![Screenshot from 2025-03-31 12-38-31](https://github.com/user-attachments/assets/942cfe5e-c11d-44d8-ac04-3358da5bbe52)

Điều khiển tay máy di chuyển bằng bàn phím
```bash
rosrun xerobotvisai2 arm.py
```
![Screenshot from 2025-03-31 12-40-21](https://github.com/user-attachments/assets/bf66ac40-b21e-447f-934a-e458505f4f8a)

### KẾT QUẢ 

![Screenshot from 2025-03-31 17-51-00](https://github.com/user-attachments/assets/706cb988-a809-4d96-ba8f-4f6a9edae095)

