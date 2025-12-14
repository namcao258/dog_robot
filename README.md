# Robot Dog - Quadruped Robot Project

Dự án robot chó bốn chân (quadruped robot) với điều khiển bằng động học (kinematics) và mô phỏng 3D.

## Tổng quan

Đây là dự án robot chó bốn chân sử dụng:
- **Kinematics**: Forward Kinematics (FK) và Inverse Kinematics (IK) cho chân robot 3 bậc tự do
- **Simulation**: Mô phỏng robot với giao diện GUI real-time
- **Trajectory Control**: Điều khiển quỹ đạo cho cả 4 chân (FL, FR, RL, RR)
- **Gait Control**: Điều khiển dáng đi trot gait với phối hợp 4 chân
- **Arduino Firmware**: Code điều khiển phần cứng cho robot thật

## Tính năng chính

### 1. Kinematics
- **Forward Kinematics (FK)**: Tính toán vị trí chân từ góc khớp (θ1, θ2, θ3)
- **Inverse Kinematics (IK)**: Tính toán góc khớp từ vị trí mong muốn
- **Dual Solution Selection**: Chọn nghiệm IK tối ưu (giảm thiểu thay đổi góc khớp)
- **Mirroring**: Xử lý đúng tọa độ cho chân phải (FR, RR)

### 2. Simulator - Mô phỏng robot
- **Keyboard Control**: Điều khiển robot bằng phím mũi tên / WASD
- **Configurable Body Dimensions**:
  - H (Front-Rear distance): Khoảng cách trước-sau giữa các chân
  - W (Left-Right distance): Khoảng cách trái-phải giữa các chân
  - Mặc định: H=600mm, W=200mm
- **13 đồ thị real-time**:
  - 1 Top view: Nhìn từ trên xuống robot trong world frame
  - 4 XY projections: Quỹ đạo chân trên mặt phẳng XY (nhìn từ trên)
  - 4 XZ projections: Quỹ đạo chân trên mặt phẳng XZ (nhìn từ bên)
  - 4 Joint angle plots: Biểu đồ góc khớp θ1, θ2, θ3 theo thời gian

### 3. Gait Controller
- **Trot Gait**: Dáng đi kiểu nước kiệu (FL-RR cùng pha, FR-RL cùng pha)
- **Motion Detection**: Tự động dừng gait khi robot đứng yên
- **Smooth Trajectory**: Quỹ đạo chân mượt mà với swing và stance phase

### 4. Visualizer
- GUI kinematics visualizer với điều khiển real-time
- Hiển thị 3D robot leg trong nhiều góc nhìn
- Kiểm tra FK/IK accuracy

## Cấu trúc thư mục

```
dog_robot/
├── software/
│   ├── python/                    # Python kinematics libraries
│   │   ├── forward_kinematics.py  # FK implementation
│   │   ├── inverse_kinematics.py  # IK implementation
│   │   └── test_*.py              # Test scripts
│   └── simulation/                # Simulators
│       ├── quadruped_simulator.py # Main quadruped simulator
│       ├── kinematics_visualizer.py # FK/IK visualizer
│       ├── trajectory_controller*.py # Trajectory controllers cho 4 chân
│       └── requirements.txt       # Python dependencies
├── Nova-SM3_mega-v4.2/           # Arduino firmware
├── Arduino Libraries/             # Arduino libraries cần thiết
├── docs/                         # Documentation
└── tools/                        # HTML visualization tools
```

## Cài đặt

### Yêu cầu
- Python 3.7+
- Arduino IDE (cho firmware)

### Cài đặt Python dependencies
```bash
cd software/simulation
pip install -r requirements.txt
```

Dependencies:
- `numpy` - Tính toán số học
- `matplotlib` - Vẽ đồ thị
- `tkinter` - GUI (thường có sẵn với Python)

## Sử dụng

### 1. Chạy Quadruped Simulator
```bash
cd software/simulation
python quadruped_simulator.py
```

**Điều khiển:**
- `↑` / `W`: Đi tới
- `↓` / `S`: Đi lùi
- `←` / `A`: Quay trái
- `→` / `D`: Quay phải
- `Space`: Dừng
- `Q`: Thoát

**Cấu hình:**
- Nhập giá trị H và W mới vào ô input
- Bấm "Apply Dimensions" để áp dụng

### 2. Chạy Kinematics Visualizer
```bash
cd software/simulation
python kinematics_visualizer.py
```

### 3. Chạy Trajectory Controller cho từng chân
```bash
cd software/simulation
python trajectory_controller.py      # Front Left
python trajectory_controller_FR.py   # Front Right
python trajectory_controller_RL.py   # Rear Left
python trajectory_controller_RR.py   # Rear Right
```

## Thông số kỹ thuật

### Kích thước chân (DH Parameters)
- L1 = 60 mm (Hip link)
- L2 = 105 mm (Thigh link)
- L3 = 132 mm (Calf link)

### Body Dimensions (mặc định)
- H = 600 mm (Khoảng cách trước-sau)
- W = 200 mm (Khoảng cách trái-phải)

### Hip Positions (body frame, origin ở giữa thân)
- FL (Front Left): (+H/2, +W/2) = (+300, +100)
- FR (Front Right): (+H/2, -W/2) = (+300, -100)
- RL (Rear Left): (-H/2, +W/2) = (-300, +100)
- RR (Rear Right): (-H/2, -W/2) = (-300, -100)

### Initial Configuration
- θ1 = 0°
- θ2 = 30°
- θ3 = -30°
- Foot position: (-52.5, ±60, -222.93) mm

### Gait Parameters
- Step duration: 0.5s
- Stance height: -222.93 mm
- Swing height: 30 mm
- Phase offsets: FL(0), RR(0), FR(0.5), RL(0.5)

## Tài liệu

Chi tiết về kinematics và cách tính toán xem tại:
- [software/python/FK_IK_Summary.md](software/python/FK_IK_Summary.md)
- [software/simulation/USAGE_GUIDE.md](software/simulation/USAGE_GUIDE.md)
- [docs/kinematics_worksheet.md](docs/kinematics_worksheet.md)

## Arduino Firmware

Firmware cho robot thật nằm trong thư mục `Nova-SM3_mega-v4.2/`. Yêu cầu các library:
- Adafruit PWM Servo Driver
- Adafruit GFX
- Adafruit SSD1331
- Adafruit NeoPixel
- NewPing
- PS2X_lib

## License

Dự án mã nguồn mở cho mục đích học tập và nghiên cứu.

## Tác giả

Robot Dog Project - Quadruped Robot Development

---

**Note**: Dự án đang trong quá trình phát triển. Các tính năng mới sẽ được bổ sung liên tục.
