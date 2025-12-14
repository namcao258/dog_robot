# Kinematics Visualizer - Nova-SM3 Robot

Giao diện kiểm tra và trực quan hóa Forward Kinematics (FK) và Inverse Kinematics (IK) cho chân trái trước của robot Nova-SM3.

## Tính năng

### 🎯 Giao diện chia 2 phần:

**Bên TRÁI - Forward Kinematics:**
- Nhập góc khớp (θ1, θ2, θ3) theo độ
- Hiển thị tọa độ end-effector (x, y, z) tính được
- Trực quan hóa 3D của cấu hình chân robot

**Bên PHẢI - Inverse Kinematics:**
- Nhập tọa độ end-effector (x, y, z) mong muốn
- Hiển thị góc khớp (θ1, θ2, θ3) tính được
- Trực quan hóa 3D của cấu hình chân robot

**PHÍA TRÊN - Thông số robot:**
- Chiều dài các khâu: L1, L2, L3 (mm)
- Góc homing ban đầu (degrees)
- Nút cập nhật thông số

## Cài đặt

### 1. Cài đặt dependencies:
```bash
pip install -r requirements.txt
```

Hoặc cài thủ công:
```bash
pip install numpy matplotlib PyQt5
```

### 2. Chạy chương trình:
```bash
python kinematics_visualizer.py
```

## Hướng dẫn sử dụng

### Test Forward Kinematics:
1. Nhập góc khớp vào ô bên trái (đơn vị: độ)
   - θ1: Hip angle (quay quanh trục X)
   - θ2: Knee angle (quay quanh trục Y)
   - θ3: Ankle angle (quay quanh trục Y)
2. Nhấn nút "Calculate FK"
3. Xem kết quả tọa độ end-effector và hình 3D

### Test Inverse Kinematics:
1. Nhập tọa độ mục tiêu vào ô bên phải (đơn vị: mm)
   - X: Forward/backward (âm = về phía trước)
   - Y: Lateral (dương = ra ngoài)
   - Z: Vertical (dương = xuống dưới)
2. Nhấn nút "Calculate IK"
3. Xem kết quả góc khớp và hình 3D

### Thay đổi thông số robot:
1. Nhập giá trị mới cho L1, L2, L3
2. Nhập góc homing (nếu cần)
3. Nhấn "Update Parameters"

## Ví dụ test cases

### Test case 1: Chân thẳng xuống
- **FK Input:** θ1=0°, θ2=0°, θ3=0°
- **Expected Output:** x=0, y=60, z=237 mm

### Test case 2: Chân ra ngoài
- **FK Input:** θ1=90°, θ2=0°, θ3=0°
- **Expected Output:** x=0, y=237, z=-60 mm

### Test case 3: Gối gập
- **FK Input:** θ1=0°, θ2=-45°, θ3=90°
- **Expected Output:** x≈-19, y=60, z≈168 mm

### Test case 4: IK straight down
- **IK Input:** x=0, y=60, z=237
- **Expected Output:** θ1≈0°, θ2≈0°, θ3≈0°

## Lưu ý

- Tất cả góc nhập vào với đơn vị **độ (degrees)**
- Tọa độ với đơn vị **mm (millimeters)**
- Hệ tọa độ:
  - X: Trục forward/backward (âm = về phía trước)
  - Y: Trục lateral (dương = ra ngoài)
  - Z: Trục vertical (dương = xuống dưới)
- IK sẽ tự động chọn elbow configuration phù hợp
- Nếu vị trí unreachable, IK sẽ hiển thị "Unreachable"

## Công thức sử dụng

Chi tiết công thức FK và IK xem trong file: `../python/FK_IK_Summary.md`

## Troubleshooting

### Lỗi "No module named PyQt5":
```bash
pip install PyQt5
```

### Lỗi "No module named matplotlib":
```bash
pip install matplotlib
```

### Cửa sổ không hiển thị 3D:
- Đảm bảo đã cài đặt matplotlib với backend Qt5

## Phát triển thêm

Có thể mở rộng thêm:
- [ ] Slider để điều chỉnh góc/tọa độ realtime
- [ ] Animation của chuyển động
- [ ] Export trajectory
- [ ] Multiple leg visualization
- [ ] Collision detection

---

**Author:** Nova-SM3 Robot Team
**Version:** 1.0
**Date:** 2025-12-12
