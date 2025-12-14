# Hướng dẫn sử dụng Kinematics Visualizer

## 🎯 Tổng quan giao diện

```
┌─────────────────────────────────────────────────────────────────────┐
│  Robot Parameters                                                    │
│  L1: [60.0]  L2: [105.0]  L3: [132.0]                              │
│  Homing θ1: [0.0]  θ2: [0.0]  θ3: [0.0]  [Update Parameters]      │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────┬──────────────────────────────────────┐
│  FORWARD KINEMATICS          │  INVERSE KINEMATICS                  │
├──────────────────────────────┼──────────────────────────────────────┤
│  Input: Joint Angles (°)     │  Input: Position (mm)                │
│  θ1 (Hip):   [____]          │  X: [____]                           │
│  θ2 (Knee):  [____]          │  Y: [____]                           │
│  θ3 (Ankle): [____]          │  Z: [____]                           │
│  [Calculate FK]              │  [Calculate IK]                      │
│                              │                                      │
│  Output: Position (mm)       │  Output: Joint Angles (°)            │
│  X: [____]                   │  θ1 (Hip):   [____]                  │
│  Y: [____]                   │  θ2 (Knee):  [____]                  │
│  Z: [____]                   │  θ3 (Ankle): [____]                  │
│                              │                                      │
│  ┌────────────────────┐      │  ┌────────────────────┐              │
│  │                    │      │  │                    │              │
│  │   3D Leg          │      │  │   3D Leg          │              │
│  │   Visualization   │      │  │   Visualization   │              │
│  │                    │      │  │                    │              │
│  └────────────────────┘      │  └────────────────────┘              │
└──────────────────────────────┴──────────────────────────────────────┘
```

## 📋 Các bước thực hiện

### A. Test Forward Kinematics (Bên trái)

**Bước 1:** Nhập góc khớp (đơn vị: độ)
- θ1: Góc hip (xoay quanh trục X)
- θ2: Góc knee (xoay quanh trục Y)
- θ3: Góc ankle (xoay quanh trục Y)

**Bước 2:** Click nút "Calculate FK"

**Bước 3:** Xem kết quả
- Tọa độ end-effector (x, y, z) hiển thị trong Output
- Hình 3D hiển thị cấu hình chân robot

**Ví dụ:**
```
Input:  θ1 = 0°,  θ2 = 0°,  θ3 = 0°
Output: x = 0 mm, y = 60 mm, z = 237 mm
→ Chân thẳng xuống dưới
```

### B. Test Inverse Kinematics (Bên phải)

**Bước 1:** Nhập tọa độ mục tiêu (đơn vị: mm)
- X: Forward/backward (âm = về phía trước)
- Y: Lateral (dương = ra ngoài)
- Z: Vertical (dương = xuống dưới)

**Bước 2:** Click nút "Calculate IK"

**Bước 3:** Xem kết quả
- Góc khớp (θ1, θ2, θ3) hiển thị trong Output
- Hình 3D hiển thị cấu hình chân robot
- Nếu vị trí unreachable → hiển thị "Unreachable"

**Ví dụ:**
```
Input:  x = -100 mm, y = 80 mm, z = -120 mm
Output: θ1 ≈ 121.73°, θ2 ≈ -15.76°, θ3 ≈ 92.58°
→ Chân duỗi về phía trước và lên trên
```

### C. Thay đổi thông số robot

**Bước 1:** Nhập giá trị mới
- L1: Chiều dài khâu basal (mm)
- L2: Chiều dài khâu thigh (mm)
- L3: Chiều dài khâu calf (mm)
- Homing angles: Góc ban đầu (degrees)

**Bước 2:** Click "Update Parameters"

**Bước 3:** Tính toán lại FK/IK với thông số mới

## 🧪 Test Cases đề xuất

### Test 1: Straight Down
```
FK Input:  θ1=0°, θ2=0°, θ3=0°
Expected:  x=0, y=60, z=237 mm
Description: Chân thẳng xuống dưới
```

### Test 2: Leg Sideways
```
FK Input:  θ1=90°, θ2=0°, θ3=0°
Expected:  x=0, y=237, z=-60 mm
Description: Chân duỗi ra ngoài bên
```

### Test 3: Knee Bent
```
FK Input:  θ1=0°, θ2=-45°, θ3=90°
Expected:  x≈-19, y=60, z≈168 mm
Description: Gối gập, cẳng chân hướng lên
```

### Test 4: IK Roundtrip
```
Step 1 - FK:  θ1=30°, θ2=-45°, θ3=90° → (x, y, z)
Step 2 - IK:  (x, y, z) → θ1', θ2', θ3'
Verify: θ1'≈30°, θ2'≈-45°, θ3'≈90° (error < 0.1°)
```

### Test 5: Unreachable Position
```
IK Input:  x=-300, y=60, z=-100
Expected:  "Unreachable" (quá xa)
```

## 📊 Hiểu hệ tọa độ

### Hệ tọa độ robot:
```
        Y (Lateral, outward)
        ↑
        |
        |
        O────→ X (Forward)
       /
      /
     ↓ Z (Downward)
```

### Ý nghĩa các trục:
- **X-axis**: Forward/Backward
  - X âm = về phía trước
  - X dương = về phía sau

- **Y-axis**: Lateral (bên hông)
  - Y dương = ra ngoài (abduction)
  - Y âm = vào trong (adduction)

- **Z-axis**: Vertical
  - Z dương = xuống dưới
  - Z âm = lên trên

### Ý nghĩa các góc:
- **θ1 (Hip)**: Xoay quanh trục X
  - Dương = chân ra ngoài
  - Âm = chân vào trong

- **θ2 (Knee)**: Xoay quanh trục Y
  - Âm = gối gập (flexion)
  - Dương = gối duỗi (extension)

- **θ3 (Ankle)**: Xoay quanh trục Y
  - Dương = cổ chân gập lên
  - Âm = cổ chân duỗi xuống

## 💡 Tips

### Kiểm tra tính đúng đắn:
1. Chạy FK với góc (0, 0, 0) → Nên cho (0, 60, 237)
2. Chạy IK với (0, 60, 237) → Nên cho góc gần (0, 0, 0)
3. FK → IK → FK: Position error < 0.01 mm
4. IK → FK → IK: Angle error < 0.01°

### Debug:
- Nếu IK cho "Unreachable": Kiểm tra xem vị trí có nằm trong workspace không
  - Distance from origin: √(x²+y²+z²) phải trong [|L2-L3|, L1+L2+L3]
  - Y² + Z² ≥ L1² (không được quá gần origin)

### Workspace limits:
- Max reach: L1 + L2 + L3 = 60 + 105 + 132 = 297 mm
- Min reach: |L2 - L3| = |105 - 132| = 27 mm

## 🔧 Troubleshooting

### Vấn đề: Góc IK không khớp với góc FK ban đầu
**Nguyên nhân:** IK có 2 nghiệm (elbow up/down)
**Giải pháp:** Đây là bình thường, IK tự động chọn nghiệm tốt nhất

### Vấn đề: 3D plot không hiển thị
**Nguyên nhân:** Thiếu matplotlib backend
**Giải pháp:** `pip install matplotlib PyQt5`

### Vấn đề: Angles lớn hơn ±180°
**Nguyên nhân:** arctan2 trả về [-π, π]
**Giải pháp:** Đây là bình thường, có thể wrap về [0, 360°] nếu cần

---

**Chúc bạn test thành công!** 🎉
