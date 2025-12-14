# Forward & Inverse Kinematics - Chân Trái Trước
## Robot: Nova-SM3 Quadruped Dog

---

## FORWARD KINEMATICS (FK)

```
x = -L₂ sin(θ₂) - L₃ sin(θ₂ + θ₃)

y = L₁ cos(θ₁) + L₂ sin(θ₁) cos(θ₂) + L₃ sin(θ₁) cos(θ₂ + θ₃)

z = L₁ sin(θ₁) - L₂ cos(θ₁) cos(θ₂) - L₃ cos(θ₁) cos(θ₂ + θ₃)
```

---

## INVERSE KINEMATICS (IK)

```
z_temp = -√(y² + z² - L₁²)

θ₁ = arctan2(-(y·z_temp + z·L₁),  y·L₁ - z·z_temp)

r_2D = √(x² + z_temp²)

θ₃ = ±arccos((r_2D² - L₂² - L₃²) / (2·L₂·L₃))  (chọn dấu ± để khớp target x)

φ = arctan2(-x, -z_temp)  (góc từ trục -Z đến target)
β = arccos((L₂² + r_2D² - L₃²) / (2·L₂·r_2D))  (góc trong tam giác)
θ₂ = φ - β
```

**Lưu ý:**
- θ₃ có 2 nghiệm (elbow up/down). Chọn nghiệm nào cho x_check khớp với x target.
- φ được đo từ trục -Z (hướng xuống), không phải từ trục X ngang.
- Công thức này đã được kiểm tra và verified với roundtrip tests.

---

## KÝ HIỆU

**Độ dài khâu:**
- **L₁** = 60 mm - Khâu basal (hip offset), nối từ thân đến khớp hip
- **L₂** = 105 mm - Khâu thigh (đùi), từ hip đến knee
- **L₃** = 132 mm - Khâu calf (cẳng chân), từ knee đến ankle

**Góc khớp:**
- **θ₁** - Góc khớp hip, quay quanh trục X, dương = chân ra ngoài
- **θ₂** - Góc khớp knee (gối), quay quanh trục Y, dương = gập lên
- **θ₃** - Góc khớp ankle (cổ chân), quay quanh trục Y, dương = gập lên

**Tọa độ end-effector:**
- **x** - Tọa độ theo trục X (forward, về phía trước)
- **y** - Tọa độ theo trục Y (lateral, ra ngoài)
- **z** - Tọa độ theo trục Z (vertical, hướng lên trên, Z dương = lên, Z âm = xuống)

**Biến phụ (dùng trong IK):**
- **z_temp** - Thành phần Z trong mặt phẳng XOZ khi θ₁ = 0
- **r_2D** - Khoảng cách 2D trong mặt phẳng XOZ
- **φ** (phi) - Góc từ gốc tọa độ tới target trong mặt phẳng XOZ
- **β** (beta) - Góc trong tam giác tại khớp knee

**Hệ tọa độ:**
- Gốc **O** tại khớp hip
- Trục **X** hướng về phía trước
- Trục **Y** hướng ra ngoài (bên trái)
- Trục **Z** hướng lên trên (Z dương = lên, Z âm = xuống)

---

**Hình minh họa:** `leg_kinematics_diagram.png`

**Chi tiết:** `test_FK.ipynb`, `IK_from_FK.ipynb`
