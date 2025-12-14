# BÀI TẬP THỰC HÀNH: ĐỘNG HỌC CHÂN ROBOT DOG

**Mục tiêu:** Tính toán Forward Kinematics (FK) và Inverse Kinematics (IK) cho một chân robot dog 3-DOF

**Công cụ cần thiết:**
- Giấy A4 trắng
- Bút chì, thước kẻ
- Máy tính (không bắt buộc)
- Kiến thức: Lượng giác cơ bản, đại số

---

## PHẦN 1: THIẾT LẬP BÀI TOÁN

### 1.1. Cấu Trúc Chân Robot

Vẽ sơ đồ chân robot với các thông số:

```
Body (Root Joint)
  │
  ├─── W (Basal segment) ───┐
  │                         [Hip Joint - θ1]
  │                              │
  │                         L1 (Thigh)
  │                              │
  │                         [Knee Joint - θ2]
  │                              │
  │                         L2 (Calf)
  │                              │
  │                         [Ankle Joint - θ3]
  │                              │
  │                         L3 (Foot - optional)
  │                              │
  └─────────────────────────► End Effector
```

**Thông số vật lý (mm):**
- W (Basal) = 60 mm
- L1 (Thigh) = 105 mm
- L2 (Calf) = 132 mm
- L3 (Foot) = 0 mm (tạm thời bỏ qua)

**Hệ tọa độ:**
- Gốc tọa độ: Tâm thân robot
- Trục X: Hướng về phía trước
- Trục Y: Hướng sang trái
- Trục Z: Hướng lên trên (vuông góc với mặt đất)

**Vị trí Root Joint (chân LF):**
- Root = (H/2, W_body/2, 0) = (90, 40, 0) mm
- H = 180 mm (khoảng cách chân trước-sau)
- W_body = 80 mm (khoảng cách chân trái-phải)

---

### 1.2. Định Nghĩa Góc Khớp

**θ1 (Hip/Root Joint):**
- Khớp tại điểm nối giữa thân và chân
- Xoay quanh trục X (trục dọc theo chiều dài thân)
- θ1 = 0°: chân duỗi thẳng ra ngoài
- θ1 > 0°: chân xoay xuống dưới
- θ1 < 0°: chân xoay lên trên
- Giới hạn: -45° ≤ θ1 ≤ 45°

**θ2 (Knee Joint):**
- Khớp giữa Thigh và Calf
- Xoay quanh trục Y (khi θ1 = 0)
- θ2 = 0°: L1 thẳng đứng (vuông góc với mặt đất)
- θ2 > 0°: L1 nghiêng về phía trước
- θ2 < 0°: L1 nghiêng về phía sau
- Giới hạn: -90° ≤ θ2 ≤ 90°

**θ3 (Ankle Joint):**
- Khớp giữa Calf và Foot
- Xoay quanh trục Y (khi θ1 = 0)
- θ3 = 0°: L2 cùng phương với L1
- θ3 > 0°: L2 gập về phía trước
- θ3 < 0°: L2 gập về phía sau
- Giới hạn: -135° ≤ θ3 ≤ 0°

**Lưu ý quan trọng:**
- Khi θ1 ≠ 0, toàn bộ chân (W, L1, L2) xoay quanh trục X
- θ2 và θ3 luôn xoay quanh trục Y (trong hệ tọa độ cục bộ của chân)

---

## PHẦN 2: FORWARD KINEMATICS (FK)

**Mục tiêu:** Cho trước các góc khớp (θ1, θ2, θ3), tính vị trí End Effector (X, Y, Z)

### 2.1. Phương Pháp Giải

**Bước 1: Tính trong hệ tọa độ cục bộ (giả sử θ1 = 0)**

Khi θ1 = 0, chân nằm trong mặt phẳng xOz:

**Vị trí Hip (sau Basal W):**
```
X_hip_local = 0
Y_hip_local = W
Z_hip_local = 0
```

**Vị trí Knee (sau Thigh L1):**
```
X_knee_local = X_hip_local - L1 × sin(θ2)
Y_knee_local = Y_hip_local
Z_knee_local = Z_hip_local - L1 × cos(θ2)
```

**Vị trí End Effector (sau Calf L2):**
```
X_end_local = X_knee_local - L2 × sin(θ2 + θ3)
Y_end_local = Y_knee_local
Z_end_local = Z_knee_local - L2 × cos(θ2 + θ3)
```

**Rút gọn:**
```
X_end_local = -L1 × sin(θ2) - L2 × sin(θ2 + θ3)
Y_end_local = W
Z_end_local = -L1 × cos(θ2) - L2 × cos(θ2 + θ3)
```

**Bước 2: Xoay quanh trục X bởi góc θ1**

Ma trận xoay quanh trục X:
```
Rx(θ1) = | 1      0           0        |
         | 0   cos(θ1)   -sin(θ1)    |
         | 0   sin(θ1)    cos(θ1)    |
```

Áp dụng:
```
X_end = X_end_local
Y_end = Y_end_local × cos(θ1) - Z_end_local × sin(θ1)
Z_end = Y_end_local × sin(θ1) + Z_end_local × cos(θ1)
```

**Bước 3: Cộng offset từ Root Joint (nếu cần tọa độ tuyệt đối)**

Nếu tính từ gốc tọa độ robot:
```
X_final = X_end + 90
Y_final = Y_end + 40
Z_final = Z_end + 0
```

---

### 2.2. Công Thức Cuối Cùng (FK)

**Tính từ Root Joint (0,0,0) cục bộ:**

```
X = -L1 × sin(θ2) - L2 × sin(θ2 + θ3)

Y = W × cos(θ1)

Z = -W × sin(θ1) - [L1 × cos(θ2) + L2 × cos(θ2 + θ3)] × cos(θ1)
```

**Chú ý:**
- Góc tính bằng radian: θ_rad = θ_deg × π/180
- Dấu (-) trong Z vì trục Z hướng lên, chân hướng xuống

---

### 2.3. Bài Tập Thực Hành FK

**Bài 1:** Tính toán với θ1 = 0°

| θ2   | θ3    | X = ? | Y = ? | Z = ?   |
|------|-------|-------|-------|---------|
| 0°   | 0°    |       |       |         |
| 45°  | -90°  |       |       |         |
| 60°  | -120° |       |       |         |
| 30°  | -60°  |       |       |         |

**Bài 2:** Tính toán với θ2 = 45°, θ3 = -90°

| θ1   | X = ? | Y = ? | Z = ?   |
|------|-------|-------|---------|
| 0°   |       |       |         |
| 15°  |       |       |         |
| 30°  |       |       |         |
| -15° |       |       |         |

**Bài 3:** Tự chọn góc

Chọn bộ góc (θ1, θ2, θ3) bất kỳ trong giới hạn cho phép:

θ1 = _____°
θ2 = _____°
θ3 = _____°

Tính toán:
```
X =




Y =




Z =




```

**Kiểm tra kết quả:**
- Mở file `tools/fk_tester.html`
- Nhập các góc vào sliders
- So sánh kết quả tính tay với kết quả trên màn hình

---

## PHẦN 3: INVERSE KINEMATICS (IK)

**Mục tiêu:** Cho trước vị trí End Effector (X, Y, Z), tính các góc khớp (θ1, θ2, θ3)

### 3.1. Phân Tích Bài Toán

**Đầu vào:**
- Vị trí mong muốn: P = (X, Y, Z)
- Chiều dài links: W, L1, L2

**Đầu ra:**
- Góc khớp: θ1, θ2, θ3

**Đặc điểm:**
- Có thể có nhiều nghiệm (multiple solutions)
- Có thể không có nghiệm (unreachable position)
- Cần chọn nghiệm phù hợp với giới hạn vật lý

---

### 3.2. Phương Pháp Giải IK

**Bước 1: Tính θ1 (Hip angle)**

Từ công thức FK:
```
Y = W × cos(θ1)
```

Suy ra:
```
θ1 = arccos(Y / W)
```

**Lưu ý:**
- Điều kiện tồn tại: |Y| ≤ W
- Nếu |Y| > W → Không đạt được (unreachable)

**Bước 2: Tính khoảng cách trong mặt phẳng xOz**

Từ công thức Z:
```
Z = -W × sin(θ1) - [L1 × cos(θ2) + L2 × cos(θ2 + θ3)] × cos(θ1)
```

Đặt:
```
Z' = Z + W × sin(θ1)
Z' = -[L1 × cos(θ2) + L2 × cos(θ2 + θ3)] × cos(θ1)
```

Rút ra:
```
Z'' = Z' / cos(θ1) = -[L1 × cos(θ2) + L2 × cos(θ2 + θ3)]
```

**Bước 3: Áp dụng định lý cosin (2-link planar IK)**

Bây giờ bài toán quy về 2D trong mặt phẳng xOz:
- Khoảng cách đích: r = √(X² + Z''²)

Sử dụng định lý cosin:
```
r² = L1² + L2² - 2 × L1 × L2 × cos(180° - θ3)
```

Suy ra:
```
cos(θ3) = -(r² - L1² - L2²) / (2 × L1 × L2)
```

```
θ3 = -arccos[(r² - L1² - L2²) / (2 × L1 × L2)]
```

**Lưu ý:**
- Điều kiện tồn tại: |cos(θ3)| ≤ 1
- Nếu r > L1 + L2 → Quá xa (unreachable)
- Nếu r < |L1 - L2| → Quá gần (unreachable)
- Dấu (-) vì θ3 thường âm (gập về phía sau)

**Bước 4: Tính θ2**

Sử dụng công thức:
```
α = arctan2(Z'', X)
β = arctan2(L2 × sin(θ3), L1 + L2 × cos(θ3))
```

```
θ2 = α + β
```

**Hoặc sử dụng công thức khác:**
```
tan(θ2) = [X × (L1 + L2 × cos(θ3)) + Z'' × L2 × sin(θ3)] /
          [Z'' × (L1 + L2 × cos(θ3)) - X × L2 × sin(θ3)]
```

---

### 3.3. Thuật Toán IK Hoàn Chỉnh

```
INPUT: Target position (X, Y, Z)

STEP 1: Tính θ1
  θ1 = arccos(Y / W)

  CHECK: |Y| ≤ W
  IF NOT → UNREACHABLE

STEP 2: Tính khoảng cách hiệu dụng
  Z' = Z + W × sin(θ1)
  Z'' = Z' / cos(θ1)
  r = √(X² + Z''²)

  CHECK: |L1 - L2| ≤ r ≤ L1 + L2
  IF NOT → UNREACHABLE

STEP 3: Tính θ3
  cos_θ3 = (r² - L1² - L2²) / (2 × L1 × L2)

  CHECK: |cos_θ3| ≤ 1
  IF NOT → UNREACHABLE

  θ3 = -arccos(cos_θ3)

STEP 4: Tính θ2
  α = arctan2(Z'', X)
  β = arctan2(L2 × sin(θ3), L1 + L2 × cos(θ3))
  θ2 = α + β

STEP 5: Kiểm tra giới hạn
  CHECK: -45° ≤ θ1 ≤ 45°
  CHECK: -90° ≤ θ2 ≤ 90°
  CHECK: -135° ≤ θ3 ≤ 0°

OUTPUT: (θ1, θ2, θ3)
```

---

### 3.4. Bài Tập Thực Hành IK

**Bài 4:** Tính IK cho các vị trí đơn giản

**Case 1:** Chân duỗi thẳng xuống
```
Target: X = 0, Y = 60, Z = -237
(Khi θ1=0, θ2=0, θ3=0 thì chân duỗi thẳng)

Bước 1: Tính θ1
θ1 = arccos(60/60) = arccos(1) = ___°

Bước 2: Tính r
Z' = -237 + 60×sin(0) = _____
Z'' = Z'/cos(0) = _____
r = √(0² + Z''²) = _____

Bước 3: Tính θ3
cos(θ3) = (r² - 105² - 132²) / (2×105×132) = _____
θ3 = ___°

Bước 4: Tính θ2
α = arctan2(Z'', 0) = ___°
β = arctan2(132×sin(θ3), 105 + 132×cos(θ3)) = ___°
θ2 = ___°
```

**Case 2:** Tư thế đứng chuẩn
```
Target: X = -104.45, Y = 60, Z = -167.45
(Khoảng cách hợp lý cho tư thế đứng)

θ1 = ___°
θ2 = ___°
θ3 = ___°
```

**Bài 5:** Workspace Analysis

Vẽ vùng làm việc (workspace) của chân robot:

1. Với θ1 = 0°, vẽ vùng đạt được trong mặt phẳng xOz:
   - Bán kính tối đa: r_max = L1 + L2 = _____
   - Bán kính tối thiểu: r_min = |L1 - L2| = _____

2. Với Y = W, chân có thể đạt những điểm nào?
   - Vẽ hình tròn tâm O, bán kính r_max
   - Vẽ hình tròn tâm O, bán kính r_min
   - Vùng giữa 2 hình tròn là workspace

3. Khi thay đổi θ1 (-45° đến 45°), workspace thay đổi như thế nào?

---

## PHẦN 4: KIỂM TRA VÀ ĐÁNH GIÁ

### 4.1. Kiểm Tra FK → IK → FK

**Quy trình kiểm tra:**

1. Chọn bộ góc ban đầu: (θ1, θ2, θ3)
2. Tính FK → Được vị trí (X, Y, Z)
3. Dùng (X, Y, Z) tính IK → Được (θ1', θ2', θ3')
4. So sánh: (θ1, θ2, θ3) với (θ1', θ2', θ3')
5. Sai số cho phép: < 0.1°

**Bài 6:** Kiểm tra chu trình FK-IK

| Góc ban đầu     | FK Result    | IK Result    | Sai số  |
|-----------------|--------------|--------------|---------|
| θ1=0, θ2=45, θ3=-90 | X=___, Y=___, Z=___ | θ1'=___, θ2'=___, θ3'=___ | Δθ=___ |
| θ1=15, θ2=60, θ3=-120 | X=___, Y=___, Z=___ | θ1'=___, θ2'=___, θ3'=___ | Δθ=___ |
| θ1=-30, θ2=30, θ3=-60 | X=___, Y=___, Z=___ | θ1'=___, θ2'=___, θ3'=___ | Δθ=___ |

---

### 4.2. Trường Hợp Đặc Biệt

**Case 1: Singularity (Kỳ dị)**
- Xảy ra khi chân duỗi thẳng hoàn toàn (θ3 = 0°)
- Hoặc khi chân gập hoàn toàn (r = r_min)
- Tại singularity, có vô số nghiệm cho θ2

**Case 2: Unreachable Position**
- Điểm nằm ngoài workspace
- |Y| > W
- r > L1 + L2
- r < |L1 - L2|

**Bài 7:** Xác định các trường hợp đặc biệt

Với các vị trí sau, xác định có đạt được không? Tại sao?

1. (X=0, Y=100, Z=-200): _______________
2. (X=-300, Y=60, Z=-100): _______________
3. (X=-100, Y=60, Z=0): _______________
4. (X=-118, Y=52, Z=-200): _______________

---

## PHẦN 5: BÀI TẬP NÂNG CAO

### 5.1.궤đạo chuyển động (Trajectory Planning)

**Mục tiêu:** Lập kế hoạch cho chân di chuyển từ điểm A đến điểm B

**Bài 8:** Trajectory thẳng (Linear Interpolation)

```
Điểm bắt đầu: A = (X_a, Y_a, Z_a) = (-74, 60, -167)
Điểm kết thúc: B = (X_b, Y_b, Z_b) = (-135, 60, -167)
Số bước: N = 10

Tại mỗi bước i (i = 0, 1, 2, ..., 10):
  t = i / N
  X_i = X_a + t × (X_b - X_a)
  Y_i = Y_a + t × (Y_b - Y_a)
  Z_i = Z_a + t × (Z_b - Z_a)

  Tính IK cho (X_i, Y_i, Z_i) → (θ1_i, θ2_i, θ3_i)
```

Điền vào bảng:

| Bước | t   | X    | Y  | Z     | θ1  | θ2  | θ3   |
|------|-----|------|----|-------|-----|-----|------|
| 0    | 0.0 | -74  | 60 | -167  |     |     |      |
| 1    | 0.1 |      | 60 |       |     |     |      |
| 2    | 0.2 |      | 60 |       |     |     |      |
| 3    | 0.3 |      | 60 |       |     |     |      |
| ...  |     |      |    |       |     |     |      |
| 10   | 1.0 | -135 | 60 | -167  |     |     |      |

---

### 5.2. Tối Ưu Hóa Vận Tốc

**Bài 9:** Tính vận tốc góc

Cho quỹ đạo từ Bài 8, tính vận tốc góc tại mỗi khớp:

```
Δt = 0.05 giây (thời gian mỗi bước)

ω1 = Δθ1 / Δt (rad/s)
ω2 = Δθ2 / Δt
ω3 = Δθ3 / Δt
```

Vẽ đồ thị vận tốc theo thời gian.

**Giới hạn vận tốc servo:**
- ω_max = 300°/s = 5.24 rad/s

Kiểm tra xem có bước nào vượt quá giới hạn không?

---

### 5.3. Phân Tích Jacobian

**Khái niệm:** Jacobian matrix mô tả mối quan hệ giữa vận tốc góc và vận tốc đầu cuối.

```
[v_x]       [∂X/∂θ1  ∂X/∂θ2  ∂X/∂θ3]   [ω1]
[v_y]   =   [∂Y/∂θ1  ∂Y/∂θ2  ∂Y/∂θ3] × [ω2]
[v_z]       [∂Z/∂θ1  ∂Z/∂θ2  ∂Z/∂θ3]   [ω3]
```

**Bài 10:** Tính Jacobian tại tư thế home (θ1=0, θ2=45, θ3=-90)

Tính đạo hàm riêng:
```
∂X/∂θ1 = ___
∂X/∂θ2 = ___
∂X/∂θ3 = ___

∂Y/∂θ1 = ___
∂Y/∂θ2 = ___
∂Y/∂θ3 = ___

∂Z/∂θ1 = ___
∂Z/∂θ2 = ___
∂Z/∂θ3 = ___
```

Ma trận Jacobian:
```
J = [___ ___ ___]
    [___ ___ ___]
    [___ ___ ___]
```

---

## PHẦN 6: TÀI LIỆU THAM KHẢO

### 6.1. Công Thức Lượng Giác Cần Nhớ

```
sin²(θ) + cos²(θ) = 1

sin(-θ) = -sin(θ)
cos(-θ) = cos(θ)

sin(α + β) = sin(α)cos(β) + cos(α)sin(β)
cos(α + β) = cos(α)cos(β) - sin(α)sin(β)

arctan2(y, x): Tính góc từ tọa độ (x, y), xét cả 4 góc phần tư
```

### 6.2. Ma Trận Xoay 3D

**Xoay quanh trục X:**
```
Rx(θ) = [1      0           0      ]
        [0   cos(θ)    -sin(θ)    ]
        [0   sin(θ)     cos(θ)    ]
```

**Xoay quanh trục Y:**
```
Ry(θ) = [cos(θ)    0    sin(θ) ]
        [   0      1       0   ]
        [-sin(θ)   0    cos(θ) ]
```

**Xoay quanh trục Z:**
```
Rz(θ) = [cos(θ)  -sin(θ)   0]
        [sin(θ)   cos(θ)   0]
        [   0        0     1]
```

### 6.3. Định Lý Cosin

Cho tam giác với các cạnh a, b, c và góc C đối diện cạnh c:

```
c² = a² + b² - 2ab×cos(C)

cos(C) = (a² + b² - c²) / (2ab)
```

---

## PHẦN 7: ĐÁP ÁN VÀ HƯỚNG DẪN GIẢI

### 7.1. Đáp Án Bài 1

| θ2   | θ3    | X       | Y    | Z        |
|------|-------|---------|------|----------|
| 0°   | 0°    | 0       | 60   | -237     |
| 45°  | -90°  | -167.3  | 60   | -116.7   |
| 60°  | -120° | -204.8  | 60   | -66.2    |
| 30°  | -60°  | -167.0  | 60   | -144.6   |

**Hướng dẫn giải chi tiết:**

**Case θ2=0°, θ3=0°:**
```
X = -L1×sin(0) - L2×sin(0) = 0
Y = W×cos(0) = 60×1 = 60
Z = -W×sin(0) - [L1×cos(0) + L2×cos(0)]×cos(0)
  = 0 - [105×1 + 132×1]×1
  = -237
```

**Case θ2=45°, θ3=-90°:**
```
X = -105×sin(45°) - 132×sin(45°-90°)
  = -105×0.707 - 132×sin(-45°)
  = -74.24 - 132×(-0.707)
  = -74.24 + 93.32
  = -167.3

Y = 60×cos(0) = 60

Z = -60×sin(0) - [105×cos(45°) + 132×cos(-45°)]×cos(0)
  = 0 - [105×0.707 + 132×0.707]×1
  = -[74.24 + 93.32]
  = -167.56

Chú ý: Giá trị chính xác hơn phụ thuộc vào độ chính xác của sin/cos
```

*(Tiếp tục các đáp án khác...)*

---

### 7.2. Lời Khuyên Khi Tính Toán

1. **Luôn chuyển độ sang radian:** θ_rad = θ_deg × π/180
2. **Kiểm tra đơn vị:** Đảm bảo tất cả chiều dài cùng đơn vị (mm)
3. **Vẽ sơ đồ:** Trước khi tính, vẽ hình để hiểu rõ bài toán
4. **Kiểm tra giới hạn:** Sau khi tính IK, kiểm tra góc có trong giới hạn không
5. **Xác minh bằng FK:** Sau khi tính IK, dùng FK để kiểm tra lại

---

## PHẦN 8: KẾ HOẠCH HỌC TẬP

### Tuần 1: Forward Kinematics
- [ ] Đọc PHẦN 1 và PHẦN 2
- [ ] Làm Bài 1, 2, 3
- [ ] Kiểm tra kết quả với FK Tester
- [ ] Vẽ sơ đồ chân robot trong các tư thế khác nhau

### Tuần 2: Inverse Kinematics
- [ ] Đọc PHẦN 3
- [ ] Làm Bài 4, 5
- [ ] Phân tích workspace
- [ ] Code thuật toán IK (Python hoặc C++)

### Tuần 3: Kiểm Tra và Nâng Cao
- [ ] Làm Bài 6, 7 (Kiểm tra FK-IK)
- [ ] Làm Bài 8 (Trajectory planning)
- [ ] Thử nghiệm trên robot thật (nếu có)

### Tuần 4: Chuyên Sâu
- [ ] Làm Bài 9, 10 (Vận tốc, Jacobian)
- [ ] Nghiên cứu các thuật toán IK khác
- [ ] Tối ưu hóa code

---

## GHI CHÚ CÁ NHÂN

*(Dành chỗ để ghi chú trong quá trình học)*

**Những điểm khó:**


**Câu hỏi cần làm rõ:**


**Ý tưởng cải tiến:**


---

**Chúc bạn học tốt! 🚀**

*Tài liệu này được tạo để hỗ trợ học tập kinematics cho robot dog.*
*Có thể in ra và điền vào các bài tập thực hành.*
