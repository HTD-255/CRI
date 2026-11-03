# Kết quả sau khi chuyển luật lốp sang mô hình Fiala

Tài liệu này ghi lại kết quả kiểm thử và mô phỏng sau khi thay luật lực lốp
tuyến tính (`Fy = C*alpha`) bằng mô hình Fiala bão hòa trong
[vehicle_model.py](./vehicle_model.py). Dùng làm tài liệu tham chiếu khi viết
lại phần kết quả/methodology; không phải báo cáo cuối cùng.

Ngày tạo: 2026-11-04. Môi trường: Python 3.12.8, pytest 9.2.0, Windows.

## 1. Thay đổi mã nguồn liên quan

- `DynamicBicycleModel._fiala_lateral_force`: cài đặt công thức Fiala brush-tire
  (tuyến tính quanh alpha=0, đa thức bậc 3 trước bão hòa, bão hòa phẳng ở
  `mu*Fz` ngoài góc trượt bão hòa).
- `DynamicBicycleModel.tire_forces`: tính tải pháp tuyến tĩnh theo trục
  (`Fzf = m*g*lr/L`, `Fzr = m*g*lf/L`), gọi hàm Fiala cho từng cầu.
- `VehicleParams`: thêm `mu=0.9` (hệ số ma sát) và `g=9.81`; `Cf`, `Cr` giữ
  nguyên giá trị cũ nhưng được đánh dấu "giả định, cần rà soát" (xem
  [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md) mục 13.8 để biết
  chi tiết nguồn tham số).

## 2. Kết quả kiểm thử tự động (pytest)

Lệnh chạy:

```powershell
python -m pytest test_simulation.py -v
```

Kết quả (9/9 PASSED):

```
test_simulation.py::test_target_synchronized_with_ego PASSED             [ 11%]
test_simulation.py::test_applied_commands_replay_ego_state PASSED        [ 22%]
test_simulation.py::test_commands_within_limits PASSED                   [ 33%]
test_simulation.py::test_applied_matches_selected_controller PASSED      [ 44%]
test_simulation.py::test_finalize_rejects_non_finite PASSED              [ 55%]
test_simulation.py::test_finalize_floor_then_clip PASSED                 [ 66%]
test_simulation.py::test_fiala_zero_slip_produces_zero_force PASSED      [ 77%]
test_simulation.py::test_fiala_matches_cornering_stiffness_at_small_slip PASSED [ 88%]
test_simulation.py::test_fiala_force_is_odd_and_saturates_at_friction_limit PASSED [100%]

============================== 9 passed in 1.18s ==============================
```

3 test mới cho mô hình Fiala:

| Test | Mục tiêu kiểm tra |
|---|---|
| `test_fiala_zero_slip_produces_zero_force` | `alpha=0` phải cho lực ngang bằng 0 ở cả hai cầu. |
| `test_fiala_matches_cornering_stiffness_at_small_slip` | Với `alpha` rất nhỏ (`1e-6 rad`), `Fy/alpha -> Cf` (đúng định nghĩa độ cứng góc trượt tuyến tính tại gốc). |
| `test_fiala_force_is_odd_and_saturates_at_friction_limit` | Lực là hàm lẻ theo `alpha` (`Fy(-a) = -Fy(a)`) và bão hòa đúng bằng `mu*Fz` khi `alpha` vượt góc bão hòa. |

## 3. Đặc tính số của luật lốp Fiala (tham số hiện tại)

Với `m=1575 kg`, `L=2.5 m`, `lf=1.2 m`, `lr=1.3 m`, `mu=0.9`, `g=9.81 m/s^2`:

- Tải pháp tuyến tĩnh: `Fzf = 8034.4 N`, `Fzr = 7416.4 N`.
- Giới hạn ma sát (lực bão hòa tối đa): `mu*Fzf = 7231.0 N`, `mu*Fzr = 6674.7 N`.
- Góc trượt bắt đầu bão hòa (`tan(alpha) = 3*mu*Fz/C`):
  - Cầu trước (`Cf=19000 N/rad`): `alpha_sat_f ≈ 48.8°`.
  - Cầu sau (`Cr=33000 N/rad`): `alpha_sat_r ≈ 31.2°`.

Bảng lực ngang theo góc trượt (tính trực tiếp từ `_fiala_lateral_force`,
`vx=15 m/s`):

| alpha (deg) | Fyf (N) | Fyr (N) |
|---:|---:|---:|
| 0 | 0.0 | 0.0 |
| 1 | 326.6 | 559.6 |
| 2 | 643.4 | 1087.3 |
| 3 | 950.7 | 1584.4 |
| 4 | 1248.9 | 2051.9 |
| 5 | 1538.2 | 2490.9 |
| 7 | 2091.0 | 3287.3 |
| 10 | 2859.4 | 4291.7 |
| 15 | 3989.7 | 5512.4 |
| 20 | 4945.1 | 6247.0 |
| 30 | 6357.5 | 6674.0 (gần bão hòa) |
| 50 | 7231.0 (bão hòa) | 6674.7 (bão hòa) |

Quan sát: cầu sau bão hòa sớm hơn cầu trước (~31° so với ~49°) do `Cr/Fzr`
lớn hơn `Cf/Fzf`; đây là hệ quả trực tiếp của việc `Cf`, `Cr` hiện chưa có
nguồn gốc độc lập đã xác minh (xem mục 13.8 của
[RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md)), nên các góc bão
hòa này chỉ mang tính minh họa, không phải số liệu đã kiểm chứng cho một lốp
thực tế.

Lệnh tái tạo:

```powershell
python -c "
import numpy as np
from vehicle_model import DynamicBicycleModel
model = DynamicBicycleModel()
p = model.params
Fzf = p.m*p.g*p.lr/p.L
Fzr = p.m*p.g*p.lf/p.L
for deg in [0,1,2,3,4,5,7,10,15,20,30,50]:
    a = np.radians(deg)
    print(deg, model._fiala_lateral_force(a, p.Cf, Fzf, p.mu),
          model._fiala_lateral_force(a, p.Cr, Fzr, p.mu))
"
```

## 4. Kết quả mô phỏng closed-loop (NMPC + Pure Pursuit + CRI)

Chạy một episode mặc định (`SimConfig(max_steps=300)`, kịch bản
`lateral_zigzag`) với luật lốp Fiala mới:

| Chỉ tiêu | Giá trị |
|---|---|
| Số bước mô phỏng thực hiện | 156 / 300 (kết thúc do `target_end`) |
| Phân bố mode | NMPC: 35, PP_FALLBACK: 10, NO_TRACK: 111 |
| Khoảng cách nhỏ nhất ego-target | 2.591 m |
| Khoảng cách cuối cùng | 64.519 m |
| Thời điểm vào ngưỡng gần (< 4 m) | t = 6.0 s |
| CRI trung bình / min / max | 0.172 / 0.000 / 0.824 |
| Gia tốc ngang động học lớn nhất (ước lượng kinematic, không phải lực lốp) | 83.124 m/s² (giá trị cao bất thường, cần kiểm tra lại — xem ghi chú bên dưới) |

**Ghi chú quan trọng:** giá trị gia tốc ngang động học lớn nhất (83 m/s²) là
dấu hiệu bất thường (vượt xa khả năng bám đường thực tế), khớp với cảnh báo
đã có trong [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md) rằng
luật Fiala hiện tại chưa có ràng buộc friction circle/ellipse và `lat_accel_kin`
chỉ là ước lượng kinematic (`v^2*tan(delta)/L`), không phải gia tốc tính từ
lực lốp thực tế. Không nên trích số này như một kết quả vật lý đã kiểm chứng;
cần bổ sung ràng buộc lực kết hợp trước khi dùng cho tuyên bố định lượng.

Lệnh tái tạo:

```powershell
python -c "
import numpy as np
from simulation import SimConfig, run_simulation
log = run_simulation(SimConfig(max_steps=300))
print('end_reason:', log['end_reason'])
print('steps:', len(log['time']))
print('counts:', log['counts'])
print('min_distance_m:', float(np.min(log['distance'])))
print('final_distance_m:', float(log['distance'][-1]))
print('proximity_time_s:', log['proximity_time'])
print('cri_mean/min/max:', float(np.mean(log['cri'])), float(np.min(log['cri'])), float(np.max(log['cri'])))
print('max_abs_lat_accel_kin:', float(np.max(log['lat_accel_kin'])))
"
```

## 5. Việc chưa làm / cần rà soát tiếp

- `Cf`, `Cr` vẫn là giá trị giả định (xem mục 13.8 của
  [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md)); cần tìm nguồn
  riêng hoặc chạy phân tích độ nhạy trước khi trình bày là tham số đã xác thực.
- Chưa có ràng buộc friction circle/ellipse kết hợp lực dọc-ngang; giá trị
  `lat_accel_kin` lớn ở mục 4 cho thấy cần bổ sung ràng buộc này.
- Tải pháp tuyến hiện là tĩnh (không có chuyển tải động khi phanh/tăng tốc
  hoặc vào cua); cần đánh giá ảnh hưởng nếu kịch bản có gia tốc dọc lớn.
