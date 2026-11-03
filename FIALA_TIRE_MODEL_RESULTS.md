# Kết quả sau khi chuyển luật lốp sang mô hình Fiala

Tài liệu này ghi lại kết quả kiểm thử và mô phỏng sau khi thay luật lực lốp
tuyến tính (`Fy = C*alpha`) bằng mô hình Fiala bão hòa trong
[vehicle_model.py](./vehicle_model.py). Dùng làm tài liệu tham chiếu khi viết
lại phần kết quả/methodology; không phải báo cáo cuối cùng.

Ngày tạo: 2026-11-04. Môi trường: Python 3.12.8, pytest 9.2.0, Windows.

## 1. Thay đổi mã nguồn liên quan

- `DynamicBicycleModel._fiala_lateral_force`: cài đặt công thức Fiala brush-tire
  (tuyến tính quanh alpha=0, đa thức bậc 3 trước bão hòa, bão hòa phẳng ở mức
  `max_lateral_force` truyền vào).
- `DynamicBicycleModel._max_lateral_friction`: **mới** — áp dụng friction
  ellipse, trả về lực ngang tối đa còn lại của một cầu sau khi đã trừ phần ma
  sát dùng cho lực dọc (`sqrt((mu*Fz)^2 - Fx^2)`).
- `DynamicBicycleModel.normal_loads`: **mới** — tính tải pháp tuyến động theo
  trục, có chuyển tải dọc khi phanh/tăng tốc (`Fzf = Fzf_static - m*ax*h/L`,
  `Fzr = Fzr_static + m*ax*h/L`), thay cho tải pháp tuyến tĩnh trước đây.
- `DynamicBicycleModel._longitudinal_force_split`: **mới** — phân bổ lực dọc
  tổng (`m*ax`) cho hai cầu theo tỉ lệ tải tĩnh; đây là giả định đơn giản hóa
  (không mô hình hệ dẫn động cụ thể), cần rà soát nếu cần độ chính xác cao hơn.
- `DynamicBicycleModel.tire_forces`: nhận thêm tham số `ax` (mặc định 0.0, nên
  tương thích ngược), dùng để tính tải pháp tuyến động và friction ellipse.
- `VehicleParams`: thêm `mu=0.9` (hệ số ma sát), `g=9.81` và `h=0.55` (chiều
  cao trọng tâm, giả định minh họa); `Cf`, `Cr` giữ nguyên giá trị cũ nhưng
  được đánh dấu "giả định, cần rà soát" (xem
  [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md) mục 13.8 để biết
  chi tiết nguồn tham số).

## 2. Kết quả kiểm thử tự động (pytest)

Lệnh chạy:

```powershell
python -m pytest test_simulation.py -v
```

Kết quả (11/11 PASSED):

```
test_simulation.py::test_target_synchronized_with_ego PASSED
test_simulation.py::test_applied_commands_replay_ego_state PASSED
test_simulation.py::test_commands_within_limits PASSED
test_simulation.py::test_applied_matches_selected_controller PASSED
test_simulation.py::test_finalize_rejects_non_finite PASSED
test_simulation.py::test_finalize_floor_then_clip PASSED
test_simulation.py::test_fiala_zero_slip_produces_zero_force PASSED
test_simulation.py::test_fiala_matches_cornering_stiffness_at_small_slip PASSED
test_simulation.py::test_fiala_force_is_odd_and_saturates_at_friction_limit PASSED
test_simulation.py::test_braking_shifts_normal_load_to_front_axle PASSED
test_simulation.py::test_friction_ellipse_reduces_lateral_capacity_under_hard_braking PASSED

============================== 11 passed ==============================
```

5 test cho mô hình lốp (Fiala + friction ellipse + tải pháp tuyến động):

| Test | Mục tiêu kiểm tra |
|---|---|
| `test_fiala_zero_slip_produces_zero_force` | `alpha=0` phải cho lực ngang bằng 0 ở cả hai cầu. |
| `test_fiala_matches_cornering_stiffness_at_small_slip` | Với `alpha` rất nhỏ (`1e-6 rad`), `Fy/alpha -> Cf` (đúng định nghĩa độ cứng góc trượt tuyến tính tại gốc). |
| `test_fiala_force_is_odd_and_saturates_at_friction_limit` | Lực là hàm lẻ theo `alpha` (`Fy(-a) = -Fy(a)`) và bão hòa đúng bằng `mu*Fz` tĩnh khi `ax=0` và `alpha` vượt góc bão hòa. |
| `test_braking_shifts_normal_load_to_front_axle` | Phanh (`ax<0`) tăng `Fzf`/giảm `Fzr`; tăng tốc (`ax>0`) ngược lại; tổng `Fzf+Fzr` không đổi. |
| `test_friction_ellipse_reduces_lateral_capacity_under_hard_braking` | Khi phanh gấp, lực ngang bão hòa thấp hơn giới hạn tính riêng từ `Fz` động (do một phần ma sát đã dùng cho lực dọc theo friction ellipse). |

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
    # ax=0 => friction ellipse không thu hẹp giới hạn, max_lateral = mu*Fz
    print(deg, model._fiala_lateral_force(a, p.Cf, p.mu*Fzf),
          model._fiala_lateral_force(a, p.Cr, p.mu*Fzr))
"
```

## 3b. Friction ellipse + tải pháp tuyến động (mới triển khai)

Minh họa ảnh hưởng của gia tốc dọc `ax` lên tải pháp tuyến và giới hạn lực
ngang bão hòa (cầu trước, `alpha` đủ lớn để bão hòa):

| `ax` (m/s²) | Tình huống | `Fzf` (N) | `Fx_f` (N, ước lượng) | `mu*Fzf` (chỉ do Fz động) | Giới hạn lực ngang bão hòa thực tế (friction ellipse) |
|---:|---|---:|---:|---:|---:|
| 0 | giữ tốc độ | 8034.4 | 0.0 | 7231.0 | 7231.0 |
| -6 | phanh gấp | 10113.4 | -4914.0 | 9102.1 | 7661.6 |
| +3 | tăng tốc | 6994.9 | 2457.0 | 6295.4 | 5796.1 |

Quan sát: phanh gấp làm `Fzf` tăng (chuyển tải về phía trước) nên nếu chỉ xét
riêng `mu*Fz` thì giới hạn lực ngang tưởng như tăng khá nhiều (7231.0 ->
9102.1 N); nhưng phần ma sát bị chiếm dụng bởi lực dọc (`Fx_f`) qua friction
ellipse kéo giới hạn thực tế xuống còn 7661.6 N — thấp hơn đáng kể so với
mức "chỉ do `Fz` động", dù vẫn còn cao hơn mốc tĩnh ban đầu. Đây chính là nội
dung được kiểm chứng trong
`test_friction_ellipse_reduces_lateral_capacity_under_hard_braking`.

Lệnh tái tạo:

```powershell
python -c "
from vehicle_model import DynamicBicycleModel
model = DynamicBicycleModel()
for ax in [0.0, -6.0, 3.0]:
    Fzf, Fzr = model.normal_loads(ax)
    Fxf, Fxr = model._longitudinal_force_split(ax)
    max_Fyf = model._max_lateral_friction(model.params.mu, Fzf, Fxf)
    print(ax, Fzf, Fxf, max_Fyf)
"
```

## 4. Kết quả mô phỏng closed-loop (NMPC + Pure Pursuit + CRI)

Chạy một episode mặc định (`SimConfig(max_steps=300)`, kịch bản
`lateral_zigzag`) với luật lốp Fiala **đã có friction ellipse + tải pháp
tuyến động**:

| Chỉ tiêu | Giá trị (sau khi thêm friction ellipse + tải động) | Giá trị trước đó (chỉ Fiala thuần, tải tĩnh) |
|---|---:|---:|
| Số bước mô phỏng thực hiện | 156 / 300 (`target_end`) | 156 / 300 (`target_end`) |
| Phân bố mode | NMPC: 35, PP_FALLBACK: 1, NO_TRACK: 120 | NMPC: 35, PP_FALLBACK: 10, NO_TRACK: 111 |
| Khoảng cách nhỏ nhất ego-target | 29.735 m | 2.591 m |
| Khoảng cách cuối cùng | 133.114 m | 64.519 m |
| Thời điểm vào ngưỡng gần (< 4 m) | **không xảy ra** (`None`) | t = 6.0 s |
| CRI trung bình / min / max | 0.1819 / 0.000 / 0.8596 | 0.172 / 0.000 / 0.824 |
| Gia tốc ngang động học lớn nhất (kinematic, từ lệnh `delta` đã áp dụng) | 0.184 m/s² | 83.124 m/s² (bất thường) |

**Phát hiện quan trọng:** việc bật friction ellipse + tải pháp tuyến động đã
thay đổi đáng kể hành vi closed-loop so với bản Fiala thuần túy trước đó,
dù `Cf`, `Cr` và kịch bản giữ nguyên. Cụ thể, trong episode mặc định đã chạy,
xe **không còn tới gần mục tiêu trong ngưỡng 4 m** (trước đó có, ở t=6s).
Gia tốc ngang kinematic giảm mạnh (83 → 0.18 m/s²) cho thấy NMPC/PP giờ chọn
lệnh lái ôn hòa hơn nhiều — khả năng là vì mô hình dự đoán bên trong NMPC
(dùng cùng `DynamicBicycleModel`) giờ phản ánh đúng hơn giới hạn lực ngang
khả dụng, khiến các lệnh lái gắt trước đây (vốn ngầm giả định lực lốp vô hạn
khi `ax` lớn) không còn được xem là khả thi/tối ưu.

Đây là một thay đổi hành vi cần được điều tra thêm (xem mục 6), **không nên
diễn giải ngay là "mô hình mới tệ hơn"** — nhiều khả năng mô hình cũ (tải
tĩnh, không friction ellipse) đã đánh giá quá cao khả năng bám đường của xe,
và kết quả "tới gần mục tiêu" trước đó dựa trên giả định vật lý không thực
tế. Trước khi dùng một trong hai bộ số cho báo cáo, cần xác định lại kịch
bản/tham số điều khiển (ví dụ chân trời NMPC, trọng số cost, ngưỡng CRI) cho
phù hợp với mô hình lốp đã thực tế hơn.

**Ghi chú về `lat_accel_kin`:** đây vẫn chỉ là ước lượng kinematic
(`v^2*tan(delta)/L`) từ lệnh lái đã áp dụng, không phải gia tốc tính trực
tiếp từ lực lốp Fiala/friction ellipse. Giá trị nhỏ hơn nhiều ở đây phản ánh
lệnh lái được chọn ôn hòa hơn, không phải một phép đo lực lốp độc lập.

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

## 5. Phân tích độ nhạy `Cf`/`Cr`

Chạy lại kịch bản closed-loop mặc định (`max_steps=300`) với `Cf`, `Cr` quét
theo hệ số 0.5×, 1×, 2×, 4× giá trị hiện tại, cộng thêm mốc so sánh
`Cf=Cr=80000 N/rad` (giá trị của Rajamani cho cùng loại sedan):

| factor | Cf (N/rad) | Cr (N/rad) | min_dist (m) | CRI trung bình | NMPC | PP_FALLBACK | NO_TRACK | steps |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 9500 | 16500 | 29.735 | 0.1819 | 35 | 1 | 120 | 156 |
| 1.0 (hiện tại) | 19000 | 33000 | 29.735 | 0.1819 | 35 | 1 | 120 | 156 |
| 2.0 | 38000 | 66000 | 29.735 | 0.1818 | 35 | 1 | 120 | 156 |
| 4.0 | 76000 | 132000 | 29.735 | 0.1815 | 35 | 1 | 120 | 156 |
| — (Rajamani) | 80000 | 80000 | 29.735 | 0.1815 | 35 | 1 | 120 | 156 |

**Nhận xét:** trong kịch bản đã chạy, các chỉ tiêu đầu ra (khoảng cách nhỏ
nhất, phân bố mode, CRI trung bình) **gần như không đổi** trên toàn dải quét
8× (0.5×–4×) và ở mốc Rajamani. Điều này không có nghĩa `Cf`/`Cr` không quan
trọng nói chung — nó cho thấy: (a) trong kịch bản/chân trời điều khiển đã
chạy, phần lớn bước thời gian là `NO_TRACK` (120/156, không có lực lốp nào
ảnh hưởng tới quyết định vì không có track), (b) số bước có điều khiển chủ
động (NMPC/PP) tương đối ít và có thể không đi vào vùng góc trượt đủ lớn để
`Cf`/`Cr` tạo khác biệt rõ rệt trong friction ellipse hiện tại.

**Kết luận cho mục đích "chứng minh tính legit":** phân tích độ nhạy này
**không thay thế** việc cần một nguồn đo/hiệu chuẩn thực tế cho `Cf`, `Cr`.
Nó chỉ cho biết: với kịch bản cụ thể đã kiểm tra, kết quả không nhạy cảm bất
thường với độ lớn `Cf`/`Cr` trong khoảng đã quét — nên nếu phải dùng giá trị
giả định, rủi ro sai lệch định lượng từ riêng `Cf`/`Cr` (ở kịch bản này) có
vẻ thấp hơn rủi ro từ friction ellipse/tải động (xem mục 4). Cần lặp lại
sweep này trên nhiều kịch bản khác (đặc biệt kịch bản có góc trượt lớn hơn)
trước khi khái quát hóa kết luận.

Script tái tạo: `_sensitivity_sweep.py` (xem nội dung bên dưới, không commit
kèm theo file kết quả này vì chỉ là script dùng một lần):

```python
import numpy as np
from simulation import SimConfig, run_simulation
from vehicle_model import VehicleParams

base_Cf, base_Cr = 19000.0, 33000.0
orig_init = VehicleParams.__init__

def make_patched(cf, cr):
    def patched(self):
        orig_init(self)
        self.Cf = cf
        self.Cr = cr
    return patched

for factor in [0.5, 1.0, 2.0, 4.0]:
    cf, cr = base_Cf * factor, base_Cr * factor
    VehicleParams.__init__ = make_patched(cf, cr)
    log = run_simulation(SimConfig(max_steps=300))
    VehicleParams.__init__ = orig_init
    print(factor, cf, cr, np.min(log['distance']), np.mean(log['cri']), log['counts'])
```

## 6. Việc đã hoàn thành ở vòng rà soát này

- Friction ellipse kết hợp lực dọc-ngang theo cầu (`_max_lateral_friction`).
- Tải pháp tuyến động với chuyển tải dọc theo `h` (`normal_loads`).
- Phân tích độ nhạy `Cf`/`Cr` trên kịch bản closed-loop mặc định (mục 5).
- 2 test mới xác nhận hành vi chuyển tải và friction ellipse (mục 2).

## 7. Việc chưa làm / cần rà soát tiếp

- `Cf`, `Cr` vẫn là giá trị giả định, **chưa có nguồn đo/hiệu chuẩn thực tế**
  (xem mục 13.8 của [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md)).
  Sweep ở mục 5 chỉ cho thấy độ nhạy thấp trong một kịch bản cụ thể, không
  phải bằng chứng tính hợp lệ của giá trị.
- `h=0.55 m` (chiều cao trọng tâm) là giả định minh họa, chưa có nguồn riêng.
- Phân bổ lực dọc giữa hai cầu (`_longitudinal_force_split`, theo tỉ lệ tải
  tĩnh) là giả định đơn giản hóa, không phản ánh hệ dẫn động cụ thể (FWD/RWD/
  AWD); cần rà soát nếu kịch bản có gia tốc dọc lớn và cần độ chính xác cao.
- **Cần điều tra thay đổi hành vi closed-loop** nêu ở mục 4 (mất sự kiện tới
  gần mục tiêu) trước khi dùng kết quả mô phỏng cho bất kỳ tuyên bố định
  lượng nào trong báo cáo; có thể cần tinh chỉnh lại tham số điều khiển
  (NMPC/PP/hysteresis) cho phù hợp với mô hình lốp thực tế hơn.
