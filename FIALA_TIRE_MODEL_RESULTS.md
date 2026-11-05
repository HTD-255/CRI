# Kết quả sau khi chuyển luật lốp sang mô hình Fiala

Tài liệu này ghi lại kết quả kiểm thử và mô phỏng sau khi thay luật lực lốp
tuyến tính (`Fy = C*alpha`) bằng mô hình Fiala bão hòa trong
[vehicle_model.py](./vehicle_model.py). Dùng làm tài liệu tham chiếu khi viết
lại phần kết quả/methodology; không phải báo cáo cuối cùng.

Ngày tạo: 2026-11-04. Cập nhật: 2026-11-06 (sửa lỗi phân bổ lực dọc, xem mục
1b). Môi trường: Python 3.12.8, pytest 9.2.0, Windows.

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
  tổng (`m*ax`) cho hai cầu theo tỉ lệ tải **động** (sau chuyển tải, xem mục
  1b); đây là giả định đơn giản hóa (không mô hình hệ dẫn động cụ thể), cần
  rà soát nếu cần độ chính xác cao hơn theo drivetrain thật.
- `DynamicBicycleModel.tire_forces`: nhận thêm tham số `ax` (mặc định 0.0, nên
  tương thích ngược), dùng để tính tải pháp tuyến động và friction ellipse.
- `VehicleParams`: thêm `mu=0.9` (hệ số ma sát), `g=9.81` và `h=0.55` (chiều
  cao trọng tâm, giả định minh họa); `Cf`, `Cr` giữ nguyên giá trị cũ nhưng
  được đánh dấu "giả định, cần rà soát" (xem
  [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md) mục 13.8 để biết
  chi tiết nguồn tham số).

## 1b. Lỗi đã phát hiện và sửa: cầu trước mất toàn bộ khả năng lái khi tăng tốc gắt

Ở lần triển khai friction ellipse đầu tiên, `_longitudinal_force_split` chia
lực dọc tổng theo tỉ lệ **tải pháp tuyến tĩnh**, trong khi `_max_lateral_friction`
(friction ellipse) lại dùng **tải pháp tuyến động** (`normal_loads`, đã gồm
chuyển tải). Khi tăng tốc gắt (`ax` gần `ax_max=8 m/s^2`), chuyển tải làm
giảm tải trọng cầu trước (`Fzf` động giảm), nhưng lực dọc yêu cầu tại cầu
trước vẫn được tính theo tỉ lệ tải **tĩnh** (lớn hơn) — dẫn tới lực dọc yêu
cầu tại cầu trước (vd. 6552 N) **vượt quá** giới hạn ma sát động của chính
cầu đó (`mu*Fzf` động, vd. 4736 N). Phần âm bên trong friction ellipse bị
chặn ở 0, khiến mô hình kết luận sai rằng **cầu trước mất 100% khả năng lái**
mỗi khi tăng tốc tối đa — dù tổng lực dọc toàn xe vẫn hoàn toàn khả thi nếu
phân bổ hợp lý hơn (cầu sau có nhiều tải hơn khi tăng tốc, nên có thể gánh
phần lớn lực dọc).

Hệ quả quan sát được trong mô phỏng closed-loop: NMPC hầu như không dùng
`delta` nữa trong các bước tăng tốc tối đa (`delta≈0.001` thay vì dao động
±0.45 như kỳ vọng), vì mô hình dự đoán bên trong NMPC cho rằng lái không còn
tạo ra lực ngang nào. Khoảng cách nhỏ nhất ego-target trong kịch bản mặc định
tăng từ 2.6 m (bản Fiala thuần, chưa có friction ellipse) lên 29.7 m (bản có
lỗi) — một hồi quy đáng kể.

**Cách sửa:** `_longitudinal_force_split` nay nhận thêm `Fzf`, `Fzr` (tải
động, từ `normal_loads`) và chia lực dọc theo tỉ lệ **tải động** thay vì tải
tĩnh — tự nhất quán với ngân sách ma sát mà `_max_lateral_friction` sử dụng
cho cùng cầu đó. Đây vẫn là một giả định đơn giản hóa (không phản ánh
drivetrain cụ thể FWD/RWD/AWD), nhưng không còn tạo ra mâu thuẫn nội tại
giữa hai hàm.

Ngân sách lực ngang khả dụng theo `ax` sau khi sửa (cầu trước):

| `ax` (m/s²) | Fzf động (N) | Fxf yêu cầu (N) | Lực ngang khả dụng cầu trước (N) | Trước khi sửa |
|---:|---:|---:|---:|---:|
| 0 | 8034.4 | 0.0 | 7231.0 | 7231.0 (không đổi) |
| +3 | 6994.9 | 2139.1 | 5920.8 | thấp hơn đáng kể (chia theo tải tĩnh, ước lượng ~5796) |
| +8 (ax_max) | 5262.4 | 4291.4 | **2003.6** | **0.0** (lỗi: mất hoàn toàn khả năng lái) |
| -5 (phanh) | 9766.9 | -4978.0 | 7244.8 | gần tương tự (ít bị ảnh hưởng vì phanh ít đổi) |

Đã thêm test hồi quy `test_front_axle_retains_lateral_capacity_at_max_acceleration`
để đảm bảo cầu trước luôn giữ được lực ngang khả dụng > 0 ở `ax=ax_max`.

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
test_simulation.py::test_front_axle_retains_lateral_capacity_at_max_acceleration PASSED

============================== 12 passed ==============================
```

6 test cho mô hình lốp (Fiala + friction ellipse + tải pháp tuyến động):

| Test | Mục tiêu kiểm tra |
|---|---|
| `test_fiala_zero_slip_produces_zero_force` | `alpha=0` phải cho lực ngang bằng 0 ở cả hai cầu. |
| `test_fiala_matches_cornering_stiffness_at_small_slip` | Với `alpha` rất nhỏ (`1e-6 rad`), `Fy/alpha -> Cf` (đúng định nghĩa độ cứng góc trượt tuyến tính tại gốc). |
| `test_fiala_force_is_odd_and_saturates_at_friction_limit` | Lực là hàm lẻ theo `alpha` (`Fy(-a) = -Fy(a)`) và bão hòa đúng bằng `mu*Fz` tĩnh khi `ax=0` và `alpha` vượt góc bão hòa. |
| `test_braking_shifts_normal_load_to_front_axle` | Phanh (`ax<0`) tăng `Fzf`/giảm `Fzr`; tăng tốc (`ax>0`) ngược lại; tổng `Fzf+Fzr` không đổi. |
| `test_friction_ellipse_reduces_lateral_capacity_under_hard_braking` | Khi phanh gấp, lực ngang bão hòa thấp hơn giới hạn tính riêng từ `Fz` động (do một phần ma sát đã dùng cho lực dọc theo friction ellipse). |
| `test_front_axle_retains_lateral_capacity_at_max_acceleration` | Hồi quy cho lỗi mục 1b: ở `ax=ax_max`, cầu trước phải còn lực ngang khả dụng `>0` (không được mất hoàn toàn khả năng lái). |

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
| -6 | phanh gấp | 10113.4 | -6185.6 | 9102.1 | 6677.3 |
| +3 | tăng tốc | 6994.9 | 2139.1 | 6295.4 | 5920.8 |
| +8 (ax_max) | tăng tốc tối đa | 5262.4 | 4291.4 | 4736.2 | 2003.6 |

Quan sát: phanh gấp làm `Fzf` tăng (chuyển tải về phía trước) nên nếu chỉ xét
riêng `mu*Fz` thì giới hạn lực ngang tưởng như tăng khá nhiều (7231.0 ->
9102.1 N); nhưng phần ma sát bị chiếm dụng bởi lực dọc (`Fx_f`) qua friction
ellipse kéo giới hạn thực tế xuống còn 6677.3 N — thấp hơn mức "chỉ do `Fz`
động", dù vẫn thấp hơn một chút so với mốc tĩnh ban đầu (7231.0 N). Ở
`ax=8 m/s²` (tăng tốc tối đa), lực ngang khả dụng giảm còn ~28% so với mốc
tĩnh (2003.6 N so với 7231.0 N) — phản ánh thực tế là xe mất phần lớn khả
năng vào cua khi tăng tốc gắt, nhưng **không** mất hoàn toàn (khác với lỗi đã
phát hiện và sửa ở mục 1b, nơi giá trị này từng bị tính sai thành 0). Đây
chính là nội dung được kiểm chứng trong
`test_friction_ellipse_reduces_lateral_capacity_under_hard_braking` và
`test_front_axle_retains_lateral_capacity_at_max_acceleration`.

Lệnh tái tạo:

```powershell
python -c "
from vehicle_model import DynamicBicycleModel
model = DynamicBicycleModel()
for ax in [0.0, -6.0, 3.0]:
    Fzf, Fzr = model.normal_loads(ax)
    Fxf, Fxr = model._longitudinal_force_split(ax, Fzf, Fzr)
    max_Fyf = model._max_lateral_friction(model.params.mu, Fzf, Fxf)
    print(ax, Fzf, Fxf, max_Fyf)
"
```

## 4. Kết quả mô phỏng closed-loop (NMPC + Pure Pursuit + CRI)

Chạy một episode mặc định (`SimConfig(max_steps=300)`, kịch bản
`lateral_zigzag`), so sánh 3 trạng thái của mô hình lốp:

| Chỉ tiêu | (A) Fiala thuần, tải tĩnh (trước friction ellipse) | (B) + friction ellipse, **có lỗi mục 1b** | (C) + friction ellipse, **đã sửa lỗi** |
|---|---:|---:|---:|
| Số bước mô phỏng thực hiện | 156 / 300 (`target_end`) | 156 / 300 (`target_end`) | 156 / 300 (`target_end`) |
| Phân bố mode | NMPC: 35, PP: 10, NO_TRACK: 111 | NMPC: 35, PP: 1, NO_TRACK: 120 | NMPC: 29, PP: 23, NO_TRACK: 104 |
| Khoảng cách nhỏ nhất ego-target | 2.591 m | 29.735 m | **6.339 m** |
| Khoảng cách cuối cùng | 64.519 m | 133.114 m | 51.146 m |
| Thời điểm vào ngưỡng gần (< 4 m) | t = 6.0 s | không xảy ra | không xảy ra (nhưng gần hơn (B) nhiều) |
| CRI trung bình / min / max | 0.172 / 0.000 / 0.824 | 0.1819 / 0.000 / 0.8596 | 0.199 / 0.000 / 0.811 |
| Gia tốc ngang kinematic lớn nhất | 83.124 m/s² (bất thường) | 0.184 m/s² | 129.978 m/s² (bất thường, xem ghi chú) |

**Kết luận:** sau khi sửa lỗi phân bổ lực dọc (mục 1b), kết quả cải thiện rõ
rệt so với (B) — khoảng cách nhỏ nhất giảm từ 29.7 m xuống 6.3 m, gần với
mốc (A) hơn nhiều, trong khi vẫn giữ được mô hình vật lý thực tế hơn (A)
(có friction ellipse + tải động). Kết quả (C) chưa đạt lại chính xác mốc
(A) — điều này **hợp lý**, vì (A) vốn cho phép xe dùng lực ngang không giới
hạn ngay cả khi tăng tốc tối đa (vật lý không thực tế), nên (A) luôn là
"best case" lạc quan hơn so với mọi mô hình có ràng buộc ma sát thực tế.

**Ghi chú về gia tốc ngang kinematic (129.978 m/s² ở cột C):** đây là một
giới hạn đã biết của chỉ số `lat_accel_kin` (`v^2*tan(delta)/L`), xảy ra khi
NMPC chọn `delta` lớn ở tốc độ cao (ví dụ `vx≈27.7 m/s`, `delta≈-0.40 rad`
tại t=3.7s) — công thức kinematic giả định xe bám đường hoàn hảo (không
trượt), nên phóng đại gia tốc thực tế khi góc lái lớn ở tốc độ cao. Gia tốc
ngang **thực tế** của xe bị giới hạn bởi lực Fiala đã bão hòa theo friction
ellipse (tối đa `max_Fyf`/`m`, xem mục 3b), không phải giá trị kinematic này.
Đây không phải hồi quy mới — chỉ là việc phục hồi khả năng lái (mục 1b) khiến
NMPC dùng `delta` lớn hơn, do đó bộc lộ rõ hơn hạn chế đã biết của chỉ số
kinematic. Khuyến nghị: nếu cần báo cáo gia tốc ngang, tính trực tiếp từ
`Fyf`, `Fyr` (gia tốc động lực học, `dvy/dt` nêu trong `continuous_dynamics`)
thay vì dùng `lat_accel_kin`.

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

## 5. Phân tích độ nhạy `Cf`/`Cr` (đã chạy lại sau khi sửa lỗi mục 1b)

Chạy lại kịch bản closed-loop mặc định (`max_steps=300`) với `Cf`, `Cr` quét
theo hệ số 0.5×, 1×, 2×, 4× giá trị hiện tại, cộng thêm mốc so sánh
`Cf=Cr=80000 N/rad` (giá trị của Rajamani cho cùng loại sedan), **trên mô
hình đã sửa lỗi friction ellipse**:

| factor | Cf (N/rad) | Cr (N/rad) | min_dist (m) | CRI trung bình | NMPC | PP_FALLBACK | NO_TRACK | steps |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 9500 | 16500 | 11.655 | 0.1830 | 22 | 25 | 109 | 156 |
| 1.0 (hiện tại) | 19000 | 33000 | 6.339 | 0.1990 | 29 | 23 | 104 | 156 |
| 2.0 | 38000 | 66000 | 2.653 | 0.2213 | 43 | 9 | 104 | 156 |
| 4.0 | 76000 | 132000 | 9.969 | 0.1525 | 30 | 2 | 124 | 156 |
| — (Rajamani) | 80000 | 80000 | 6.623 | 0.1893 | 33 | 8 | 115 | 156 |

**Nhận xét (đã thay đổi so với kết quả sweep trước khi sửa lỗi mục 1b):**
sau khi sửa lỗi friction ellipse, kết quả **nhạy cảm rõ rệt** với `Cf`/`Cr`
— `min_dist` dao động từ 2.65 m (factor 2.0) đến 11.66 m (factor 0.5), và
phân bố mode (NMPC/PP_FALLBACK) cũng thay đổi đáng kể theo từng mức. Đây là
hành vi hợp lý hơn nhiều so với sweep trước đó (gần như phẳng tuyệt đối),
vốn là dấu hiệu gián tiếp của chính lỗi đã sửa (khi `Cf`/`Cr` không còn ảnh
hưởng nhiều vì cầu trước đã mất khả năng lái do lỗi, không phải vì hệ thống
thực sự không nhạy cảm với tham số này).

Biến thiên **không đơn điệu** theo `factor` (không phải "Cf/Cr càng lớn thì
càng tốt" một cách tuyến tính) — đây là hệ quả tự nhiên của một hệ vòng kín
phi tuyến có chuyển mode rời rạc (NMPC/PP/hysteresis/mất track), không phải
dấu hiệu lỗi. Giá trị `Cf`/`Cr` hiện tại (factor 1.0) cho kết quả gần tương
đương mốc Rajamani (6.34 m so với 6.62 m) — một điểm dữ liệu khác hỗ trợ (dù
không chứng minh) rằng độ lớn hiện tại không quá bất hợp lý so với tài liệu
tham chiếu, trong kịch bản cụ thể này.

**Kết luận cho mục đích "chứng minh tính legit":** phân tích độ nhạy này
**không thay thế** việc cần một nguồn đo/hiệu chuẩn thực tế cho `Cf`, `Cr`.
Với mô hình đã sửa lỗi, kết quả hiện **nhạy cảm đáng kể** với `Cf`/`Cr` (khác
với kết luận "ít nhạy cảm" rút ra từ bản có lỗi) — nghĩa là việc chưa có
nguồn đo thực tế cho `Cf`/`Cr` là một rủi ro định lượng thực sự, cần ưu tiên
xử lý trước khi dùng kết quả closed-loop cho báo cáo. Cần lặp lại sweep này
trên nhiều kịch bản khác, nhiều seed, trước khi khái quát hóa kết luận.

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

rows = []
for factor in [0.5, 1.0, 2.0, 4.0]:
    cf, cr = base_Cf * factor, base_Cr * factor
    VehicleParams.__init__ = make_patched(cf, cr)
    log = run_simulation(SimConfig(max_steps=300))
    VehicleParams.__init__ = orig_init
    rows.append((factor, cf, cr, np.min(log['distance']), np.mean(log['cri']), log['counts']))

VehicleParams.__init__ = make_patched(80000.0, 80000.0)  # Rajamani
log_raj = run_simulation(SimConfig(max_steps=300))
VehicleParams.__init__ = orig_init
for r in rows:
    print(*r)
print('Rajamani:', np.min(log_raj['distance']), np.mean(log_raj['cri']), log_raj['counts'])
```

## 6. Việc đã hoàn thành ở vòng rà soát này

- Friction ellipse kết hợp lực dọc-ngang theo cầu (`_max_lateral_friction`).
- Tải pháp tuyến động với chuyển tải dọc theo `h` (`normal_loads`).
- **Phát hiện và sửa lỗi self-consistency** giữa `_longitudinal_force_split`
  (trước đây dùng tải tĩnh) và `_max_lateral_friction` (dùng tải động) — lỗi
  này từng khiến cầu trước mất toàn bộ khả năng lái khi gia tốc gần `ax_max`
  (xem mục 1b). Đã thêm test hồi quy
  `test_front_axle_retains_lateral_capacity_at_max_acceleration`.
- Chạy lại kết quả closed-loop (mục 4) và sweep `Cf`/`Cr` (mục 5) trên mô
  hình đã sửa lỗi; kết luận cũ rút ra từ dữ liệu có lỗi (ví dụ "độ nhạy
  `Cf`/`Cr` thấp") đã được thay thế bằng kết luận mới, chính xác hơn.
- Tổng cộng 12 test tự động xác nhận hành vi chuyển tải, friction ellipse và
  tính self-consistency của phân bổ lực dọc/ngang (mục 2).

## 7. Việc chưa làm / cần rà soát tiếp

- `Cf`, `Cr` vẫn là giá trị giả định, **chưa có nguồn đo/hiệu chuẩn thực tế**
  (xem mục 13.8 của [RESEARCH_PLAN_CRI_SEDAN.md](./RESEARCH_PLAN_CRI_SEDAN.md)).
  Sweep ở mục 5 (đã chạy lại sau khi sửa lỗi mục 1b) cho thấy **độ nhạy đáng
  kể** với `Cf`/`Cr` — khác với kết luận "ít nhạy cảm" rút ra trước đây từ dữ
  liệu có lỗi — nên đây là rủi ro định lượng thực sự cần ưu tiên xử lý (tìm
  nguồn đo riêng hoặc mở rộng sweep sang nhiều kịch bản/seed khác) trước khi
  trình bày `Cf`/`Cr` là tham số đã xác thực.
- `h=0.55 m` (chiều cao trọng tâm) là giả định minh họa, chưa có nguồn riêng.
- Phân bổ lực dọc giữa hai cầu (`_longitudinal_force_split`, nay theo tỉ lệ
  tải **động**) vẫn là giả định đơn giản hóa, không phản ánh hệ dẫn động cụ
  thể (FWD/RWD/AWD); cần rà soát nếu kịch bản có gia tốc dọc lớn và cần độ
  chính xác cao hơn (ví dụ mô hình riêng cho xe RWD/FWD).
- Chỉ số `lat_accel_kin` (gia tốc ngang kinematic log để debug) vẫn là ước
  lượng không có giới hạn vật lý (`v^2*tan(delta)/L`), có thể cho giá trị rất
  lớn và gây hiểu nhầm (xem ghi chú mục 4); nên cân nhắc thay bằng gia tốc
  tính trực tiếp từ `Fyf`, `Fyr` nếu dùng chỉ số này cho báo cáo.
- Sweep `Cf`/`Cr` và kết quả closed-loop (mục 4, 5) mới chạy trên **một**
  kịch bản (`lateral_zigzag`, seed mặc định); cần lặp lại trên nhiều kịch
  bản/seed khác trước khi coi là đại diện chung.
