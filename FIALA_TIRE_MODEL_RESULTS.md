# Kết quả sau khi chuyển luật lốp sang mô hình Fiala

Tài liệu này ghi lại kết quả kiểm thử và mô phỏng sau khi thay luật lực lốp
tuyến tính (`Fy = C*alpha`) bằng mô hình Fiala bão hòa trong
[vehicle_model.py](./vehicle_model.py). Dùng làm tài liệu tham chiếu khi viết
lại phần kết quả/methodology; không phải báo cáo cuối cùng.

Ngày tạo: 2026-11-04. Cập nhật gần nhất: 2026-11-07 (regularization tốc độ
thấp, chạy lại kết quả closed-loop và sweep `Cf`/`Cr`). Môi trường: Windows,
Python 3.12.9 trong `.venv`, NumPy 2.5.4, pytest 9.2.0.

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
- `VehicleParams`: `Cf=Cr=80000 N/rad` là độ cứng góc trượt **cả cầu** được
  công bố trong bộ sedan tham chiếu của Rajamani (2012, ch. 3); đây không phải
  giá trị tự đặt. Giới hạn là chưa có dữ liệu hiệu chuẩn để khẳng định bộ
  tham số tham chiếu mô tả chính xác một xe vật lý cụ thể. `mu=0.9` là giá trị
  mô phỏng được đối chiếu khoảng kiểm định trong Taylor (2015), không phải đo
  cho một cặp lốp-mặt đường cụ thể; `h=0.55 m` vẫn là giả định minh họa.
- `VehicleParams.v_epsilon=0.5 m/s` và
  `DynamicBicycleModel._regularized_speed`: regularize mẫu số vận tốc bằng một
  smooth floor để tránh kỳ dị ở tốc độ thấp mà không làm đổi đáng kể góc
  trượt ở tốc độ trên ngưỡng.

## 1b. Lỗi đã phát hiện và sửa: cầu trước mất toàn bộ khả năng lái khi tăng tốc gắt

Ở lần triển khai friction ellipse đầu tiên, `_longitudinal_force_split` chia
lực dọc tổng theo tỉ lệ **tải pháp tuyến tĩnh**, trong khi `_max_lateral_friction`
(friction ellipse) lại dùng **tải pháp tuyến động** (`normal_loads`, đã gồm
chuyển tải). Khi tăng tốc gắt (`ax` gần `ax_max=8 m/s^2`), chuyển tải làm
giảm tải trọng cầu trước (`Fzf` động giảm), nhưng lực dọc yêu cầu tại cầu
trước vẫn được tính theo tỉ lệ tải **tĩnh** (lớn hơn) — dẫn tới lực dọc yêu
cầu trước (7418.9 N tại `ax=8 m/s^2`) **vượt quá** giới hạn ma sát động
của chính cầu đó (`mu*Fzf` động = 5863.4 N). Phần âm bên trong friction ellipse bị
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

| `ax` (m/s²) | Fzf động (N) | Fxf hiện tại (N) | Lực ngang khả dụng hiện tại (N) | Chia Fx theo Fz tĩnh (N) |
|---:|---:|---:|---:|---:|
| 0 | 9097.5 | 0.0 | 8187.7 | 8187.7 |
| +3 | 8129.0 | 2485.9 | 6880.8 | 6766.5 |
| +8 (ax_max) | 6514.9 | 5312.9 | **2480.5** | **0.0** (lỗi phân bổ cũ) |
| -5 (phanh) | 10711.5 | -5459.5 | 7945.5 | 8452.0 |

Đã thêm test hồi quy `test_front_axle_retains_lateral_capacity_at_max_acceleration`
để đảm bảo cầu trước luôn giữ được lực ngang khả dụng > 0 ở `ax=ax_max`.

## 1c. Regularization góc trượt ở tốc độ thấp

Trước cập nhật 2026-11-07, mô hình đặt sàn cho `vx` nhưng vẫn dùng góc lái
để tạo slip angle ở `vx=0`; xe đứng yên với `delta != 0` vì vậy có thể nhận
lực lốp giả. Hiện mô hình tính các thành phần vận tốc trong hệ tọa độ bánh
trước/sau, sau đó dùng mẫu số vận tốc được làm trơn:

```text
v_reg = 0.5 * (abs(v) + epsilon
               + hypot(abs(v) - epsilon, epsilon * 1e-3))
epsilon = 0.5 m/s
```

Đây là smooth approximation của `max(abs(v), epsilon)`: mẫu số luôn dương,
không kỳ dị khi đổi chiều, và hầu như không làm đổi slip ở tốc độ lớn hơn
ngưỡng. Fiala vẫn là mô hình phi tuyến theo `tan(alpha)`; friction ellipse
vẫn giới hạn lực ngang còn lại sau lực dọc. Khi `vx=vy=r=0`, lực ngang bằng
0 với mọi góc lái; khi xe còn vận tốc ngang hoặc yaw rate khác 0, lực ngang
có thể vẫn có ý nghĩa vật lý.

Giá trị kiểm tra với `delta=0.3 rad`, `vy=r=0`:

| `vx` (m/s) | `Fyf` (N) |
|---:|---:|
| 0 | 0.0 |
| 0.0001 | 4.727 |
| 0.01 | 463.789 |
| 0.5 | 8187.279 (gần bão hòa) |
| 1.0 | 8187.712 (bão hòa) |
| 10.0 | 8187.712 (bão hòa) |

Hai test hồi quy `test_fiala_stationary_vehicle_has_no_steering_force` và
`test_fiala_force_tends_to_zero_near_standstill` kiểm tra lực bằng 0 tại trạng
thái đứng yên và tiến về 0 khi tốc độ giảm. Ngưỡng `0.5 m/s` là regularization
số học, chưa được nhận dạng từ dữ liệu lốp.

Phạm vi: NMPC hiện tại dùng mô phỏng phi tuyến và sai phân hữu hạn trong
projected-gradient solver. Các đạo hàm affine được nêu trong đề xuất Convex
MPC và matrix-exponential discretization **chưa được triển khai**; nếu đổi
solver, cần tính Jacobian nhất quán với regularizer này. Cũng chưa có luật
Ackermann khi xe dừng; mô hình lực không sinh lực ngang chỉ vì đổi `delta`.

## 2. Kết quả kiểm thử tự động (pytest)

Lệnh chạy:

```powershell
.\.venv\Scripts\python.exe -m pytest test_simulation.py -q
```

Kết quả hiện tại: **14/14 passed** (Python 3.12.9, NumPy 2.5.4, pytest 9.2.0).
Tám test liên quan trực tiếp đến lốp kiểm tra Fiala tại slip bằng 0, giới hạn
ma sát, độ cứng góc nhỏ, trạng thái đứng yên, hội tụ tốc độ thấp, chuyển tải
phanh/tăng tốc, friction ellipse và khả năng còn lực ngang tại `ax_max`.

```
..............                                                           [100%]
14 passed
```

Tám test cho mô hình lốp (Fiala + regularization tốc độ thấp + friction
ellipse + tải pháp tuyến động):

| Test | Mục tiêu kiểm tra |
|---|---|
| `test_fiala_zero_slip_produces_zero_force` | `alpha=0` phải cho lực ngang bằng 0 ở cả hai cầu. |
| `test_fiala_stationary_vehicle_has_no_steering_force` | Với `vx=vy=r=0`, thay đổi `delta` không được tự sinh lực ngang. |
| `test_fiala_force_tends_to_zero_near_standstill` | Lực ngang giảm về 0 khi `vx` tiến về 0. |
| `test_fiala_matches_cornering_stiffness_at_small_slip` | Với `alpha` rất nhỏ (`1e-6 rad`), `Fy/alpha -> Cf` (đúng định nghĩa độ cứng góc trượt tuyến tính tại gốc). |
| `test_fiala_force_is_odd_and_saturates_at_friction_limit` | Lực là hàm lẻ theo `alpha` (`Fy(-a) = -Fy(a)`) và bão hòa đúng bằng `mu*Fz` tĩnh khi `ax=0` và `alpha` vượt góc bão hòa. |
| `test_braking_shifts_normal_load_to_front_axle` | Phanh (`ax<0`) tăng `Fzf`/giảm `Fzr`; tăng tốc (`ax>0`) ngược lại; tổng `Fzf+Fzr` không đổi. |
| `test_friction_ellipse_reduces_lateral_capacity_under_hard_braking` | Khi phanh gấp, lực ngang bão hòa thấp hơn giới hạn tính riêng từ `Fz` động (do một phần ma sát đã dùng cho lực dọc theo friction ellipse). |
| `test_front_axle_retains_lateral_capacity_at_max_acceleration` | Hồi quy cho lỗi mục 1b: ở `ax=ax_max`, cầu trước phải còn lực ngang khả dụng `>0` (không được mất hoàn toàn khả năng lái). |

## 3. Đặc tính số của luật lốp Fiala (tham số hiện tại)

Với bộ tham số Rajamani 2012 (`m=1573 kg`, `lf=1.1 m`, `lr=1.58 m`, `L=lf+lr=2.68 m`, `Cf=Cr=80000 N/rad`), `mu=0.9`, `g=9.81 m/s^2`:

- Tải pháp tuyến tĩnh: `Fzf = 9097.5 N`, `Fzr = 6333.7 N`.
- Giới hạn ma sát (lực bão hòa tối đa): `mu*Fzf = 8187.7 N`, `mu*Fzr = 5700.3 N`.
- Góc trượt bắt đầu bão hòa (`tan(alpha) = 3*mu*Fz/C`):
  - Cầu trước: `alpha_sat_f ≈ 17.6°`.
  - Cầu sau: `alpha_sat_r ≈ 12.2°`.

Bảng lực ngang theo góc trượt (tính trực tiếp từ `_fiala_lateral_force`,
`vx=15 m/s`):

| alpha (deg) | Fyf (N) | Fyr (N) |
|---:|---:|---:|
| 0 | 0.0 | 0.0 |
| 1 | 1318.5 | 1285.5 |
| 2 | 2488.0 | 2362.1 |
| 3 | 3517.7 | 3248.7 |
| 4 | 4416.8 | 3963.7 |
| 5 | 5194.2 | 4525.3 |
| 7 | 6418.3 | 5260.9 |
| 10 | 7556.0 | 5669.7 |
| 15 | 8170.8 | 5700.3 (bão hòa) |
| 20 | 8187.7 (bão hòa) | 5700.3 (bão hòa) |
| 30 | 8187.7 | 5700.3 |
| 50 | 8187.7 | 5700.3 |

Quan sát: cầu sau bão hòa sớm hơn cầu trước (~12° so với ~18°) do `Cr/Fzr`
lớn hơn `Cf/Fzf` (cùng `C`, tải sau nhỏ hơn). Toàn bộ tham số xe nay lấy từ
một nguồn công bố duy nhất (Rajamani 2012, ch. 3), nên không còn "bộ trộn".
Ngoại lệ vẫn là `h=0.55 m` (giả định) và `mu=0.9` (tham chiếu Taylor 2015).

Lệnh tái tạo:

```powershell
.\.venv\Scripts\python.exe -c "
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
| 0 | giữ tốc độ | 9097.5 | 0.0 | 8187.7 | 8187.7 |
| -6 | phanh gấp | 11034.4 | -6748.8 | 9930.9 | 7285.4 |
| +3 | tăng tốc | 8129.0 | 2485.9 | 7316.1 | 6880.8 |
| +8 (ax_max) | tăng tốc tối đa | 6514.9 | 5312.9 | 5863.4 | 2480.5 |

Quan sát: phanh gấp làm `Fzf` tăng (chuyển tải về phía trước) nên nếu chỉ xét
riêng `mu*Fz` thì giới hạn lực ngang tưởng như tăng (8187.7 -> 9930.9 N);
nhưng phần ma sát bị chiếm dụng bởi lực dọc (`Fx_f`) qua friction ellipse
kéo giới hạn thực tế xuống còn 7285.4 N — thấp hơn mức "chỉ do `Fz` động" và
thấp hơn mốc tĩnh (8187.7 N). Ở `ax=8 m/s²` (tăng tốc tối đa), lực ngang
khả dụng giảm còn ~30% so với mốc tĩnh (2480.5 N so với 8187.7 N) — xe mất
phần lớn khả năng vào cua khi tăng tốc gắt, nhưng **không** mất hoàn toàn
(khác với lỗi đã sửa ở mục 1b, nơi giá trị này từng bị tính sai thành 0).
Đây chính là nội dung được kiểm chứng trong
`test_friction_ellipse_reduces_lateral_capacity_under_hard_braking` và
`test_front_axle_retains_lateral_capacity_at_max_acceleration`.

Lệnh tái tạo:

```powershell
.\.venv\Scripts\python.exe -c "
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

Chạy lại một episode mặc định (`SimConfig(max_steps=300)`, kịch bản
`lateral_zigzag`) trên code hiện tại, gồm regularization tốc độ thấp. Cột
(D) là lần chạy mới. Các cột (A)-(C) là số liệu lưu từ các phiên bản trước;
chúng chưa được chạy lại cùng regularizer mới nên chỉ giữ làm lịch sử, không
phải phép so sánh có kiểm soát với (D).

| Chỉ tiêu | (A) Fiala thuần, tải tĩnh | (B) + friction ellipse, **có lỗi mục 1b** | (C) + ellipse đã sửa, tham số cũ (`Cf=19000`, `Cr=33000`) | (D) + ellipse đã sửa, **tham số Rajamani** |
|---|---:|---:|---:|---:|
| Số bước mô phỏng | 156 / 300 (`target_end`) | 156 / 300 | 156 / 300 | 156 / 300 |
| Phân bố mode | NMPC: 35, PP: 10, NO_TRACK: 111 | NMPC: 35, PP: 1, NO_TRACK: 120 | NMPC: 29, PP: 23, NO_TRACK: 104 | NMPC: 32, PP: 10, NO_TRACK: 114 |
| Khoảng cách nhỏ nhất ego-target | 2.591 m | 29.735 m | 6.339 m | **5.955 m** |
| Khoảng cách cuối cùng | 64.519 m | 133.114 m | 51.146 m | 96.909 m |
| Thời điểm vào ngưỡng gần (< 4 m) | t = 6.0 s | không | không | không |
| CRI trung bình / min / max | 0.172 / 0 / 0.824 | 0.1819 / 0 / 0.8596 | 0.199 / 0 / 0.811 | 0.1868 / 0 / 0.8662 |
| `lat_accel_kin` lớn nhất | 83.124 m/s² | 0.184 m/s² | 129.978 m/s² | 174.183 m/s² |

**Kết quả bản hiện tại:** mô phỏng kết thúc theo `target_end` sau 156 bước;
`min_dist=5.955 m`, không vào ngưỡng gần `<4 m`. Đây là một episode của một
kịch bản/seed, không đủ để khái quát hiệu năng chung. Chênh lệch với số liệu
(D) cũ (`6.266 m`) xuất hiện sau cập nhật regularization tốc độ thấp.

`lat_accel_kin` cực đại `174.183 m/s^2` vẫn là ước lượng động học
`v^2*tan(delta)/L`, không phải gia tốc ngang tính từ lực lốp. Không dùng số
này làm bằng chứng rằng xe thực có thể đạt gia tốc đó; nếu báo cáo gia tốc
ngang, cần tính từ lực Fiala và phương trình động lực học.

Lệnh tái tạo:

```powershell
.\.venv\Scripts\python.exe -c "
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

## 5. Phân tích độ nhạy `Cf`/`Cr` (code hiện tại)

Chạy lại kịch bản closed-loop mặc định (`max_steps=300`), nhân `Cf` và `Cr`
(cơ sở `80000 N/rad`, Rajamani 2012) với hệ số 0.5×, 1×, 2×, 4×:

| factor | Cf = Cr (N/rad) | min_dist (m) | CRI trung bình | NMPC | PP_FALLBACK | NO_TRACK | steps |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 40000 | 4.856 | 0.2084 | 31 | 23 | 102 | 156 |
| 1.0 (Rajamani, hiện tại) | 80000 | 5.955 | 0.1868 | 32 | 10 | 114 | 156 |
| 2.0 | 160000 | 5.330 | 0.2059 | 35 | 13 | 108 | 156 |
| 4.0 | 320000 | 5.282 | 0.2006 | 32 | 16 | 108 | 156 |

**Nhận xét:** `min_dist` nằm trong khoảng 4.856–5.955 m trên dải 8× của
`Cf`/`Cr`, và không đơn điệu (hệ vòng kín phi tuyến có chuyển mode rời rạc).
Ở cả bốn hệ số, episode này không vào ngưỡng gần `<4 m`. Đây là sweep một
kịch bản/seed; cần thêm kịch bản và seed trước khi khái quát hóa.

Script tái tạo (dùng một lần, không commit):

```python
import numpy as np
from simulation import SimConfig, run_simulation
from vehicle_model import VehicleParams

orig_init = VehicleParams.__init__
def make_patched(f):
    def patched(self):
        orig_init(self)
        self.Cf *= f
        self.Cr *= f
    return patched

for f in [0.5, 1.0, 2.0, 4.0]:
    VehicleParams.__init__ = make_patched(f)
    log = run_simulation(SimConfig(max_steps=300))
    print(f, np.min(log['distance']), np.mean(log['cri']), log['counts'])
VehicleParams.__init__ = orig_init
```
## 6. Việc đã hoàn thành ở vòng rà soát này

- Friction ellipse kết hợp lực dọc-ngang theo cầu (`_max_lateral_friction`).
- Tải pháp tuyến động với chuyển tải dọc theo `h` (`normal_loads`).
- Regularization tốc độ thấp cho slip angle, loại bỏ lực ngang giả do riêng
  góc lái tại trạng thái `vx=vy=r=0` (mục 1c).
- **Phát hiện và sửa lỗi self-consistency** giữa `_longitudinal_force_split`
  (trước đây dùng tải tĩnh) và `_max_lateral_friction` (dùng tải động) — lỗi
  này từng khiến cầu trước mất toàn bộ khả năng lái khi gia tốc gần `ax_max`
  (xem mục 1b). Đã thêm test hồi quy
  `test_front_axle_retains_lateral_capacity_at_max_acceleration`.
- Chạy lại kết quả closed-loop (mục 4) và sweep `Cf`/`Cr` (mục 5) trên mô
  hình đã sửa lỗi; kết luận cũ rút ra từ dữ liệu có lỗi (ví dụ "độ nhạy
  `Cf`/`Cr` thấp") đã được thay thế bằng kết luận mới, chính xác hơn.
- Tổng cộng 14 test tự động, trong đó 8 test về Fiala, regularization tốc độ
  thấp, chuyển tải và friction ellipse (mục 2).

## 7. Việc chưa làm / cần rà soát tiếp

- `Cf=Cr=80000 N/rad` là độ cứng **cả cầu** được công bố trong bộ sedan
  tham chiếu Rajamani (2012, ch. 3), không phải giá trị giả định/tự đặt. Giới
  hạn còn lại là chưa hiệu chuẩn bộ sedan tham chiếu theo một xe cụ thể; khi
  trích dẫn học thuật cần ghi đúng ấn bản và trang/bảng của nguồn.
- `mu=0.9` là giả định mô phỏng có đối chiếu khoảng kiểm định Taylor (2015),
  không phải phép đo mặt đường; `h=0.55 m` cũng là giả định minh họa.
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
- `v_epsilon=0.5 m/s` được chọn để regularize số học, chưa được nhận dạng từ
  dữ liệu lốp; cần sweep ngưỡng và xác nhận bằng dữ liệu nếu kết quả vùng tốc
  độ thấp có ý nghĩa định lượng.
