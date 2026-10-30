# Project Development Log
## Online Self-Calibrating Steering Control Using RLS with IMU Feedback

> **Project:** AutoCar III — IMU Self-Calibration Research  
> **Bắt đầu:** 2026-05-26  
> **Mục tiêu:** Conference paper cho hội thảo khoa học quốc tế (trong nước)  
> **Platform:** Hanback AutoCar III

---

## Mục Lục

- [Session 1 — Khởi tạo ý tưởng & Lập kế hoạch](#session-1--2026-05-26-2057---2130)
- [Session 2 — Offline Analysis Pipeline](#session-2--2026-05-27-2130---2200)

---

## Session 1 — 2026-05-26, 20:57 – 21:30

### 1.1 Đánh giá ý tưởng ban đầu (20:57)

**Yêu cầu ban đầu từ người dùng:**
- Hướng nghiên cứu: Tự hiệu chuẩn lái dựa trên IMU
- Ý tưởng: Từ notebook wheel alignment (8.1) và pose control (8.2), xây bộ điều khiển tự học quan hệ steering-yaw/gyro online
- Thực nghiệm: So sánh linear regression hiện tại, PID, RLS, MPC nhẹ; đo yaw error và path deviation

**Phân tích đã thực hiện:**

1. **Khảo sát codebase** — Đọc toàn bộ 26 notebook trong `samplecode/`:
   - `8.1 휠 얼라인먼트.ipynb` — Wheel Alignment: sweep steering -0.9→1.0, đọc Euler yaw, fit Linear Regression offline (5000 epochs)
   - `8.2 차량 자세 제어.ipynb` — Pose Control: thu thập cả Gyro-Z lẫn Euler yaw, 2 mô hình LR, vòng điều khiển real-time
   - `3. AIoT AutoCarⅢ 제어.ipynb` — Khảo sát API phần cứng: getAccel, getGyro, getEuler, getMagnetic, getGravity, getUltrasonic

2. **Survey landscape nghiên cứu 2024–2025:**
   - RLS cho steering dynamics đã có (KSAE 2024, MDPI journals)
   - IMU-based yaw rate calibration với factor graph optimization đã có
   - **Gap tìm được:** So sánh systematic online vs offline trên educational platform, forgetting factor analysis, multi-sensor IMU fusion

3. **Đánh giá tổng quan:** 4/5 ⭐ — Đủ tốt, cần điều chỉnh framing

**Quyết định thiết kế quan trọng:**
- Frame RLS online là contribution chính (không phải chỉ so sánh)
- Thêm perturbation test để chứng minh adaptive advantage
- Phân tích forgetting factor λ là contribution phụ
- Dùng cả Gyro-Z và Euler-yaw cho multi-sensor fusion

**Output:** `research_evaluation.md` (artifact)

---

### 1.2 Xác nhận tiêu đề & Tạo research plan (21:05)

**Tiêu đề được duyệt:**
> *"Online Self-Calibrating Steering Control Using Recursive Least Squares with IMU Feedback on a Small-Scale Autonomous Vehicle"*

**Tạo file:** `research_plan.md` — 14 phần chi tiết bao gồm:

| Phần | Nội dung |
|------|----------|
| §1 | Bối cảnh & động lực (steering misalignment problem) |
| §2 | 3 câu hỏi nghiên cứu (RQ1–RQ3) |
| §3 | 4 phương pháp so sánh với pseudo-code (LR, PID, RLS, MPC) |
| §4 | Kiến trúc hệ thống (system diagram) |
| §5 | 4 thí nghiệm với protocol chi tiết |
| §6 | Metrics & statistical testing |
| §7 | Cấu trúc thư mục project |
| §8 | Dự kiến 6 figures cho paper |
| §9 | Cấu trúc paper 7 sections |
| §10 | 3 contributions tuyên bố |
| §11 | Timeline 6 tuần (02/06 → 13/07) |
| §12 | Rủi ro & phương án dự phòng |
| §13 | Dependencies |
| §14 | References sơ bộ |

---

### 1.3 Code controllers (21:16)

**Thiết kế kiến trúc controller:**

Tất cả controllers kế thừa `BaseController` với interface thống nhất:
```
controller.update(car) → dict (telemetry record)
```

Mỗi `update()` tự động:
1. Đọc sensor từ car (gyro, euler, accel)
2. Fix Euler wrap-around (0°/360°)
3. Build feature vector
4. Gọi `compute_steering()` (abstract, mỗi controller implement riêng)
5. Apply gain × 1.5 + clip [-1, +1]
6. Ghi telemetry vào history

**Files đã tạo:**

```
src/
├── __init__.py
├── controllers/
│   ├── __init__.py                 (941 B)   — Package exports
│   ├── base_controller.py        (8,076 B)   — Abstract base class
│   ├── lr_controller.py          (6,000 B)   — Linear Regression offline
│   ├── pid_controller.py         (4,895 B)   — PID + anti-windup
│   ├── rls_controller.py         (9,362 B)   — RLS online ★
│   └── mpc_controller.py         (8,004 B)   — Lightweight MPC
└── utils/
    ├── __init__.py                  (16 B)
    ├── config.py                 (3,342 B)   — Hyperparameters tập trung
    ├── metrics.py                (6,797 B)   — 7 metrics computation
    └── logger.py                 (5,732 B)   — CSV + JSON logger
```

**Chi tiết từng controller:**

| Controller | Vai trò | Đặc điểm kỹ thuật |
|------------|---------|-------------------|
| `LRController` | Baseline offline | `calibrate(car)` tự sweep + OLS fit; `set_weights()` cho mock mode; model cố định sau deploy |
| `PIDController` | Baseline online (model-free) | Anti-windup clamp cho integral; EMA filter cho derivative; `ziegler_nichols(Ku, Tu)` helper |
| `RLSController` | **Contribution chính** | Forgetting factor λ; multi-sensor input [gyro, euler, accel, bias]; `warm_start()` từ LR; 2 supervision modes (`gyro_feedback`, `error_driven`); `get_convergence_time()`, `get_theta_trajectory()` diagnostics |
| `MPCController` | Upper-bound performance | Linearised bicycle model; dense QP (không cần cvxpy); slew-rate constraint; `update_dynamics()` hot-swap |

**Quyết định thiết kế đáng chú ý:**

1. **RLS supervision mode** — Đây là vấn đề khó: trong online operation, không có "ground truth" steering. Giải quyết bằng 2 mode:
   - `gyro_feedback`: target = 0 khi gyro_z = 0 (đi thẳng)
   - `error_driven`: correction nhỏ dựa trên yaw error, tích lũy dần

2. **MPC không dùng cvxpy** — Dense QP solve bằng `np.linalg.solve` để tránh dependency nặng trên embedded. Prediction matrices S_x, S_u build inline.

3. **Config tập trung** — Tất cả hyperparameters ở `config.py` để experiment scripts không hard-code giá trị.

**Test suite:** `tests/test_controllers.py` — 8 test cases với `MockCar` simulator:
- MockCar mô phỏng dynamics: `gyro_z = -8.0 × steering + 1.5 + noise`
- Test LR calibration R² > 0.7
- Test PID anti-windup
- Test RLS θ convergence + P trace shrinkage
- Test RLS λ sweep {0.90, 0.95, 0.99, 1.00}
- Test RLS warm-start vs cold-start
- Test MPC bounded output + finite cost
- Test metrics + logger pipeline end-to-end

> ⚠️ **Lưu ý:** Chưa chạy được test tự động do terminal sandbox không tìm thấy powershell PATH trên máy này. Cần chạy thủ công: `python tests/test_controllers.py`

---

### 1.4 Code data collection & experiments (21:23)

**Data collection — 2 files:**

| File | Class | Chức năng |
|------|-------|-----------|
| `sensor_preprocessor.py` | `SensorPreprocessor` | Bias calibration tĩnh (50 samples); EMA low-pass filter (α=0.3); Euler unwrap liên tục |
| `collector.py` | `SweepCollector` | Sweep steering tự động (matching notebook 8.1/8.2 protocol) |
| | `RunCollector` | Drive loop dưới controller + real-time logging + perturbation hook |
| | `execute_single_run()` | Bundle: drive → log CSV → compute metrics → save JSON |

**Experiments — 4 files + 1 master runner:**

| File | Experiment | Trả lời | Protocol |
|------|-----------|---------|----------|
| `exp1_straight_line.py` | Straight-Line Calibration | RQ1 | 4 controllers × 10 runs × 5s; so sánh MAE_ψ, RMSE_ψ |
| `exp2_lambda_sweep.py` | Forgetting Factor Sweep | RQ2 | RLS với λ ∈ {0.90–1.00} × 10 runs; plot convergence vs stability |
| `exp3_perturbation.py` | Perturbation Test ★ | RQ3 | 3s ổn định → thêm 200g → 5s recovery; `PerturbationManager` tự shift bias (mock) hoặc beep+prompt (hardware) |
| `exp4_multisensor.py` | Multi-Sensor Fusion | Bonus | RLS λ=0.95 với 3 config: Gyro-only / Euler-only / Full-IMU |
| `run_all_experiments.py` | Master runner | — | `--mock` / `--only exp3` / `--runs 2` / `--duration 2` |

**Thiết kế đáng chú ý:**

1. **`--mock` mode xuyên suốt** — Tất cả experiment đều hỗ trợ dry-run trên PC bằng `MockCar`, không cần phần cứng. Hữu ích để debug logic trước khi chạy thật.

2. **Perturbation simulation** — `PerturbationManager` trong exp3 có 2 mode:
   - Mock: tự động tăng `car.TRUE_BIAS += 3.0` tại t=3s
   - Hardware: `car.alarm()` beep + console prompt cho operator đặt vật nặng

3. **Mỗi experiment tự chứa** — Có thể chạy độc lập (`python -m src.experiments.exp1_straight_line --mock`) hoặc qua master runner.

4. **Output format thống nhất:**
   ```
   data/exp1/lr/LR-Offline_run00.csv        ← per-step telemetry
   data/exp1/lr/LR-Offline_run00_meta.json   ← metrics + config
   data/exp1/exp1_summary.json               ← aggregated results
   ```

---

## Tổng Kết Session 1

### Files đã tạo: 20 files

```
AutocarIII_Project/
├── research_plan.md                    ← Kế hoạch nghiên cứu 14 phần
├── log_project.md                      ← File này
├── run_all_experiments.py              ← Master experiment runner
│
├── src/
│   ├── __init__.py
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── base_controller.py          ← Abstract base (8.0 KB)
│   │   ├── lr_controller.py            ← Linear Regression baseline (6.0 KB)
│   │   ├── pid_controller.py           ← PID + anti-windup (4.9 KB)
│   │   ├── rls_controller.py           ← RLS online ★ contribution (9.4 KB)
│   │   └── mpc_controller.py           ← Lightweight MPC (8.0 KB)
│   │
│   ├── data_collection/
│   │   ├── __init__.py
│   │   ├── sensor_preprocessor.py      ← IMU conditioning (5.5 KB)
│   │   └── collector.py                ← Sweep + Run collectors (7.8 KB)
│   │
│   ├── experiments/
│   │   ├── __init__.py
│   │   ├── exp1_straight_line.py       ← Exp 1: Straight-line (8.2 KB)
│   │   ├── exp2_lambda_sweep.py        ← Exp 2: λ sweep (6.8 KB)
│   │   ├── exp3_perturbation.py        ← Exp 3: Perturbation (9.1 KB)
│   │   └── exp4_multisensor.py         ← Exp 4: Multi-sensor (7.0 KB)
│   │
│   └── utils/
│       ├── __init__.py
│       ├── config.py                   ← Tất cả hyperparameters (3.3 KB)
│       ├── metrics.py                  ← 7 metrics (6.8 KB)
│       └── logger.py                   ← CSV + JSON logger (5.7 KB)
│
├── tests/
│   └── test_controllers.py             ← 8 test cases (10.9 KB)
│
└── samplecode/                         ← Hanback gốc (26 notebooks, không thay đổi)
```

### Tổng dung lượng code mới: ~100 KB

---

## Session 2 — 2026-05-27, 21:30 – 22:00

### 2.1 Hoàn thiện offline analysis pipeline

Bổ sung 3 file mới vào `analysis/` (session trước đã có 3 file cơ bản: `plot_results.py`, `statistical_tests.py`, `generate_tables.py`).

---

**File 1: `analysis/data_explorer.py` — Exploratory Data Analysis (EDA)**

| Chức năng | Mô tả |
|-----------|-------|
| `check_data_integrity()` | Scan toàn bộ `data/` directory, kiểm tra file CSV/JSON có đủ không, có bị corrupt không, có NaN không |
| `print_run_summaries()` | In bảng per-run metrics (MAE, RMSE, compute time, steps) cho mỗi controller |
| `detect_outliers()` | Z-score outlier detection (threshold |z| > 2.5), flag run cần xem lại |
| `plot_diagnostics()` | 4-panel diagnostic plot cho mỗi controller: gyro traces, steering traces, gyro distribution, compute time distribution |
| `print_correlation_matrix()` | Ma trận correlation giữa MAE, RMSE, steady-state error, compute time |

Mục đích: Chạy **trước** khi phân tích chính thức, để phát hiện data issues sớm.

---

**File 2: `analysis/export_for_paper.py` — Paper Asset Exporter**

| Chức năng | Mô tả |
|-----------|-------|
| `export_figures()` | Copy all PDF figures → `paper/figures/` |
| `export_tables()` | Generate LaTeX tables (individual `.tex` files) → `paper/tables/` |
| `generate_bib()` | Tạo `references.bib` starter với 6 references quan trọng (Ljung, Haykin, Rajamani, Rawlings, Kim2024, Hanback manual) |
| `generate_paper_skeleton()` | Tạo `main.tex` IEEEtran conference format, 7 sections matching research plan, với `TODO` markers |
| `print_paper_checklist()` | Kiểm tra 13 assets cần thiết (figures + tables + tex + bib), in ✅/❌ |

Mục đích: One-command export toàn bộ assets cần thiết cho viết paper.

---

**File 3: `analysis/run_analysis.py` — Master Analysis Pipeline**

Orchestrate 5 bước phân tích theo thứ tự:

```
Step 1: explore   → Data integrity + EDA + outlier detection
Step 2: figures   → Generate 7 paper figures (or demo with synthetic data)
Step 3: stats     → Statistical significance tests (Shapiro-Wilk, t-test,
                     Wilcoxon, Cohen's d, Holm-Bonferroni, Spearman,
                     Kruskal-Wallis, Fisher exact)
Step 4: tables    → Generate Markdown + LaTeX tables
Step 5: export    → Copy figures/tables/bib → paper/
```

Features:
- `--demo` mode: tạo demo figures bằng synthetic data (không cần data thật)
- `--only figures`: chạy 1 bước duy nhất
- `--alpha 0.01`: thay đổi mức ý nghĩa

---

### 2.2 Review 3 files analysis có sẵn (từ Session 1)

| File | Lines | Chức năng | Trạng thái |
|------|-------|-----------|-----------|
| `plot_results.py` | 857 | 7 figure generators + 5 demo generators + CLI | ✅ Hoàn chỉnh |
| `statistical_tests.py` | 475 | 8 statistical tests cho 4 experiments + CLI | ✅ Hoàn chỉnh |
| `generate_tables.py` | 326 | 4 tables × 2 formats (Markdown + LaTeX) + CLI | ✅ Hoàn chỉnh |

---

## Tổng Kết Session 2

### Files mới tạo: 3 files | Tổng analysis: 6 files

```
analysis/
├── run_analysis.py              ← 🆕 Master pipeline (5 bước)
├── data_explorer.py             ← 🆕 EDA + integrity + outliers
├── export_for_paper.py          ← 🆕 Paper export + main.tex skeleton
├── plot_results.py              ← 7 figures + demo mode (31.8 KB)
├── statistical_tests.py         ← 8 stat tests (16.0 KB)
└── generate_tables.py           ← 4 tables × 2 formats (10.7 KB)
```

### Trạng thái tổng thể project

| Hạng mục | Trạng thái |
|----------|-----------|
| Research plan | ✅ Hoàn thành, đã duyệt |
| Controllers (LR, PID, RLS, MPC) | ✅ Code xong |
| Data collection (sweep, run, preprocess) | ✅ Code xong |
| Experiments (exp1–exp4) | ✅ Code xong |
| Unit tests (MockCar) | ✅ Code xong, chưa chạy verify |
| Analysis — Figure generation | ✅ Code xong (7 figures + demo mode) |
| Analysis — Statistical tests | ✅ Code xong (8 tests) |
| Analysis — Table generation | ✅ Code xong (4 tables × 2 formats) |
| Analysis — Data explorer | ✅ Code xong |
| Analysis — Paper export | ✅ Code xong (main.tex + bib + checklist) |
| Chạy thực nghiệm trên phần cứng | ⬜ Chưa bắt đầu |
| Paper writing | ⬜ Chưa bắt đầu |

### Cách sử dụng analysis pipeline

| Mục đích | Lệnh |
|----------|-------|
| **Demo figures** (xem trước mẫu) | `python analysis/run_analysis.py --demo` |
| **Full pipeline** (sau khi có data) | `python analysis/run_analysis.py --data data/` |
| **Chỉ vẽ figure** | `python analysis/run_analysis.py --data data/ --only figures` |
| **Chỉ chạy stats** | `python analysis/run_analysis.py --data data/ --only stats` |
| **Kiểm tra data** | `python analysis/data_explorer.py --data data/ --show-outliers --plot` |
| **Export cho paper** | `python analysis/export_for_paper.py --data data/ --output paper/` |

### Việc cần làm tiếp theo

1. **Chạy test suite** trên PC: `python tests/test_controllers.py`
2. **Dry-run mock** toàn bộ: `python run_all_experiments.py --mock --runs 2 --duration 2`
3. **Demo figures**: `python analysis/run_analysis.py --demo`
4. **Deploy lên AutoCar III** — Copy `src/` sang xe, cài numpy, matplotlib, scipy
5. **Chạy thực nghiệm thật** theo timeline tuần 1–4
6. **Chạy analysis pipeline** trên real data
7. **Viết paper** từ skeleton `paper/main.tex`

---

> *Log này được cập nhật sau mỗi session làm việc.*

