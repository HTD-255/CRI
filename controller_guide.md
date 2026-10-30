# Hướng dẫn các file controller trong dự án

Tài liệu này mô tả ngắn gọn cách hoạt động của các file điều khiển đang được dùng trong workspace hiện tại. Phần `NMPC` được bỏ ra theo yêu cầu.

## 1. Tổng quan luồng điều khiển

Mọi controller trong `src/controllers/` đều đi theo cùng một quy trình chung:

1. Đọc tín hiệu từ cảm biến trên xe: `gyro_z`, `euler_yaw`, `accel_y`.
2. `BaseController` chuẩn hóa dữ liệu, sửa lỗi vòng quay Euler quanh mốc `0/360`, và dựng vector đặc trưng.
3. Controller con tính giá trị lái thô trong `compute_steering()`.
4. `BaseController.update()` nhân gain, cộng trim/heading-hold nếu bật, rồi chặn trong giới hạn lái an toàn.
5. Kết quả được ghi vào `history` để phục vụ log, phân tích và vẽ đồ thị.

Điểm quan trọng là tất cả controller đều cùng giao tiếp qua một interface thống nhất: `update(car) -> dict`. Nhờ vậy các experiment có thể thay controller mà không phải sửa logic vòng lặp.

## 2. Vai trò từng file

### `src/controllers/base_controller.py`

Đây là lớp nền của toàn bộ hệ thống.

- Định nghĩa interface bắt buộc cho controller con.
- Cung cấp `update()` để gom toàn bộ luồng xử lý đầu vào/đầu ra.
- Xử lý bù wrap-around của `euler_yaw` khi góc đi qua mốc `360 -> 0`.
- Xây vector đặc trưng theo cấu hình cảm biến đang bật.
- Quản lý `history`, `reset()`, và các thông tin telemetry dùng cho phân tích offline.
- Có thêm hai lớp hiệu chỉnh chung:
  - `auto_trim`: bù lệch chạy thẳng theo phản hồi gyro.
  - `heading_hold`: giữ hướng ổn định quanh đường thẳng.

Nói ngắn gọn, file này không quyết định chiến lược điều khiển mà chỉ chuẩn hóa cách mọi controller chạy trong hệ thống.

### `src/controllers/lr_controller.py`

Đây là controller baseline kiểu học máy tuyến tính, nhưng học offline.

- `calibrate(car)` chạy một sweep góc lái, thu cặp `(sensor, steering)` rồi fit mô hình tuyến tính bằng OLS.
- Sau khi đã có `theta`, `update()` chỉ thực hiện phép nhân `x @ theta` để suy ra lệnh lái.
- `theta` không đổi trong lúc chạy, nên đây là controller không thích nghi online.
- Có `static_bias` để cộng bù lệch cơ khí sau khi đã có output từ mô hình.
- `estimate_static_bias()` dùng bài probe chạy thẳng để ước lượng trim lái cố định.

File này phù hợp để làm mốc so sánh cho các controller khác vì nó đơn giản, dễ hiểu và ổn định.

### `src/controllers/pid_controller.py`

Đây là controller phản hồi cổ điển, chạy online và không cần học mô hình.

- Sai số được tính từ `target_yaw_rate - gyro_z`.
- Thành phần P, I, D được tính rời rạc theo `dt`.
- Integral có chống windup bằng giới hạn biên.
- Derivative được lọc EMA để giảm nhiễu từ cảm biến.
- Kết quả cuối cùng có thể thêm `static_bias` để bù lệch cơ khí.

PID thường được dùng như baseline điều khiển truyền thống: đơn giản, phản ứng nhanh, nhưng không tự học tham số.

### `src/controllers/rls_controller.py`

Đây là controller quan trọng nhất của đề tài vì nó tự hiệu chỉnh online.

- Mô hình giả định là tuyến tính: `steer = theta^T x`.
- Mỗi bước chạy, controller vừa dự đoán steering vừa cập nhật `theta` bằng Recursive Least Squares.
- `forgetting_factor` `λ` quyết định mức độ quên dữ liệu cũ và thích nghi với điều kiện mới.
- `P` là ma trận hiệp phương sai, dùng để theo dõi độ tin cậy của tham số.
- Có hai chế độ tạo target huấn luyện:
  - `gyro_feedback`: dùng yaw-rate như tín hiệu phản hồi chính.
  - `error_driven`: dùng sai số hiện tại để kéo mô hình về trạng thái chạy thẳng.
- `warm_start()` cho phép khởi tạo từ `LRController` để hội tụ nhanh hơn.

Đây là file thể hiện đúng ý tưởng "tự hiệu chỉnh" của hệ thống: xe chạy đến đâu thì mô hình tự cập nhật đến đó.

### `src/controllers/mpc_controller.py`

Đây là controller tối ưu hóa dựa trên mô hình tuyến tính đơn giản.

- Dựng mô hình trạng thái dạng bicycle linearized với `yaw_error` và `yaw_rate`.
- Mỗi bước, controller giải bài toán tối ưu trong một horizon ngắn `N`.
- Hàm mục tiêu cân bằng giữa bám quỹ đạo, giảm yaw-rate và giảm effort lái.
- Có ràng buộc slew-rate để tránh đổi lái quá đột ngột.
- Với chạy thẳng, controller giữ đầu ra cẩn thận để tránh dao động nhỏ quanh 0.
- Kết quả trả về là steering đầu tiên trong chuỗi tối ưu, cộng thêm `static_bias` nếu có.

File này là lựa chọn “mạnh hơn” PID/LR vì có nhìn trước tương lai ngắn hạn, nhưng vẫn nhẹ hơn một NMPC đầy đủ.

### `src/controllers/__init__.py`

Đây là file xuất public API cho package controller.

- Tập trung import các controller chính để dùng thống nhất qua `from src.controllers import ...`.
- Giúp experiment script chỉ cần import một nơi thay vì phải nhớ từng file con.
- File tài liệu này chỉ mô tả các controller đang dùng thực tế trong luồng chính: `BaseController`, `LRController`, `PIDController`, `RLSController`, `MPCController`.

## 3. Controller nào được dùng trong experiment nào

- `exp1_straight_line.py`: so sánh `LR`, `PID`, `RLS`, `MPC` trên bài chạy thẳng.
- `exp2_lambda_sweep.py`: chỉ tập trung vào `RLS` với nhiều giá trị `lambda`.
- `exp3_perturbation.py`: so sánh khả năng hồi phục của `LR`, `PID`, `RLS`, `MPC` khi có nhiễu/perturbation.
- `exp4_multisensor.py`: so sánh `RLS` với các cấu hình cảm biến khác nhau.

## 4. Cách đọc telemetry khi debug

Khi gọi `update()`, mỗi controller đều trả về một dict có `clipped_steering` và các trường phụ trợ riêng.

- `pid_*`: phục vụ kiểm tra thành phần P/I/D.
- `lr_*`: phục vụ xem output từ mô hình tuyến tính và bias.
- `rls_*`: phục vụ theo dõi hội tụ tham số, ma trận `P`, và mức thay đổi `theta`.
- `mpc_*`: phục vụ xem cost, trạng thái dự đoán, và chuỗi input tối ưu.

Nếu cần debug nhanh, hãy nhìn theo thứ tự này:

1. Cảm biến đầu vào có hợp lý không.
2. `raw_steering` có đúng dấu không.
3. `gained_steering` sau gain có bị quá mạnh không.
4. `clipped_steering` có đang chạm biên không.

## 5. Tóm tắt ngắn

- `BaseController`: khung chung và xử lý hậu cần.
- `LRController`: học tuyến tính offline.
- `PIDController`: điều khiển phản hồi cổ điển.
- `RLSController`: tự hiệu chỉnh online, là trọng tâm của nghiên cứu.
- `MPCController`: tối ưu hóa ngắn hạn dựa trên mô hình tuyến tính.

Nếu bạn muốn, tôi có thể viết tiếp một bản ngắn hơn theo kiểu “README nội bộ” hoặc một bản chi tiết hơn theo kiểu tài liệu báo cáo khoa học.