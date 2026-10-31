# Mô hình plant và lốp (nhánh `plant-tire`)

Tài liệu này mô tả phần plant/lốp đã sửa trong [vehicle_model.py](./vehicle_model.py), lý do sửa, cách kiểm chứng và những điểm còn mở. Công thức được mô tả bằng lời.

## 1. Vấn đề trước khi sửa

- Plant là bicycle động lực học phi tuyến 6 trạng thái (vị trí X, Y, góc yaw, vận tốc dọc, vận tốc ngang, yaw rate) với lực lốp **tuyến tính theo góc trượt**.
- Không có giới hạn ma sát: lực lốp tăng vô hạn theo góc trượt, nên xe có thể "bám" vượt khả năng vật lý.
- NMPC, CRI và mô phỏng đều dùng chung plant này, nên cả bộ điều khiển lẫn bộ đánh giá đều tin vào khả năng bám không có thật. Đây là gốc của phản biện số 2 (gia tốc ngang 16–29 m/s²).
- Gia tốc ngang trong log trước đây chỉ là ước lượng động học (bình phương tốc độ nhân tang góc lái chia chiều dài cơ sở), không phản ánh lực lốp.

## 2. Mô hình lốp mới

Có hai chế độ, chọn qua tham số `tire_model` trong `VehicleParams`:

| Chế độ | Ý nghĩa |
|---|---|
| `saturated` (mặc định) | Lực ngang bão hòa theo giới hạn vòng tròn ma sát |
| `linear` | Mô hình cũ, giữ lại để đối chiếu/tái lập kết quả cũ |

### Cách hoạt động của chế độ `saturated`

1. **Tải trọng tĩnh từng trục** được tính từ khối lượng và vị trí trọng tâm (trục trước chịu tải theo khoảng cách từ trọng tâm đến trục sau, và ngược lại). Chưa xét chuyển tải khi tăng/giảm tốc hoặc vào cua.
2. **Giới hạn lực ngang mỗi trục** bằng tải trục nhân với hệ số bám, rồi giảm theo mức lực dọc đang dùng (vòng tròn ma sát): lực dọc càng lớn thì lực ngang còn lại càng nhỏ. Lực dọc được chia cho hai trục theo tỷ lệ tải trọng.
3. **Lực ngang** là hàm tanh của góc trượt: ở góc trượt nhỏ trùng với lốp tuyến tính (cùng độ cứng Cf, Cr), ở góc trượt lớn bão hòa đúng bằng giới hạn ở bước 2. Hàm này trơn và đạo hàm liên tục nên phù hợp với NMPC.
4. **Gia tốc dọc hiệu dụng** bị chặn ở một tỷ lệ của khả năng bám (`long_grip_frac` = 0.9 của hệ số bám nhân g). Lý do: lệnh gia tốc tối đa của NMPC là 8 m/s² gần bằng giới hạn bám (khoảng 7.85 m/s²), nếu để dùng hết thì lực ngang còn lại bằng 0 và mô hình mất ý nghĩa.

### Tham số mới

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| `tire_model` | `saturated` | hoặc `linear` |
| `mu` | 0.8 | nhựa đường khô, **giả định**, cần nguồn trích dẫn |
| `g` | 9.81 | |
| `long_grip_frac` | 0.9 | **lựa chọn kỹ thuật**, cần giải thích/độ nhạy |

## 3. Các hàm mới

- Gia tốc ngang thân xe tính từ lực lốp (thay cho ước lượng động học).
- Mức sử dụng độ bám: tỷ lệ giữa lực tuyến tính mà lệnh yêu cầu và lực tối đa của trục chặt nhất. Lớn hơn 1 nghĩa là lốp đã bão hòa, lệnh vượt khả năng. Đây là ứng viên làm tín hiệu khả thi cho CRI ở bước sau.
- Giới hạn lực ngang theo trục, góc trượt và gia tốc dọc hiệu dụng: tách riêng để NMPC/CRI dùng lại.

Mô phỏng [simulation.py](./simulation.py) ghi thêm hai trường: `lat_accel_dyn` (gia tốc ngang từ lực lốp) và `tire_util` (mức sử dụng độ bám). Trường `lat_accel_kin` cũ vẫn còn để đối chiếu.

## 4. Kiểm chứng

[test_tire.py](./test_tire.py) có 7 test, cùng 6 test mô phỏng, tổng 13 test đều pass:

- Góc trượt nhỏ: lực bão hòa trùng lực tuyến tính.
- Lực ngang không vượt giới hạn ma sát ở mọi góc lái và mức gia tốc dọc.
- Gia tốc ngang từ lực lốp không vượt hệ số bám nhân g.
- Chế độ tuyến tính thực sự vượt giới hạn ma sát (xác nhận lỗi cũ).
- Tăng gia tốc dọc làm giảm lực ngang cho phép.
- Gia tốc dọc hiệu dụng bị giới hạn bởi độ bám.
- Mức sử dụng độ bám nhỏ hơn 1 với lệnh nhẹ và lớn hơn 1 với lệnh lái gấp.

## 5. Kết quả so sánh sơ bộ (một kịch bản, một seed)

Kịch bản `lateral_zigzag`, tốc độ target 17 m/s, seed 1, cùng bộ điều khiển và tham số cũ:

| | Tuyến tính | Bão hòa |
|---|---|---|
| Số bước NMPC / PP / mất bám | 13 / 28 / 115 | 21 / 20 / 115 |
| Khoảng cách tối thiểu | 5.26 m | 0.89 m |
| Gia tốc ngang từ lực lốp, lớn nhất / trung bình | 4.93 / 0.62 m/s² | 6.02 / 1.27 m/s² |
| Tỷ lệ bước lốp bão hòa | không có ý nghĩa* | 26% |

\* Với chế độ tuyến tính, giới hạn ma sát không áp dụng vào động lực học nên tỷ lệ bão hòa chỉ mang tính tham khảo.

**Cách đọc:** đây chỉ là một lần chạy, **không phải bằng chứng thống kê**. Điều quan trọng là: khi lốp có giới hạn, hành vi vòng kín thay đổi rõ (khoảng cách tối thiểu rút từ 5.26 m xuống 0.89 m), và hơn một phần tư số bước lốp đã bão hòa dù lệnh vẫn nằm trong ràng buộc. Điều này xác nhận các đánh giá CRI trước đây dựa trên plant sai khả năng bám.

Lưu ý: gia tốc ngang trong số liệu cũ của bài báo (16–29 m/s²) đến từ cách tính khác nên không so sánh trực tiếp được với cột trên. Cần chạy lại các bảng bằng plant mới và định nghĩa gia tốc ngang thống nhất.

## 6. Hạn chế đã biết

- Chưa có chuyển tải dọc/ngang; tải trục là tĩnh.
- Mô hình lốp tanh là xấp xỉ, không phải Pacejka. Chưa có ghép lực dọc–ngang chi tiết theo từng bánh.
- Mô hình bicycle gộp hai bánh mỗi trục.
- Phương trình vận tốc dọc chưa có thành phần lực ngang trục trước chiếu lên phương dọc.
- Hệ số bám và `long_grip_frac` chưa có nguồn hay phân tích độ nhạy.
- Giới hạn lệnh gia tốc tối đa 8 m/s² trong ràng buộc điều khiển vẫn lớn hơn khả năng bám; hiện được chặn ngầm bên trong plant. Cần quyết định có hạ ràng buộc NMPC cho khớp không.
- NMPC chưa có ràng buộc gia tốc ngang hay giới hạn độ bám tường minh; hiện chỉ "biết" qua plant bão hòa.

## 7. Cần làm rõ / quyết định

1. Giá trị hệ số bám và nguồn trích dẫn (khô/ướt?). Nên chạy độ nhạy cho vài giá trị.
2. Có hạ giới hạn gia tốc dọc của NMPC xuống phù hợp độ bám không (ảnh hưởng đến điểm phản biện số 4).
3. Có thêm chuyển tải tĩnh/động không, hay chấp nhận tải tĩnh và nêu rõ là giả định.
4. Định nghĩa thống nhất của "gia tốc ngang" khi báo cáo (từ lực lốp hay động học).
5. Dùng mức sử dụng độ bám làm một thành phần của cổng khả thi cứng trong CRI.

## 8. Bước tiếp theo

1. Thêm kiểm tra khả thi chung (gia tốc ngang, độ bám) dùng cho NMPC, PP và CRI.
2. Đưa tín hiệu bão hòa vào thiết kế CRI mới.
3. Chạy lại baseline với nhiều seed/kịch bản bằng plant mới, đặt kết quả cạnh bản tuyến tính để trả lời phản biện số 2.
