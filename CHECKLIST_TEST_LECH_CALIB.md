# CHECKLIST TEST LỆCH CALIB VÀ CƠ KHÍ

## 1. Mục đích

Tài liệu này dùng để kiểm tra và khoanh vùng nguyên nhân khi xe:

- chạy thẳng ổn ở đầu đoạn nhưng lệch dần ở cuối đoạn
- không đánh lái nhiều nhưng vẫn bị sai số tích lũy
- có nghi ngờ về calib chưa ổn, lệch cơ khí, lệch servo hoặc drift cảm biến

Mục tiêu chính là tách rõ 3 nhóm nguyên nhân:

1. Lệch cơ khí cố định
2. Lệch thay đổi theo thời gian, tải trọng, tốc độ hoặc pin
3. Drift cảm biến IMU hoặc sai lệch heading measurement

## 2. Nguyên tắc thực hiện

1. Mỗi lần chỉ thay đổi 1 biến.
2. Lặp mỗi bài test tối thiểu 3 lần.
3. Giữ nguyên mặt sàn, pin, vị trí xuất phát và tải trọng nếu bài test không nhắm vào các biến đó.
4. Luôn ghi cả số đo tay và log cảm biến, không chỉ dựa vào cảm giác.
5. Nếu phát hiện lỗi cơ khí rõ ràng thì sửa cơ khí trước khi tune controller.

## 3. Dụng cụ cần chuẩn bị

- Xe AutoCar III
- Pin đủ điện cho cả buổi test
- Thước đo hoặc thước lá để đo độ lệch ngang
- Băng dính để đánh dấu đường tim và các mốc 1/3, 2/3, cuối đoạn
- Bút đánh dấu vị trí xuất phát
- Sổ tay hoặc bảng ghi kết quả
- Điện thoại hoặc camera nếu cần quay lại hành vi xe
- Mặt sàn phẳng, ít trượt, không có vật cản

## 4. Thông tin chung phải ghi cho mỗi run

Ghi đầy đủ các mục sau cho từng run:

1. Run ID
2. Ngày giờ
3. Tên bài test
4. Chiều chạy: thuận hoặc nghịch
5. Controller: open-loop, LR, PID, RLS, MPC
6. Tốc độ đặt
7. Mức pin hoặc điện áp pin
8. Mặt sàn
9. Cấu hình tải trọng
10. Vị trí xuất phát có giống run trước hay không
11. Hướng lệch: trái hoặc phải
12. Ghi chú bất thường: rung, servo hú, bánh bị cạ, xe bị va chạm nhẹ

## 5. Tiêu chí đánh giá tổng quát

Kỳ vọng xe ổn định khi:

1. Đẩy tay không motor mà vẫn giữ hướng gần thẳng.
2. Steering = 0 trả về tâm lặp lại được sau nhiều lần.
3. Open-loop drift nhỏ và không tăng vọt ở cuối đoạn.
4. Gyro bias đứng yên ổn định qua nhiều lần đo.
5. Chạy hai chiều không cho kết quả mâu thuẫn.
6. Tăng tốc độ không làm sai số bùng lên bất thường.
7. Controller chỉ cần bù nhẹ, không phải giữ lái quá mạnh.

## 6. Bài test 1 - Kiểm tra cơ khí thụ động

### Mục tiêu

Kiểm tra xem xe có lệch do hình học khung, bánh sau hoặc ma sát cơ khí hay không khi không có tác động từ motor và controller.

### Cách làm

1. Tắt motor hoặc đảm bảo xe không tự chạy.
2. Đặt bánh trước ở vị trí thẳng.
3. Đặt xe vào vạch xuất phát.
4. Đẩy xe bằng tay trên đoạn thẳng 2 đến 3 m.
5. Lặp lại ít nhất 3 lần.

### Thông số thực tế cần ghi

1. Chiều dài đoạn đẩy tay, đơn vị m.
2. Độ lệch ngang cuối đoạn, đơn vị cm.
3. Xe lệch trái hay phải.
4. Có rung, lắc, kéo nhẹ về một bên hay không.
5. Bánh nào có dấu hiệu ma sát bất thường.

### Mong muốn

- Xe đẩy tay vẫn đi gần thẳng.
- Độ lệch cuối đoạn rất nhỏ và lặp lại tương đối ổn định.

### Cách kết luận

- Nếu đẩy tay vẫn lệch rõ, ưu tiên sửa cơ khí trước.
- Nếu đẩy tay thẳng nhưng chạy motor mới lệch, nguyên nhân nghiêng về servo, tải động, nguồn hoặc controller.

## 7. Bài test 2 - Kiểm tra servo về tâm

### Mục tiêu

Kiểm tra steering = 0 có thực sự cho bánh trước về cùng một vị trí sau mỗi lần hay không.

### Cách làm

1. Bật xe nhưng không cho chạy tới.
2. Gửi lệnh steering = 0.
3. Quay bánh sang trái nhẹ, trả về 0.
4. Quay bánh sang phải nhẹ, trả về 0.
5. Lặp chu kỳ này ít nhất 5 lần.
6. Đánh dấu vị trí bánh hoặc đo khoảng cách tại mép vành.

### Thông số thực tế cần ghi

1. Số lần lặp.
2. Vị trí bánh khi về tâm mỗi lần.
3. Độ chênh lớn nhất giữa các lần, đơn vị mm hoặc độ.
4. Cảm giác độ rơ linkage hoặc backlash.
5. Có tiếng servo kêu bất thường hay không.

### Mong muốn

- Mỗi lần trả về 0 đều cho ra cùng một vị trí hoặc sai khác rất nhỏ.

### Cách kết luận

- Nếu vị trí về tâm không lặp lại, calibration một lần sẽ không đủ giá trị.
- Nếu tâm ổn định mà xe vẫn lệch, quay lại kiểm tra bánh sau, sensor và dynamic load.

## 8. Bài test 3 - Open-loop với steering = 0

### Mục tiêu

Đo sai lệch cơ bản của xe khi không có controller can thiệp, để phân biệt bias cố định và drift tăng dần theo thời gian.

### Cách làm

1. Đặt steering = 0 cố định.
2. Cho xe chạy cùng một tốc độ trên cùng một đoạn thẳng.
3. Đánh dấu mốc 1/3, 2/3 và cuối đoạn.
4. Ở mỗi run, ghi độ lệch tại 3 mốc này.
5. Lặp ít nhất 3 lần.

### Thông số thực tế cần ghi

1. Tốc độ đặt.
2. Thời gian chạy.
3. Độ lệch ngang tại mốc 1/3, đơn vị cm.
4. Độ lệch ngang tại mốc 2/3, đơn vị cm.
5. Độ lệch ngang cuối đoạn, đơn vị cm.
6. Hướng lệch trái hoặc phải.
7. Nếu có log, ghi gyro_z trung bình cả run.
8. Nếu có log, ghi gyro_z trung bình 2 giây cuối.

### Mong muốn

- Độ lệch nhỏ và tăng đều nếu xe có bias nhẹ.
- Không có kiểu ổn lúc đầu nhưng lệch vọt ở cuối đoạn.

### Cách kết luận

- Nếu độ lệch tăng gần tuyến tính, khả năng là bias cố định.
- Nếu độ lệch tăng mạnh về cuối đoạn, nghi ngờ drift do nhiệt, pin, ma sát, tải trọng hoặc sensor.

## 9. Bài test 4 - Chạy hai chiều ngược nhau

### Mục tiêu

Phân biệt lỗi do xe và lỗi do mặt sàn.

### Cách làm

1. Chạy một run theo chiều thuận.
2. Đặt lại xe ở đầu đoạn còn lại.
3. Chạy lại cùng điều kiện theo chiều nghịch.
4. Lặp ít nhất 3 cặp run.

### Thông số thực tế cần ghi

1. Chiều chạy.
2. Độ lệch tại 1/3, 2/3, cuối đoạn.
3. Hướng lệch tính theo thân xe.
4. Hướng lệch tính theo phòng hoặc mặt sàn.
5. Có khác biệt giữa hai chiều hay không.

### Mong muốn

- Nếu nền sàn tốt, kết quả hai chiều không được mâu thuẫn lớn.

### Cách kết luận

- Nếu xe luôn lệch cùng một phía theo thân xe, nghi cơ khí hoặc servo.
- Nếu xe luôn lệch cùng một phía của sàn dù đảo chiều chạy, nghi mặt sàn bị dốc hoặc ma sát không đều.

## 10. Bài test 5 - Drift cảm biến khi đứng yên

### Mục tiêu

Kiểm tra gyro bias và accel bias có ổn định theo thời gian hay không.

### Cách làm

1. Đặt xe đứng yên trên mặt sàn phẳng.
2. Chạy calibrate bias lần 1.
3. Ghi kết quả.
4. Chờ 3 đến 5 phút để xe và IMU nóng lên.
5. Chạy calibrate bias lần 2.
6. Ghi kết quả và so sánh.

### Thông số thực tế cần ghi

1. Gyro bias lần 1.
2. Gyro std lần 1.
3. Accel bias lần 1.
4. Accel std lần 1.
5. Gyro bias lần 2.
6. Gyro std lần 2.
7. Accel bias lần 2.
8. Accel std lần 2.
9. Chênh lệch gyro bias giữa hai lần.

### Mong muốn

- Bias thay đổi rất nhỏ giữa các lần đo.
- Độ lệch chuẩn không quá lớn khi xe đứng yên.

### Cách kết luận

- Nếu bias đổi rõ sau vài phút, cần nghi drift nhiệt của IMU.
- Nếu std lớn khi đứng yên, sensor đang nhiễu hoặc lắp đặt chưa ổn.

## 11. Bài test 6 - Sweep theo tốc độ

### Mục tiêu

Kiểm tra sai lệch có phụ thuộc động lực học và tải động theo tốc độ hay không.

### Cách làm

1. Chọn ít nhất 3 mức tốc độ, ví dụ 20, 30, 40.
2. Chạy cùng một bài test open-loop hoặc controller giới hạn.
3. Ghi cùng một nhóm thông số cho mỗi tốc độ.
4. Lặp lại ít nhất 3 run mỗi mức tốc độ.

### Thông số thực tế cần ghi

1. Tốc độ.
2. Độ lệch 1/3, 2/3, cuối đoạn.
3. Gyro_z trung bình nếu có log.
4. Gyro_z 2 giây cuối nếu có log.
5. Steering trung bình nếu có controller.
6. Có rung hoặc dao động tăng theo tốc độ hay không.

### Mong muốn

- Sai lệch không bùng lên bất thường khi tăng tốc độ.

### Cách kết luận

- Nếu tốc độ càng cao xe càng lệch mạnh, nghi geometry, servo dưới tải, ma sát bánh hoặc khung.
- Nếu sai lệch gần như không đổi theo tốc độ, nghi bias tĩnh.

## 12. Bài test 7 - Ảnh hưởng của pin

### Mục tiêu

Kiểm tra servo và động cơ có đổi đặc tính khi pin giảm hay không.

### Cách làm

1. Test một nhóm run khi pin đầy.
2. Test lại một nhóm run khi pin đã giảm rõ.
3. Giữ nguyên mặt sàn, tốc độ và tải trọng.

### Thông số thực tế cần ghi

1. Mức pin hoặc điện áp pin đầu bài.
2. Mức pin hoặc điện áp pin cuối bài.
3. Độ lệch cuối đoạn ở từng mức pin.
4. Gyro_z trung bình nếu có log.
5. Steering correction trung bình nếu có controller.
6. Có dấu hiệu servo yếu, chậm, hụt hoặc rung hay không.

### Mong muốn

- Kết quả không thay đổi nhiều giữa pin đầy và pin thấp vừa phải.

### Cách kết luận

- Nếu chỉ pin thấp mới lệch mạnh, cần xử lý nguồn trước khi kết luận về calibration.

## 13. Bài test 8 - Ảnh hưởng của tải trọng

### Mục tiêu

Kiểm tra xe có nhạy cảm với vị trí tải trọng hay không.

### Cách làm

1. Test không tải thêm.
2. Test với tải đặt giữa xe.
3. Test với tải lệch trái.
4. Test với tải lệch phải.
5. Giữ nguyên các điều kiện còn lại.

### Thông số thực tế cần ghi

1. Cấu hình tải trọng.
2. Khối lượng tải thêm.
3. Vị trí đặt tải.
4. Độ lệch 1/3, 2/3, cuối đoạn.
5. Hướng lệch.
6. Nếu có log, ghi thêm gyro_z và steering trung bình.

### Mong muốn

- Xe không đổi hướng lệch quá nhạy khi thay đổi nhẹ về tải trọng.

### Cách kết luận

- Nếu đổi tải là đổi pattern lệch rõ, khung xe hoặc phân bố trọng tâm đang là biến quan trọng.

## 14. Bài test 9 - So sánh controller trong cùng điều kiện

### Mục tiêu

Xác nhận lại xem lỗi nằm ở phần xe hay phần điều khiển.

### Cách làm

1. Chạy open-loop cùng điều kiện tham chiếu.
2. Chạy LR.
3. Chạy PID.
4. Chạy RLS.
5. Chạy MPC nếu cần.
6. Ghi cùng một bộ thông số cho mỗi phương án.

### Thông số thực tế cần ghi

1. Controller.
2. Độ lệch 1/3, 2/3, cuối đoạn.
3. Gyro_z trung bình cả run.
4. Gyro_z trung bình 2 giây cuối.
5. Steering trung bình.
6. Steering có chạm biên hay không.
7. Cảm giác xe có phải đang "gồng" để giữ thẳng hay không.

### Mong muốn

- Các controller tốt hơn open-loop một cách rõ ràng.
- Không controller nào phải sửa quá mạnh chỉ để che một lỗi cơ khí.

### Cách kết luận

- Nếu tất cả controller đều chung một pattern lệch cuối đoạn, lỗi gốc khả năng cao nằm ở phần cứng hoặc sensor.
- Nếu chỉ một controller bị, khi đó mới quay lại tune tham số.

## 15. Thứ tự thực hiện khuyến nghị

Để tiết kiệm thời gian, nên làm theo thứ tự sau:

1. Bài test 1 - Kiểm tra cơ khí thụ động
2. Bài test 2 - Kiểm tra servo về tâm
3. Bài test 4 - Chạy hai chiều ngược nhau
4. Bài test 5 - Drift cảm biến khi đứng yên
5. Bài test 3 - Open-loop với steering = 0
6. Bài test 6 - Sweep theo tốc độ
7. Bài test 7 - Ảnh hưởng của pin
8. Bài test 8 - Ảnh hưởng của tải trọng
9. Bài test 9 - So sánh controller trong cùng điều kiện

## 16. Mẫu bảng ghi tay để điền nhanh

Có thể in hoặc chép bảng dưới đây ra giấy:

| Run ID | Test | Direction | Controller | Speed | Battery | Load | Surface | Dev 1/3 (cm) | Dev 2/3 (cm) | Dev End (cm) | Drift Dir | Mean gyro_z | Tail gyro_z | Mean steer | Notes |
|---|---|---|---|---:|---|---|---|---:|---:|---:|---|---:|---:|---:|---|
| 01 | Open-loop | Thuận | None | 30 | Đầy | None | Gạch |  |  |  |  |  |  |  |  |
| 02 | Open-loop | Nghịch | None | 30 | Đầy | None | Gạch |  |  |  |  |  |  |  |  |
| 03 | Servo center | N/A | N/A | 0 | Đầy | None | Bàn test |  |  |  |  |  |  |  |  |

## 17. Mẫu kết luận sau khi test

Sau mỗi nhóm bài test, nên ghi kết luận ngắn theo form sau:

1. Hiện tượng quan sát được.
2. Biến giữ nguyên trong nhóm test này.
3. Biến đã thay đổi.
4. Kết quả đo tay.
5. Kết quả log cảm biến.
6. Nhận định tạm thời: cơ khí, sensor, servo, pin, tải trọng hay controller.
7. Hành động tiếp theo.

## 18. Dấu hiệu để ưu tiên sửa cơ khí trước

Nên dừng việc tune controller và sửa cơ khí trước nếu gặp các dấu hiệu sau:

1. Xe đẩy tay vẫn lệch rõ.
2. Bánh trước về tâm không lặp lại.
3. Hai chiều chạy cho kết quả đối lập theo mặt sàn.
4. Xe có rung, kéo, cạ bánh hoặc ma sát một bên.
5. Bias cần bù thay đổi mỗi run một cách lớn.

## 19. Dấu hiệu để ưu tiên kiểm tra sensor

Nên ưu tiên kiểm tra IMU và pipeline sensor nếu gặp các dấu hiệu sau:

1. Xe thực tế lệch nhưng gyro_z ghi nhận rất đẹp.
2. Gyro bias thay đổi rõ sau vài phút.
3. Std của gyro cao khi xe đứng yên.
4. Bật Euler vào thì kết quả xấu hơn rõ.

## 20. Dấu hiệu để ưu tiên xem lại controller

Chỉ nên ưu tiên tune controller khi:

1. Cơ khí và sensor đã qua được các bài test cơ bản.
2. Open-loop ổn định hơn trước nhưng controller vẫn over-correct.
3. Steering thường xuyên chạm biên.
4. Chỉ một controller có pattern lệch xấu hơn hẳn các controller còn lại.

## 14. Bai test 9 - So sanh controller trong cung dieu kien

### Muc tieu

Xac nhan lai xem loi nam o phan xe hay phan dieu khien.

### Cach lam

1. Chay open-loop cung dieu kien tham chieu.
2. Chay LR.
3. Chay PID.
4. Chay RLS.
5. Chay MPC neu can.
6. Ghi cung mot bo thong so cho moi phuong an.

### Thong so thuc te can ghi

1. Controller.
2. Do lech 1/3, 2/3, cuoi doan.
3. Gyro_z trung binh ca run.
4. Gyro_z trung binh 2 giay cuoi.
5. Steering trung binh.
6. Steering co cham bien hay khong.
7. Cam giac xe co phai dang "gong" de giu thang hay khong.

### Mong muon

- Cac controller tot hon open-loop mot cach ro rang.
- Khong controller nao phai sua qua manh chi de che mot loi co khi.

### Cach ket luan

- Neu tat ca controller deu chung mot pattern lech cuoi doan, loi goc kha nang cao nam o phan cung hoac sensor.
- Neu chi mot controller bi, khi do moi quay lai tune tham so.

## 15. Thu tu thuc hien khuyen nghi

De tiet kiem thoi gian, nen lam theo thu tu sau:

1. Bai test 1 - Kiem tra co khi thu dong
2. Bai test 2 - Kiem tra servo ve tam
3. Bai test 4 - Chay hai chieu nguoc nhau
4. Bai test 5 - Drift cam bien khi dung yen
5. Bai test 3 - Open-loop voi steering = 0
6. Bai test 6 - Sweep theo toc do
7. Bai test 7 - Anh huong cua pin
8. Bai test 8 - Anh huong cua tai trong
9. Bai test 9 - So sanh controller trong cung dieu kien

## 16. Mau bang ghi tay de dien nhanh

Co the in hoac chep bang duoi day ra giay:

| Run ID | Test | Direction | Controller | Speed | Battery | Load | Surface | Dev 1/3 (cm) | Dev 2/3 (cm) | Dev End (cm) | Drift Dir | Mean gyro_z | Tail gyro_z | Mean steer | Notes |
|---|---|---|---|---:|---|---|---|---:|---:|---:|---|---:|---:|---:|---|
| 01 | Open-loop | Thuan | None | 30 | Day | None | Gach |  |  |  |  |  |  |  |  |
| 02 | Open-loop | Nghich | None | 30 | Day | None | Gach |  |  |  |  |  |  |  |  |
| 03 | Servo center | N/A | N/A | 0 | Day | None | Ban test |  |  |  |  |  |  |  |  |

## 17. Mau ket luan sau khi test

Sau moi nhom bai test, nen ghi ket luan ngan theo form sau:

1. Hien tuong quan sat duoc.
2. Bien giu nguyen trong nhom test nay.
3. Bien da thay doi.
4. Ket qua do tay.
5. Ket qua log cam bien.
6. Nhan dinh tam thoi: co khi, sensor, servo, pin, tai trong hay controller.
7. Hanh dong tiep theo.

## 18. Dau hieu de uu tien sua co khi truoc

Nen dung viec tune controller va sua co khi truoc neu gap cac dau hieu sau:

1. Xe day tay van lech ro.
2. Banh truoc ve tam khong lap lai.
3. Hai chieu chay cho ket qua doi lap theo mat san.
4. Xe co rung, keo, cat banh hoac ma sat mot ben.
5. Bias can bu thay doi moi run mot cach lon.

## 19. Dau hieu de uu tien kiem tra sensor

Nen uu tien kiem tra IMU va pipeline sensor neu gap cac dau hieu sau:

1. Xe thuc te lech nhung gyro_z ghi nhan rat dep.
2. Gyro bias thay doi ro sau vai phut.
3. Std cua gyro cao khi xe dung yen.
4. Bat Euler vao thi ket qua xau hon ro.

## 20. Dau hieu de uu tien xem lai controller

Chi nen uu tien tune controller khi:

1. Co khi va sensor da qua duoc cac bai test co ban.
2. Open-loop on dinh hon truoc nhung controller van over-correct.
3. Steering thuong xuyen cham bien.
4. Chi mot controller co pattern lech xau hon han cac controller con lai.