# Research Plan đề xuất

## CRI được hiệu chuẩn cho chuyển đổi NMPC–PP trong mô phỏng sedan

**Trạng thái tài liệu:** Bản đề xuất để thảo luận với giảng viên hướng dẫn  
**Phạm vi:** Một xe sedan tham chiếu trong mô phỏng; phối hợp NMPC và Pure Pursuit (PP) bằng CRI; đánh giá giới hạn động lực học, chất lượng tương đối của hai ứng viên điều khiển và hiệu năng theo dõi/interception.  
**Lưu ý phạm vi:** Nếu giữ contact trong mô phỏng, contact chỉ là sự kiện terminal hình học. Kế hoạch không nghiên cứu mức độ va chạm, hư hại hoặc cách chuyển kết quả thành hành vi ngoài mô phỏng.

---

## 1. Tóm tắt đề xuất

Nghiên cứu đề xuất thiết kế lại cơ chế chuyển đổi giữa NMPC và Pure Pursuit (PP) cho bài toán xe sedan theo dõi/interception một mục tiêu trong mô phỏng. Cả hai bộ điều khiển tạo ứng viên điều khiển; CRI là cơ chế trung tâm để đánh giá ứng viên NMPC so với PP và quyết định mode nào nên được áp dụng. Điểm mới dự kiến là **CRI được định nghĩa lại như một ước lượng đã hiệu chuẩn về xác suất NMPC là lựa chọn phù hợp hơn PP trong một chân trời ngắn**, thay vì một tổng trọng số heuristic được diễn giải như độ tin cậy của toàn hệ thống.

CRI là thành phần điều phối chính, không phải một chỉ số chỉ để ghi log. Mỗi chu kỳ, hệ thống tính ứng viên NMPC và ứng viên PP; CRI so sánh chất lượng dự kiến của hai ứng viên, sau đó hysteresis/dwell-time quyết định mode đang hoạt động. Cổng kiểm tra solver và feasibility là lớp ràng buộc cứng bổ trợ: nó không thay CRI trong các quyết định bình thường, nhưng có thể từ chối ứng viên không hợp lệ. Sai số giữa trạng thái dự đoán và trạng thái quan sát ở bước kế tiếp được đưa vào đánh giá CRI ở các chu kỳ sau.

Song song, mô hình sedan và cách tính gia tốc được rà soát để bảo đảm các kết quả có ý nghĩa vật lý. Code hiện dùng mô hình động lực học bicycle phi tuyến với luật lực lốp Fiala cho mô phỏng và dự đoán NMPC. Luật Fiala bão hòa lực ngang theo tải pháp tuyến tĩnh và hệ số ma sát giả định; cần thẩm định miền hợp lệ, độ nhạy và sai số mô hình. Mô hình chưa áp dụng tải động hay giới hạn lực dọc-ngang kết hợp; không phải toàn bộ mô hình xe là tuyến tính và không mặc định phải thay bằng mô hình bốn bánh.

## 2. Bối cảnh và động lực

Phiên bản mô phỏng hiện tại có các thành phần của một hệ thống hybrid: NMPC, PP, CRI, hysteresis, UKF và cảm biến giới hạn FOV. NMPC và PP là hai mode của cùng bộ điều khiển hybrid; CRI đánh giá chuyển đổi giữa hai mode, không phải chỉ giám sát riêng nghiệm NMPC. Tuy nhiên, các đánh giá phản biện đặt ra ba vấn đề liên quan:

1. CRI hiện là tổng trọng số của các chỉ số heuristic; chưa chứng minh rằng điểm CRI phân biệt đúng thời điểm NMPC tốt hơn PP trong closed loop.
2. Mô hình lốp tuyến tính và giới hạn đầu vào dạng hộp chưa đủ để giải thích các gia tốc ngang rất lớn hoặc xác nhận tính khả thi vật lý.
3. Luật hậu xử lý có thể làm lệnh thực tế khác với lệnh mà CRI/NMPC đã đánh giá.

Vì vậy, nghiên cứu không chỉ điều chỉnh trọng số CRI. Cần xác định một ngữ nghĩa đo lường rõ ràng cho CRI, tích hợp nó thành một phần có ảnh hưởng thực sự tới quyết định điều khiển, và đánh giá đóng góp của nó bằng dữ liệu tách biệt khỏi dữ liệu tuning.

### 2.1 Hiện trạng mã liên quan

- [`cri.py`](./cri.py): CRI hiện cộng bốn thành phần optimality, feasibility, superiority và smoothness theo trọng số cố định. Feasibility hiện tập trung vào giới hạn trạng thái/điều khiển dạng hộp; superiority dùng dự đoán giữ điều khiển không đổi.
- [`hysteresis.py`](./hysteresis.py): đã có hai ngưỡng hysteresis và dwell theo số lần gọi `decide`, nhưng chưa có đánh giá dwell-time thực tế hoặc xử lý đầy đủ khi mất track.
- [`NMPC.py`](./NMPC.py): solver hiện dùng projected gradient với phép chiếu lên ràng buộc hộp; chưa có xử lý trực tiếp các ràng buộc phi tuyến về lực lốp.
- [`vehicle_model.py`](./vehicle_model.py): code đã có mô hình động lực học bicycle phi tuyến/single-track. Động lực học có các thành phần phi tuyến như lượng giác, arctan và tích trạng thái; lực lốp dùng luật Fiala bão hòa theo tải pháp tuyến tĩnh, chưa có tải động hoặc giới hạn ma sát kết hợp.
- [`main.py`](./main.py): mode quyết định chọn trực tiếp NMPC hoặc PP; chưa thấy blending. Lệnh gia tốc được hậu xử lý sau lựa chọn mode, vì vậy lệnh được CRI đánh giá có thể khác lệnh áp dụng.
- [`main.py`](./main.py): gia tốc ngang được ghi bằng công thức động học dựa trên tốc độ, góc lái và chiều dài cơ sở; cần phân biệt nó với gia tốc ngang tính từ phương trình lực của mô hình.
- [`metric.py`](./metric.py): đã có PEI, khoảng cách nhỏ nhất và proximity-time; chưa có nhãn đánh giá so sánh chất lượng closed-loop của hai mode NMPC và PP trong một cửa sổ tương lai.

Các cấu hình tạo ra toàn bộ bảng trong bản thảo cần được truy vết riêng trước khi xem bất kỳ kết quả nào là baseline tái lập.

## 3. Phạm vi và giả định nghiên cứu

### 3.1 Phạm vi

- Một sedan tham chiếu trong mô phỏng, không tuyên bố đại diện cho tất cả phân khúc xe.
- Hệ hybrid gồm NMPC và PP; PP không chỉ là xử lý lỗi mà là mode điều khiển thay thế được CRI so sánh và chọn trong quá trình vận hành.
- Target được mô tả bằng một mô hình chuyển động đã công bố trong bài; giả định về gia tốc, khả năng đổi hướng và khả năng quan sát cần được nêu rõ.
- Mô phỏng cảm biến/UKF được giữ trong phạm vi nếu nó ảnh hưởng tới CRI thông qua độ bất định trạng thái.
- Nếu contact được giữ làm sự kiện của mô phỏng, phát hiện contact theo footprint hai vật thể và dừng mô phỏng tại sự kiện đầu tiên. Sự kiện proximity khi khoảng cách giữa hai xe dưới 4 m vẫn được lưu như metric cũ để đối chiếu.
- Không đánh giá mức độ hư hại, lực va chạm, chấn thương hoặc cách triển khai ngoài mô phỏng.

### 3.2 Giả định cần xác nhận

1. Với mô hình bicycle phi tuyến hiện có, cần bổ sung tire-force saturation/friction constraint ở mức nào để đủ trả lời reviewer? Có cần plant tham chiếu chi tiết hơn để xác nhận kết quả không?
2. Có nguồn tham số sedan/lốp đủ tin cậy cho mô hình hiện tại hay cần dùng bộ tham số tham chiếu từ tài liệu?
3. Hàm mục tiêu của NMPC ưu tiên những đại lượng nào và vì sao?
4. CRI sẽ đánh giá lợi thế tương đối giữa NMPC và PP theo cửa sổ bao nhiêu bước/thời gian?
5. Phạm vi chứng minh phù hợp là khả thi ràng buộc, boundedness/practical stability hay chỉ đánh giá thực nghiệm?

## 4. Mục tiêu nghiên cứu

### Mục tiêu tổng quát

Thiết kế và đánh giá một bộ điều khiển hybrid NMPC–PP trong đó CRI là cơ chế trung tâm đánh giá và quyết định chuyển mode dựa trên chất lượng tương đối quan sát/ước lượng được, đồng thời các kiểm tra vật lý cứng vẫn được thực thi độc lập.

### Mục tiêu cụ thể

1. Xây dựng plant sedan và cách tính gia tốc/lực lốp nhất quán với phạm vi mô phỏng.
2. Định nghĩa nhãn so sánh NMPC với PP và đặc trưng cho CRI sao cho có thể đo calibration trên tập kiểm tra độc lập.
3. Thiết kế chuyển mode trong đó CRI quyết định chuyển đổi thông thường, đi cùng cổng feasibility cứng, hysteresis và dwell-time theo thời gian.
4. Định lượng CRI có cải thiện quyết định chuyển mode của hybrid NMPC–PP so với ngưỡng đơn giản hoặc không dùng CRI hay không.
5. Nêu và kiểm tra rõ điều kiện áp dụng của mọi tuyên bố lý thuyết.

## 5. Câu hỏi nghiên cứu và giả thuyết

| Mã | Câu hỏi nghiên cứu | Giả thuyết cần kiểm tra |
|---|---|---|
| RQ1 | CRI đã hiệu chuẩn có dự đoán được khi nào NMPC là lựa chọn phù hợp hơn PP trong chân trời ngắn không? | CRI có calibration tốt hơn một điểm heuristic chưa hiệu chuẩn trên các kịch bản/seed giữ lại để kiểm tra. |
| RQ2 | Chuyển mode dựa trên CRI có cải thiện hệ hybrid NMPC–PP so với ngưỡng đơn giản hoặc không dùng CRI không? | Với cùng các giới hạn cứng, CRI giảm số lần chọn mode kém phù hợp mà không khiến hệ thống gần như luôn ở một mode. |
| RQ3 | Độ bất định xe, lốp và cảm biến ảnh hưởng thế nào tới calibration và quyết định mode? | Calibration suy giảm khi phân phối kiểm tra lệch khỏi tập huấn luyện; mức suy giảm đo được qua các kịch bản độ nhạy. |
| RQ4 | Dwell-time và hysteresis ảnh hưởng thế nào đến chattering và hiệu năng? | Dwell/hysteresis giảm chuyển mode ngắn hạn; tác động tới hiệu năng được đo riêng và không được suy diễn thành chứng minh ổn định. |

Các giả thuyết trên là giả thuyết thực nghiệm; không được xem là kết quả đã biết trước.

## 6. Methodology đề xuất

### 6.1 Mô hình sedan và kiểm chứng vật lý

#### Chọn cấu trúc mô hình

- **Mô hình hiện có:** giữ làm baseline là động lực học bicycle phi tuyến; mô tả đúng trạng thái, phương trình động lực học và luật lực lốp đang được triển khai.
- **Vấn đề cần xử lý:** luật lực lốp Fiala bão hòa theo tải pháp tuyến tĩnh và hệ số ma sát giả định. Cần kiểm chứng miền hợp lệ, độ nhạy tham số và xem xét tải động/lực dọc kết hợp; mô hình này chưa phải friction circle/ellipse.
- **Mô hình dự đoán NMPC:** nếu khác plant dùng đánh giá, định lượng sai số dự đoán và ảnh hưởng tới quyết định CRI; nếu dùng cùng mô hình, vẫn cần kiểm tra tính hợp lệ của tire model ở các trạng thái đã chạy.
- Không gọi bicycle model là mô hình tuyến tính toàn xe. Cần phân biệt rõ “động lực học bicycle phi tuyến” với “luật lực lốp Fiala có bão hòa”.

#### Nguồn tham số

Mỗi tham số phải được đánh dấu thuộc một trong các nhóm:

1. đo/hiệu chuẩn;
2. trích từ tài liệu hoặc bộ benchmark;
3. giả định mô phỏng.

Với tham số bất định, khai báo một miền hợp lý và dùng trong phân tích độ nhạy. Không chọn một giá trị chỉ vì nó tạo ra kết quả tốt.

**Rà soát hiện trạng (sau khi chuyển sang luật lốp Fiala):**

- `m=1575 kg` và `Iz=2875 kg*m^2` gần khớp với ví dụ sedan tham chiếu trong **Rajesh Rajamani**, *Vehicle Dynamics and Control* (Springer, 2012, chương 3), trong đó `m=1573 kg`, `Iz=2873 kg*m^2` được mô tả là đại diện cho một sedan hành khách. Có thể xếp hai tham số này vào nhóm (2).
- `lf=1.2 m`, `lr=1.3 m`, `Cf=19000 N/rad`, `Cr=33000 N/rad` **không khớp** với giá trị tương ứng của cùng nguồn đó (**Rajamani**, `lf=1.1`, `lr=1.58`, `Cf=Cr=80000 N/rad`). Không được coi `Cf`/`Cr` hiện tại là đã có nguồn gốc cùng ví dụ với `m`/`Iz`. Cần làm một trong hai việc trước khi báo cáo: (a) tìm nguồn riêng hợp lý cho `lf`, `lr`, `Cf`, `Cr` và ghi rõ nguồn, hoặc (b) xếp các tham số này vào nhóm (3) giả định mô phỏng và chạy phân tích độ nhạy, không trình bày như giá trị đã được xác thực.
- `mu=0.9` đối chiếu được với khoảng giá trị `Umin=0.9`/`Umax=1.0` dùng trong **Michael Taylor**, *Implementation and Validation of the Fiala Tire Model in Chrono*, Technical Report TR-2015-13 (2015), nhưng nguồn đó dùng giá trị này để kiểm định triển khai mô hình Fiala, không phải để mô tả một cặp lốp-mặt đường cụ thể. Xếp vào nhóm (3) giả định mô phỏng, có đối chiếu độ lớn với tài liệu.
- `h=0.55 m` (chiều cao trọng tâm, dùng cho chuyển tải dọc) là giá trị minh họa nằm trong khoảng điển hình của sedan hành khách, chưa có nguồn riêng. Xếp vào nhóm (3) giả định mô phỏng.
- Đã chạy phân tích độ nhạy `Cf`/`Cr` (sweep 0.5×–4× giá trị hiện tại, cộng mốc so sánh `Cf=Cr=80000 N/rad` của Rajamani) trên kịch bản closed-loop đầy đủ; kết quả chi tiết ở [FIALA_TIRE_MODEL_RESULTS.md](./FIALA_TIRE_MODEL_RESULTS.md) mục 5. Kết quả **không thay thế** việc cần một nguồn đo thực tế cho `Cf`/`Cr`, chỉ cho biết mức độ nhạy của các chỉ tiêu đã chọn trong kịch bản đã chạy.

#### Lực lốp và gia tốc

- Đánh giá riêng luật Fiala đã triển khai so với luật lực lốp tuyến tính trước đây; xác định độ nhạy theo độ cứng góc, tải pháp tuyến và hệ số ma sát.
- **Đã triển khai** giới hạn lực kết hợp cho mỗi bánh bằng friction ellipse (`DynamicBicycleModel._max_lateral_friction`): tổng hợp lực dọc (ước lượng từ `ax` theo tỉ lệ tải tĩnh, giả định đơn giản hóa cần rà soát) và lực ngang tại cầu không vượt quá `mu*Fz`. Tải pháp tuyến cũng đã chuyển sang động (`normal_loads`, có chuyển tải dọc theo `h`) thay vì tĩnh như trước. Xem chi tiết công thức, test và dữ liệu minh họa ở [FIALA_TIRE_MODEL_RESULTS.md](./FIALA_TIRE_MODEL_RESULTS.md) mục 3–4.
- Việc bật friction ellipse + tải pháp tuyến động đã làm thay đổi đáng kể hành vi closed-loop so với bản chỉ có Fiala thuần túy (ví dụ: không còn xảy ra sự kiện tới gần mục tiêu trong kịch bản mặc định đã chạy thử) — cần coi đây là phát hiện cần điều tra thêm (tinh chỉnh kịch bản/tham số), không phải lỗi cần ẩn đi.
- Xác định cách tính tải pháp tuyến và hệ số ma sát từ mô hình/giả định; không đưa ràng buộc này vào bài như thể đã dùng nếu solver hoặc plant không thực thi nó.
- Báo cáo riêng gia tốc ngang động học và gia tốc ngang từ phương trình động lực học. Với mô hình bicycle hiện tại, gia tốc ngang động lực học được tính bằng tổng của đạo hàm vận tốc ngang trong hệ thân xe và tích giữa vận tốc dọc với tốc độ yaw.
- Kiểm tra sự ổn định theo bước thời gian và tính nhất quán giữa lực lốp, trạng thái và gia tốc báo cáo.

### 6.2 Định nghĩa CRI so sánh NMPC với PP

CRI được định nghĩa là:

> Ước lượng đã hiệu chuẩn của xác suất ứng viên NMPC là lựa chọn tốt hơn ứng viên PP trong một chân trời đánh giá đã xác định, theo tiêu chí chất lượng và phân phối kịch bản mô phỏng đã công bố.

CRI **không** là xác suất an toàn phổ quát và không thay thế kiểm tra ràng buộc cứng. CRI trả lời câu hỏi so sánh giữa hai mode; nó không thay thế NMPC hoặc PP và không tự điều khiển phương tiện nếu thiếu bộ chuyển mode.

#### Nhãn so sánh hai ứng viên

Với mỗi cặp ứng viên NMPC và PP ở bước hiện tại, tạo hai nhánh mô phỏng bắt đầu từ cùng trạng thái, cùng trạng thái target, cùng nhiễu và cùng giới hạn vật lý. Trong mỗi nhánh, tiếp tục chạy đúng chính sách của mode tương ứng qua các bước receding-horizon trong cửa sổ đánh giá, thay vì chỉ giữ cố định lệnh đầu tiên:

- Gán nhãn “NMPC tốt hơn” nếu ứng viên NMPC hợp lệ, không vi phạm ràng buộc cứng và đạt tiêu chí so sánh định trước so với PP.
- Gán nhãn “PP tốt hơn hoặc tương đương” nếu PP cho kết quả tốt hơn, hoặc chênh lệch không đủ để ưu tiên NMPC theo biên đã định nghĩa.
- Nếu ứng viên NMPC không hợp lệ, đánh dấu là không được chấp nhận bởi cổng cứng; không để một nhãn xác suất CRI hợp thức hóa ứng viên lỗi.

Tiêu chí gán nhãn phải được chốt trước khi tạo tập test. Nên dùng các kết quả đầu ra độc lập với đặc trưng CRI để tránh vòng lặp/tự xác nhận, ví dụ mức giảm sai số nhiệm vụ, sai số dự đoán plant, vi phạm ràng buộc và mức điều khiển trong cùng cửa sổ. Nếu dùng một hàm utility tổng hợp để xếp hạng hai nhánh, phải nêu rõ ý nghĩa từng thành phần, cách chuẩn hóa và phân tích độ nhạy với lựa chọn utility. Không dùng chính chênh lệch cost đầu vào CRI làm nhãn duy nhất. Không dùng contact cuối cùng làm nhãn duy nhất vì nó phụ thuộc cả kịch bản, trạng thái target, cảm biến và quãng thời gian còn lại.

Các nhánh mô phỏng song song tạo nhãn phản thực ngoại tuyến; trong vận hành closed loop chỉ một mode được áp dụng tại mỗi thời điểm. Dùng cùng nhiễu ngẫu nhiên giữa cặp nhánh khi có thể để giảm nhiễu so sánh. CRI được huấn luyện trên các cặp kết quả mô phỏng nhưng phải được đánh giá trên rollouts closed loop độc lập, nơi chính sách chuyển mode dựa trên CRI quyết định chuỗi điều khiển thực tế.

#### Đặc trưng đầu vào đề xuất

Chỉ giữ các đặc trưng có thể tính tại thời điểm ra quyết định và có ý nghĩa:

- trạng thái solver/điều kiện dừng;
- residual feasibility và stationarity;
- biên nhỏ nhất tới các ràng buộc vật lý trên chân trời dự báo;
- độ bất định trạng thái từ UKF;
- sai số dự đoán một bước trước đã chuẩn hóa theo độ bất định;
- chênh lệch chi phí NMPC/PP được tính cùng mô hình, cùng chân trời và cùng định nghĩa;
- độ lớn thay đổi điều khiển, chỉ dùng như đặc trưng phụ nếu chứng minh có giá trị dự báo.

Không giả định rằng từng đặc trưng hiện có trong CRI đã được định nghĩa đúng; phải kiểm tra đơn vị, chuẩn hóa và khả năng dự báo của chúng.

#### Mô hình hiệu chuẩn

- Bắt đầu bằng mô hình đơn giản, có thể giải thích như logistic regression; so sánh isotonic calibration nếu dữ liệu đủ.
- Chia train/validation/test theo kịch bản hoặc seed, tránh chia ngẫu nhiên các timestep tương quan từ cùng một rollout sang cả train và test.
- Đóng băng mô hình và các ngưỡng trước khi chạy đánh giá cuối.
- Đánh giá calibration bằng reliability diagram, Brier score và/hoặc ECE; đồng thời báo cáo false acceptance, false rejection và coverage.
- Kiểm tra calibration khi phân phối xe, nhiễu cảm biến hoặc kịch bản target thay đổi.

### 6.3 Cấu trúc vòng kín CRI–NMPC–PP

Luồng điều khiển đề xuất:

1. UKF/cảm biến cập nhật trạng thái và độ bất định.
2. NMPC và PP cùng tạo ứng viên điều khiển từ trạng thái hiện tại.
3. **Cổng cứng** kiểm tra tính hợp lệ và feasibility của từng ứng viên theo giới hạn áp dụng.
4. Nếu chỉ một ứng viên qua cổng cứng, chọn ứng viên hợp lệ; nếu NMPC lỗi cứng thì chuyển sang PP. Nếu cả hai đều không hợp lệ, chuyển sang trạng thái lỗi/mất khả năng điều khiển đã định nghĩa, không giả định có mode an toàn.
5. Nếu cả hai ứng viên hợp lệ, CRI ước lượng khả năng NMPC tốt hơn PP theo tiêu chí đã hiệu chuẩn.
6. Hysteresis/dwell-time dùng CRI để quyết định mode NMPC hay PP.
7. Áp dụng lệnh của mode đã chọn; lưu chính lệnh cuối cùng đó làm lệnh trước cho bước sau.
8. Khi có trạng thái mới, tính sai số dự đoán thực tế và cập nhật đặc trưng đầu vào cho lần tính CRI tiếp theo.

#### Logic mode

- Khởi tạo ở PP hoặc mode khởi tạo được xác định trước; cả NMPC và PP tiếp tục được tính như các ứng viên khi trạng thái đầu vào còn hợp lệ.
- Chuyển PP → NMPC khi cả hai ứng viên được đánh giá theo cùng tiêu chí, ứng viên NMPC qua cổng cứng, CRI vượt ngưỡng bật và số bước xác nhận đạt yêu cầu.
- Chuyển NMPC → PP khi CRI thấp hơn ngưỡng tắt đủ số bước; chuyển ngay nếu ứng viên NMPC gặp lỗi cứng.
- Với suy giảm mềm của CRI, dùng ngưỡng tắt thấp hơn ngưỡng bật và yêu cầu tình trạng kéo dài một số bước.
- Dwell-time áp dụng cho chuyển mode bình thường; không chặn chuyển khẩn cấp khi cổng cứng thất bại.
- Trạng thái mất track cần được mô tả riêng; không giả định PP có thể hoạt động bình thường khi target không còn được quan sát.

Chưa đưa distance-based blending vào bản đầu tiên của method mới. Nếu sau này thêm blending, lệnh hỗn hợp phải qua lại cổng khả thi vì sự kết hợp hai lệnh riêng lẻ khả thi không tự đảm bảo tính khả thi phi tuyến.

### 6.4 Xử lý tham số

Phân loại và chọn tham số riêng:

| Loại tham số | Phương pháp xác định |
|---|---|
| Tham số xe/lốp | Đo, hiệu chuẩn hoặc trích nguồn; khai báo miền bất định nếu chưa chắc chắn |
| Tham số UKF/cảm biến | Nguồn từ mô hình cảm biến/tài liệu hoặc giả định công khai; kiểm tra độ nhạy |
| Trọng số hàm mục tiêu | Chọn trên tập tuning riêng với tiêu chí định trước; không chọn theo tập test |
| Ngưỡng CRI và dwell | Chọn theo calibration/false-acceptance trade-off và kiểm tra trên tập giữ lại |
| Ngưỡng đánh giá | Định nghĩa trước khi chạy kết quả cuối; không đổi sau khi xem kết quả để tăng tỷ lệ thành công |

Không tuyên bố mọi tham số đã “hết heuristic”. Mục tiêu là tham số có nguồn hoặc có quy trình chọn, cùng với độ nhạy và giới hạn diễn giải rõ ràng.

## 7. Phân tích lý thuyết và phạm vi chứng minh

### 7.1 Không dùng CRI làm chứng minh ổn định

CRI đã hiệu chuẩn là một ước lượng thống kê trên phân phối mô phỏng; tự nó không chứng minh an toàn, ổn định hay hội tụ cho mọi trạng thái.

### 7.2 Các mệnh đề có thể xem xét

Tùy khả năng của mô hình và solver, ưu tiên chứng minh một thuộc tính hẹp, ví dụ:

1. Nếu bộ điều khiển qua cổng cứng và các giả định mô hình đúng, lệnh được chấp nhận nằm trong miền ràng buộc đã định nghĩa.
2. PP duy trì một thuộc tính boundedness/feasibility trong miền xác định, nếu có thể xây dựng chứng minh cho controller đó.
3. Hệ hybrid có boundedness/practical stability nếu từng mode có hàm Lyapunov phù hợp và ảnh hưởng chuyển mode được chặn.

Với multiple Lyapunov functions, có thể tìm điều kiện đủ theo hướng sau: trong mỗi mode, hàm Lyapunov tương ứng phải giảm theo từng bước hoặc giảm sau một số bước xác định; khi chuyển mode, mức tăng của hàm ứng với mode mới phải bị chặn. Dwell-time cần đủ dài để mức giảm khi một mode hoạt động bù được mức tăng tại thời điểm chuyển. Điều kiện này chỉ có ý nghĩa nếu các bất đẳng thức cần thiết được chứng minh trong cùng miền áp dụng; đây là hướng phân tích, không phải kết quả đã có.

Nếu không chứng minh được các giả thiết cho NMPC gần đúng, PP, sai số UKF và mô hình lốp, cần giới hạn bài ở kết quả thực nghiệm. Không suy ra “contact chắc chắn xảy ra” từ tính ổn định; đó là một mệnh đề riêng cần điều kiện reachability và mô hình target.

## 8. Thiết kế thực nghiệm

### 8.1 Tập dữ liệu và tránh rò rỉ

- Tạo rollouts cho huấn luyện/hiệu chuẩn với nhiều seed và điều kiện ban đầu có kiểm soát.
- Tách tập train, validation và test theo kịch bản/seed, không theo từng timestep.
- Giữ một số kịch bản hoặc dải tham số làm kiểm tra ngoài phân phối nếu mục tiêu là đánh giá độ bền.
- Lưu cấu hình xe, lốp, target, FOV, nhiễu, seed, phiên bản code và toàn bộ metric.

### 8.2 Baseline

Tối thiểu so sánh:

1. NMPC-only.
2. PP-only.
3. Hybrid dùng ngưỡng cố định nhưng không CRI đã hiệu chuẩn.
4. Hybrid với CRI mới.
5. Hybrid chỉ có cổng cứng, không dùng điểm CRI.

Mọi baseline phải dùng cùng plant, giới hạn vật lý, cách xử lý lệnh, cảm biến và tiêu chí đánh giá. Nếu khác, ghi rõ lý do.

### 8.3 Ablation

- Bỏ từng nhóm đặc trưng CRI.
- Không hysteresis.
- Không dwell-time.
- Có/không có acceleration floor hiện tại.
- Dùng CRI heuristic cũ so với CRI hiệu chuẩn mới.
- Đánh giá luật Fiala hiện tại và so sánh với mô hình tire-force/friction chi tiết hơn nếu cần làm plant tham chiếu.

### 8.4 Metric

**CRI:**

- Brier score/ECE hoặc calibration metric tương ứng;
- false acceptance và false rejection;
- coverage/tỷ lệ thời gian NMPC được chấp nhận;
- calibration theo nhóm điều kiện (FOV, nhiễu, sai lệch tham số).

**Điều khiển và tính khả thi:**

- tỷ lệ vi phạm mỗi ràng buộc;
- gia tốc ngang theo từng định nghĩa rõ ràng, cực đại/RMS/phân vị;
- biên lực lốp và sai số dự đoán;
- số lần chuyển mode, dwell-time, thời gian ở từng mode;
- độ trễ tính toán/solver failure nếu đo được.

**Nhiệm vụ:**

- hiệu năng theo metric đã định nghĩa trước;
- proximity khi khoảng cách giữa hai xe dưới 4 m để đối chiếu với kết quả trước;
- nếu dùng contact terminal, contact theo footprint và thời điểm sự kiện;
- vận tốc tương đối tại sự kiện chỉ là metric mô tả, không phải mục tiêu tối ưu mức độ va chạm.

### 8.5 Thí nghiệm độ nhạy

Thay đổi có kiểm soát các nhóm bất định đã có căn cứ: tham số xe/lốp, độ trễ/nhiễu cảm biến, FOV và điều kiện target. Trước khi chạy, xác định dải biến thiên và lý do chọn dải đó. Báo cáo trung bình cùng độ phân tán/khoảng tin cậy phù hợp; không chỉ báo cáo seed tốt nhất.

## 9. Kế hoạch trả lời reviewer

| Nhận xét | Cách phản hồi dự kiến | Bằng chứng cần có |
|---|---|---|
| CRI heuristic, chưa liên hệ ổn định/an toàn | Định nghĩa CRI là xác suất/ước lượng chất lượng nghiệm được hiệu chuẩn; tách khỏi cổng ràng buộc cứng; giới hạn tuyên bố | Quy trình gán nhãn, calibration trên test, false acceptance/coverage, ablation |
| Gia tốc ngang quá cao, thiếu mô hình lốp | Xác minh metric; xác định sedan tham chiếu; đưa giới hạn lốp vào plant/controller phù hợp | Tham số có nguồn, mô tả tire model, bảng gia tốc/lực và số lần vi phạm, toàn bộ kết quả chạy lại |
| Tiêu chí khoảng cách dưới 4 m có thể là fly-by | Giữ metric proximity cũ; nếu cần contact, phát hiện theo footprint và dừng mô phỏng | Định nghĩa hình học, kiểm tra theo bước thời gian, metric mới/cũ |
| NMPC và post-processing không nhất quán | Đồng bộ lệnh được đánh giá với lệnh thực thi; áp dụng cùng chính sách cho baseline | Log lệnh trước/sau, ablation acceleration floor, kết quả baseline công bằng |
| Methodology/ổn định chưa chặt | Nêu giả thiết; chứng minh chỉ mệnh đề đủ cơ sở; nếu không, trình bày như đánh giá thực nghiệm | Định lý khớp với code hoặc phần limitation rõ ràng |
| Reviewer 2 chưa thấy thay đổi đáng kể | Liệt kê thay đổi mã, mô hình, thí nghiệm và bảng/hình mới theo từng comment | Bản thảo marked-up và response letter có vị trí cụ thể |

## 10. Các giai đoạn và tiêu chí hoàn thành

### Giai đoạn A — Tái lập và đóng băng phạm vi

- Truy vết script/cấu hình đã tạo các bảng cũ.
- Tái lập các kết quả chính hoặc giải thích sai khác.
- Chốt mô hình sedan, target, cảm biến và tiêu chí đánh giá.

**Đầu ra:** cấu hình baseline có thể chạy lại và tài liệu giả định mô hình.

### Giai đoạn B — Plant và tính khả thi

- Xác nhận mô hình bicycle phi tuyến với luật lực lốp Fiala; đánh giá độ nhạy và chốt phạm vi hợp lệ.
- Sửa cách tính/báo cáo gia tốc.
- Xác định ràng buộc lốp và cách thực thi trong solver/plant.

**Đầu ra:** kiểm thử mô hình và bảng kiểm tra tính khả thi.

### Giai đoạn C — CRI và chuyển mode

- Tạo dữ liệu nhãn bằng rollouts có kiểm soát.
- Huấn luyện/hiệu chuẩn mô hình CRI trên tập riêng.
- Thiết kế hard gate, hysteresis, dwell và xử lý mất track.
- Đảm bảo lệnh được đánh giá là lệnh sẽ thực thi.

**Đầu ra:** CRI có ngữ nghĩa, calibration report và bộ chuyển mode kiểm thử được.

### Giai đoạn D — Đánh giá đối chứng

- Chạy baseline, ablation, nhiều seed và độ nhạy.
- Báo cáo calibration, coverage, vi phạm ràng buộc, metric nhiệm vụ và switching.
- Không dùng lại tập tuning làm tập test.

**Đầu ra:** bảng/hình tái lập cho các nhận xét của reviewer.

### Giai đoạn E — Chứng minh và viết lại

- Chọn mệnh đề lý thuyết có thể chứng minh từ mô hình/solver thực tế.
- Nếu chứng minh không đủ, giảm tuyên bố và nêu limitation.
- Viết lại method, kết quả, abstract, conclusion và response letter đồng bộ.

**Đầu ra:** bản thảo sạch, bản đánh dấu thay đổi và response letter có tham chiếu vị trí.

## 11. Những tuyên bố không nên đưa ra nếu chưa có bằng chứng

- CRI là xác suất an toàn phổ quát.
- CRI cao đồng nghĩa với nghiệm tối ưu toàn cục.
- Hysteresis/dwell-time tự chứng minh hệ hybrid ổn định.
- Gia tốc báo cáo là khả thi đối với xe sedan nếu mô hình lốp chưa giới hạn lực.
- Contact/proximity trong mô phỏng chứng minh khả năng triển khai trên xe thật.
- Kết quả của một sedan tham chiếu đại diện cho mọi xe.

## 12. Câu hỏi cần xin ý kiến giảng viên

1. Có thống nhất định nghĩa CRI là xác suất đã hiệu chuẩn rằng NMPC tốt hơn PP trong cửa sổ đánh giá, hay muốn giữ CRI là một chỉ số xếp hạng không xác suất?
2. Kết quả nào được xem là “tốt hơn” khi so NMPC và PP, và có cần hàm utility tổng hợp hay dùng tiêu chí nhiều mục tiêu?
3. Nguồn tham số sedan/lốp nào đủ phù hợp cho mô hình bicycle phi tuyến hiện tại? Có cần plant tham chiếu chi tiết hơn để xác nhận mô hình lốp, hay có thể biện minh bằng dữ liệu/benchmark và phân tích độ nhạy?
4. Kịch bản target, FOV/camera và nhiễu nào là phạm vi chính cần đánh giá?
5. Định nghĩa proximity/contact nào cần giữ trong bài và contact có thực sự cần thiết để trả lời câu hỏi nghiên cứu?
6. Cần ưu tiên chứng minh thuộc tính nào: tính khả thi, boundedness/practical stability, hay đánh giá thực nghiệm có kiểm soát?
7. Thời gian và tài nguyên có đủ để tạo dữ liệu cặp phản thực, hiệu chuẩn CRI và kiểm tra trên tập kịch bản độc lập không?

## 13. Thông tin cần thu thập trước khi chốt method

Hãy thu thập các dữ liệu dưới đây và ghi nguồn, ngày truy cập, đơn vị và độ tin cậy. Nếu chưa tìm được nguồn, đánh dấu rõ là giả định để đưa vào phân tích độ nhạy; không chọn giá trị chỉ dựa trên kết quả mong muốn.

### 13.1 Bản thảo và khả năng tái lập kết quả hiện tại

- Bản thảo nguồn và phiên bản đã gửi reviewer; response letter vòng trước.
- Script/config cụ thể tạo Table 5, Table 14 và các kết quả CRI; map mỗi bảng/hình tới file dữ liệu.
- Điều kiện khởi tạo, seed, số lần lặp, thời gian chạy, tham số target và sensor cho từng bảng.
- Định nghĩa chính xác các đại lượng “mean lateral acceleration” trong bảng: công thức, thời điểm lấy mẫu, dùng trị tuyệt đối hay signed, mean theo bước hay theo run.
- Xác nhận distance-based blending có tồn tại trong phiên bản code/bản thảo dùng để nộp hay không.

### 13.2 Sedan và plant mô phỏng

- Tên/nguồn của sedan tham chiếu hoặc nguồn benchmark; khối lượng, chiều dài/rộng, chiều dài cơ sở, vị trí trọng tâm, mô-men quán tính yaw. Xác định đại lượng nào mô hình bicycle hiện tại dùng trực tiếp và đại lượng nào chỉ cần cho plant tham chiếu.
- Tham số lốp: độ cứng hoặc dữ liệu đường cong lực-góc trượt, lực dọc/ngang cực đại, điều kiện tải và áp suất nếu nguồn cung cấp.
- Tải trọng tĩnh từng cầu/bánh, mô hình chuyển tải khi tăng tốc/phanh/quay vòng, giả định hệ dẫn động và phân bố lực phanh.
- Hệ số ma sát mặt đường, điều kiện mặt đường, và nguồn/biện minh cho miền biến thiên.
- Miền hợp lệ của luật lốp Fiala hiện tại (bão hòa theo tải pháp tuyến tĩnh và hệ số ma sát giả định); tire-force saturation/friction model đầy đủ hơn (ví dụ friction circle/ellipse, tải động) nếu cần, và phương pháp kiểm tra nó.
- Nếu không có thông số thật: nguồn benchmark thay thế, các tham số giả định và phạm vi kết luận cho cấu hình mô phỏng.

### 13.3 Target và định nghĩa nhiệm vụ

- Mô hình động học của target, vận tốc và gia tốc có thể có, cách sinh quỹ đạo và các trường hợp khó.
- Target có footprint/kích thước xe hay chỉ là điểm mục tiêu; điểm tham chiếu trong mô hình nằm ở đâu.
- Mục tiêu khoa học chính: giảm khoảng cách, tiếp cận, bám theo, hay sự kiện terminal mô phỏng; metric nào là primary outcome.
- Cơ sở của ngưỡng proximity 4 m hiện tại; cần giữ để đối chiếu nhưng không tự xem đó là hình học contact.
- Nếu dùng contact terminal: kích thước/footprint của hai vật thể, cách kiểm tra contact giữa hai bước và cách dừng mô phỏng.
- Cách xử lý target bị mất khỏi FOV, track stale, target rời miền mô phỏng hoặc hết thời gian mô phỏng.

### 13.4 Camera, sensor và UKF

- Cấu hình FOV đang dùng cho từng kết quả, bao gồm 60° và 360° nếu có; ý nghĩa của giả định 360° trong mô phỏng.
- Tầm phát hiện, tần số cập nhật, độ trễ, nhiễu vị trí/vận tốc/góc, xác suất bỏ sót/phát hiện sai và mô hình che khuất nếu có.
- Nguồn các tham số sensor hoặc giải thích rằng đó là giả định mô phỏng.
- Ma trận nhiễu quá trình/đo của UKF, cách chọn/hiệu chuẩn và kiểm tra độ nhất quán của covariance.
- Các metric estimation cần báo cáo, ví dụ sai số trạng thái và độ phủ khoảng tin cậy.

### 13.5 NMPC, PP và luật hiện có

- Chu kỳ lấy mẫu, prediction/control horizon, hàm mục tiêu đầy đủ và đơn vị/tỷ lệ từng hạng tử.
- Giới hạn trạng thái, điều khiển, tốc độ lái, gia tốc và nguồn của từng giới hạn.
- Solver status, tiêu chí dừng, số vòng lặp, thời gian tính toán và cách xử lý nghiệm không hội tụ.
- PP lookahead/geometry, gain gia tốc, giới hạn lái và cách chọn desired speed.
- Lý do và phạm vi áp dụng của acceleration floor dưới tốc độ thấp; có áp dụng giống nhau cho mọi baseline không.
- Lệnh nào được đưa vào CRI, biến lưu làm `last_mv`, và lệnh cuối cùng tác động lên plant sau post-processing.
- Điều kiện một ứng viên NMPC hoặc PP được xem là không hợp lệ; trường hợp cả hai đều không hợp lệ được xử lý thế nào.

### 13.6 CRI và dữ liệu hiệu chuẩn

- Định nghĩa từng thành phần CRI hiện tại, công thức tính, đơn vị, tham số chuẩn hóa và lý do chọn trọng số/ngưỡng.
- Ý nghĩa chính xác của “NMPC tốt hơn PP”: outcome/utility, cửa sổ đánh giá, biên tương đương và xử lý trường hợp hòa.
- Các đặc trưng online nào thực sự sẵn có tại thời điểm quyết định; loại bỏ đặc trưng gây rò rỉ thông tin tương lai.
- Quy mô dữ liệu có thể tạo: số scenario, seed, trạng thái khởi tạo và cặp rollout NMPC/PP.
- Cách chia train/validation/test theo scenario hoặc seed; phân phối nào được giữ lại để kiểm tra tổng quát hóa.
- Chi phí tính toán cho CRI online và việc có đáp ứng được chu kỳ điều khiển không.
- Tiêu chí chọn ngưỡng chuyển mode và mức chấp nhận false switch; các giới hạn này cần được thống nhất trước khi đánh giá test.

### 13.7 Phạm vi lý thuyết

- Thuộc tính cụ thể muốn chứng minh và miền trạng thái/điều kiện target áp dụng.
- Mô hình toán của NMPC, PP, plant, sai số mô hình và estimator đủ để xây dựng phân tích.
- Có thể chứng minh PP có tính chất cần thiết trong miền dự định làm mode thay thế hay không.
- Điều kiện solver NMPC bảo đảm tính khả thi đệ quy hay chỉ tìm nghiệm gần đúng.
- Tính chất nào sẽ được báo cáo thực nghiệm thay vì gọi là chứng minh.

### 13.8 Bảng theo dõi nguồn tham số

Tạo bảng làm việc với các cột: **Tên tham số | Giá trị/miền | Đơn vị | Nhóm (vật lý/thiết kế/đánh giá) | Nguồn hoặc quy trình chọn | Độ bất định | Nơi sử dụng trong code | Thí nghiệm độ nhạy**. Bảng này giúp phát hiện tham số nào còn tùy ý trước khi đóng băng cấu hình cuối.

**Rà soát sơ bộ cho `vehicle_model.py` (luật lốp Fiala), dùng làm điểm khởi đầu khi điền bảng trên:**

| Tham số | Giá trị hiện tại | Nhóm | Tác giả | Nguồn đối chiếu | Ghi chú/độ bất định |
|---|---|---|---|---|---|
| `m` | 1575 kg | (2) trích tài liệu | Rajesh Rajamani | Rajamani, *Vehicle Dynamics and Control*, Springer 2012, ch. 3 (`m=1573 kg`, mô tả "representative of parameters for a passenger sedan") | Gần khớp, chênh lệch không đáng kể; có thể trích dẫn trực tiếp. |
| `Iz` | 2875 kg·m² | (2) trích tài liệu | Rajesh Rajamani | Cùng nguồn Rajamani (`Iz=2873 kg*m^2`) | Gần khớp, cùng nguồn với `m`. |
| `lf`, `lr` | 1.2 m, 1.3 m | (3) giả định mô phỏng | Rajesh Rajamani (nguồn đối chiếu, không khớp) | Không khớp Rajamani (`lf=1.1`, `lr=1.58`) | Không nên trích cùng nguồn với `m`/`Iz`; cần nguồn riêng hoặc công bố là giả định + sensitivity theo `lf/lr`. |
| `Cf`, `Cr` | 19000, 33000 N/rad | (3) giả định mô phỏng, cần rà soát | Rajesh Rajamani (nguồn đối chiếu, không khớp) | Thấp hơn 2.4–4 lần so với `Cf=Cr=80000 N/rad` của Rajamani cho cùng loại sedan | Điểm yếu nhất của bộ tham số hiện tại; ưu tiên tìm nguồn riêng (đo/hiệu chuẩn hoặc benchmark khác phù hợp quy ước "theo cầu") trước khi viết bài, hoặc chạy sensitivity rộng và nêu rõ giới hạn. |
| `mu` | 0.9 | (3) giả định mô phỏng, đối chiếu độ lớn | Michael Taylor | Taylor, *Implementation and Validation of the Fiala Tire Model in Chrono*, Technical Report TR-2015-13 (2015): `Umin=0.9`, `Umax=1.0` dùng để kiểm định triển khai Fiala | Không phải giá trị đo cho một cặp lốp-mặt đường cụ thể; chỉ dùng được để nói "cùng khoảng giá trị với tài liệu kiểm định mô hình Fiala". |
| `h` (chiều cao trọng tâm) | 0.55 m | (3) giả định mô phỏng | Không có nguồn cụ thể; nằm trong khoảng điển hình được nhắc tới trong tài liệu động lực học xe (vd. Rajamani 2012, ch.2, công thức chuyển tải dọc) | Dùng cho `normal_loads()` (chuyển tải dọc khi phanh/tăng tốc) và friction ellipse; cần nguồn riêng nếu dùng cho tuyên bố định lượng. |

Khi viết lại phần methodology, nên trình bày tách bạch: `m`, `Iz` là tham số có đối chiếu tài liệu; `lf`, `lr`, `Cf`, `Cr` là giả định mô phỏng cần sensitivity; `mu` là giả định có đối chiếu độ lớn nhưng không phải giá trị đo thực địa. Tránh gộp chung cả bốn/năm tham số thành một câu "tham số lấy từ tài liệu tham khảo".