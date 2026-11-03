# Kế hoạch xử lý phản biện bài báo Interception

## 1. Mục tiêu và nguyên tắc

Phạm vi nghiên cứu được giới hạn vào **một cấu hình sedan tham chiếu trong mô phỏng**; không tuyên bố kết quả áp dụng cho mọi loại xe. Plant hiện là mô hình động lực học bicycle phi tuyến, không phải mô hình xe bốn bánh tính lực riêng từng bánh. Interception được định nghĩa là sự kiện tiếp xúc hình học giữa footprint hai xe và là điều kiện kết thúc mô phỏng; không dùng nó để đánh giá an toàn hay hậu quả va chạm ngoài đời.

Mục tiêu là tái thiết kế methodology và bằng chứng thực nghiệm ở mức tổng thể, không chỉ vá riêng CRI, gia tốc hoặc mô hình lốp. Có thể giữ cấu trúc NMPC–PP–CRI làm hướng nghiên cứu, nhưng cần thiết kế lại cách mô hình hóa, tiêu chí khả thi, ý nghĩa CRI, luật chuyển mode, mục tiêu tối ưu và quy trình chọn tham số. Không mặc định giữ lại các trọng số, ngưỡng, luật hậu xử lý hay kết luận hiện tại. Mọi thay đổi phải được áp dụng nhất quán cho các baseline và đánh giá lại toàn bộ kết quả bị ảnh hưởng.

Code hiện dùng mô hình động lực học bicycle phi tuyến cho mô phỏng. Cần phân biệt mô hình này với mô hình xe bốn bánh chi tiết: phi tuyến nằm trong động lực học (như lượng giác, arctan và tích trạng thái), còn luật lực lốp hiện tuyến tính theo góc trượt và chưa có bão hòa/giới hạn lực lốp. Không mặc định phải thay bằng mô hình bốn bánh; cần thẩm định luật lực lốp và tính khả thi theo mục tiêu bài báo.

### Định hướng tái thiết kế methodology

**Nguyên tắc:** thiết kế theo thứ tự *mô hình và miền hợp lệ → ràng buộc vật lý → bộ điều khiển ứng viên → định nghĩa CRI và luật chuyển mode → phân tích lý thuyết → hiệu chuẩn tham số → thực nghiệm*. Không chọn trọng số/ngưỡng để làm đẹp kết quả sau khi đã xem test set. CRI vẫn là đóng góp trung tâm, nhưng chỉ được gọi là reliability index theo đúng đại lượng mà nó dự báo và bằng chứng hiệu chuẩn hỗ trợ.

#### 1. Chốt bài toán và tiêu chí thành công

- Định nghĩa trạng thái ego/target, tín hiệu quan sát được, giả định về target motion, giới hạn cảm biến và miền trạng thái mà phương pháp tuyên bố hoạt động.
- Định nghĩa interception là tiếp xúc footprint trong mô phỏng; báo cáo riêng proximity, thời gian tiếp xúc và vận tốc tương đối để mô tả kết quả, không tối ưu mức độ va chạm.
- Xác định mục tiêu điều khiển theo thứ tự ưu tiên có thể diễn giải vật lý: trước hết giữ khả thi/ràng buộc; tiếp theo đạt interception; sau đó mới tối ưu thời gian/độ mượt/nỗ lực điều khiển theo yêu cầu đã biện minh. Ưu tiên ràng buộc hoặc thứ tự từ điển thay cho tổng trọng số tùy ý khi có thể.

#### 2. Xây dựng plant và giới hạn vật lý nhất quán

- Giữ mô hình single-track/nonlinear bicycle làm mô hình sedan tham chiếu nếu nó đủ cho câu hỏi nghiên cứu; không cần chuyển sang mô hình bốn bánh chỉ để tăng độ phức tạp.
- Thay luật lực lốp tuyến tính không bão hòa bằng luật lốp có bão hòa phù hợp với độ chi tiết của plant. Nếu tuyên bố combined-slip/friction ellipse, phải biểu diễn lực dọc và cách phân bổ lực dọc/lực ngang phù hợp; nếu không, chỉ gọi đó là giới hạn gia tốc xấp xỉ và nêu rõ giới hạn.
- Chốt nguồn hoặc cách nhận dạng tham số khối lượng, hình học, quán tính, độ cứng/đặc tính lốp, hệ số bám, độ trễ và nhiễu cảm biến. Nếu không có dữ liệu xe thật, ghi rõ đây là sedan tham chiếu mô phỏng và đánh giá độ nhạy/tham số bất định.
- Đưa giới hạn lực lốp, gia tốc ngang/dọc, góc lái, tốc độ thay đổi góc lái và gia tốc vào cùng một chính sách khả thi. Loại bỏ các hậu xử lý không được optimizer/CRI biết đến; kiểm tra lệnh cuối cùng được áp dụng.

#### 3. Thiết kế lại NMPC và PP như hai candidate công bằng

- Dùng chung plant, dự báo target, chu kỳ lấy mẫu, miền điều khiển và safety filter cho NMPC và PP.
- NMPC phải tính đúng ràng buộc được tuyên bố; kiểm tra trạng thái solver, khả thi sau tối ưu và khả năng duy trì khả thi qua các bước. Nếu solver hiện tại không xử lý được ràng buộc phi tuyến cần thiết, chọn lại solver/phương pháp hoặc giảm phạm vi tuyên bố.
- PP phải được đánh giá như một controller ứng viên thực sự, không mặc định là fallback luôn an toàn. Cả hai candidate phải qua cùng cổng kiểm tra lệnh/giới hạn vật lý.
- Không để một acceleration floor hoặc clip riêng làm thay đổi lệnh sau khi CRI đã chấm điểm. Nếu có safety filter chung, CRI đánh giá lệnh sau filter hoặc đánh giá toàn bộ chuỗi candidate → filter → plant.

#### 4. Định nghĩa CRI có thể kiểm chứng, không dựa vào tổng trọng số tùy ý

- Định nghĩa sự kiện mà CRI dự báo trước: ví dụ candidate NMPC hội tụ, khả thi trong horizon ngắn dưới bất định và có hiệu năng dự báo tốt hơn PP theo tiêu chí đã chốt. Không nhập nhằng “solver hội tụ”, “khả thi”, “an toàn” và “tốt hơn PP” thành một khái niệm duy nhất.
- Tính đánh giá candidate từ rollout closed-loop tương ứng với NMPC receding-horizon và PP cập nhật lệnh, dùng cùng trạng thái, target forecast, horizon, uncertainty samples và giới hạn.
- Dùng uncertainty từ UKF và bất định tham số/mô hình để tạo các trường hợp kiểm tra. CRI có thể được xây dựng như xác suất thực nghiệm/tỷ lệ rollout đạt sự kiện dự báo hoặc như một decision score đã hiệu chuẩn. Chỉ gọi là xác suất tin cậy nếu reliability/calibration được xác nhận trên episode độc lập.
- Chọn ngưỡng switching từ yêu cầu chấp nhận rủi ro/false acceptance đã công bố và tập tuning; báo cáo hiệu năng trên tập scenario/seed riêng. Không tối ưu ngưỡng trên test set.
- Tách hard feasibility/safety gate ra khỏi CRI: nghiệm không hữu hạn, solver thất bại, constraint vi phạm hoặc lệnh không hợp lệ phải bị loại trước khi xét score. CRI không phải safety certificate.
- Hysteresis dùng thời gian thực và có nhánh override tức thời cho lỗi cứng. Minimum dwell-time giúp hạn chế chuyển mode quá dày nhưng tự nó không chứng minh ổn định.

#### 5. Đặt phạm vi chứng minh thực tế

- Với bài toán interception hữu hạn thời gian, ưu tiên chứng minh các tính chất có thể hỗ trợ trực tiếp: tính khả thi đệ quy của NMPC trong miền xác định, thỏa ràng buộc trong plant đã mô hình hóa, boundedness trong miền hoạt động và điều kiện mà switching supervisor không làm mất tính khả thi.
- Nêu rõ giả thiết về target acceleration, sai số UKF, sai lệch mô hình, miền trạng thái ban đầu và khả năng solver. Nếu chứng minh cần một safety controller/terminal invariant set, phải xây dựng và kiểm tra được thành phần đó trong code.
- Chỉ tuyên bố ổn định của hybrid closed-loop nếu chứng minh áp dụng cho cả các mode NMPC/PP và luật chuyển thực tế. Dwell-time hoặc CRI đơn lẻ không đủ làm chứng minh.
- Nếu không thể chứng minh ổn định toàn cục, giới hạn kết luận thành đảm bảo khả thi/boundedness có điều kiện hoặc kết quả thực nghiệm trong miền/kịch bản đã thử; đưa giới hạn vào abstract/conclusion và phản hồi reviewer.

#### 6. Thay “tham số heuristic” bằng quy trình xác định và kiểm chứng

- **Tham số vật lý:** lấy từ nguồn/tài liệu hoặc nhận dạng; lưu nguồn, đơn vị, miền bất định và quy trình dùng.
- **Giới hạn an toàn/khả thi:** suy ra từ thông số sedan, giả định mặt đường và giới hạn mô hình; không tune theo hiệu năng interception.
- **Tham số cảm biến/UKF:** quy đổi noise sensor sang covariance nhất quán, hiệu chuẩn trên dữ liệu mô phỏng/đo được và kiểm tra consistency của estimate.
- **Tham số bộ điều khiển:** xác định từ thời gian lấy mẫu, horizon, actuator limits và mục tiêu định lượng; dùng tuning có cấu trúc cùng validation thay vì chọn thủ công theo một scenario.
- **Tham số CRI/switching:** fit/calibrate trên tập episode tuning; chọn operating threshold theo mức false acceptance/độ trễ chuyển mode được chấp nhận; đánh giá độ nhạy và báo cáo trên tập độc lập.
- Phân loại rõ tham số nào là vật lý, nhận dạng, thiết kế hay thống kê. Nếu một tham số vẫn do người thiết kế chọn, ghi rõ rationale và phân tích độ nhạy thay vì gọi nó là “tối ưu” hay “được chứng minh”.

#### 7. Thực nghiệm phải kiểm tra đúng tuyên bố mới

- Tái lập baseline cũ rồi chạy lại toàn bộ thí nghiệm sau khi thay plant, constraint, lệnh áp dụng hoặc switching; không tái sử dụng số liệu được tạo bởi cấu hình cũ.
- So sánh tối thiểu NMPC-only, PP-only, switching cố định/hysteresis nếu phù hợp, và phương pháp NMPC–PP–CRI đề xuất; dùng cùng model, limits, target estimates và initial conditions.
- Báo cáo riêng hiệu năng interception, thời gian, gia tốc/lực lốp, constraint violations, solver failures, chuyển mode, CRI calibration và hiệu năng theo mức bất định.
- Tách episode theo scenario/seed thành tuning và evaluation; chạy nhiều seed và báo cáo phân bố/độ tin cậy thống kê. Một kịch bản duy nhất hoặc một seed cố định không đủ xác nhận phương pháp tổng quát.

**Hướng khuyến nghị:** nếu mục tiêu là thiết lập quan hệ chặt nhất giữa CRI và hành vi đóng vòng, ưu tiên một CRI dạng **chỉ số chứng nhận dựa trên biên khả thi và tiến triển robust**, không phải tổng trọng số heuristic hoặc xác suất dự đoán. Mỗi candidate NMPC/PP chỉ được chấp nhận khi thỏa cùng các điều kiện robust về ràng buộc và tiến triển tới tập interception; CRI tổng hợp mức dư địa của các điều kiện đó, còn lợi thế hiệu năng và độ mượt dùng sau bước chứng nhận để lựa chọn giữa các candidate hợp lệ. Tuyên bố cần giới hạn theo tập bất định và miền trạng thái đã xác định. Nếu không thể xây dựng/kiểm chứng các biên worst-case, dùng phiên bản CRI xác suất đã hiệu chuẩn và chỉ tuyên bố bảo đảm thống kê, không tuyên bố chứng minh an toàn/ổn định.

#### Phương án chặt nhất: CRI là chứng nhận robust cho switching

Đây là phương án được ưu tiên nếu đủ nguồn lực để thiết kế lại optimizer, plant và phân tích lý thuyết. Nó tạo quan hệ trực tiếp giữa CRI và điều kiện mà bộ supervisor được phép chọn mode.

1. **Xác định miền và bất định:** xây dựng tập trạng thái hoạt động, giới hạn lực lốp/chấp hành, sai số estimate UKF, sai lệch mô hình và gia tốc mục tiêu tối đa. Mọi bảo đảm chỉ áp dụng trong các tập/giới hạn này; cần chứng minh điều kiện ban đầu nằm trong miền đó.
2. **Xác định tập interception và đại lượng tiến triển:** biểu diễn contact bằng footprint; tạo đại lượng đo tiến triển trong hệ tọa độ tương đối tới tập contact. Xác định điều kiện reachable: nếu target nằm ngoài khả năng động lực học hoặc mất quan sát vượt giả thiết thì không tuyên bố chắc chắn interception.
3. **Tạo mô hình dự báo robust:** dùng tube/ensemble/worst-case rollout bao phủ các sai số nằm trong tập bất định. Nếu dùng mô phỏng mẫu hữu hạn, kết quả chỉ là đảm bảo thống kê trừ khi có phương pháp bổ sung để chặn toàn bộ phần bất định chưa lấy mẫu.
4. **Tạo chứng nhận cho mỗi candidate:** candidate NMPC và PP phải được kiểm tra trên cùng tập uncertainty và cùng safety filter. Candidate chỉ được chấp nhận nếu lệnh cuối khả thi, các giới hạn được thỏa trong toàn rollout và điều kiện tiến triển/terminal được thỏa. PP không được coi là an toàn mặc định.
5. **Định nghĩa CRI không bù trừ giữa các điều kiện:** dùng biên có dấu hoặc trạng thái pass/fail từ các điều kiện chứng nhận. Nếu cần một scalar, nó phải là biên dư địa bảo thủ được chuẩn hóa bằng các ngưỡng vật lý/numeric đã định nghĩa; không dùng tổng trọng số để một thành phần tốt bù cho vi phạm ràng buộc cứng. CRI không hợp lệ khi solver trả nghiệm lỗi hoặc dữ liệu đầu vào nằm ngoài miền chứng nhận.
6. **Tách certification khỏi performance ranking:** sau khi loại candidate không đạt chứng nhận, dùng superiority so với PP, thời gian dự đoán hoặc độ mượt làm tiêu chí thứ cấp giữa các candidate còn hợp lệ. Các tiêu chí thứ cấp không được ghi đè chứng nhận.
7. **Dùng safety filter/backup có điều kiện rõ:** nếu cả NMPC và PP đều không qua chứng nhận, phải có hành động backup nằm trong miền khả thi hoặc báo rõ hệ thống không thể bảo đảm điều kiện tại trạng thái đó. Không được âm thầm phát lệnh ngoài miền để tiếp tục mô phỏng.
8. **Thiết lập chứng minh:** dưới giả thiết sai số bị chặn, khởi tạo trong miền khả thi, solver trả nghiệm thỏa tolerance đã quy định, candidate/backup qua cùng cổng và terminal/backup set được duy trì, chứng minh ràng buộc được thỏa và feasibility được duy trì tới contact hoặc tới khi rời miền giả thiết. Nếu có một đại lượng Lyapunov/CLF chung giảm cho mọi lệnh được chấp nhận, có thể suy ra boundedness hoặc tiến triển tới tập đích trong miền đó. Nếu chỉ chứng minh giảm theo mode riêng, cần chứng minh điều kiện khi chuyển và dwell-time thỏa bất đẳng thức tương ứng.
9. **Giới hạn tuyên bố:** dwell-time tự nó không chứng minh ổn định; CRI đạt ngưỡng chỉ là chứng nhận trong assumptions/domain đã chứng minh. Nếu không chứng minh được một CLF chung, không gọi toàn hệ switching là ổn định. Với bài toán target chuyển động, ưu tiên tuyên bố finite-time reachability/recursive feasibility có điều kiện hoặc boundedness; interception không thể được bảo đảm nếu bài toán không reachable.
10. **Đánh giá ngoài mẫu bổ sung:** dù có chứng minh worst-case trong miền mô hình, vẫn chạy Monte Carlo nhiều seed để kiểm tra độ bảo thủ và mức hữu ích. Nếu dùng một tầng xác suất để nới miền bất định, đánh giá calibration riêng và gọi rõ đó là đảm bảo xác suất theo phân phối đã chọn.

**Lưu ý về tính khả thi:** đây là phương án chặt nhất về mặt logic nhưng có thể đòi hỏi thay solver, bổ sung mô hình bất định/tube, xây dựng miền terminal/backup và chứng minh reachability. Nếu không thể hoàn tất các thành phần đó, phương án trung thực tiếp theo là CRI xác suất đã hiệu chuẩn cộng hard feasibility gate; không trình bày nó như một chứng nhận Lyapunov hoặc bảo đảm an toàn tuyệt đối.

## 2. Hiện trạng được nhận diện trong mã nguồn

- [`cri.py`](./cri.py): CRI đã được cài đặt dưới dạng tổng có trọng số của optimality, feasibility, superiority và smoothness; trọng số hiện tại là 0.25, 0.20, 0.40 và 0.15. Các thành phần được tính bằng heuristic và một số tham số chuẩn hóa (`eps1`–`eps4`). `phi2` kiểm tra giới hạn trạng thái/điều khiển dạng hộp khi mô phỏng tối đa 5 bước, không kiểm tra friction circle. Lệnh NMPC đã được solver chiếu vào giới hạn hộp nên phần kiểm tra giới hạn điều khiển trong `phi2` không chứng minh lệnh khả thi theo lực lốp. CRI hiện không sử dụng `ExitFlag` hoặc `GradNorm` từ solver.
- [`hysteresis.py`](./hysteresis.py): đã có hysteresis với `theta_on=0.65`, `theta_off=0.45` và `n_dwell=2`. Bộ đếm được tăng khi gọi `decide`, và được đặt lại sau khi chuyển mode; vì vậy có một điều kiện dwell tối thiểu theo số lần gọi. Tuy nhiên, `decide` chỉ được gọi khi có track trong [`main.py`](./main.py), nên không nên diễn giải con số này là dwell-time theo thời gian thực trong mọi tình huống nếu chưa xử lý khoảng mất track. Với `Ts=0.1 s`, hai bước điều khiển bình thường tương đương khoảng 0.2 s. Code hiện chưa thống kê số lần chuyển hoặc dwell thực tế.
- [`main.py`](./main.py): khi có track, controller chọn trực tiếp lệnh NMPC hoặc PP theo mode; chưa thấy distance-based blending. Nếu bản thảo mô tả blending thì cần xác nhận có phiên bản code khác hay bổ sung blending vào code đánh giá. Khi solver ném exception, code chuyển sang PP, nhưng không thấy nhánh quyết định rõ ràng theo `ExitFlag` hay `GradNorm`.
- [`vehicle_model.py`](./vehicle_model.py) và [`NMPC.py`](./NMPC.py): NMPC dùng giới hạn hộp cho đầu vào, `a_x ∈ [-5, 8] m/s²`, `delta ∈ [-0.45, 0.45] rad`, cùng các giới hạn trạng thái. Mô hình hiện tại là động lực học bicycle phi tuyến với luật lực lốp Fiala bão hòa theo tải pháp tuyến tĩnh và hệ số ma sát giả định; chưa có ràng buộc friction circle/ellipse hay tải trọng động. Đây không phải mô hình xe bốn bánh tính lực riêng từng bánh, nhưng điều đó không đồng nghĩa toàn bộ mô hình là tuyến tính.
- [`main.py`](./main.py): tiêu chí hiện tại đặt `intercepted=True` khi khoảng cách Euclid giữa hai vị trí nhỏ hơn 4 m tại một bước. Với mục tiêu mô phỏng va chạm, khoảng cách tâm không phải hình học va chạm của hai thân xe. Cần xác định footprint của sedan/target trong mô phỏng, kiểm tra giao nhau của hai footprint và dừng mô phỏng tại sự kiện đó; vận tốc tương đối tại thời điểm sự kiện chỉ dùng để mô tả kết quả, không biến thành tiêu chí tối ưu mức độ va chạm.
- [`main.py`](./main.py): sau khi chọn NMPC/PP, code áp dụng `a_x >= 4 m/s²` khi tốc độ nhỏ hơn 10 m/s, sau đó áp dụng `a_x >= -4 m/s²`. Trong nhánh không có track, gia tốc được tính theo tốc độ rồi kẹp dưới ở -4 m/s²; không thấy floor 4 m/s² của nhánh có track được áp dụng tại đây.
- [`main.py`](./main.py): CRI được tính trên lệnh NMPC và PP trước hậu xử lý; sau khi chọn mode, `last_mv` được gán lệnh đã chọn cũng trước khi áp dụng floor/kẹp. Vì vậy lệnh cuối cùng thực thi có thể khác lệnh được CRI đánh giá và lệnh được dùng làm đầu vào điều khiển bước sau. Cần đồng bộ hoặc giải thích/đánh giá sai khác này.
- [`NMPC.py`](./NMPC.py): solver hiện dùng projected gradient với số vòng lặp nhỏ và giới hạn hộp cho đầu vào. Các giới hạn trạng thái được khai báo trong mô hình không tự động trở thành ràng buộc trong bài toán tối ưu. Cần kiểm tra tiêu chuẩn hội tụ, nghiệm tốt nhất và cost được trả về sau lần cập nhật cuối; không xem cờ hội tụ hiện có là bằng chứng nghiệm tối ưu hay khả thi vật lý.
- [`cri.py`](./cri.py): các phép tính optimality/feasibility/superiority hiện chưa tương ứng hoàn toàn với rollout closed-loop của hai controller. Một số phép dự báo giữ lệnh cố định trong horizon, trong khi NMPC thực tế tái tối ưu và PP cập nhật lệnh theo trạng thái. CRI cũng chưa dùng covariance UKF để phản ánh bất định mục tiêu. Cần làm cho đánh giá candidate dùng cùng trạng thái, dự báo, thời hạn và giới hạn vật lý, đồng thời đại diện đúng cách vận hành NMPC và PP.
- [`main.py`](./main.py): trong luồng đã rà soát, cấu hình chạy tập trung vào một kịch bản, seed cố định và FOV 60 độ; chưa thấy distance-based blending. Nếu bài báo trình bày blending hoặc kết quả 360 độ, cần truy đúng entry point, cấu hình và dữ liệu sinh các kết quả đó trước khi sửa nội dung hoặc tái lập bảng.
- [`main.py`](./main.py) và [`scenario.py`](./scenario.py): cần kiểm tra đồng bộ thời điểm cập nhật mục tiêu, ego, estimate và log. Luồng hiện tại có nguy cơ ghép trạng thái ego trước khi tiến bước với trạng thái mục tiêu sau khi tiến bước khi tính khoảng cách/điều khiển. Xác nhận bằng timestamp thống nhất trước khi dùng lại metric hoặc kết quả interception.
- [`ukf.py`](./ukf.py) và [`tracker.py`](./tracker.py): covariance/noise cần được đối chiếu với độ lệch chuẩn mà sensor sinh ra; phép lấy trung bình sigma cho góc cần xử lý tính tuần hoàn thay vì dùng trung bình tuyến tính gần điểm đổi dấu góc. Đánh giá ảnh hưởng bằng sai số ước lượng và độ nhạy, không mặc định cấu hình hiện tại đã hiệu chuẩn.
- [`pure_pursuit.py`](./pure_pursuit.py) và [`fallback.py`](./fallback.py): PP và simple fallback có giới hạn gia tốc dọc `[-4, 8] m/s²` và góc lái `[-0.45, 0.45] rad`; các giới hạn này không tự đảm bảo giới hạn gia tốc ngang/lực lốp.
- [`main.py`](./main.py): gia tốc ngang được lưu bằng công thức `v² tan(delta) / L` với tốc độ toàn phần và được lấy trị tuyệt đối. Đây là đại lượng ước tính theo công thức động học, không phải lực lốp hoặc gia tốc ngang tính trực tiếp từ động lực học bicycle đang mô phỏng. Cần kiểm tra công thức/đơn vị và đối chiếu với đại lượng động lực học trước khi so sánh với số liệu trong bài.
- [`main.py`](./main.py): cấu hình nhìn thấy trong file dùng `np.random.seed(1)` và tạo sensor với `fov_deg=60`. Chưa xác định từ phần mã đã rà soát cấu hình tạo kết quả 360°, các noise setting tương ứng, hoặc script sinh Table 5/Table 14; cần truy vết cấu hình và dữ liệu trước khi khẳng định các bảng có thể tái tạo từ entry point này.
- [`metric.py`](./metric.py): đã có metric PEI, khoảng cách nhỏ nhất trong `calculate_pei`, tracking distance/time theo ngưỡng và proximity-time theo các vùng khoảng cách. Chưa thấy metric contact dựa trên footprint sedan/target; một số metric có hằng số/ngưỡng hard-coded nên cần trình bày riêng các đại lượng cơ sở, không dùng PEI làm bằng chứng duy nhất.
- [`run_all_experiments.py`](./run_all_experiments.py) và các file summary hiện có thuộc nhánh thí nghiệm RLS/AutoCar, không phải runner hay bằng chứng kết quả cho interception NMPC–PP. Không dùng các kết quả đó thay cho số liệu interception. Trong workspace đã rà soát chưa tìm thấy thư mục `src`/`tests` hay bản thảo `.tex`, `.bib`, `.pdf` hoặc `.docx`; cần xác nhận đây có phải đúng phiên bản workspace đã tạo bài báo hay không.

### Phân loại nhanh theo review

- **Đã có, chủ yếu cần mô tả rõ hơn:** 4 thành phần CRI và trọng số; hysteresis với dwell theo số lần gọi; giới hạn hộp NMPC và giới hạn PP/fallback; động lực học bicycle phi tuyến với luật lực lốp tuyến tính; luật hậu xử lý gia tốc; UKF; các metric khoảng cách và PEI.
- **Đã có nhưng chưa đủ để đáp ứng reviewer:** dwell theo số lần gọi nhưng chưa có thống kê/đánh giá thời gian; khoảng cách nhỏ nhất nhưng chưa có tiêu chí contact theo footprint; metric gia tốc ngang nhưng chưa xác nhận đúng đại lượng vật lý; kiểm tra feasibility theo box constraint nhưng chưa có tính khả thi lốp.
- **Có vẻ không khớp mô tả bài báo hoặc cần xác nhận phiên bản:** distance-based blending chưa thấy trong luồng điều khiển hiện tại; cấu hình 360°/các bảng được reviewer nêu chưa truy được từ entry point đã xem.
- **Cần bổ sung code/thí nghiệm hoặc chứng minh trong bài:** đánh giá miền hợp lệ của luật lực lốp hiện tại và bổ sung saturation/friction constraint phù hợp; plant chi tiết hơn chỉ khi cần làm mô hình đối chiếu; tiêu chí contact terminal theo footprint; xử lý nhất quán lệnh trước và sau hậu xử lý; xác minh hội tụ và ràng buộc solver; hiệu chuẩn/đánh giá CRI trên kịch bản tách biệt; đồng bộ timestamp và hiệu chuẩn UKF; ablation floor; log chuyển mode/dwell; phân tích ổn định hình thức nếu muốn đưa ra tuyên bố tương ứng.

### Phạm vi mô hình được chọn

- **Loại xe:** một sedan tham chiếu, với một bộ tham số cố định có nguồn/biện minh. Không chọn nhiều phân khúc xe trong vòng sửa này.
- **Mô hình plant:** giữ mô hình động lực học bicycle phi tuyến hiện có làm baseline; thẩm định và bổ sung luật lực lốp bão hòa/giới hạn ma sát. Chỉ cân nhắc plant chi tiết hơn làm mô hình đối chiếu nếu cần và có đủ nguồn lực; không xem thay plant là điều kiện bắt buộc từ đầu.
- **Mô hình điều khiển:** có thể dùng mô hình dự đoán rút gọn trong NMPC nếu được mô tả và đối chiếu với plant đánh giá. Cần định lượng ảnh hưởng của sai khác mô hình.
- **Va chạm trong nghiên cứu:** chỉ dùng giao nhau footprint như sự kiện terminal trong mô phỏng. Dừng mô phỏng tại sự kiện; không phân tích/điều chỉnh để tăng vận tốc va chạm hoặc mức độ phá hủy.
- **Cảm biến:** dùng một cấu hình camera mô phỏng có thông số giả định/nguồn rõ ràng (FOV, độ trễ, tần số cập nhật, nhiễu và điều kiện phát hiện). Không gọi một camera là “tốt nhất”; đánh giá độ nhạy với các điều kiện quan sát.

## 3. Kế hoạch theo ưu tiên

### Ưu tiên 0 — Chốt phiên bản và lập đường cơ sở

**Mục đích:** bảo đảm mọi thay đổi có thể truy nguyên tới kết quả bài báo.

1. Xác nhận mã, tham số, seed, kịch bản và phiên bản bản thảo đã tạo các bảng được reviewer nêu.
2. Lập bảng liên kết: *số bảng/hình trong bài → script chạy → file dữ liệu → metric → controller/config*.
3. Xác nhận mọi trạng thái được ghép theo cùng timestamp: trạng thái target thật, measurement, estimate UKF, trạng thái ego, lệnh điều khiển và metric. Sửa thứ tự cập nhật nếu đang so sánh trạng thái ở các thời điểm khác nhau.
4. Chạy lại nguyên trạng một lần để kiểm tra có tái tạo được các con số chính hay không; lưu log và cấu hình.
5. Ghi nhận số lần chuyển NMPC/PP, phân phối CRI và từng thành phần, trạng thái solver, lệnh đề xuất/cuối cùng, thời gian ở mỗi mode, thời gian mô phỏng, khoảng cách nhỏ nhất, gia tốc ngang và gia tốc dọc để có baseline trước khi sửa.
6. Không ghi đè dữ liệu cũ. Lưu kết quả mới theo tên phiên bản/cấu hình có thể phân biệt rõ.

**Điều kiện hoàn tất:** có thể tái tạo các kết quả nền hoặc đã giải thích được sai khác; có danh sách đầy đủ các bảng/hình chịu ảnh hưởng.

### Ưu tiên 1 — Làm rõ tính khả thi vật lý của chuyển động

**Phản hồi trọng tâm:** các gia tốc ngang 16.02 và 28.87 m/s² rất cao với sedan; code dùng động lực học bicycle phi tuyến nhưng lực lốp tuyến tính theo góc trượt và chưa giới hạn lực, nên cần xác minh các đại lượng gia tốc và tính khả thi của luật lốp.

#### Công việc kỹ thuật

1. Kiểm tra lại đơn vị, cách tính, dữ liệu và điều kiện chạy đã tạo ra các giá trị reviewer nêu. Phân biệt rõ:
   - gia tốc ngang động học từ vận tốc/quỹ đạo;
   - gia tốc ngang của mô hình động lực học từ phương trình trạng thái/lực lốp;
   - gia tốc đo được nếu có dữ liệu cảm biến.
2. Chốt một sedan tham chiếu và truy nguyên từng tham số tới tài liệu, dữ liệu đo hoặc giả định được công khai. Nếu không có dữ liệu hiệu chuẩn, gọi đây là cấu hình tham chiếu trong mô phỏng, không phải một mẫu xe đã được xác nhận.
3. Giữ mô hình bicycle phi tuyến hiện tại làm baseline, rồi bổ sung/kiểm tra mô hình lực lốp có bão hòa và ràng buộc ma sát. Chỉ dùng plant chi tiết hơn như mô hình đối chiếu nếu việc đó cần thiết để xác nhận hành vi lốp và có đủ tham số; nếu không, giới hạn tuyên bố theo mô hình hiện có.
4. Xác định giới hạn dựa trên thông số sedan và giả định mặt đường, không chọn hệ số ma sát hoặc giới hạn gia tốc chỉ để làm đẹp kết quả.
5. Làm rõ cách biểu diễn lực dọc trước khi gọi một ràng buộc là friction circle/ellipse. Plant hiện nhận gia tốc dọc như đầu vào trực tiếp, chưa phân bổ lực dọc cho từng lốp; do đó ràng buộc tổng hợp trên gia tốc chỉ là xấp xỉ nếu chưa bổ sung mô hình lực dọc/phân bổ lực. Hoặc bổ sung mức mô hình lực cần thiết để diễn giải ràng buộc lốp, hoặc gọi đúng tên và giới hạn kết luận của xấp xỉ được chọn.
6. Đưa tính khả thi vào bài toán điều khiển và/hoặc kiểm tra plant. Tùy mô hình và khả năng solver, xem xét:
   - giới hạn gia tốc ngang;
   - giới hạn lực ngang từng lốp/cầu, phù hợp với mức chi tiết mô hình;
   - friction circle/ellipse kết hợp gia tốc dọc và ngang;
   - giới hạn góc trượt để tránh dùng mô hình lốp tuyến tính ngoài vùng hợp lệ.
7. Nếu dùng ràng buộc phi tuyến nhưng solver hiện tại chỉ chiếu lên giới hạn hộp, cần thay đổi phương pháp tối ưu hoặc dùng cách xử lý ràng buộc được kiểm chứng. Xác minh cả hội tụ của solver lẫn tính khả thi của lệnh cuối; không coi giới hạn trạng thái khai báo trong plant là ràng buộc NMPC nếu optimizer không áp đặt chúng.
8. Áp dụng cùng giới hạn vật lý cho NMPC, PP/fallback và các baseline; nếu mỗi controller xử lý khác nhau, phải nêu lý do và đánh giá ảnh hưởng.
9. Nếu có post-processing để đảm bảo tính khả thi, ghi rõ thứ tự xử lý và kiểm tra đầu ra cuối cùng sau toàn bộ các phép giới hạn.

#### Công việc thực nghiệm và báo cáo

- Chạy lại mọi kịch bản và bảng chịu tác động.
- Báo cáo tối thiểu gia tốc ngang cực đại, RMS/phân vị cao, số bước hoặc tỷ lệ thời gian vượt giới hạn; đồng thời báo cáo gia tốc dọc và mức sử dụng giới hạn lốp nếu có.
- Báo cáo cả hiệu năng interception và tính khả thi; không chỉ đưa một chỉ số trung bình có thể che khuất các đỉnh lớn.
- So sánh trước/sau khi áp dụng ràng buộc, bao gồm tỷ lệ thành công, thời gian bắt mục tiêu, khoảng cách nhỏ nhất và các đại lượng điều khiển.
- Nếu mô hình hiện tại không đủ để xác nhận lực lốp thực tế, nêu giới hạn này rõ trong phần Limitations; không diễn giải gia tốc động học như bằng chứng lực lốp đã bão hòa hoặc vẫn khả thi.

**Điều kiện hoàn tất:** con số gia tốc được tính và định nghĩa nhất quán; mọi bảng bị ảnh hưởng đã chạy lại; các vi phạm và giả định mô hình được công khai.

### Ưu tiên 2 — Định nghĩa sự kiện interception trong mô phỏng

**Phản hồi trọng tâm:** điều kiện tâm xe cách nhau dưới 4 m không biểu diễn rõ va chạm giữa hai thân xe. Trong phạm vi bài này, interception được mô phỏng như một sự kiện terminal hình học; không diễn giải nó thành phép đánh giá độ an toàn hay mức độ va chạm ngoài đời.

#### Định nghĩa cần chốt

1. Xác định footprint xe sedan đánh chặn và footprint mục tiêu trong mặt phẳng, điểm tham chiếu của mỗi thân xe và quy ước hướng. Nếu mục tiêu dùng động học đơn giản thay vì plant sedan, ghi rõ điều đó.
2. Định nghĩa sự kiện contact bằng giao nhau của hai footprint trong mô phỏng; dừng mô phỏng ở sự kiện đầu tiên. Dùng kiểm tra liên tục/swept geometry giữa hai bước hoặc kiểm chứng hội tụ theo bước thời gian để tránh bỏ sót contact do rời rạc hóa. Tránh dùng khoảng cách giữa tâm xe làm đại diện cho va chạm.
3. Giữ tiêu chí cũ `d < 4 m` dưới tên **proximity event** để đối chiếu với bảng/kết quả cũ; không gọi riêng tiêu chí này là va chạm.
4. Ghi thời điểm contact và vận tốc tương đối tại thời điểm đó như thông tin mô tả để reviewer đánh giá fly-by và tính nhất quán của mô hình. Không dùng vận tốc tương đối để tối ưu mức độ va chạm, và không phân tích hư hại/động học sau va chạm.
5. Xác định cách xử lý mục tiêu bị mất khỏi FOV hoặc track không được xác nhận; báo cáo riêng các lần không quan sát được thay vì gán thành contact thành công.

#### Metric và thí nghiệm

- Báo cáo riêng tỷ lệ proximity theo tiêu chí cũ và tỷ lệ sự kiện contact theo footprint.
- Báo cáo thời điểm sự kiện, khoảng cách tâm và vận tốc tương đối tại contact như metric mô tả; không dùng các metric này để tối ưu mức độ va chạm.
- Kiểm tra độ nhạy của kết quả với kích thước footprint và chu kỳ mô phỏng để loại trừ lỗi phát hiện contact do rời rạc hóa.
- Dùng cùng định nghĩa đánh giá cho tất cả controller/baseline.

**Điều kiện hoàn tất:** contact là sự kiện terminal xác định bằng hình học, có thể tái lập; tiêu chí cũ và mới được đặt tên riêng, và phạm vi kết luận giới hạn ở mô phỏng.

### Ưu tiên 3 — Đồng bộ giới hạn NMPC với hậu xử lý gia tốc

**Phản hồi trọng tâm:** NMPC cho phép `a_x` từ -5 đến 8 m/s², trong khi lệnh áp dụng bị giới hạn dưới ở -4 m/s²; dưới 10 m/s lại bị ép lên ít nhất 4 m/s².

#### Công việc kỹ thuật

1. Vẽ/ghi lại luồng lệnh từ NMPC hoặc PP → chọn mode → hậu xử lý → cập nhật mô hình. Xác định mọi phép clip/floor ở các nhánh có tracking, mất tracking, fallback và baseline.
2. Quyết định chính sách thống nhất:
   - đưa các giới hạn thực sự áp dụng vào miền khả thi của optimizer nếu có thể; hoặc
   - giữ hậu xử lý nhưng đưa chính sách này thành một phần rõ ràng của bộ điều khiển và đánh giá nghiệm theo lệnh sau hậu xử lý.
3. Mặc định loại bỏ floor gia tốc tốc độ thấp nếu chưa có lý do vật lý/điều khiển được biện minh. Nếu xe cần duy trì tốc độ, thể hiện điều đó bằng mục tiêu tốc độ và giới hạn gia tốc trong bộ điều khiển, thay vì ghi đè lệnh sau khi CRI đã đánh giá.
4. Nếu vẫn giữ floor, giải thích lý do kỹ thuật và đánh giá nó có làm thay đổi bản chất nhiệm vụ hoặc che khuất lỗi điều khiển hay không; chạy ablation có/không có floor.
5. Đảm bảo lệnh cuối cùng luôn tuân thủ giới hạn gia tốc dọc và ràng buộc vật lý sau khi kết hợp với giới hạn lateral/friction.
6. Áp dụng cùng chính sách cho baseline. Nếu baseline không dùng floor hoặc dùng floor khác, so sánh không còn công bằng và phải được nêu rõ.

#### Ablation cần có

- Hybrid có và không có floor tốc độ thấp.
- Tách kết quả khi tốc độ thấp và khi tốc độ bình thường.
- So sánh lệnh do optimizer/fallback yêu cầu với lệnh cuối cùng thực sự áp dụng; báo cáo số bước và biên độ bị thay đổi.
- Nếu đổi miền NMPC (chẳng hạn giới hạn dưới từ -5 sang -4), đánh giá lại toàn bộ kết quả liên quan.

**Điều kiện hoàn tất:** miền tối ưu, luật hậu xử lý và lệnh thực thi không mâu thuẫn hoặc mâu thuẫn được mô tả/đánh giá minh bạch; baseline được xử lý thống nhất.

### Ưu tiên 4 — Phân tích CRI, hysteresis và hành vi hybrid

**Phản hồi trọng tâm:** CRI là tổ hợp heuristic, chưa có quan hệ hình thức với ổn định/an toàn; hysteresis hiện chưa đủ để kết luận ổn định.

#### Định nghĩa và hiệu chuẩn CRI

1. Giữ CRI là cơ chế trung tâm điều phối giữa NMPC và PP, nhưng không mặc định giữ nguyên cách diễn giải “độ tin cậy” nếu điểm hiện tại chưa được hiệu chuẩn.
2. Chốt CRI đang dự báo điều gì trong một khoảng thời gian xác định. Một hướng kiểm chứng được là dùng CRI như điểm đã hiệu chuẩn cho khả năng candidate NMPC vừa thỏa các điều kiện khả thi vừa có kết quả rollout tốt hơn candidate PP theo tiêu chí đã định nghĩa. Chỉ gọi CRI là xác suất nếu kiểm tra hiệu chuẩn trên dữ liệu độc lập hỗ trợ cách diễn giải đó; nếu không, gọi là calibrated decision score hoặc chỉ số ra quyết định.
3. Định nghĩa nhãn đánh giá trước khi tuning: điều kiện solver hợp lệ, các giới hạn vật lý được thỏa trong rollout và tiêu chí so sánh kết quả với PP. Không chọn nhãn hoặc trọng số sau khi xem kết quả test.
4. Tạo tập scenario/seed riêng cho tuning ngưỡng và trọng số, và tập khác để đánh giá cuối. Không chia ngẫu nhiên các bước thời gian liền kề của cùng một episode giữa hai tập vì sẽ gây rò rỉ thông tin.
5. So sánh hai controller bằng cùng trạng thái ban đầu, cùng dự báo mục tiêu, cùng thời hạn dự đoán và cùng giới hạn. Rollout cần phản ánh NMPC receding-horizon và PP cập nhật lệnh theo trạng thái; không dùng một lệnh hằng cho cả horizon nếu đó không phải hành vi thực tế của controller.
6. Đưa các lỗi cứng ra khỏi trung bình có trọng số: nghiệm không hữu hạn, solver thất bại, ràng buộc cứng bị vi phạm, hoặc dữ liệu target không còn hợp lệ phải kích hoạt nhánh xử lý rõ ràng. CRI cao không được ghi đè cổng khả thi/an toàn này; PP cũng phải được kiểm tra theo chính sách tương ứng.
7. Ghi log riêng từng thành phần CRI, lệnh ứng viên và lệnh cuối áp dụng, trạng thái solver, biên an toàn, covariance/độ bất định target, mode và lý do chuyển. Đánh giá CRI bằng các trường hợp chấp nhận nhầm/từ chối nhầm và chỉ số hiệu chuẩn phù hợp trên tập đánh giá.

#### Phân tích lý thuyết

1. Mô tả CRI đúng với bản chất: chỉ số heuristic để đánh giá chất lượng nghiệm/điều khiển theo các thành phần đã định nghĩa; không gọi CRI là xác suất tin cậy hay chứng minh an toàn nếu chưa hiệu chuẩn/thiết lập quan hệ tương ứng.
2. Làm rõ mỗi thành phần, chuẩn hóa, đơn vị, ngưỡng, trọng số và cách xử lý khi optimizer lỗi hoặc không hội tụ. Tách rõ kết quả hiệu chuẩn thực nghiệm khỏi bất kỳ bảo đảm hình thức nào.
3. Phân tích việc dùng `ExitFlag`, feasibility và giới hạn vật lý như điều kiện chặn độc lập. CRI trung bình cao không nên tự nó chấp nhận một lệnh vi phạm ràng buộc cứng.
4. Phân biệt ba mức kết luận trong bài:
   - ổn định/boundedness có chứng minh dưới giả thiết nêu rõ;
   - bảo đảm cục bộ/thực dụng nếu chỉ chứng minh trong tập trạng thái xác định;
   - xu hướng thực nghiệm nếu chỉ có mô phỏng.
5. Nếu có đủ cơ sở cho phân tích Lyapunov hybrid, xây dựng từ mô hình đóng vòng và các controller thực tế, nêu miền áp dụng, giả thiết target/quan sát, giới hạn mô hình và điều kiện chuyển. Chỉ viện dẫn multiple Lyapunov functions/minimum dwell time khi chứng minh được các bất đẳng thức tương ứng.
6. Nếu không thể chứng minh điều kiện đó, không trình bày một điều kiện tổng quát như định lý đã được thỏa mãn. Thay vào đó, giới hạn tuyên bố, bổ sung phân tích boundedness/thực nghiệm và nêu rõ đây là giới hạn nghiên cứu.

#### Hysteresis và dwell-time

- Ghi dwell dưới dạng thời gian thực, không chỉ số bước; kiểm tra liệu trạng thái mới được giữ tối thiểu đúng thời lượng dự định.
- Đánh giá các mức dwell-time đã chọn dựa trên chu kỳ lấy mẫu và đáp ứng xe, không chốt mặc định từ `n_dwell=2`.
- Ghi log mỗi lần chuyển, mode, CRI, lý do chuyển, thời gian ở mode trước/sau và số lần chuyển trên một đơn vị thời gian.
- Kiểm tra ảnh hưởng của dwell-time tới chattering, thời gian phản ứng, tỷ lệ interception và giới hạn vật lý.

#### Độ nhạy CRI và ablation

- Thay đổi có kiểm soát trọng số CRI và/hoặc ngưỡng `theta_on`, `theta_off`; giải thích dải thay đổi dựa trên mức hợp lý, không chọn sau khi nhìn kết quả tốt nhất.
- Báo cáo kết quả trung bình và độ phân tán qua nhiều seed/kịch bản.
- So sánh hybrid đầy đủ với các ablation hợp lý: không hysteresis/dwell, không từng thành phần CRI khi phù hợp, và NMPC-only/PP-only.
- Không loại bỏ ràng buộc an toàn cứng trong ablation nhằm tạo baseline yếu một cách thiếu công bằng.

**Điều kiện hoàn tất:** các tuyên bố ổn định được hỗ trợ đúng mức; dwell-time được đo và phân tích; có kết quả độ nhạy và ablation đủ để đánh giá vai trò của CRI.

### Ưu tiên 5 — Sửa bản thảo, hình và trích dẫn

1. Sửa Fig. 1 để nhãn/ký hiệu không chồng lấn; kiểm tra ở kích thước in thực tế và định dạng cuối của nhà xuất bản.
2. Rà soát câu mô tả biến, thêm vị ngữ rõ ràng. Ví dụ: “where \((X,Y)\) denotes the global position, \(\psi\) is the yaw angle, \(v_x\) and \(v_y\) are the longitudinal and lateral body-frame velocities, \(r\) is the yaw rate, \(a_x\) is the longitudinal acceleration, and \(\delta\) is the front steering angle.”
3. Sửa trích dẫn lặp “Kong et al. Kong et al. (2015)” bằng đúng cú pháp theo hệ thống trích dẫn đang dùng; kiểm tra toàn văn các trích dẫn tác giả/năm bị lặp.
4. Rà soát toàn bộ bài cho câu thiếu vị ngữ, ký hiệu chưa định nghĩa, đơn vị, dấu thập phân, tên controller, viết tắt, tham chiếu hình/bảng và nhất quán giữa nội dung với kết quả mới.
5. Cập nhật phần mô hình, hàm mục tiêu/ràng buộc, tiêu chí interception, hậu xử lý và metric đúng theo mã chạy cuối cùng.
6. Kiểm tra mọi con số trong abstract, bảng, hình, caption và conclusion có khớp dữ liệu mới; xóa hoặc đánh dấu rõ kết quả cũ không còn dùng.

**Điều kiện hoàn tất:** kiểm tra từng hình/bảng ở bản xuất cuối và đối chiếu mọi số liệu với file nguồn.

### Ưu tiên 6 — Trả lời Reviewer 2 và hoàn thiện hồ sơ nộp lại

Reviewer 2 cho rằng vòng trước chưa có cập nhật đáng kể. Cần đáp lại bằng các thay đổi có thể kiểm chứng, không chỉ bằng lời cam kết.

1. Tạo response letter theo từng reviewer và từng ý, bao gồm cả góp ý từ vòng trước nếu có.
2. Với mỗi nhận xét, dùng cấu trúc:
   - **Comment:** trích hoặc tóm tắt chính xác ý reviewer;
   - **Response:** trả lời trực tiếp, lịch sự, nêu giới hạn nếu có;
   - **Changes made:** thay đổi cụ thể trong mô hình, thuật toán, thí nghiệm hoặc văn bản;
   - **Location:** mục, phương trình, hình/bảng và số trang/dòng trong bản thảo đã đánh dấu.
3. Chỉ tuyên bố “đã thêm” hoặc “đã chứng minh” khi thay đổi đã có trong bản thảo và dữ liệu chạy xác nhận.
4. Cung cấp bản thảo có đánh dấu thay đổi và bản clean nếu tạp chí yêu cầu.
5. Đảm bảo bảng phản hồi liên kết đến phiên bản cuối, không dùng số trang/dòng của bản trước khi dàn trang.

## 4. Ma trận nhận xét – hành động – bằng chứng

| Reviewer | Mối quan tâm | Thay đổi cần có | Bằng chứng phải chuẩn bị |
|---|---|---|---|
| 1.1 | CRI heuristic; rollout chưa phản ánh đầy đủ NMPC/PP thực tế; thiếu phân tích ổn định/boundedness; dwell chưa đo theo thời gian | Giữ CRI làm cơ chế chuyển mode; hiệu chuẩn/đánh giá trên tập scenario tách biệt; tách hard feasibility gate; ghi dwell-time; chỉ đưa chứng minh nếu có đủ giả thiết | Kết quả hiệu chuẩn và đánh giá độc lập; log thành phần CRI, solver, lý do chuyển; thống kê dwell/chattering và ablation |
| 1.2 | Gia tốc ngang rất cao; thiếu giới hạn ma sát/lốp; metric hiện tại là ước lượng động học | Xác minh đại lượng gia tốc; làm rõ mô hình lực dọc/lốp; bổ sung giới hạn phù hợp với plant và áp dụng nhất quán | Bảng max/RMS/phân vị/vi phạm; mức sử dụng giới hạn lốp; kết quả chạy lại hiệu năng interception |
| 1.3 | `d < 4 m` một bước không phân biệt proximity với va chạm giữa thân xe | Dùng contact terminal theo footprint; giữ `d < 4 m` làm proximity metric để đối chiếu | Tỷ lệ proximity/contact; thời điểm contact và vận tốc tương đối tại sự kiện dưới dạng mô tả; kiểm tra phụ thuộc bước thời gian |
| 1.4 | Miền NMPC, lệnh CRI đánh giá và lệnh sau hậu xử lý không nhất quán; floor dưới 10 m/s có thể thiên lệch | Đồng bộ candidate → CRI → lệnh áp dụng; bỏ floor nếu không có lý do; dùng cùng chính sách cho baseline | Ablation floor; thống kê lệnh bị thay đổi; kết quả baseline chạy cùng chính sách |
| 1.5 | Fig. 1 chồng nhãn; câu thiếu vị ngữ; trích dẫn lặp | Sửa hình, tiếng Anh, trích dẫn và rà soát toàn bản | Bản thảo clean/marked; kiểm tra hình xuất bản; checklist biên tập |
| 2 | Không có cập nhật đáng kể từ vòng trước | Thực hiện thay đổi kỹ thuật/thực nghiệm có thể kiểm chứng và phản hồi từng điểm | Response letter có vị trí thay đổi, bảng/hình mới, bản thảo có đánh dấu |

## 5. Kế hoạch chạy thí nghiệm và quản lý kết quả

1. **Baseline tái lập:** chạy lại đúng cấu hình cũ trước mọi sửa đổi.
2. **Bản sedan tham chiếu:** cố định nguồn tham số cho mô hình bicycle phi tuyến, mô hình lốp và giả định mặt đường; ghi rõ giới hạn của plant và liệu có dùng mô hình đối chiếu chi tiết hơn hay không.
3. **Bản khả thi vật lý:** thêm giới hạn đã được xác định, chạy lại tất cả kịch bản và baseline.
4. **Đánh giá contact terminal:** tính lại proximity event cũ và contact theo footprint trên cùng dữ liệu; dừng mô phỏng ở contact đầu tiên.
5. **Ablation acceleration floor:** giữ nguyên mọi yếu tố khác và so sánh có/không có floor.
6. **Ablation switching/CRI:** so sánh các cấu hình chuyển mode, thành phần CRI và ngưỡng; ghi số lần switch, dwell-time, lý do chuyển, chấp nhận nhầm/từ chối nhầm.
7. **Hiệu chuẩn CRI:** chọn ngưỡng/trọng số trên tập scenario/seed tuning; đánh giá cuối trên tập scenario/seed khác. So sánh CRI với NMPC-only và PP-only, đồng thời báo cáo hiệu năng lẫn khả năng dự báo tính khả thi/lợi thế của NMPC.
8. **Độ nhạy mô hình/cảm biến:** thay đổi có kiểm soát tham số xe và giả định camera trong phạm vi đã biện minh; tách bộ tuning khỏi bộ đánh giá.
9. **Độ lặp:** chạy nhiều seed hoặc điều kiện ban đầu có kiểm soát; nêu số lần thử và cách tổng hợp (mean/std, median/IQR hoặc khoảng tin cậy thích hợp).
10. **Lưu cấu hình:** với mỗi run, lưu controller, tham số xe, chu kỳ lấy mẫu, ngưỡng, seed, commit/version nếu có, kết quả metric và log cần thiết.
11. **Cập nhật bài:** sinh bảng/hình từ kết quả mới thay vì nhập số thủ công; kiểm tra từng số trong phần tóm tắt và kết luận.

## 6. Các quyết định cần chốt trước khi đóng băng bản sửa

- Kích thước/định nghĩa footprint sedan và mục tiêu trong mô phỏng; cách phát hiện giao nhau của hai footprint.
- Bộ tham số sedan tham chiếu, nguồn/hiệu chuẩn, hệ số ma sát và miền hợp lệ của luật lực lốp.
- Mô hình bicycle phi tuyến hiện tại có cần đối chiếu với plant/benchmark chi tiết hơn hay không; nếu có, cách định lượng sai khác.
- Cách áp đặt giới hạn friction circle/ellipse với solver hiện tại.
- Lý do vật lý của gia tốc floor dưới tốc độ thấp và việc có giữ luật này trong bản cuối hay không.
- Cấu hình camera mô phỏng, nguồn/giả định FOV, tần số cập nhật, độ trễ và nhiễu; không tuyên bố có một cấu hình camera “tốt nhất” chung.
- Định nghĩa sự kiện mà CRI dự báo, cách đưa bất định UKF/model vào đánh giá candidate, cách kiểm tra calibration và operating point/false acceptance được chấp nhận.
- Cách chia episode thành tập tuning và evaluation; cách xác định tham số bộ điều khiển, trọng số/ngưỡng CRI và dwell-time để tránh tuning theo test set.
- Khả năng chứng minh feasibility đệ quy, boundedness hoặc stability theo đúng mô hình đóng vòng và logic switching thực tế. Nếu không đủ điều kiện, cần chốt rõ phạm vi tuyên bố thực nghiệm.
- Luật lốp và cách biểu diễn lực dọc đủ nhất quán để tuyên bố giới hạn lốp/ma sát; nếu chưa đủ, chọn mức mô hình hóa phù hợp hoặc giới hạn tuyên bố.
- Bản thảo nguồn, thư phản biện vòng trước và yêu cầu định dạng của tạp chí/hội nghị.

## 7. Thứ tự thực hiện đề xuất

1. Xác nhận đúng phiên bản source/kịch bản sinh bảng; đồng bộ timestamp giữa target, ego, measurement, estimate và metric; tái lập baseline cũ trước khi thay đổi.
2. Chốt bài toán, tiêu chí interception theo footprint, mục tiêu và miền hoạt động được tuyên bố.
3. Chốt plant sedan, luật lốp, tham số có nguồn/nhận dạng và giới hạn vật lý; xác minh định nghĩa gia tốc và phạm vi solver.
4. Thiết kế lại NMPC/PP và safety gate sao cho dùng chung giới hạn; bỏ hậu xử lý ẩn; kiểm tra khả thi của lệnh cuối và phản ánh đúng rollout mỗi controller.
5. Định nghĩa sự kiện CRI dự báo; tạo quy trình đánh giá/hiệu chuẩn từ uncertainty, scenario/seed tách biệt; chọn switching/hysteresis theo tiêu chí đã công bố.
6. Chốt mức chứng minh có thể đạt được cho feasibility, boundedness hoặc stability; đảm bảo giả thiết bao phủ controller, target uncertainty và logic chuyển mode.
7. Chạy baseline, ablation, độ nhạy và đánh giá độc lập nhiều seed; sinh lại mọi bảng/hình từ log và cấu hình mới.
8. Sửa Fig. 1, ngữ pháp, ký hiệu và trích dẫn; viết response letter theo từng comment và đối chiếu từng tuyên bố với bằng chứng.
9. Kiểm tra cuối: giả thiết → method → code/config → dữ liệu → bảng/hình → bản thảo → response letter phải nhất quán.

## 8. Checklist hoàn tất trước khi nộp

- [ ] Tái lập baseline hoặc giải thích sai khác.
- [ ] Phạm vi được giới hạn rõ ở sedan tham chiếu trong mô phỏng; không khái quát hóa cho mọi xe.
- [ ] Mô hình cuối được mô tả đúng theo luật lực lốp thực sự đã cài đặt; miền hợp lệ, bão hòa và giới hạn vật lý được nêu rõ.
- [ ] Phiên bản method cuối đã xác định rõ luật lốp có bão hòa/giới hạn ma sát và cách biểu diễn lực dọc; nếu chỉ dùng xấp xỉ, giới hạn đó được nêu minh bạch.
- [ ] Kiểm tra lại các giá trị gia tốc được reviewer nêu và cách tính gia tốc ngang.
- [ ] Ràng buộc/giới hạn vật lý được mô tả chính xác, áp dụng đúng và kiểm tra trên lệnh cuối.
- [ ] Mọi controller/baseline được đánh giá với chính sách giới hạn công bằng.
- [ ] Contact terminal được xác định tái lập bằng footprint; tiêu chí proximity cũ được giữ riêng để đối chiếu.
- [ ] Vận tốc tương đối tại contact chỉ được báo cáo mô tả, không dùng để tối ưu mức độ va chạm hoặc suy luận hư hại.
- [ ] Kết quả cũ và mới được phân biệt, mọi bảng/hình chịu ảnh hưởng đã chạy lại.
- [ ] Dwell-time được biểu diễn theo thời gian và có thống kê chuyển mode.
- [ ] CRI được giữ làm cơ chế chuyển mode, có mục tiêu/nhãn đánh giá rõ; cách diễn giải là chỉ số, decision score hay xác suất phù hợp với kết quả hiệu chuẩn độc lập.
- [ ] Candidate NMPC/PP được đánh giá theo cùng điều kiện và rollout phản ánh đúng cách vận hành; hard feasibility gate không bị điểm CRI ghi đè.
- [ ] Timestamp của target, ego, measurement, estimate, lệnh điều khiển và metric nhất quán.
- [ ] Solver được đánh giá bằng trạng thái hội tụ và tính khả thi thực tế; giới hạn khai báo nhưng không được tối ưu áp đặt không bị mô tả nhầm là constraint NMPC.
- [ ] Tham số vật lý, cảm biến, bộ điều khiển và switching được phân loại theo nguồn/nhận dạng/thiết kế/hiệu chuẩn; rationale và độ nhạy được báo cáo.
- [ ] Tập tuning và evaluation được tách theo episode/scenario/seed; không chọn ngưỡng CRI theo test set.
- [ ] Mức chứng minh feasibility/boundedness/stability đúng với giả thiết và toàn bộ logic hybrid được cài đặt.
- [ ] Có phân tích độ nhạy/ablation đủ trả lời vai trò CRI, hysteresis và acceleration floor.
- [ ] Tuyên bố stability/safety/boundedness không vượt quá bằng chứng.
- [ ] Fig. 1 không chồng nhãn; lỗi predicate và trích dẫn lặp đã được rà soát toàn bài.
- [ ] Response letter trả lời từng ý, dẫn đúng vị trí thay đổi trong bản thảo cuối.
- [ ] Các số liệu trong abstract, nội dung, bảng, hình, kết luận và response letter đồng nhất.
