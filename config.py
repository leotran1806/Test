# cấu hình tầng điều phối (đặc tả mục 10)
# file này KHÔNG import gì trong package
# goalRule: luật giải X4 (2 robot tranh 1 đích quay lui)
#     'paper'   = khóa (-v, theta, ID) đúng ĐX (robot ở xa hơn thắng)
#     'nearest' = khóa (v, ID), robot gần hơn thắng
# thetaEps: làm tròn theta (và v) khi so ưu tiên, tránh sai số dấu phẩy động
# deadlockK: số bước bị chặn OCCUPIED liên tiếp trước khi xử lý kẹt
# tabuSteps: số bước robot ko được chọn lại đích vừa bỏ (R2a, R2b)
# lockMaxSteps: khóa nhường đường tự hết hạn sau số bước này (an toàn)
# maxSteps: chặn vòng lặp vô hạn khi test -> RuntimeError
# checkInvariants: kiểm tra bất biến mỗi bước (tắt khi chạy thực nghiệm lớn)
# filterNearCover: (Mức 2) bỏ cặp có c_u nằm trên đoạn thẳng sắp đi của robot đang quét
# strictDiagonal: (bổ sung, mặc định tắt = đúng đặc tả) robot ở 2 ô cạnh của bước chéo cũng tính là xung đột
#     giống luật "ko cắt góc" với vật cản: thân robot đi chéo sẽ chạm robot đứng ở ô cạnh

from dataclasses import dataclass


@dataclass
class CoordConfig:
    goalRule: str = 'paper'
    thetaEps: float = 1e-9
    deadlockK: int = 2
    tabuSteps: int = 10
    lockMaxSteps: int = 60
    maxSteps: int = 100000
    checkInvariants: bool = True
    filterNearCover: bool = False
    strictDiagonal: bool = False

    def __post_init__(self):
        if self.goalRule not in ('paper', 'nearest'):
            raise ValueError("goalRule phai la 'paper' hoac 'nearest'")
        if self.thetaEps <= 0:
            raise ValueError("thetaEps phai > 0")
        if self.deadlockK < 1:
            raise ValueError("deadlockK phai >= 1")
        if self.tabuSteps < 0 or self.lockMaxSteps < 1 or self.maxSteps < 1:
            raise ValueError("tabuSteps >= 0, lockMaxSteps >= 1, maxSteps >= 1")
