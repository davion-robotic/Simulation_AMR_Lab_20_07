from collections import defaultdict


# ============================================================
# 1. LỚP NODE - đại diện cho 1 trạng thái trong không gian tìm kiếm
# ============================================================
class Node:
    def __init__(self, name, w=0, par=None):  # w: weight (trọng số)
        self.name = name
        self.par = par      # con trỏ tới node cha (dùng để truy vết đường đi)
        self.w = w

    def display(self):
        print(self.name, self.par)

    def __lt__(self, other):
        # dùng khi cần sắp xếp / hàng đợi ưu tiên theo trọng số (UCS, A*...)
        return self.w < other.w


# ============================================================
# 2. DỮ LIỆU ĐỒ THỊ (danh sách kề)
# ============================================================
data = defaultdict(list)
data['A'] = ['B', 'C', 'D']
data['B'] = ['E', 'F']
data['C'] = ['G', 'H']
data['D'] = ['I', 'J']
data['F'] = ['K', 'L', 'M']
data['H'] = ['N', 'O']


# ============================================================
# 3. CÁC HÀM HỖ TRỢ
# ============================================================
def equal(O, G):
    """Kiểm tra 2 node có phải cùng 1 trạng thái không"""
    return O.name == G.name


def checkInArray(tmp, Open):
    """Kiểm tra node tmp đã tồn tại trong danh sách Open chưa"""
    for x in Open:
        if equal(x, tmp):
            return True
    return False


def path(O):
    """In đường đi từ đích ngược về gốc (dựa vào con trỏ cha .par)"""
    print(O.name)
    if O.par is not None:
        path(O.par)
    else:
        return


# ============================================================
# 4. THUẬT TOÁN TÌM KIẾM THEO CHIỀU RỘNG (BFS)
# ============================================================
def BFS(S=Node('A'), G=Node('M')):
    Open = []
    Closed = []
    Open.append(S)

    while True:
        if len(Open) == 0:
            print('Tim kiem that bai')
            return

        O = Open.pop(0)        # lấy phần tử ĐẦU tiên (FIFO) -> đặc trưng của BFS
        Closed.append(O)
        print('Dang xet:', O.name)

        if equal(O, G):
            print('\nTim thay duong di! In duong di (tu dich ve goc):')
            path(O)
            return

        for x in data[O.name]:
            tmp = Node(x)
            tmp.par = O
            ok1 = checkInArray(tmp, Open)
            ok2 = checkInArray(tmp, Closed)
            if not ok1 and not ok2:
                Open.append(tmp)


# ============================================================
# 5. CHẠY THỬ CHƯƠNG TRÌNH
# ============================================================
if __name__ == '__main__':
    BFS(S=Node('A'), G=Node('M'))