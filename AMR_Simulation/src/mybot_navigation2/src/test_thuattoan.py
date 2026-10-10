from collections import defaultdict

words = ['a', 'b', 'a', 'c', 'b', 'a']
count = defaultdict(int)

for w in words:
    count[w] += 1   # không cần kiểm tra key đã tồn tại chưa

print(count)   # defaultdict(<class 'int'>, {'a': 3, 'b': 2, 'c': 1})