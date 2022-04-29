
SCAN_HEADER = ['s', 'ns', 'amax', 'amin', 'ai', 'ti', 'st', 'rmin', 'rmax']

r = []
I = []
for i in range(1080):
    r += ['r' + str(i)]
    I += ['I' + str(i)]
SCAN_HEADER += r + I
print()
print(r)
print()
print(I)
print()
print(SCAN_HEADER)
print()