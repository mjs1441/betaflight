import sys

if len(sys.argv) > 1:
    f = open(sys.argv[1], "r")
else:
    f = sys.stdin

mods = dict(
    (
        ("00", "01"),
        ("01", "00"),
        ("10", "11"),
        ("11", "00")
    )
)

row = 0
header = f.readline().strip()
print("#include <stdint.h>")
print()
print("// %s font"%header)
print("const uint8_t fontData[18*3*256] = {", end="")
for x in f:
    z = row % 64
    if (z >= 54):
        row += 1
        continue
    rc = z % 3
    if (rc == 0):
        if (row > 0 and z == 0):
            print()
        print("\n    ", end="")
    print("0b", end="")
    for i in range(4):
        print(mods[x[2*i:2*(i+1)]], end="")
    print(",", end="")
    if (rc != 2):
        print(" ", end="")
    row += 1
print("\n};")

        
    
