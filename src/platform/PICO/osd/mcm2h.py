import sys

if len(sys.argv) > 1:
    f = open(sys.argv[1], "r")
else:
    f = sys.stdin

mods = dict(
    (
        # MCM   FB bit-reversed
        ("00", "01"), # Black
        ("01", "00"), # Transparent
        ("10", "11"), # White
        ("11", "00")  # Transparent
    )
)

# https://www.analog.com/en/resources/design-notes/generating-custom-characters-and-graphics-by-using-the-max7456s-memory-and-ev-kit-file-formats.html
# This is the format of .mcm files for BetaFlight etc.
# each 3 bytes, first byte is left column of 4 pixels, 2nd byte is middle, 3rd byte is right
# each byte is 4 pixels as 4x 2 bits (see mods), with left to right being high to low
# so, for ease of sending buffer to fifo to screen ("wire order"), we want to reverse the order of the 4 pixels in each byte
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
    a=""
    for i in range(4):
        a += mods[x[2*i:2*(i+1)]]
    print(a[::-1], end="") # reverse order of bits in byte
    print(",", end="")
    if (rc != 2):
        print(" ", end="")
    row += 1
print("\n};")
