import struct
import csv

INPUT_FILE = "log.bin"
OUTPUT_FILE = "log.csv"

rec = struct.Struct("<I8f")

with open(INPUT_FILE, "rb") as fin, open(OUTPUT_FILE, "w", newline="") as fout:
    writer = csv.writer(fout)
    writer.writerow(["t_us","p1","p2","p3","p4","p5","p6","p7","p8"])

    while True:
        data = fin.read(rec.size)
        if len(data) == 0:
            break
        if len(data) != rec.size:
            print("partial record:", len(data))
            break

        row = rec.unpack(data)
        writer.writerow(row)