import struct
import csv
import shutil
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
CSV_DIR = BASE_DIR / "csv"
RAW_DIR = BASE_DIR / "raw"

CSV_DIR.mkdir(exist_ok=True)
RAW_DIR.mkdir(exist_ok=True)

rec = struct.Struct("<I8f")

for input_file in sorted(BASE_DIR.glob("*.bin")):
    output_file = CSV_DIR / f"{input_file.stem}.csv"

    print(f"Processing {input_file.name} -> {output_file.name}")

    try:
        with open(input_file, "rb") as fin, open(output_file, "w", newline="") as fout:
            writer = csv.writer(fout)
            writer.writerow(["t_us","p1","p2","p3","p4","p5","p6","p7","p8"])

            while True:
                data = fin.read(rec.size)
                if len(data) == 0:
                    break
                if len(data) != rec.size:
                    print(f"partial record in {input_file.name}: {len(data)} bytes")
                    break

                row = rec.unpack(data)
                writer.writerow(row)

        shutil.move(str(input_file), str(RAW_DIR / input_file.name))
        print(f"Done: {input_file.name}")

    except Exception as e:
        print(f"Failed: {input_file.name}: {e}")