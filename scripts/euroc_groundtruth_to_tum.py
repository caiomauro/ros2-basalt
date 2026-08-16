#!/usr/bin/env python3

# Copyright 2026 Caio Mauro
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Caio Mauro nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from pathlib import Path
import csv
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: euroc_groundtruth_to_tum.py <state_groundtruth_estimate0/data.csv> <output.tum>",
            file=sys.stderr,
        )
        return 2

    input_csv = Path(sys.argv[1]).expanduser()
    output_tum = Path(sys.argv[2]).expanduser()

    if not input_csv.is_file():
        print(f"input csv not found: {input_csv}", file=sys.stderr)
        return 1

    output_tum.parent.mkdir(parents=True, exist_ok=True)

    with input_csv.open(newline="") as fh, output_tum.open("w") as out:
        reader = csv.reader(fh)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            if len(row) < 8:
                continue

            timestamp_ns = int(row[0])
            tx = float(row[1])
            ty = float(row[2])
            tz = float(row[3])
            qw = float(row[4])
            qx = float(row[5])
            qy = float(row[6])
            qz = float(row[7])

            timestamp_s = timestamp_ns / 1e9
            out.write(
                f"{timestamp_s:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )

    print(f"wrote {output_tum}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
