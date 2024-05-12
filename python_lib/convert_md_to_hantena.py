"""通常のMarkdownファイルをはてな記法に変換するスクリプト
TexをはてなTex記法に変換するのがメイン
(1) $$を変換
$$
F = ma
$$
↓
<div align='center' class='scroll'>
[tex: \displaystyle
F = ma
]
</div>
(2) 文中の$x$を[tex: x]に変換
"""

import argparse
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_path', type=Path)
    return parser.parse_args()

if __name__=="__main__":
    args = parse_args()
    input_path = args.input_path
    output_path = input_path.parent / (input_path.stem + "_hantena" + input_path.suffix)

    in_f = open(input_path, 'r')
    lines = in_f.readlines()
    in_f.close()

    out_f = open(output_path, 'w')
    num_single_dollars = 0
    num_double_dollars = 0
    for line in lines:
        # まず$$を見つけたら変換
        while "$$" in line:
            pos = line.find("$$")
            if num_double_dollars % 2 == 0:
                line = line[:pos] + "<div align='center' class='scroll'>[tex: \\displaystyle" + line[pos+2:]
            else:
                line = line[:pos] + "]</div>" + line[pos+2:]
            num_double_dollars += 1
        # 残った$を変換
        while "$" in line:
            pos = line.find("$")
            if num_single_dollars % 2 == 0:
                line = line[:pos] + "[tex: " + line[pos+1:]
            else:
                line = line[:pos] + "]" + line[pos+1:]
            num_single_dollars += 1
        out_f.write(line)
    out_f.close()
