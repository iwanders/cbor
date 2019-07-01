#!/usr/bin/env python3

import glob
import re
import os
import sys

def convert_input(input):
    #  0xFB, 0x40, 0xCA, 0x20, 0x92, 0x19, 0x65, 0x2B, 0xD4 
    # 1, 2, 3, ...
    splitted = [z.strip() for z in input.split(",")]
    result = bytearray([])
    try:
        for k in splitted:
            if "0x" in k:
                result.append(int(k.replace("0x", ""), 16))
            else:
                result.append(int(k, 10))
    except ValueError:
        pass
    return result
    

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(sys.argv)
        print("./generate_fuzz_corpus.py INPUT_DIR OUTPUT_DIR")
        sys.exit(1)
    indir = sys.argv[1]
    outdir = sys.argv[2]
    files = glob.glob(os.path.join(indir, "*.cpp"))
    possible_inputs = []
    for file_name in files:
        with open(file_name) as f:
            lnum = 0
            for l in f:
                lnum += 1
                matches = re.findall("\{([^}\"]+?)\}", l)
                inputs = [convert_input(a) for a in matches]
                if (not len(inputs)):
                    continue
                sample = inputs[-1] # just take the last one, first one is empty most of the time.
                short_name = os.path.basename(file_name)
                possible_inputs.append(("file_" + short_name + "_l_{}.bin".format(lnum), sample))

    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    for name, input in possible_inputs:
        with open(os.path.join(outdir, name), "wb") as f:
            f.write(input)
