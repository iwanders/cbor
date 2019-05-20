#!/usr/bin/env python3

import glob
import re
import os

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
    files = glob.glob("*.cpp")
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
                possible_inputs.append(("file_" + file_name + "_l_{}.bin".format(lnum), sample))
    outdir = "/tmp/fuzz_corpus/"
    #os.makedirs(outdir)
    for name, input in possible_inputs:
        with open(os.path.join(outdir, name), "wb") as f:
            f.write(input)
