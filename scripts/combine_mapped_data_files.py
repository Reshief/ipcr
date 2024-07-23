#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Script to plot a before and after of the positions of cells in stretching to verify mapping."
    )
    parser.add_argument(
        "input_mapping", help='Path to the "_mapping.txt" mapping output'
    )
    parser.add_argument(
        "input_before",
        help="Path to the full input file associated with the before",
    )
    parser.add_argument(
        "input_after",
        help="Path to the full input file associated with the after",
    )
    parser.add_argument(
        "-o",
        "--output_path",
        required=True,
        help="Path to write output file to",
    )

    args = parser.parse_args()

    input_mapping = args.input_mapping
    input_before_path = args.input_before
    input_after_path = args.input_after

    output_path = args.output_path

    mappings = np.loadtxt(input_mapping, dtype=np.int32, ndmin=2)
    before_data = np.loadtxt(input_before_path, ndmin=2)
    after_data = np.loadtxt(input_after_path, ndmin=2)

    with open(output_path, "w") as out:
        for i in range(len(mappings)):
            out.write(
                "{}\t{}\t{}\t{}\n".format(
                    input_mapping[i, 0],
                    input_mapping[i, 1],
                    list(before_data[input_mapping[i, 0]]).join("\t"),
                    list(after_data[input_mapping[i, 1]]).join("\t"),
                )
            )
