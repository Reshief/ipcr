#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Script to plot before and after statistics of area')
    parser.add_argument('input_path',
                        help='Path to the "_mapping.txt" mapping output')
    parser.add_argument('input_before',
                        help='Path to the full input areas of before')
    parser.add_argument('input_after',
                        help='Path to the full input areas of after')
    parser.add_argument('-o','--output_path', default=None,
                        help='Optional path to write output files to. Defaults to current directory')

    args = parser.parse_args()

    input_path = args.input_path
    input_before_path = args.input_before
    input_after_path = args.input_after

    output_path = args.output_path

    if output_path is None:
        output_path = "./"
    
    before_data = np.loadtxt(input_before_path, ndmin=1)
    after_data = np.loadtxt(input_after_path, ndmin=1)

    mapping_data = np.loadtxt(input_path, ndmin=2, dtype=np.int32)


    mapped_before = []
    mapped_after = []

    for i in range(len(mapping_data)):
        mapped_before.append(before_data[mapping_data[i,0]])
        mapped_after.append(after_data[mapping_data[i,1]])

    mapped_before = np.array(mapped_before)
    mapped_after = np.array(mapped_after)

    ratio = mapped_after/mapped_before

    hist, edges = np.histogram(ratio)

    plt.hist(ratio, bins='auto')  # arguments are passed to np.histogram
    plt.title("Histogram with 'auto' bins")
    plt.savefig(output_path+"1d_histogram.pdf")

    plt.clf()
    
    H, xedges, yedges = np.histogram2d(mapped_before, mapped_after, bins='auto')
    H = H.T
    fig, ax1 = plt.subplots(ncols=1)

    ax1.pcolormesh(xedges, yedges, H, cmap='rainbow')
    ax1.plot(x, 2*np.log(x), 'k-')
    ax1.set_xlim(mapped_before.min(), mapped_before.max())
    ax1.set_ylim(mapped_after.min(), mapped_after.max())
    ax1.set_xlabel('before')
    ax1.set_ylabel('after')
    ax1.set_title('Area histogram')

    ax1.grid()
    fig.savefig(output_path+"2d_histogram.pdf")
