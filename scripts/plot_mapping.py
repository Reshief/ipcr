#!/usr/bin/env python3

import argparse
import numpy as np

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Script to plot a before and after of the positions of cells in stretching to verify mapping.')
    parser.add_argument('input_path', required=True,
                        help='Path to the "_all.txt" mapping output')
    parser.add_argument('-o','--output_path', default=None,
                        help='Optional path to write output files to. Defaults to current directory')

    args = parser.parse_args()

    input_path = args.input_path

    output_path = args.output_path
    

    cell_data = np.loadtxt(input_path, ndmin=2)

    # Sort by y position of before
    cell_data = cell_data[cell_data[:,2].argsort()]

    cell_data_before = cell_data[:,:3]
    cell_data_after = cell_data[:,:3]



    cell_index_before = cell_data[:,0]
    cell_position_before = cell_data[:,1:3]
    cell_index_after = cell_data[:,3]
    cell_position_after= cell_data[:,4:6]