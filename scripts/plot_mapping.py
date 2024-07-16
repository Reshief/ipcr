#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Script to plot a before and after of the positions of cells in stretching to verify mapping.')
    parser.add_argument('input_path',
                        help='Path to the "_mapping.txt" mapping output')
    parser.add_argument('input_before',
                        help='Path to the full input positions of before')
    parser.add_argument('input_after',
                        help='Path to the full input positions of after')
    parser.add_argument('-o','--output_path', default=None,
                        help='Optional path to write output files to. Defaults to current directory')

    args = parser.parse_args()

    input_path = args.input_path
    input_before_path = args.input_before
    input_after_path = args.input_after

    output_path = args.output_path
    


    full_before = np.loadtxt(input_before_path)
    full_after = np.loadtxt(input_after_path)

    cell_data = np.loadtxt(input_path, ndmin=2, dtype=np.int32)

    cell_data_before = cell_data[:,0]
    cell_data_after = cell_data[:,1]

    mean_x_before = np.average(full_before[cell_data_before[0],0])
    mean_y_before = np.average(full_before[cell_data_before[0],1])

    mean_x_after = np.average(full_after[cell_data_after[0],0])
    mean_y_after = np.average(full_after[cell_data_after[0],1])

    cell_index_before = cell_data_before#[:,0]
    cell_index_after = cell_data_after#[:,0]


    full_before[:,0] -= mean_x_before
    full_before[:,1] -= mean_y_before

    full_after[:,0] -= mean_x_after
    full_after[:,1] -= mean_y_after


    plt.clf()
    plt.scatter(full_before[:,0],full_before[:,1], c="tab:blue", label="before")
    plt.scatter(full_after[:,0],full_after[:,1], c="tab:red", label="after")

    for i in range(len(cell_data)):
        before_ind = int(cell_index_before[i])
        after_ind = int(cell_index_after[i])
        
        plt.plot([full_before[before_ind,0],full_after[after_ind,0]],[full_before[before_ind,1],full_after[after_ind,1]], c="k")

    plt.legend()

    plt.show()