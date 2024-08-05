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
    #parser.add_argument('input_before_img',
    #                    help='Path to the full image of before')
    #parser.add_argument('input_after_img',
    #                    help='Path to the full image of after')
    parser.add_argument('-o','--output_path', default=None,
                        help='Optional path to write output files to. Defaults to current directory')

    args = parser.parse_args()

    input_path = args.input_path
    input_before_path = args.input_before
    input_after_path = args.input_after
    #image_before_path = args.input_before_img
    #image_after_path = args.input_after_img

    output_path = args.output_path
    


    full_before = np.loadtxt(input_before_path)
    full_after = np.loadtxt(input_after_path)
    full_after[:,0] *=0.9

    cell_data = np.loadtxt(input_path, ndmin=2, dtype=np.int32)

    cell_data_before = cell_data[:,0]
    cell_data_after = cell_data[:,1]

    mean_x_before = np.average(full_before[cell_data_before[0],0])
    mean_y_before = np.average(full_before[cell_data_before[0],1])
    print("Mapped:", cell_data_before[0], cell_data_after[0])

    mean_x_after = np.average(full_after[cell_data_after[0],0])
    mean_y_after = np.average(full_after[cell_data_after[0],1])

    ref_before = full_before[92]#cell_data_before[0]]
    ref_after = full_after[92]#cell_data_after[0]]

    cell_index_before = cell_data_before#[:,0]
    cell_index_after = cell_data_after#[:,0]

    full_after[:,0] += ref_before[0] - ref_after[0]
    full_after[:,1] += ref_before[1] - ref_after[1] 

    x_scale = 413./270.24
    y_scale = 369./241.45

    plt.clf()
    plt.scatter(full_before[:,0]*x_scale,full_before[:,1]*y_scale, c="tab:blue", label="before")
    plt.scatter(full_after[:,0]*x_scale,full_after[:,1]*y_scale, c="tab:red", label="after")

    plt.scatter([ref_before[0]*x_scale], [ref_before[1]*y_scale], c="g")


    # import matplotlib.image as mpimg
    # img = mpimg.imread(image_before_path)
    #imgplot = plt.imshow(img, alpha=0.5, cmap="Blues")

    #img = mpimg.imread(image_after_path)
    #imgplot = plt.imshow(img, alpha=0.5, cmap="Reds")

    for i in range(len(cell_data)):
        before_ind = int(cell_index_before[i])
        after_ind = int(cell_index_after[i])
        
        plt.plot([full_before[before_ind,0]*x_scale,full_after[after_ind,0]*x_scale],[full_before[before_ind,1]*y_scale,full_after[after_ind,1]*y_scale], c="k")

    plt.legend()

    plt.savefig(output_path+"map.pdf")
    plt.show()