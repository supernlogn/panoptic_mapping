import os
import sys
import typing

from pathlib import Path
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json


def calcSNR(sig_orig:np.ndarray, sig_with_noise: np.ndarray) -> np.ndarray:
    sig_noise = sig_with_noise - sig_orig
    clean_orig = sig_orig - np.mean(sig_orig)
    clean_noise = sig_noise - np.mean(sig_noise)
    Porig = np.sum(clean_orig ** 2, axis=0)    
    Pnoise = np.sum(clean_noise ** 2, axis=0)
    snr = Porig / Pnoise
    return snr

def plotPathAndNoise(load_history_path:Path):
    """ creates a plot ot the original history signal
        and the noise history signal from a previous experiment execution.
        It needs the json file where the signals were saved.
    """
    assert(load_history_path.exists()), f"{load_history_path} does not exist"
    NUM_BINS = 24
    sig_data = {}
    with open(str(load_history_path), "r") as fr:
        sig_data = json.load(fr)
    
    sig_time = np.array(sig_data.get("timestamps", []))
    sig_time = sig_time - sig_time[0]
    sig_with_noise = np.reshape(np.array(sig_data.get("noise_history",[])), [-1,4])
    sig_orig = np.reshape(np.array(sig_data.get("orig_history",[])), [-1,4])
    pose_noise_sigma = sig_data["pose_noise_sigma"]

    fig = plt.figure(figsize=(12,9))
    fig.suptitle(f'path signal behaviour analysis under noise ~ N(0,{pose_noise_sigma})', fontsize=16)
    fig.subplots_adjust(hspace=1)

    # 3d paths comparison
    ax = fig.add_subplot(3,2,1,projection='3d')
    ax.set_title("3d paths comparison")
    ax.set_xlabel(r'$X\,(m)$')
    ax.set_ylabel(r'$Y\,(m)$')
    ax.set_zlabel(r'$Z\,(m)$')

    ax.plot(sig_with_noise[:,0],sig_with_noise[:,1],sig_with_noise[:,2])
    ax.plot(sig_orig[:,0],sig_orig[:,1],sig_orig[:,2])    
    
    # original path
    ax = fig.add_subplot(3,2,2,projection='3d')
    ax.set_title("original path")
    ax.plot(sig_orig[:,0],sig_orig[:,1],sig_orig[:,2])
    ax.set_xlabel(r'$X\,(m)$')
    ax.set_ylabel(r'$Y\,(m)$')
    ax.set_zlabel(r'$Z\,(m)$')

    # plot difference of signal per time 
    ax = fig.add_subplot(3,2,3)
    ax.set_title("signal's measure 1st difference vs time")
    dsig_orig = np.diff(sig_orig)
    measure_dsig_orig = np.sqrt( dsig_orig[:,0] ** 2 + dsig_orig[:,1] ** 2 + dsig_orig[:,2] ** 2 )
    ax.plot(sig_time, measure_dsig_orig)
    ax.set_xlabel('time (sec)')
    ax.set_ylabel(r'$\|x_{t+1}-x_{t}\|\,(m)$')
    
    # plot histogram of measure of signal's diference
    ax = fig.add_subplot(3,2,4)
    ax.set_title("measure of orig. signal's diference hist.")
    ax.hist(measure_dsig_orig, bins=NUM_BINS)
    ax.set_xlabel(r'$\|x_{t+1}-x_{t}\|\,(m)$')
    ax.set_ylabel('counts')

    # plot time vs step
    ax = fig.add_subplot(3,2,5)
    ax.set_title("time vs step")
    ax.plot(sig_time)
    ax.set_xlabel('step')
    ax.set_ylabel('time (sec)')


    # plot noise vector measure distribution
    ax = fig.add_subplot(3,2,6)
    ax.set_title("measure of signal's noise hist.")
    cleaned_noise = sig_with_noise - sig_orig
    measure_noise = np.sqrt(cleaned_noise[:,0] ** 2 + cleaned_noise[:,1] ** 2 + cleaned_noise[:,2] ** 2)
    ax.hist(measure_noise, bins=NUM_BINS)
    ax.set_xlabel(r'$n_{t}$')
    ax.set_ylabel('counts')

    # statistics of signal and noise
    print("metrics for difference of original signal:")
    print(f"std = {np.std(dsig_orig)}")
    print(f"mean = {np.mean(dsig_orig)}")
    print(f"max = {np.max(measure_dsig_orig)}")
    print(f"min = {np.min(measure_dsig_orig)}")
    snr = calcSNR(sig_orig, sig_with_noise)
    print(f"SNR = {snr}")
    
    # save figure
    i = 0
    fig_path = Path(f"/home/ioannis/datasets/noise/lastFig_{i}.pdf")
    while(fig_path.exists()):
        i += 1
        fig_path = Path(f"/home/ioannis/datasets/noise/lastFig_{i}.pdf")
    
    plt.savefig(str(fig_path))
    
    # reveal figure
    plt.show()

def main(args):
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "sans-serif",
        "font.sans-serif": ["Helvetica"]})

    load_path = Path("/home/ioannis/datasets/noise/history_p0.5.json")
    if(len(args) > 1):
        load_path = Path(args[1])

    plotPathAndNoise(load_path)

if __name__ == "__main__":
    main(sys.argv)