import numpy as np
import matplotlib.pyplot as plt
import csv

map = {
    'pos': {
        'label': 'position',
        'units': 'deg'
    },
    'vel': {
        'label': 'velocity',
        'units': 'deg/s'
    },
    'tor': {
        'label': 'torque',
        'units': 'mN*m'
    }
}
def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'same') / w

def two_side_mov_av(x, w):
    forward = moving_average(x, w)
    backward = moving_average(x[::-1], w)[::-1]
    return (forward + backward)/2


def plot_data(plot='pos', name=None, strt_time=0, **kwargs):
    """Plots motor position with respect to time. Used to test PID controller

    Args:
        Kp (float): Kp parameter
        Kd (float): Kd parameter
        Ki (float): Ki parameter
        P (int): Position (degrees units)
        V (int): Velocity (deg/sec)
        F (int): Velocity filter (parameter defining how much measurements are smoothed)
        B (int): Torque boundary
        R (int): Frequency reducer parameter (main freq/R)
    """
    fig = plt.figure()
    ax = plt.subplot(111)
    data = []
    with open('MOTOR_LOG.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        labels = spamreader.__next__()
        for row in spamreader:
            sas = []
            for j in row:
                if j != '':
                    sas.append(float(j))
                else:
                    sas.append(np.nan)
            data.append(sas)
    data = np.array(data)
    pos_raw = data[:, 0]
    turns = data[:, 4]
    pos = pos_raw/(2**14)*360 + turns*360
    vel = data[:, 1]
    tor = moving_average(data[:, 2], 1500)
    print(tor.shape)
    data_map = {
        'pos': pos,
        'vel': vel,
        'tor': tor
    }
    time = data[:, 5]
    print(time.shape)
    ax.plot(time, data_map[plot], label=map[plot]['label']) # add time at x
    if strt_time != 0:
        plt.plot(time, np.sin(time + strt_time)/np.pi*180 + 90, 'r', label='desired pos')
    # if plot == 'pos':
    #     ax.plot(time, time*0 + kwargs['P'], 'r', label=f'desired {map[plot]["label"]}')
    # if plot == 'vel':
    #     ax.plot(time, time * 0 + kwargs['V'], 'r', label=f'desired {map[plot]["label"]}')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel(f'{map[plot]["label"]} ({map[plot]["units"]})')
    ax.legend()
    ax.grid()
    plt.tight_layout()
    file_str = ''
    for k, v in kwargs.items():
        file_str += f'{k}_{v}_'
    file_str = file_str[:-1]
    if name is not None:
        file_str = name
    fig.savefig(f'plots/{name}.pdf', format='pdf')
    fig.clf()
    plt.close(fig)