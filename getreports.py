#!/usr/bin/python

# This script
# - pulls reports.nc data from odroids
# - plots results

# Run ./getreports.py --help for available options.

# Usage: ./getreports.py [options] arg
# Arguments:
#   1: local root folder of orocos components
#   2: remote root folder of orocos components
#  if arguments are not provided, default values are used:
#   1: current directory/orocos/ourbot
#   2: /home/odroid/orocos

# Options:
#   -u, --username: ssh username
#   -p, --password: ssh password
#   -s, --show: show plots

# Ruben Van Parys - 2016

import optparse, os, paramiko
from netCDF4 import Dataset
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TKAgg')
import collections as col
import math
import matplotlib.patches as patches

# Default parameters
current_dir = os.path.dirname(os.path.realpath(__file__))
local_root = os.path.join(current_dir,'orocos/emperor')
remote_root = '/home/odroid/orocos'
username = 'odroid'
password = 'odroid'
show_plots = True
hosts = col.OrderedDict()
hosts['dave'] = '192.168.11.120'
#hosts['kurt'] = '192.168.11.121'
#hosts['krist'] = '192.168.11.122'


def get_file(ftp, rem_file, loc_file):
    ftp.get(rem_file, loc_file)


def plot_nc(file):
    data = Dataset(file, mode='r')
    time = data.variables['TimeStamp']
    signals = {}

    for key, value in data.variables.items():
        if key == 'TimeStamp':
            continue
        split = key.split('.')
        component = split[0]
        signal = split[1]
        if component not in signals:
            signals[component] = {}
        if signal not in signals[component]:
            signals[component][signal] = []
        signals[component][signal].append(value)

    for component, sgnls in signals.items():
        x_data = []
        y_data = []
        for signal, values in sgnls.items():
            cnt = 1
            if signal == 'est_pose_port':
                plt.figure()
                plt.hold(True)
                legend = []
                for k, value in enumerate(values):
                    plt.plot(time[:], value[:])
                    if k == 0:
                        x_data.extend(value[:])
                    if k == 1:
                        y_data.extend(value[:])
                    legend.append(k)
                plt.legend(legend)
                plt.xlabel('time')
                plt.ylabel(signal)
                cnt += 1
                plt.title('component ' + component)
    #plt.figure()
    #plt.title('measured lidar')
    #plt.xlabel('x')
    #plt.ylabel('y')
    #plt.plot(x_m, y_m)
        if len(x_data) != 0 and len(y_data) != 0: #checking if data is empty
            fig1 = plt.figure()
            plt.plot(x_data, y_data)
            plt.plot([-1.22,-1.22, 1.22, 1.22,-1.22],[-1.22, 1.22, 1.22,-1.22,-1.22])
            plt.plot([ 0.28, 0.28,-0.28,-0.28, 0.28],[-0.41,-0.75,-0.75,-0.41,-0.41])
            plt.plot([-0.71,-0.71,-0.81,-0.81,-0.71],[ 0.87,0.715,0.715, 0.87, 0.87])
            plt.plot([0.5875,0.5875,0.4325,0.4325,0.5875],[0.5,0.4,0.4,0.5,0.5])
            plt.plot([-0.04,-0.04, 0.04, 0.04,-0.04],[ 0.54, 0.46, 0.46, 0.54, 0.54])
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('movement')
    data.close()

def plot_maps(file):
    data = Dataset(file, mode='r')
    time = data.variables['TimeStamp']
    signals = {}

    for key, value in data.variables.items():
        #print key
        if key == 'TimeStamp':
            continue
        split = key.split('.')
        component = split[0]
        signal = split[1]
        if component not in signals:
            signals[component] = {}
            #print component
        if signal not in signals[component]:
            signals[component][signal] = []
        signals[component][signal].append(value)

    distances_measured = []
    angles_measured = []
    distances_calculated = []
    angles_calculated = []
    for component, sgnls in signals.items():
        for signal, values in sgnls.items():
            for k, value in enumerate(values):
                sample = 3
                if signal == 'cor_lidar_distance_port':
                    distances_measured.append(value[sample])
                elif signal == 'cor_lidar_angle_port':
                    angles_measured.append(value[sample])
                elif signal == 'artificial_lidar_distances_port':
                    distances_calculated.append(value[sample])
                elif signal == 'artificial_lidar_angles_port':
                    angles_calculated.append(value[sample])
    plt.figure()
    plt.hold(True)
    plt.title('map')
    plt.xlabel('x')
    plt.ylabel('y')
    x_measured=[]
    y_measured=[]
    x_calculated=[]
    y_calculated=[]
    #print len(distances_measured)
    for i in range(len(distances_measured)):
        x_measured.append(distances_measured[i]*math.cos(angles_measured[i]))
        y_measured.append(distances_measured[i]*math.sin(angles_measured[i]))
    plt.plot(x_measured, y_measured)
    for i in range(len(distances_calculated)):
        x_calculated.append(distances_calculated[i]*math.cos(angles_calculated[i]))
        y_calculated.append(distances_calculated[i]*math.sin(angles_calculated[i]))
    plt.plot(x_calculated, y_calculated)
    data.close()

def save_maps(file):
    data = Dataset(file, mode='r')
    time = data.variables['TimeStamp']
    signals = {}

    for key, value in data.variables.items():
        #print key
        if key == 'TimeStamp':
            continue
        split = key.split('.')
        component = split[0]
        signal = split[1]
        if component not in signals:
            signals[component] = {}
            print component
        if signal not in signals[component]:
            signals[component][signal] = []
        signals[component][signal].append(value)


    distances_measured = [[]]
    angles_measured = [[]]
    distances_calculated = [[]]
    angles_calculated = [[]]
    for component, sgnls in signals.items():
        for signal, values in sgnls.items():
            for k, value in enumerate(values):
                if signal == 'cor_lidar_distance_port':
                    distances_measured.append(value[:])
                elif signal == 'cor_lidar_angle_port':
                    angles_measured.append(value[:])
                elif signal == 'artificial_lidar_distances_port':
                    distances_calculated.append(value[:])
                elif signal == 'artificial_lidar_angles_port':
                    angles_calculated.append(value[:])
    if len(distances_measured) > 3:
        print len(distances_measured[3])
        for i in range(0,len(distances_measured[3])):
            fig = plt.figure()
            plt.hold(True)
            plt.title('map')
            plt.xlabel('x')
            plt.ylabel('y')
            x_measured=[]
            y_measured=[]
            x_calculated=[]
            y_calculated=[]
            for j in range(3,len(distances_measured)):
                x_measured.append(distances_measured[j][i]*math.cos(angles_measured[j][i]))
                y_measured.append(distances_measured[j][i]*math.sin(angles_measured[j][i]))
            plt.plot(x_measured, y_measured)
            for k in range(3,len(distances_calculated)):
                x_calculated.append(distances_calculated[k][i]*math.cos(angles_calculated[k][i]))
                y_calculated.append(distances_calculated[k][i]*math.sin(angles_calculated[k][i]))
            plt.plot(x_calculated, y_calculated)
            plt.ylim([-2.5, 2.5])
            plt.xlim([-2.5, 2.5])
            figname = "/home/michiel/ourbot/maps/map" + str(i) + ".png"
            fig.savefig(figname)
            plt.close(fig)
            if i%20 == 0:
                print figname
    data.close()

if __name__ == "__main__":
    usage = ('''Usage: %prog [options] arg. There are 2 arguments:
                local root path & remote root path. It can be left blank to use
                default values.''')
    op = optparse.OptionParser(usage=usage)
    op.add_option("-u", "--username", dest="username",
                  default=username, help="ssh username")
    op.add_option("-p", "--password", dest="password",
                  default=password, help="ssh password")
    op.add_option("-s", "--show", dest="show_plots",
                  default=show_plots, help="show plots")
    options, args = op.parse_args()

    if len(args) == 2:
        local_root = args(0)
        remote_root = args(1)

    username = options.username
    password = options.password
    show_plots = options.show_plots

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    index = 0
    for host, address in hosts.items():
        print 'Host %s ...' % host
        ssh.connect(address, username=username, password=password)
        ftp = ssh.open_sftp()
        remote_dir = [d for d in ftp.listdir(remote_root) if 'd'
            in str(ftp.lstat(os.path.join(remote_root, d))).split()[0]]

        # Sending source files
        print 'Pulling reports.nc file'
        get_file(ftp, os.path.join(remote_root, 'reports.nc'), os.path.join(local_root, 'reports_'+str(index)+'.nc'))
        ftp.close()
        ssh.close()
        index += 1

    if show_plots:
        for index, host in enumerate(hosts.values()):
            plot_nc(os.path.join(local_root, 'reports_'+str(index)+'.nc'))
            #plot_maps(os.path.join(local_root, 'reports_'+str(index)+'.nc'))
            save_maps(os.path.join(local_root, 'reports_'+str(index)+'.nc'))
        plt.show()

