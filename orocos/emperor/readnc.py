from netCDF4 import Dataset
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TKAgg')

file_name = 'reports_0.nc'
data = Dataset(file_name, mode='r')

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
    plt.figure()
    cnt = 0
    for signal, values in sgnls.items():
        plt.subplot(len(sgnls.values()), 1, cnt)
        plt.hold(True)
        legend = []
        for k, value in enumerate(values):
            plt.plot(time, value)
            legend.append(k)
        plt.legend(legend)
        plt.xlabel('time')
        plt.ylabel(signal)
        cnt += 1
    plt.title('component ' + component)
plt.show()
data.close()
