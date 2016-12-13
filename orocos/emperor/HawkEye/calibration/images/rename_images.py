import sys, os

directory = os.getcwd()
for k, filename in enumerate(os.listdir(directory)):
    os.rename(filename, 'calibration'+str(k)+'.jpg')

