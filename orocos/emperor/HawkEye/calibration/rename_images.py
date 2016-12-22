import sys, os

directory = os.getcwd()+'/images3/'
print directory
print os.listdir(directory)

for k, filename in enumerate(os.listdir(directory)):
    os.rename(directory + filename, directory + 'calibration'+str(k)+'.jpg')

