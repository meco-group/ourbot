import sys, os
from PIL import Image

directory = os.getcwd()+'/images4'
directory_new = directory + '_resize'

if not os.path.exists(directory_new):
    os.makedirs(directory)

for k, filename in enumerate(os.listdir(directory)):
    path = os.path.join(directory, filename)
    im = Image.open(path)
    w, h = im.size
    im_new = im.resize((int(w*0.5), int(h*0.5)))
    im_new.save(os.path.join(directory_new, filename))
