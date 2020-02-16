import PIL
import numpy as np
from os import walk
from moviepy import editor

duration = 5.25
fps = 4.0
timelist = list(np.arange(0, duration, 1./fps))

framelist = []
for (dirpath, dirnames, filenames) in walk("../png"):
    filenames.sort()
    path = [dirpath + "/" + filepath for filepath in filenames]
    print(filenames)
    framelist.extend(path)
    break
img_dict = {a:f for a,f in zip(timelist, framelist)}

def make_frame(t):    
    fpath= img_dict[t]
    im = PIL.Image.open(fpath)
    ar = np.asarray(im)
    return ar

gif_path = "ObsticleDetect.gif"
clip = editor.VideoClip(make_frame, duration=duration)
clip.write_gif(gif_path, fps=fps)