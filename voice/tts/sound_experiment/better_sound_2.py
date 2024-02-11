import queue
import sys
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import sounddevice as sd

device = 0 # id of the audio device by default
window = 1000 # window for the data
downsample = 1 # how much samples to drop
channels = [1] # a list of audio channels
interval = 30 # this is update interval in miliseconds for plot

q = queue.Queue()
# Please note that this sd.query_devices has an s in the end.
device_info =  sd.query_devices(device, 'input')
samplerate = device_info['default_samplerate']
length  = int(window*samplerate/(1000*downsample))

print("Sample Rate: ", samplerate)
# Typical sample rate is 44100 so lets see.

plotdata =  np.zeros((length,len(channels)))
# Lets look at the shape of this plotdata 
print("plotdata shape: ", plotdata.shape)
# So its vector of length 44100

# next is to make fig and axis of matplotlib plt
fig,ax = plt.subplots(figsize=(8,4))

# lets set the title
ax.set_title("PyShine")

# Make a matplotlib.lines.Line2D plot item of color green
lines = ax.plot(plotdata,color = (0,1,0.29))

def audio_callback(indata,frames,time,status):
	q.put(indata[::downsample,[0]])


# def update_plot(frame):
# 	global plotdata
# 	global ax
# 	while True:
# 		try: 
# 			data = q.get_nowait()
# 		except queue.Empty:
# 			break
# 		shift = len(data)
# 		plotdata = np.roll(plotdata, -shift,axis = 0)
# 		plotdata[-shift:,:] = data
# 	for column, line in enumerate(lines):
# 		print(plotdata[:,column])
# 		line.set_ydata(plotdata[:,column])
#         if np.any(data > 0.0001):
#             print('voice detected')
# 	return lines
def update_plot(frame):
    global plotdata
    while True:
        try:
            data = q.get_nowait()
        except queue.Empty:
            break
        shift = len(data)
        plotdata = np.roll(plotdata, -shift, axis=0)
        plotdata[-shift:, :] = data

        # Add a vertical line when values are greater than 0
        for column, line in enumerate(lines):
            line.set_ydata(plotdata[:, column])
            if np.any(data > 0.001):
                print('voice detected')
    return lines

ax.set_facecolor((0,0,0))
ax.set_yticks([0])
ax.yaxis.grid(True)

stream  = sd.InputStream( device = device, channels = max(channels), samplerate = samplerate, callback  = audio_callback)
ani  = FuncAnimation(fig,update_plot, interval=interval,blit=True)
with stream:
	plt.show()
