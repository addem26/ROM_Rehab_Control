import tkinter 
from pylive import live_plotter

import matplotlib.pyplot as plt 
import numpy as np


root = tkinter.Tk()
root.wm_title("Embedding in Tk")
root.geometry('800x800')


def on_key_press():
    size = 100
    x_vec = np.linspace(0,1,size+1)[0:-1]
    y_vec = np.random.randn(len(x_vec))
    line1 = []
    while True:
        rand_val = np.random.randn(1)
        y_vec[-1] = rand_val
        line1 = live_plotter(x_vec,y_vec,line1)
        y_vec = np.append(y_vec[1:],0.0)



my_button = tkinter.Button(root, text = "Graph It!", command = on_key_press)
my_button.pack() 
root.mainloop()
# If you put root.destroy() here, it will cause an error if the window is
# closed with the window manager.