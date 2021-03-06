import tkinter as tk
from pylive import live_plotter

import matplotlib.pyplot as plt 
import numpy as np


root = tk.Tk()
root.wm_title("Embedding in Tk")
root.geometry('800x800')


def on_key_press():
    size = 100
    x_vec = np.linspace(0,1,size+1)[0:-1]
    y_vec = np.random.randn(len(x_vec))
    line1 = []
    line2 = []
    while True:
        rand_val = np.random.randn(1)
        y_vec[-1] = rand_val
        line1 = live_plotter(x_vec,y_vec,line1)
        lin2 = line1
        y_vec = np.append(y_vec[1:],0.0)

def rom_callback(): 
    return 0; 



start_workout = tk.Button(root, text = "Begin Exercise", command = on_key_press).place(x=5, y=0)
enter_rom = tk.Button(root, text = "Enter ROM",command = rom_callback).place(x=50, y=40)




#my_button.pack() 

root.mainloop()
# If you put root.destroy() here, it will cause an error if the window is
# closed with the window manager.