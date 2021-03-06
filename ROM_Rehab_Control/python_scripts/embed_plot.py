from tkinter import *
class MyWindow:
    def __init__(self, win):
        self.lbl1=Label(win, text='Enter ROM:')
        self.lbl2=Label(win, text='Enter Velocity:')
        self.lbl3=Label(win, text='Reps')
        self.t1=Entry(bd=3)
        self.t2=Entry()
        self.t3=Entry()
        self.btn1 = Button(win, text='Add')
        self.btn2=Button(win, text='Subtract')
        self.lbl1.place(x=100, y=50)
        self.t1.place(x=200, y=50)
        self.lbl2.place(x=100, y=100)
        self.t2.place(x=200, y=100)
        self.b1=Button(win, text='Send Data', command=self.add)
        self.b2=Button(win, text='Begin Exercise')
        self.b2.bind('<Button-1>', self.sub)
        self.b1.place(x=100, y=200)
        self.b2.place(x=200, y=200)
        self.lbl3.place(x=100, y=150)
        self.t3.place(x=200, y=150)
    def add(self):
        self.t3.delete(0, 'end')
        num1=int(self.t1.get())
        num2=int(self.t2.get())
        result=num1+num2
        self.t3.insert(END, str(result))
    def sub(self, event):
        self.t3.delete(0, 'end')
        num1=int(self.t1.get())
        num2=int(self.t2.get())
        result=num1-num2
        self.t3.insert(END, str(result))

window=Tk()
mywin=MyWindow(window)
window.title('Hello Python')
window.geometry("500x600+10+10")
window.mainloop()