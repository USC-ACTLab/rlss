from Tkinter import *
import time
from threading import Thread

class PathGui(Frame):

    def __init__(self, master=None):
        self.master = master
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()

    def createWidgets(self):
        self.QUIT = Button(self,text="Quit",fg="red",command=self.quit,width=6,height=1)
        self.QUIT.grid(row=1,column=2)

        Button(self,text="Select All",command=self.select_all_hyperplanes,width=6,height=1).grid(row=0,column=0)
        Button(self,text="Deselect All",command=self.deselect_all_hyperplanes,width=6,height=1).grid(row=1,column=0)



        Button(self,text="Select All",command=self.select_all_robots,width=6,height=1).grid(row=0,column=1)
        Button(self,text="Deselect All",command=self.deselect_all_robots,width=6,height=1).grid(row=1,column=1)
        Button(self,text=u"\u23EA",command=self.dum,width=6,height=1).grid(row=0,column=2)
        Button(self,text=u"\u23EF",command=self.dum,width=6,height=1).grid(row=0,column=3)

        '''
        self.prices_info = Text(self,width=20,height=10)
    	#self.info_text.tag_configure("center", justify='center')
        self.prices_info.grid(row=3,column=0)#,columnspan=2,rowspan=1)


        self.price_analytics = Text(self,width=20,height=10)
    	#self.info_text.tag_configure("center", justify='center')
        self.price_analytics.grid(row=3,column=4)#,columnspan=2,rowspan=1)
        '''
        lenhyp = 25
        self.hyperplane_array = [IntVar() for x in range(lenhyp)]
        for i in range(lenhyp):
            Checkbutton(self, text="Hyperplane {:0>2d}".format(i+1), variable=self.hyperplane_array[i]).grid(row=4+i,column=0)

        lenrob = 25
        self.robot_array = [IntVar() for x in range(lenrob)]
        for i in range(lenrob):
            Checkbutton(self, text="Robot {:0>2d}".format(i+1), variable=self.robot_array[i]).grid(row=4+i,column=1)


    def select_all_hyperplanes(self):
        for v in self.hyperplane_array:
            v.set(1)

    def deselect_all_hyperplanes(self):
        for v in self.hyperplane_array:
            v.set(0)

    def select_all_robots(self):
        for v in self.robot_array:
            v.set(1)

    def deselect_all_robots(self):
        for v in self.robot_array:
            v.set(0)

    def dum(self):
        pass




if __name__=="__main__":
	root = Tk()
	root.geometry("500x900")
	app = PathGui(master=root)
	app.master.title("Path Visualization Manager")
	app.master.maxsize(500, 700)
	app.mainloop()
	root.destroy()
	sys.exit()
