# -*- coding: utf-8 -*-
#!/usr/bin/env python3

#########################################################
#                                                       #
#             VMX Real Time Clock Utility               #
#                                                       #
#  This program sets the real time clock on the VMX-pi  #
#  board.  This utility shows the current RTC time and  #
#  allows the user to enter a new date and time for     #
#  the VMX-pi real time clock.                          #
#                                                       #
#  @Author: Team4121                                    #
#  @Created: 2020-09-08                                 #
#  @Version: 1.0                                        #
#                                                       #
#########################################################

"""VMX-pi real time clock utility"""

# System imports
import sys
import os
import imp

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')
sys.path.append('/usr/local/lib/vmxpi/')

# Module imports
from tkinter import *
from tkinter import ttk

#Team 4121 module imports
from FRCNavxLibrary import FRCNavx


# Define the main window
class MainWindow(ttk.Frame):

    # Override initialization method
    def __init__(self, master = None):

         # Declare class variables
        self.currentMonth = StringVar()
        self.currentDay = StringVar()
        self.currentYear = StringVar()
        self.currentHour = StringVar()
        self.currentMinute = StringVar()
        self.currentSecond = StringVar()
        self.newMonth = StringVar()
        self.newDay = StringVar()
        self.newYear = StringVar()
        self.newHour = StringVar()
        self.newMinute = StringVar()
        self.newSecond = StringVar()

        # Set master window
        super().__init__(master)
        self.master = master
        self.master.title('VMX-pi Real Time Clock Utility')
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.master.geometry('640x240+30+30')
        self.master.option_add('*tearOff',FALSE)
        self.mastercontent = ttk.PanedWindow(self.master, 
                                             orient=VERTICAL)
        self.mastercontent.grid(column=0, row=0, sticky=(N,S,E,W))

        # Create Navx object
        self.navx = FRCNavx('NavxStream')
        self.navx.start_navx()

        # Initialize time variables
        self.currentTime = self.navx.read_time()
        self.currentDate = self.navx.read_date()
        self.currentMonth.set(str(self.currentDate[3]))
        self.currentDay.set(str(self.currentDate[2]))
        self.currentYear.set(str(2000 + self.currentDate[4]))
        self.currentHour.set(str(self.currentTime[1]))
        self.currentMinute.set(str(self.currentTime[2]))
        self.currentSecond.set(str(self.currentTime[3]))
        
        # Create main areas
        self.current_time_frame()
        self.new_time_frame()


    # Create current date/time area
    def current_time_frame(self):

        # Create current time frame
        self.currentFrame = ttk.Frame(self.mastercontent, 
                                      borderwidth=1, 
                                      relief='solid', 
                                      padding=(5))
        self.mastercontent.add(self.currentFrame, weight=1)

        # Define display grid
        self.currentFrame.rowconfigure(0)
        self.currentFrame.rowconfigure(1)
        self.currentFrame.rowconfigure(2)
        self.currentFrame.columnconfigure(0, weight=1)
        self.currentFrame.columnconfigure(1, weight=1)
        self.currentFrame.columnconfigure(2, weight=1)
        self.currentFrame.columnconfigure(3, weight=1)
        self.currentFrame.columnconfigure(4, weight=1)
        self.currentFrame.columnconfigure(5, weight=1)

        # Create header
        self.currHeaderLabel = Label(self.currentFrame, 
                                     text='Current Date / Time', 
                                     justify='center')
        self.currHeaderLabel.grid(row=0, column=0, columnspan=6)

        # Create month display
        self.currMonthLabel = Label(self.currentFrame, 
                                    text='Month', 
                                    justify='center')
        self.currMonthLabel.grid(row=1, column=0, sticky=(E,W))
        self.currMonth = Label(self.currentFrame, 
                               textvariable=self.currentMonth, 
                               justify='center',
                               borderwidth=1)
        self.currMonth.grid(row=2, column=0, sticky=(E,W))

        # Create day display
        self.currDayLabel = Label(self.currentFrame, 
                                  text='Day', 
                                  justify='center')
        self.currDayLabel.grid(row=1, column=1, sticky=(E,W))
        self.currDay = Label(self.currentFrame, 
                             textvariable=self.currentDay, 
                             justify='center')
        self.currDay.grid(row=2, column=1, sticky=(E,W))

        # Create year display
        self.currYearLabel = Label(self.currentFrame, 
                                   text='Year', 
                                   justify='center')
        self.currYearLabel.grid(row=1, column=2, sticky=(E,W))
        self.currYear = Label(self.currentFrame, 
                              textvariable=self.currentYear, 
                              justify='center')
        self.currYear.grid(row=2, column=2, sticky=(E,W))

        # Create hours display
        self.currHourLabel = Label(self.currentFrame, 
                                   text='Hours', 
                                   justify='center')
        self.currHourLabel.grid(row=1, column=3, sticky=(E,W))
        self.currHour = Label(self.currentFrame, 
                              textvariable=self.currentHour, 
                              justify='center')
        self.currHour.grid(row=2, column=3, sticky=(E,W))

        # Create minutes display
        self.currMinuteLabel = Label(self.currentFrame, 
                                     text='Minutes', 
                                     justify='center')
        self.currMinuteLabel.grid(row=1, column=4, sticky=(E,W))
        self.currMinute = Label(self.currentFrame, 
                                textvariable=self.currentMinute, 
                                justify='center')
        self.currMinute.grid(row=2, column=4, sticky=(E,W))

        # Create seconds display
        self.currSecondLabel = Label(self.currentFrame, 
                                     text='Seconds', 
                                     justify='center')
        self.currSecondLabel.grid(row=1, column=5, sticky=(E,W))
        self.currSecond = Label(self.currentFrame, 
                                textvariable=self.currentSecond, 
                                justify='center')
        self.currSecond.grid(row=2, column=5, sticky=(E,W))


    # Define time set frame
    def new_time_frame(self):

        # Create set time frame
        self.setFrame = ttk.Frame(self.mastercontent, 
                                  borderwidth=1, 
                                  relief='solid', 
                                  padding=(5))
        self.mastercontent.add(self.setFrame, weight=1)

        # Define display grid
        self.setFrame.rowconfigure(0, weight=1)
        self.setFrame.rowconfigure(1, weight=1)
        self.setFrame.rowconfigure(2, weight=1)
        self.setFrame.rowconfigure(3, weight=1)
        self.setFrame.rowconfigure(4, weight=1)
        self.setFrame.columnconfigure(0, weight=1)
        self.setFrame.columnconfigure(1, weight=1)
        self.setFrame.columnconfigure(2, weight=1)
        self.setFrame.columnconfigure(3, weight=1)
        self.setFrame.columnconfigure(4, weight=1)
        self.setFrame.columnconfigure(5, weight=1)

        # Create header
        self.setHeaderLabel = Label(self.setFrame, 
                                    text='Set Date / Time', 
                                    justify='center')
        self.setHeaderLabel.grid(row=0, column=0, columnspan=6)

        # Create month input
        self.setMonthLabel = Label(self.setFrame, 
                                   text='Month', 
                                   justify='center')
        self.setMonthLabel.grid(row=1, column=0, sticky=(E,W))
        self.monthEntry = Entry(self.setFrame, 
								textvariable=self.newMonth)
        self.monthEntry.grid(row=2, column=0, sticky=(E,W))
        
        # Create day input
        self.setDayLabel = Label(self.setFrame,
									text='Day',
									justify='center')
		self.setDayLabel.grid(row=1,column=1,sticky=(E,W))
		self.dayEntry = Entry(self.setFrame,
							  textvariable=self.newDay)
		self.dayEntry.grid(row=2, column=1, sticky=(E,W))
		
		# Create year input
		self.setYearLabel = Label(self.setFrame,
								  text='Year',
								  justify='center')
		self.setYearLabel.grid(row=1, column=2, sticky=(E,W))
		self.yearEntry = Entry(self.setFrame,
							   textvariable=self.newYear)
		self.yearEntry.grid(row=2, column=2, sticky=(E,W))
		
		# Create hour input
		self.setHourLabel = Label(self.setFrame,
								  text='Hour',
								  justify='center')
		self.setHourLabel.grid(row=1, column=3, sticky=(E,W))
		self.hourEntry = Entry(self.setFrame,
							   textvariable=self.newHour)
		self.hourEntry.grid(row=2, column=3, sticky=(E,W))
		
		# Create minute input
		self.setMinuteLabel = Label(self.setFrame,
									text='Minute',
									justify='center')
		self.setMinuteLabel.grid(row=1, column=4, sticky=(E,W))
		self.minuteEntry = Entry(self.setFrame,
								 textvariable=self.newMinute)
		self.minuteEntry.grid(row=2, column=4, sticky=(E,W))
		
		# Create second input
		self.setSecondLabel = Label(self.setFrame,
									text='Second',
									justify='center')
		self.setSecondLabel.grid(row=1, column=5, sticky=(E,W))
		self.secondEntry = Entry(self.setFrame,
								 textvariable=self.newSecond)
		self.secondEntry.grid(row=2, column=5, sticky=(E,W))
		
		# Create set button
		self.setButton = Button(self.setFrame,
								text='Set Time',
								command=self.set_time)
		self.setButton.grid(row=4, column=5, sticky=(E,W))
		
		
	# Handle button press
	def set_time(self):
		


# Define main method
def main():

    # Create root window
    rootwindow = Tk()

    # Create an instance of the main window class
    app = MainWindow(rootwindow)

    # Run the application
    app.mainloop()


# Call main function on startup
if __name__ == '__main__':
    main()
