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

# Setup paths
sys.path.append('/usr/local/lib/vmxpi/')

# Module imports
from tkinter import *
from tkinter import ttk


# Define the main window
class MainWindow(ttk.Frame):

    # Override initialization method
    def __init__(self, master = None):

        # Set master window
        super().__init__(master)
        self.master = master
        self.master.title('VMX-pi Real Time Clock Utility')
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.master.geometry('640x480+30+30')
        self.master.option_add('*tearOff',FALSE)
        self.mastercontent = ttk.PanedWindow(self.master, orient=VERTICAL)
        self.mastercontent.grid(column=0, row=0, sticky=(N,S,E,W))

        # Declare class variables
        self.currentMonth = StringVar()
        self.currentDay = StringVar()
        self.currentYear = StringVar()
        self.currentHour = StringVar()
        self.currentMinute = StringVar()
        self.currentSecond = StringVar()

        # Initialize variables
        self.currentMonth.set('September')
        self.currentDay.set('22')
        self.currentYear.set('2020')
        self.currentHour.set('12')
        self.currentMinute.set('00')
        self.currentSecond.set('00')

        # Create main areas
        self.create_current_frame()
        self.set_time_frame()


    # Create current date/time area
    def create_current_frame(self):

        # Create current time frame
        self.currentFrame = ttk.Frame(self.mastercontent, borderwidth=1, relief='solid', padding=(5))
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
        self.currHeaderLabel = Label(self.currentFrame, text='Current Date / Time', justify='center')
        self.currHeaderLabel.grid(row=0, column=0, columnspan=6)

        # Create month display
        self.currMonthLabel = Label(self.currentFrame, text='Month', justify='center')
        self.currMonthLabel.grid(row=1, column=0, sticky=(E,W))
        self.currMonth = Label(self.currentFrame, textvariable=self.currentMonth, justify='center')
        self.currMonth.grid(row=2, column=0, sticky=(E,W))

        # Create day display
        self.currDayLabel = Label(self.currentFrame, text='Day', justify='center')
        self.currDayLabel.grid(row=1, column=1, sticky=(E,W))
        self.currDay = Label(self.currentFrame, textvariable=self.currentDay, justify='center')
        self.currDay.grid(row=2, column=1, sticky=(E,W))

        # Create year display
        self.currYearLabel = Label(self.currentFrame, text='Year', justify='center')
        self.currYearLabel.grid(row=1, column=2, sticky=(E,W))
        self.currYear = Label(self.currentFrame, textvariable=self.currentYear, justify='center')
        self.currYear.grid(row=2, column=2, sticky=(E,W))

        # Create hours display
        self.currHourLabel = Label(self.currentFrame, text='Hours', justify='center')
        self.currHourLabel.grid(row=1, column=3, sticky=(E,W))
        self.currHour = Label(self.currentFrame, textvariable=self.currentYear, justify='center')
        self.currHour.grid(row=2, column=3, sticky=(E,W))

        # Create minutes display
        self.currMinuteLabel = Label(self.currentFrame, text='Minutes', justify='center')
        self.currMinuteLabel.grid(row=1, column=4, sticky=(E,W))
        self.currMinute = Label(self.currentFrame, textvariable=self.currentMinute, justify='center')
        self.currMinute.grid(row=2, column=4, sticky=(E,W))

        # Create seconds display
        self.currSecondLabel = Label(self.currentFrame, text='Seconds', justify='center')
        self.currSecondLabel.grid(row=1, column=5, sticky=(E,W))
        self.currSecond = Label(self.currentFrame, textvariable=self.currentSecond, justify='center')
        self.currSecond.grid(row=2, column=5, sticky=(E,W))


    # Define time set frame
    def set_time_frame(self):

        # Create set time frame
        self.setFrame = ttk.Frame(self.mastercontent, borderwidth=1, relief='solid', padding=(5))
        self.mastercontent.add(self.setFrame, weight=1)

        # Define display grid
        self.setFrame.rowconfigure(0)
        self.setFrame.rowconfigure(1)
        self.setFrame.rowconfigure(2)
        self.setFrame.columnconfigure(0, weight=1)
        self.setFrame.columnconfigure(1, weight=1)
        self.setFrame.columnconfigure(2, weight=1)
        self.setFrame.columnconfigure(3, weight=1)
        self.setFrame.columnconfigure(4, weight=1)
        self.setFrame.columnconfigure(5, weight=1)

        # Create header
        self.setHeaderLabel = Label(self.setFrame, text='Set Date / Time', justify='center')
        self.setHeaderLabel.grid(row=0, column=0, columnspan=6)

        # Create month input
        self.setMonthLabel = Label(self.setFrame, text='Set Month', justify='center')
        self.setMonthLabel.grid(row=1, column=0, sticky=(E,W))
        self.setMonth = Label(self.setFrame, text='September', justify='center')
        self.setMonth.grid(row=2, column=0, sticky=(E,W))



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
