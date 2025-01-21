import time
import serial 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# # Initialize the figure
# fig, ax = plt.subplots()
# x_data, y_data = [], []
# line, = ax.plot(x_data, y_data, color='b')


def animate(i, dataList, ser):
    # 'i' is a incrementing variable based upon frames = x argument
    ser.write(b'g')                                     # Transmit the char 'g' to receive the Arduino data point
    arduinoData_string = ser.readline().decode('utf-8').strip() # Decode receive Arduino data as a formatted string
   # arduinoData_string = ser.readline().decode('ascii') # Decode receive Arduino data as a formatted string

    try:
        arduinoData_float = float(arduinoData_string)   # Convert to float
        dataList.append(arduinoData_float)              # Add to the list holding the fixed number of points to animate

    except:                                             # Pass if data point is bad                               
        pass

    dataList = dataList[-100:]                           # Fix the list size so that the animation plot 'window' is x number of points
                                                        # keeping last 100 data points
    
    ax.clear()                                          # Clear last data frame
    ax.plot(dataList)                                   # Plot new data frame
    
    ax.set_ylim([-180, 180])                              # Set Y axis limit of plot
    ax.set_title("Arduino Tilt Angle Through Complementary Filter")                        # Set title of figure
    ax.set_ylabel("Tilt (degree)")                              # Set title of y axis 
    ax.set_xlabel("Time")

dataList = []                                           # Initialize list to hold data points for animation
fig = plt.figure()                                      
ax = fig.add_subplot(111)                               

# Change the serial port and board rate to match the Arduino
ser = serial.Serial("COM5", 9600)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(dataList, ser), interval=100) 

plt.show()                                              
ser.close()                                             



