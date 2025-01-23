# import time
# import serial 
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation


# accel_data = []
# gyro_data = []
# comp_data = []

# # Set up the figure and axis
# fig, ax = plt.subplots()
# ax.set_ylim([-180, 180])  # Set Y-axis limits
# ax.set_title("Arduino Tilt Angle Using Complementary Filter")
# ax.set_ylabel("Tilt Angle (degrees)")
# ax.set_xlabel("Time (samples)")

# # Create three line objects for plotting
# (accel_line,) = ax.plot([], [], label="Accelerometer", linestyle="dotted", color="red")
# (gyro_line,) = ax.plot([], [], label="Gyroscope", linestyle="dashed", color="blue")
# (comp_line,) = ax.plot([], [], label="Complementary Filter", linestyle="solid", color="green")

# ax.legend()  # Add legend to differentiate the plots

# def animate(i, dataList, ser):
#     # 'i' is a incrementing variable based upon frames = x argument
#     ser.write(b'g')                                     # Transmit the char 'g' to receive the Arduino data point
#     arduinoData_string = ser.readline().decode('utf-8').strip() # Decode receive Arduino data as a formatted string
#    # arduinoData_string = ser.readline().decode('ascii') # Decode receive Arduino data as a formatted string
    
#     # Expecting comma-separated values: "accel,gyro,comp"
#     values = arduinoData_string.split(",")
#     try:
#         accel, gyro, comp = map(float, values)
#         accel_data.append(accel)
#         gyro_data.append(gyro)
#         comp_data.append(comp)

#     except:                                             # Pass if data point is bad                               
#         pass

#     accel_data[:] = accel_data[-100:]
#     gyro_data[:] = gyro_data[-100:]
#     comp_data[:] = comp_data[-100:]                                                        # keeping last 100 data points
    
#     # ploting data
#     accel_line.set_data(range(len(accel_data)), accel_data)
#     gyro_line.set_data(range(len(gyro_data)), gyro_data)
#     comp_line.set_data(range(len(comp_data)), comp_data)

#     ax.set_xlim(0, len(comp_data))  # Adjust X-axis dynamically

# # dataList = []                                           # Initialize list to hold data points for animation
# # fig = plt.figure()                                      
# # ax = fig.add_subplot(111)                               

# # Change the serial port and board rate to match the Arduino
# ser = serial.Serial("COM5", 9600)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
# time.sleep(2)                                           # Time delay for Arduino Serial initialization 

#                                                         # Matplotlib Animation Fuction that takes takes care of real time plot.
#                                                         # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
# ani = animation.FuncAnimation(fig,animate, interval=100, cache_frame_data=False) 

# plt.show()                                              
# ser.close()                                             


import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Data storage
accel_data = []
gyro_data = []
comp_data = []

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_ylim([-180, 180])  # Y-axis limits
ax.set_title("Arduino Tilt Angle Using Complementary Filter")
ax.set_ylabel("Tilt Angle (degrees)")
ax.set_xlabel("Time (samples)")

# Create three line objects for plotting
(accel_line,) = ax.plot([], [], label="Accelerometer", linestyle="dotted", color="red")
(gyro_line,) = ax.plot([], [], label="Gyroscope", linestyle="dashed", color="blue")
(comp_line,) = ax.plot([], [], label="Complementary Filter", linestyle="solid", color="green")

ax.legend()  # Add legend

# Set up Serial connection (change "COM5" to your actual port)
ser = serial.Serial("COM5", 9600, timeout=1)
time.sleep(2)  # Allow Arduino time to initialize

def animate(i):
    """Read and update sensor data from Arduino in real time."""
    try:
        ser.write(b'g')  # Send request to Arduino
        arduinoData_string = ser.readline().decode("utf-8").strip()  # Read line

        # Expecting data in format: "accel,gyro,comp"
        values = arduinoData_string.split(",")
        if len(values) == 3:  # Ensure valid data format
            accel, gyro, comp = map(float, values)
            accel_data.append(accel)
            gyro_data.append(gyro)
            comp_data.append(comp)

            # Keep last 100 data points
            accel_data[:] = accel_data[-100:]
            gyro_data[:] = gyro_data[-100:]
            comp_data[:] = comp_data[-100:]

            # Update plots
            accel_line.set_data(range(len(accel_data)), accel_data)
            gyro_line.set_data(range(len(gyro_data)), gyro_data)
            comp_line.set_data(range(len(comp_data)), comp_data)

            ax.set_xlim(0, len(comp_data))  # Adjust X-axis dynamically

    except Exception as e:
        print(f"Error: {e}")  # Print error if data is invalid

ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)

plt.show()
ser.close()  # Close serial connection after the plot closes


