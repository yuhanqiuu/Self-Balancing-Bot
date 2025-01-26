import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(i, theta_an_list, theta_gn_list, theta_n_list,ser):
    # Request new data from Arduino
    ser.write(b'g')  
    arduinoData_string = ser.readline().decode('utf-8').strip()  

    try:
        # Expecting two comma-separated values (e.g., "45.6,-30.2")
        value1, value2,value3 = map(float, arduinoData_string.split(","))  

        theta_an_list.append(value1)  # Store first data set
        theta_gn_list.append(value2)  # Store second data set
        theta_n_list.append(value3)  # Store third data set

    except Exception as e:  # Ignore bad data points
        print(f"Data error: {e}")

    # Keep only last 100 data points
    theta_an_list[:] = theta_an_list[-100:]
    theta_gn_list[:] = theta_gn_list[-100:]
    theta_n_list[:] = theta_n_list[-100:]

    ax.clear()  # Clear previous frame
    ax.plot(theta_an_list, label="Accelerometer", linestyle="dashed", color="red")  
    ax.plot(theta_gn_list, label="Gyroscope", linestyle="dashed", color="blue")  
    ax.plot(theta_n_list, label="Complementary", linestyle="dashed", color="green")  


    ax.set_ylim([-100, 100])  # Set Y-axis limits
    ax.set_yticks(range(-100, 100, 10))  # Set Y-axis ticks every 5 degrees
    ax.set_title("Arduino Sensor Data")  
    ax.set_ylabel("Sensor Values (degrees)")  
    ax.set_xlabel("Time (samples)")  
    ax.legend()  # Add legend

    ax.grid(True, color='gray', linestyle='--', linewidth=0.5, alpha=0.7) # Add grids

# Initialize data storage
theta_an_list = []
theta_gn_list = []
theta_n_list = []

# Set up figure
fig = plt.figure()
ax = fig.add_subplot(111)

# Establish Serial Connection (Update COM Port)
ser = serial.Serial("COM5", 9600)
time.sleep(2)  # Allow Arduino time to initialize

# Set up animation function
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(theta_an_list, theta_gn_list,theta_n_list, ser), interval=50)

plt.show()
ser.close()


