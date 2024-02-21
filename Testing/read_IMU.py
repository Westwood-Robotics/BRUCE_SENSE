from BRUCE_SENSE import Manager as sense_manager
import numpy as np
import matplotlib.pyplot as plt
import datetime as dt
import matplotlib.animation as animation

accel_new = np.array([0., 0., 0.])  # new      accelerometer reading
omega_new = np.array([0., 0., 0.])  # new      gyroscope     reading
PICO_baudrate = 1000000
PICO_port = 'COM44'
sm = sense_manager.SenseManager(port=PICO_port, baudrate=PICO_baudrate)

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
y_ay = []
y_az = []


# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    sm.read_data()
    # Add x and y to lists
    xs.append(dt.datetime.now())
    ys.append(sm.accel[0])

    # Limit x and y lists to 20 items
    xs = xs[-200:]
    ys = ys[-200:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.title('IMU')
    plt.ylabel('acc-x')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1)
plt.show()
