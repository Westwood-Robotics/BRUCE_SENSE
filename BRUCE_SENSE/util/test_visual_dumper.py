import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

from bruce_sense import Manager
s = Manager.SENSOR(port='COM12', baudrate=2000000)

fig = plt.figure()
ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)
ts = []
rs = []
ps = []
ys = []
rxs = []
rys = []
rzs = []
ddxs = []
ddys = []
ddzs = []
ddxs_nog = []
ddys_nog = []
ddzs_nog = []
t0 = time.time()
data = {}

def animate(i, ts, rs, ps, ys, rxs, rys, rzs, ddxs, ddys, ddzs, ddxs_nog, ddys_nog, ddzs_nog):
    data, contact, error = s.get_dump()

    ts.append(time.time() - t0)

    ddxs_nog.append(data[6])
    ddys_nog.append(data[7])
    ddzs_nog.append(data[8])

    ddxs.append(data[0])
    ddys.append(data[1])
    ddzs.append(data[2]-9.7)

    rxs.append(data[3])
    rys.append(data[4])
    rzs.append(data[5])

    rs.append(data[9])
    ps.append(data[10])
    ys.append(data[11])
    print(contact)

    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()
    # ax1.plot(ts[-100:], rs[-100:], label='roll (x)')
    # ax1.plot(ts[-100:], ps[-100:], label='pitch (y)')
    ax1.plot(ts[-100:], ys[-100:], label='yaw (z)')
    ax1.legend(loc='upper left')
    #ax2.plot(ts[-100:], rxs[-100:], label='du (x)')
    #ax2.plot(ts[-100:], rys[-100:], label='dv (y)')
    ax2.plot(ts[-100:], rzs[-100:], label='dw (z)')
    ax2.legend(loc='upper left')
    ax3.plot(ts[-100:], ddxs[-100:], label='ddx')
    ax3.plot(ts[-100:], ddys[-100:], label='ddy')
    ax3.plot(ts[-100:], ddzs[-100:], label='ddz')
    ax3.legend(loc='upper left')
    ax4.plot(ts[-100:], ddxs_nog[-100:], label='ddx_no_g')
    ax4.plot(ts[-100:], ddys_nog[-100:], label='ddy_no_g')
    ax4.plot(ts[-100:], ddzs_nog[-100:], label='ddz_no_g')
    ax4.legend(loc='upper left')


ani = animation.FuncAnimation(fig, animate, fargs = (ts, rs, ps, ys, rxs, rys, rzs, ddxs, ddys, ddzs, ddxs_nog, ddys_nog,
                                                     ddzs_nog), interval = 1)
plt.show()
