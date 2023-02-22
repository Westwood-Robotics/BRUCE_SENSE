import time

from bruce_sense import Manager
s = Manager.SENSOR(port='COM9', baudrate=115200)

start_time = time.time()
for i in range(5000):
    data = s.get_dump()[0]
end_time = time.time()
freq = 5000/(end_time-start_time)
print("Frequency: %f" % freq)
