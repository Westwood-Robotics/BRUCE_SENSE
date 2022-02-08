import time
import os

from bruce_sense import Manager
s = Manager.SENSOR(port='COM9', baudrate=115200)
# data = s.get_dump()[0]
# print(data)
# start_time = time.time()
# for i in range(10000):
#     #os.system('cls' if os.name == 'nt' else 'clear')
#     data = s.get_dump()[0]
#
# end_time = time.time()
# freq = 10000/(end_time-start_time)
# print("Frequency: %f" % freq)
end_time = time.time()
while True:
    rtn = s.get_dump()
    data = rtn[0]
    contact = rtn[1]
    error = rtn[2]
    # os.system('cls' if os.name == 'nt' else 'clear')
    t = time.time()-end_time
    end_time = t
    # if not error:
    # print('Acceleration:')
    # print('X: %2.2f, Y: %2.2f, Z: %2.2f, time: %5.2f' % (data[0], data[1], data[2], t))
    print(bin(contact))
