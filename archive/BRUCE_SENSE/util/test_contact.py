import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

from bruce_sense import Manager
s = Manager.SENSOR(port='COM12', baudrate=2000000)



while True:
    data, contact, error = s.get_dump()
    print(contact)
