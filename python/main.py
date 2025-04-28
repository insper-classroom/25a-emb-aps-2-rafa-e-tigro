from sensor_capt import MPU6050Visualizer
import time
import numpy as np
import pyautogui as pg
port_name = '/dev/cu.usbmodem102'  # Update this to your actual port
visualizer = MPU6050Visualizer(port_name)
visualizer.open_port()

last_move = time.time()
while True:

    try:
        data, buttons = visualizer.get_state()
        ax = data[3]
        ay = data[4]
        az = data[5]
        if time.time() - last_move > 0.5:
            if (ax) < -1.5:
                pg.press('space')
                last_move = time.time()                           
            if buttons[1]:
                pg.press('a')
                last_move = time.time()
            if buttons[2]:
                last_move = time.time()
                pg.press('d')
            if buttons[3]:
                last_move = time.time()
                pg.press('w')
    except:
        pass
    