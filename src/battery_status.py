#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
import tkinter as tk
from tkinter import ttk
import time
import threading
import signal
import sys
import queue

class BatteryWindow:
    def __init__(self, root):
        self.root = root
        self.root.title("Battery Status")
        self.root.geometry("200x400+1500+100")
        self.root.attributes('-topmost', True)

        self.canvas = tk.Canvas(self.root, width=200, height=400)
        self.canvas.pack()

        self.battery_label = tk.Label(self.root, text="No Data", font=("Arial", 24))
        self.battery_label.place(x=50, y=160)

        self.percentage = None
        self.last_update_time = time.time()

        self.update_timer()

    def update_battery(self, percentage):
        self.percentage = percentage
        self.last_update_time = time.time()
        self.root.after(0, self.update_graphics)

    def update_graphics(self):
        self.canvas.delete("all")
        if self.percentage is not None:
            battery_height = 300
            charge_height = int(battery_height * self.percentage)
            self.canvas.create_rectangle(50, 50, 150, 350, outline="black", width=2)
            self.canvas.create_rectangle(50, 350 - charge_height, 150, 350, fill="green")
            self.battery_label.config(text=f'{int(self.percentage * 100)}%')
        else:
            self.battery_label.config(text="No Data")

    def update_timer(self):
        if time.time() - self.last_update_time > 1:
            self.battery_label.config(text="Disconnected")
            self.canvas.delete("all")
            self.canvas.create_rectangle(50, 50, 150, 350, outline="black", width=2)
        self.root.after(1000, self.update_timer)

def callback(data):
    update_queue.put(data.percentage)

def listener():
    rospy.Subscriber("/mirte/power/power_watcher", BatteryState, callback)
    rospy.spin()

def process_queue():
    try:
        while True:
            percentage = update_queue.get_nowait()
            app.update_battery(percentage)
    except queue.Empty:
        pass
    root.after(100, process_queue)

def signal_handler(sig, frame):
    print('Ctrl+C pressed, exiting...')
    rospy.signal_shutdown('Shutdown signal received')
    root.quit()

if __name__ == '__main__':
    rospy.init_node('battery_listener', anonymous=True)

    root = tk.Tk()
    app = BatteryWindow(root)

    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Queue for thread-safe communication
    update_queue = queue.Queue()

    # Start the ROS subscriber in a separate thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    # Start processing the queue
    root.after(100, process_queue)

    root.mainloop()

    # Ensure the listener thread finishes
    listener_thread.join()



# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import BatteryState
# import tkinter as tk
# from tkinter import ttk
# import time
# import threading
# import signal
# import sys
# import queue

# class BatteryWindow:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("Battery Status")
#         self.root.geometry("200x400")
#         self.root.attributes('-topmost', True)

#         self.canvas = tk.Canvas(self.root, width=200, height=400)
#         self.canvas.pack()

#         self.battery_label = tk.Label(self.root, text="No Data", font=("Arial", 24))
#         self.battery_label.place(x=50, y=160)

#         self.percentage = None
#         self.last_update_time = time.time()

#         self.update_timer()

#     def update_battery(self, percentage):
#         self.percentage = percentage
#         self.last_update_time = time.time()
#         self.root.after(0, self.update_graphics)

#     def update_graphics(self):
#         self.canvas.delete("all")
#         if self.percentage is not None:
#             battery_height = 300
#             charge_height = int(battery_height * self.percentage)
#             self.canvas.create_rectangle(50, 50, 150, 350, outline="black", width=2)
#             self.canvas.create_rectangle(50, 350 - charge_height, 150, 350, fill="green")
#             self.battery_label.config(text=f'{int(self.percentage * 100)}%')
#         else:
#             self.battery_label.config(text="No Data")

#     def update_timer(self):
#         if time.time() - self.last_update_time > 1:
#             self.battery_label.config(text="Disconnected")
#             self.canvas.delete("all")
#             self.canvas.create_rectangle(50, 50, 150, 350, outline="black", width=2)
#         self.root.after(1000, self.update_timer)

# def callback(data):
#     update_queue.put(data.percentage)

# def listener():
#     rospy.Subscriber("/mirte/power/power_watcher", BatteryState, callback)
#     rospy.spin()

# def process_queue():
#     try:
#         while True:
#             percentage = update_queue.get_nowait()
#             app.update_battery(percentage)
#     except queue.Empty:
#         pass
#     root.after(100, process_queue)

# def signal_handler(sig, frame):
#     print('Ctrl+C pressed, exiting...')
#     root.quit()
#     sys.exit(0)

# if __name__ == '__main__':
#     rospy.init_node('battery_listener', anonymous=True)

#     root = tk.Tk()
#     app = BatteryWindow(root)

#     # Set up signal handler for graceful shutdown
#     signal.signal(signal.SIGINT, signal_handler)

#     # Queue for thread-safe communication
#     update_queue = queue.Queue()

#     # Start the ROS subscriber in a separate thread
#     listener_thread = threading.Thread(target=listener)
#     listener_thread.start()

#     # Start processing the queue
#     root.after(100, process_queue)

#     root.mainloop()


