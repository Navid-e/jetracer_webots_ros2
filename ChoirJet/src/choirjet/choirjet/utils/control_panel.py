from tkinter import * 
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Empty
from ackermann_msgs.msg import AckermannDrive



class ControlPanel(Node):

    def __init__(self):
        super().__init__('control_panel',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True)

        self.n_agents = self.get_parameter('n_agents').value
        self.vicon_list = self.get_parameter('vicon_list').value
        self.publishers_cmd_ackermann = {}

        for i in range(self.n_agents):
            topic = '/agent_{}/cmd_ackermann'.format(i)
            print(topic)
            self.publishers_cmd_ackermann[i] = self.create_publisher(AckermannDrive, topic, 1)

        # initialize pose subscription
        self.publisher_experiment = self.create_publisher(Empty, '/experiment_trigger', 1)
        self.publisher_plot = self.create_publisher(Empty, '/plot', 1)
        self.publisher_save_pickle = self.create_publisher(Empty, '/pickle', 1)
            
        self.publisher_stop = self.create_publisher(Empty, '/stop', 1)
        self.root = Tk()
        self.root.geometry("400x250")
        B2 = Button(self.root, text = "Start", command = self.start_experiment)
        B2.place(x = 50,y = 50)
        B4 = Button(self.root, text = "Stop", command = self.stop)
        B4.place(x = 50,y = 100)
        B5 = Button(self.root, text = "Plot", command = self.plot)
        B5.place(x = 50,y = 150)
        B7 = Button(self.root, text = "pickle", command = self.save_pickle)
        B7.place(x = 50,y = 200)

        ### Trajectory Canvas
        self.canvas1 = Canvas(self.root, width = 100, height = 300)
        self.canvas1.pack()
        self.canvas1.place(x=150,y=0)

        agent_id_label = Label(self.root, text='ID')
        self.canvas1.create_window(0, 50, window=agent_id_label)
        self.agent_id = Entry (self.root) 
        self.canvas1.create_window(110, 50, window=self.agent_id)

        traj_time_label = Label(self.root, text='throttle')
        self.canvas1.create_window(0, 80, window=traj_time_label)
        self.cmd_throttle = Entry (self.root) 
        self.canvas1.create_window(110, 80, window=self.cmd_throttle)

        traj_x_label = Label(self.root, text='steering')
        self.canvas1.create_window(0, 110, window=traj_x_label)
        self.cmd_steering_angle = Entry (self.root) 
        self.canvas1.create_window(110, 110, window=self.cmd_steering_angle)

        B13 = Button(text='Send Commands', command=self.send_cmd)
        self.canvas1.create_window(100, 210, window=B13)

        self.root.mainloop()


    def send_cmd(self):
        id = int(self.agent_id.get())
        throttle = np.clip(-1.0,float(self.cmd_throttle.get()),1.0)
        steering_angle = np.clip(-1.0,float(self.cmd_steering_angle.get()),1.0)

        string = 'cmd sent to agent_{}: ({}, {})'.format(id,throttle, steering_angle)
        label1 = Label(self.root, text= string)
        self.canvas1.create_window(100, 240, window=label1)

        msg = AckermannDrive()
        msg.acceleration = throttle
        msg.steering_angle = steering_angle
        self.publishers_cmd_ackermann[id].publish(msg)

        self.get_logger().info('cmd ({}, {}) sent to agent_{} from control panel'.format(throttle, steering_angle,id))


    def start_experiment(self):
        msg = Empty()
        self.publisher_experiment.publish(msg)

    def stop(self):
        msg = Empty()
        self.publisher_stop.publish(msg)

    def plot(self):
        msg = Empty()
        self.publisher_plot.publish(msg)

    def save_pickle(self):
        msg = Empty()
        self.publisher_save_pickle.publish(msg)


