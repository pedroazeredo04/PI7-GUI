import tkinter as tk
import numpy as np

LEG_WINDOW_WIDTH = 800
LEG_WINDOW_HEIGHT = 600

def rad_to_deg(rad):
    return rad * 180 / np.pi

class MechanicalLegInterface:
    def __init__(self, master, RPi_Serial):
        self.master = master
        self.RPi_Serial = RPi_Serial
        
        self.canvas = tk.Canvas(master, width=LEG_WINDOW_WIDTH, height=LEG_WINDOW_HEIGHT, bg="white")
        self.canvas.grid(row=0, column=0, rowspan=7)

        self.control_frame = tk.Frame(master)
        self.control_frame.grid(row=0, column=1, padx=10, pady=10)

        # Thigh motor PID controls
        self.thigh_pid_frame = tk.Frame(self.control_frame)
        self.thigh_pid_frame.pack(pady=10)
        tk.Label(self.thigh_pid_frame, text="Thigh Motor PID").grid(row=0, columnspan=2)
        self.thigh_kp_label = tk.Label(self.thigh_pid_frame, text="Kp:")
        self.thigh_kp_label.grid(row=1, column=0, sticky="w")
        self.thigh_kp_entry = tk.Entry(self.thigh_pid_frame)
        self.thigh_kp_entry.grid(row=1, column=1)
        self.thigh_ki_label = tk.Label(self.thigh_pid_frame, text="Ki:")
        self.thigh_ki_label.grid(row=2, column=0, sticky="w")
        self.thigh_ki_entry = tk.Entry(self.thigh_pid_frame)
        self.thigh_ki_entry.grid(row=2, column=1)
        self.thigh_kd_label = tk.Label(self.thigh_pid_frame, text="Kd:")
        self.thigh_kd_label.grid(row=3, column=0, sticky="w")
        self.thigh_kd_entry = tk.Entry(self.thigh_pid_frame)
        self.thigh_kd_entry.grid(row=3, column=1)
        self.send_thigh_pid_button = tk.Button(self.thigh_pid_frame, text="Send Thigh PID", command=self.send_thigh_pid)
        self.send_thigh_pid_button.grid(row=4, columnspan=2, pady=5)

        # Calf motor PID controls
        self.calf_pid_frame = tk.Frame(self.control_frame)
        self.calf_pid_frame.pack(pady=10)
        tk.Label(self.calf_pid_frame, text="Calf Motor PID").grid(row=0, columnspan=2)
        self.calf_kp_label = tk.Label(self.calf_pid_frame, text="Kp:")
        self.calf_kp_label.grid(row=1, column=0, sticky="w")
        self.calf_kp_entry = tk.Entry(self.calf_pid_frame)
        self.calf_kp_entry.grid(row=1, column=1)
        self.calf_ki_label = tk.Label(self.calf_pid_frame, text="Ki:")
        self.calf_ki_label.grid(row=2, column=0, sticky="w")
        self.calf_ki_entry = tk.Entry(self.calf_pid_frame)
        self.calf_ki_entry.grid(row=2, column=1)
        self.calf_kd_label = tk.Label(self.calf_pid_frame, text="Kd:")
        self.calf_kd_label.grid(row=3, column=0, sticky="w")
        self.calf_kd_entry = tk.Entry(self.calf_pid_frame)
        self.calf_kd_entry.grid(row=3, column=1)
        self.send_calf_pid_button = tk.Button(self.calf_pid_frame, text="Send Calf PID", command=self.send_calf_pid)
        self.send_calf_pid_button.grid(row=4, columnspan=2, pady=5)

        # Command buttons
        self.command_frame = tk.Frame(self.control_frame)
        self.command_frame.pack(pady=10)
        self.start_button = tk.Button(self.command_frame, text="Start", command=self.start_command)
        self.start_button.grid(row=0, columnspan=2, pady=5)
        self.stop_button = tk.Button(self.command_frame, text="Stop", command=self.stop_command)
        self.stop_button.grid(row=1, columnspan=2, pady=5)
        self.goto_button = tk.Button(self.command_frame, text="Go To", command=self.goto_command)
        self.goto_button.grid(row=2, columnspan=2, pady=5)
        self.command_text = tk.Label(self.command_frame, text="")
        self.command_text.grid(row=3, columnspan=2, pady=5)

        self.canvas.bind("<B1-Motion>", self.move_leg)

        # Leg parameters
        self.leg_lines = []
        self.thigh_length = 190  # 190 mm for the thigh
        self.calf_length = 200  # 200 mm for the calf
        self.hip_x, self.hip_y = (LEG_WINDOW_WIDTH/2), 100
        self.foot_x, self.foot_y = (LEG_WINDOW_WIDTH/2), (self.thigh_length + self.calf_length)
        self.knee_x = 0
        self.knee_y = 0
        self.knee_angle = 0

        self.draw_leg()

        # Obstacle parameters
        self.obstacle_lines = []
        self.obstacle_lenght = 24
        self.obstacle_height = 104
        self.obstacle_x, self.obstacle_y = (LEG_WINDOW_WIDTH/2), 400
        self.draw_obstacle()

        # Floor parameters
        self.floor_lines = []
        self.floor_lenght = LEG_WINDOW_WIDTH
        self.draw_floor()


    def get_thigh_angle_degrees(self):
        return rad_to_deg(np.arctan2(self.knee_x - self.hip_x, self.knee_y - self.hip_y))
    
    def get_knee_angle_degrees(self):
        return rad_to_deg(self.knee_angle) * 2

    def draw_leg(self):
        if self.leg_lines:
            for line in self.leg_lines:
                self.canvas.delete(line)
        # Calculate knee position
        dx = self.foot_x - self.hip_x
        dy = self.foot_y - self.hip_y
        d = np.hypot(dx, dy)
        if d > self.thigh_length + self.calf_length:
            d = self.thigh_length + self.calf_length
        angle = np.arctan2(dy, dx)
        self.knee_angle = np.arccos((self.thigh_length ** 2 + d ** 2 - self.calf_length ** 2) / (2 * self.thigh_length * d))
        self.knee_x = self.hip_x + self.thigh_length * np.cos(angle - self.knee_angle)
        self.knee_y = self.hip_y + self.thigh_length * np.sin(angle - self.knee_angle)
        self.leg_lines = [
            self.canvas.create_line(self.hip_x, self.hip_y, self.knee_x, self.knee_y, fill="blue", width=5),
            self.canvas.create_line(self.knee_x, self.knee_y, self.foot_x, self.foot_y, fill="red", width=5)
        ]

    def draw_obstacle(self):
        if self.obstacle_lines:
            for line in self.obstacle_lines:
                self.canvas.delete(line)
        self.obstacle_lines = [
            self.canvas.create_line(self.obstacle_x - self.obstacle_lenght/2, self.obstacle_y, self.obstacle_x + self.obstacle_lenght/2, self.obstacle_y, fill="green", width=5),
            self.canvas.create_line(self.obstacle_x - self.obstacle_lenght/2, self.obstacle_y, self.obstacle_x - self.obstacle_lenght/2, self.obstacle_y + self.obstacle_height, fill="green", width=5),
            self.canvas.create_line(self.obstacle_x + self.obstacle_lenght/2, self.obstacle_y, self.obstacle_x + self.obstacle_lenght/2, self.obstacle_y + self.obstacle_height, fill="green", width=5),
            self.canvas.create_line(self.obstacle_x - self.obstacle_lenght/2, self.obstacle_y + self.obstacle_height, self.obstacle_x + self.obstacle_lenght/2, self.obstacle_y + self.obstacle_height, fill="green", width=5)
        ]

    def draw_floor(self):
        if self.floor_lines:
            for line in self.floor_lines:
                self.canvas.delete(line)
        self.floor_lines = [
            self.canvas.create_line(0, 500, LEG_WINDOW_WIDTH, 500, fill="black", width=5)
        ]

    def move_leg(self, event):
        dx = event.x - self.hip_x
        dy = event.y - self.hip_y
        d = np.hypot(dx, dy)
        if d > self.thigh_length + self.calf_length:
            angle = np.arctan2(dy, dx)
            event.x = self.hip_x + (self.thigh_length + self.calf_length) * np.cos(angle)
            event.y = self.hip_y + (self.thigh_length + self.calf_length) * np.sin(angle)
        self.foot_x, self.foot_y = event.x, event.y
        self.draw_leg()

    def goto_command(self):
        coordinates = np.array([[self.foot_x + (500 - LEG_WINDOW_WIDTH/2)], [500 - self.foot_y]])
        self.command_text["text"] = f"x: {round(coordinates.item(0),2)} | y: {round(coordinates.item(1),2)}\n" + f"angulo da coxa: {round(self.get_thigh_angle_degrees(), 2)}\n" + f"angulo da panturras: {round(self.get_knee_angle_degrees(), 2)}"
        #print(self.RPi_Serial.goto(coordinates))

    def start_command(self):
        self.command_text["text"] = "Start command"
        # print(RPi_Serial.send_trajectory(t_coordinates))

    def stop_command(self):
        self.command_text["text"] =  "Stop command"
        # self.RPi_Serial.write_register(globals.REG_STOP, 1)

    def send_thigh_pid(self):
        kp = float(self.thigh_kp_entry.get()) if (self.thigh_kp_entry.get() != "") else 0
        ki = float(self.thigh_ki_entry.get()) if (self.thigh_ki_entry.get() != "") else 0
        kd = float(self.thigh_kd_entry.get()) if (self.thigh_kd_entry.get() != "") else 0
        self.set_thigh_pid(kp, ki, kd)
        self.command_text["text"] =  f"Send Thigh PID: \nKp={kp}, Ki={ki}, Kd={kd}"

    def send_calf_pid(self):
        kp = float(self.calf_kp_entry.get()) if (self.calf_kp_entry.get() != "") else 0
        ki = float(self.calf_ki_entry.get()) if (self.calf_ki_entry.get() != "") else 0
        kd = float(self.calf_kd_entry.get()) if (self.calf_kd_entry.get() != "") else 0
        self.set_calf_pid(kp, ki, kd)
        self.command_text["text"] = f"Send Calf PID: \nKp={kp}, Ki={ki}, Kd={kd}"

    def set_thigh_pid(self, kp, ki, kd):
        self.command_text["text"] = f"Set THIGH PID: \nKp={kp}, Ki={ki}, Kd={kd}"
        # self.RPi_Serial.write_register(globals.REG_KP, kp)
        # self.RPi_Serial.write_register(globals.REG_KI, ki)
        # self.RPi_Serial.write_register(globals.REG_KD, kd)

    def set_calf_pid(self, kp, ki, kd):
        self.command_text["text"] =  f"Set CALF PID: \nKp={kp}, Ki={ki}, Kd={kd}"
        # self.RPi_Serial.write_register(globals.REG_KP, kp)
        # self.RPi_Serial.write_register(globals.REG_KI, ki)
        # self.RPi_Serial.write_register(globals.REG_KD, kd)


def main():
    root = tk.Tk()
    root.title("Mechanical Leg Control Interface")
    RPi_Serial = None  # Replace with your SerialComm instance
    app = MechanicalLegInterface(root, RPi_Serial)
    root.mainloop()

if __name__ == '__main__':
    main()
