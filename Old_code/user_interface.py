import tkinter as tk

braking_mode = 0
steering_value = 0
prop_value = 0

class UserInterface:
    def __init__(self, root):
        self.root = root
        global braking_mode, steering_value, prop_value

        # Create a frame to hold the controls
        frame_controls = tk.Frame(root)
        frame_controls.pack(side=tk.LEFT, pady=20, padx=20)

        # Create a frame to hold the label and button for braking mode
        frame_brake = tk.Frame(frame_controls)
        frame_brake.pack(pady=10)

        # Create a label for braking mode
        self.label_brake = tk.Label(frame_brake, text="Braking Mode", font=('Helvetica', 16))
        self.label_brake.pack(side=tk.LEFT)

        # Create a button for braking mode
        self.button = tk.Button(frame_brake, text=str(braking_mode), command=self.toggle_braking_mode, font=('Helvetica', 16), width=10, height=3)
        self.button.pack(side=tk.LEFT)

        # Create a frame to hold the label and slider for steering value
        frame_steering = tk.Frame(frame_controls)
        frame_steering.pack(pady=10)

        # Create a label for steering value
        self.label_steering = tk.Label(frame_steering, text="Steering Value", font=('Helvetica', 16))
        self.label_steering.pack(side=tk.LEFT)

        # Create a scale (slider) for steering value
        self.steering_scale = tk.Scale(frame_steering, from_=-100, to=100, orient=tk.HORIZONTAL, length=300)
        self.steering_scale.set(0)  # Initial value at the center
        self.steering_scale.pack(side=tk.LEFT)

        # Create a frame to hold the label and slider for propulsion value
        frame_propulsion = tk.Frame(frame_controls)
        frame_propulsion.pack(pady=10)

        # Create a label for propulsion value
        self.label_propulsion = tk.Label(frame_propulsion, text="Propulsion Value", font=('Helvetica', 16))
        self.label_propulsion.pack(side=tk.LEFT)

        # Create a scale (slider) for propulsion value
        self.propulsion_scale = tk.Scale(frame_propulsion, from_=0, to=1023, orient=tk.HORIZONTAL, length=300)
        self.propulsion_scale.set(0)  # Initial value at the left
        self.propulsion_scale.pack(side=tk.LEFT)

        # Create a text widget to act as a terminal
        self.terminal = tk.Text(root, height=20, width=50, font=('Helvetica', 12))
        self.terminal.pack(side=tk.RIGHT, padx=20, pady=20)

        # Bind event to slider release
        self.steering_scale.bind("<ButtonRelease-1>", self.update_steering_value)
        self.propulsion_scale.bind("<ButtonRelease-1>", self.update_propulsion_value)

    def toggle_braking_mode(self):
        global braking_mode
        braking_mode = 1 if braking_mode == 0 else 0
        self.button.config(text=str(braking_mode))
        self.log_to_terminal(f"Braking Mode: {braking_mode}")

    def update_steering_value(self, event):
        global steering_value
        steering_value = int(self.steering_scale.get())
        self.log_to_terminal(f"Steering Value: {steering_value}")

    def update_propulsion_value(self, event):
        global prop_value
        prop_value = int(self.propulsion_scale.get())
        self.log_to_terminal(f"Propulsion Value: {prop_value}")

    def log_to_terminal(self, message):
        self.terminal.insert(tk.END, message + '\n')
        self.terminal.see(tk.END)  # Auto-scroll to the end

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Braking Mode and Steering Control")
    app = UserInterface(root)
    root.mainloop()
