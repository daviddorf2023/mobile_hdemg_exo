import tkinter as tk
from tkinter import ttk
import rospy

class GUIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SRAL hdEMG Exoskeleton")

        # Initialize dropdown inputs
        self.device_var = tk.StringVar()
        self.method_var = tk.StringVar()
        self.analyzer_var = tk.StringVar()
        self.side_var = tk.StringVar()
        self.muscles_var = tk.StringVar()

        # Set window size
        self.root.geometry("400x300")  # Adjust dimensions as needed

        # Initialize ROS node
        rospy.init_node("gui_node")

        self.create_widgets()

    def create_widgets(self):
        device_label = ttk.Label(self.root, text="Device:")
        device_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")

        device_choices = ["Quattrocento", "MuoviPro", "Simulation"]
        self.device_dropdown = ttk.Combobox(self.root, values=device_choices, textvariable=self.device_var)
        self.device_dropdown.grid(row=0, column=1, padx=10, pady=5)

        method_label = ttk.Label(self.root, text="Method:")
        method_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")

        method_choices = ["RMS", "CST"]
        self.method_dropdown = ttk.Combobox(self.root, values=method_choices, textvariable=self.method_var)
        self.method_dropdown.grid(row=1, column=1, padx=10, pady=5)

        analyzer_label = ttk.Label(self.root, text="Analyzer:")
        analyzer_label.grid(row=2, column=0, padx=10, pady=5, sticky="w")

        analyzer_choices = ["On", "Off"]
        self.analyzer_dropdown = ttk.Combobox(self.root, values=analyzer_choices, textvariable=self.analyzer_var)
        self.analyzer_dropdown.grid(row=2, column=1, padx=10, pady=5)

        side_label = ttk.Label(self.root, text="Side:")
        side_label.grid(row=3, column=0, padx=10, pady=5, sticky="w")

        side_choices = ["Left", "Right"]
        self.side_dropdown = ttk.Combobox(self.root, values=side_choices, textvariable=self.side_var)
        self.side_dropdown.grid(row=3, column=1, padx=10, pady=5)

        muscles_label = ttk.Label(self.root, text="Muscles:")
        muscles_label.grid(row=4, column=0, padx=10, pady=5, sticky="w")

        muscles_choices = ["1", "2", "3", "4"]
        self.muscles_dropdown = ttk.Combobox(self.root, values=muscles_choices, textvariable=self.muscles_var)
        self.muscles_dropdown.grid(row=4, column=1, padx=10, pady=5)

        start_button = ttk.Button(self.root, text="Start", command=self.start_process)
        start_button.grid(row=5, columnspan=2, pady=10)

    def start_process(self):
        selected_device = self.device_var.get()
        selected_method = self.method_var.get()
        selected_analyzer = self.analyzer_var.get()
        selected_side = self.side_var.get()
        selected_muscles = self.muscles_var.get()

        rospy.set_param("/device", selected_device)
        rospy.set_param("/method", selected_method)
        rospy.set_param("/latency_analyzer", selected_analyzer)
        rospy.set_param("/side", selected_side)
        rospy.set_param("/muscle_count", selected_muscles)

        print("Selected Device:", selected_device)
        print("Selected Method:", selected_method)
        print("Selected Analyzer:", selected_analyzer)
        print("Selected Side:", selected_side)
        print("Selected Muscles:", selected_muscles)

        # Close the GUI window
        self.root.destroy()

        # Set flag to completed
        rospy.set_param("/gui_completed", True)

if __name__ == "__main__":
    root = tk.Tk()
    app = GUIApp(root)
    root.mainloop()
