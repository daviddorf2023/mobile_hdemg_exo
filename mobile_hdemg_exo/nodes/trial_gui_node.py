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
        self.analyzer_var = tk.BooleanVar()
        self.side_var = tk.StringVar()
        self.muscles_var = tk.StringVar()

        # Set window size
        self.root.geometry("500x320")

        # Initialize ROS node
        rospy.init_node("gui_node")

        self.create_widgets()

    def create_widgets(self):
        device_label = ttk.Label(self.root, text="Trial Type:")
        device_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        device_choices = ["Flat", "Trapezoid",
                          "Sinusoid", "Bidirectional Sinusoid"]
        self.device_dropdown = ttk.Combobox(
            self.root, values=device_choices, textvariable=self.device_var)
        self.device_dropdown.grid(row=0, column=1, padx=10, pady=5, sticky="w")

        method_label = ttk.Label(self.root, text="Method:")
        method_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        method_choices = ["RMS", "CST"]
        self.method_dropdown = ttk.Combobox(
            self.root, values=method_choices, textvariable=self.method_var)
        self.method_dropdown.grid(row=1, column=1, padx=10, pady=5, sticky="w")

        analyzer_label = ttk.Label(self.root, text="Latency analyzer:")
        analyzer_label.grid(row=2, column=0, padx=10, pady=5, sticky="w")
        analyzer_choices = ["Off", "On"]
        self.analyzer_dropdown = ttk.Combobox(
            self.root, values=analyzer_choices, textvariable=self.analyzer_var)
        self.analyzer_dropdown.grid(
            row=2, column=1, padx=10, pady=5, sticky="w")

        side_label = ttk.Label(self.root, text="Side:")
        side_label.grid(row=3, column=0, padx=10, pady=5, sticky="w")
        side_choices = ["Left", "Right"]
        self.side_dropdown = ttk.Combobox(
            self.root, values=side_choices, textvariable=self.side_var)
        self.side_dropdown.grid(row=3, column=1, padx=10, pady=5, sticky="w")

        muscles_label = ttk.Label(self.root, text="Muscles:")
        muscles_label.grid(row=4, column=0, padx=10, pady=5, sticky="w")
        muscles_choices = ["1", "2", "3", "4"]
        self.muscles_dropdown = ttk.Combobox(
            self.root, values=muscles_choices, textvariable=self.muscles_var)
        self.muscles_dropdown.grid(
            row=4, column=1, padx=10, pady=5, sticky="w")

        trials_label = ttk.Label(self.root, text="Number of Trials [1 to 10]:")
        trials_label.grid(row=5, column=0, padx=10, pady=5, sticky="w")
        self.trials_var = tk.StringVar()
        self.trials_var.set("1")  # Default value
        trials_entry = ttk.Entry(self.root, textvariable=self.trials_var)
        trials_entry.grid(row=5, column=1, padx=10, pady=5, sticky="w")

        remove_channels_label = ttk.Label(
            self.root, text="Channels to Remove (comma-separated):")
        remove_channels_label.grid(
            row=6, column=0, padx=10, pady=5, sticky="w")
        self.remove_channels_var = tk.StringVar()
        remove_channels_entry = ttk.Entry(
            self.root, textvariable=self.remove_channels_var)
        remove_channels_entry.grid(
            row=6, column=1, padx=10, pady=5, sticky="w")

        run_button = ttk.Button(
            self.root, text="Run", command=self.start_process)
        run_button.grid(row=7, columnspan=2, pady=10)

        exit_button = ttk.Button(
            self.root, text="Exit", command=self.root.destroy)
        exit_button.grid(row=8, columnspan=2, pady=10)

    def start_process(self):

        # Set flag to completed
        rospy.set_param("/trial_gui_completed", True)
