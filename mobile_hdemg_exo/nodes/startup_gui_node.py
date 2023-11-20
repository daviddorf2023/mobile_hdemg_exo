import tkinter as tk
from tkinter import ttk
import rospy


class GUIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SRAL hdEMG Exoskeleton")
        self.device_var = tk.StringVar()
        self.method_var = tk.StringVar()
        self.analyzer_var = tk.BooleanVar()
        self.exo_var = tk.StringVar()
        self.side_var = tk.StringVar()
        self.muscles_var = tk.StringVar()
        self.root.geometry("520x280")
        self.create_widgets()
        rospy.on_shutdown(self.shutdown)

    def create_widgets(self):
        device_label = ttk.Label(self.root, text="Device:")
        device_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        device_choices = ["Muovi+Pro", "Quattrocento", "File"]
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
            row=2, column=1, padx=10, pady=5, sticky="w")\

        muscles_label = ttk.Label(self.root, text="Muscles:")
        muscles_label.grid(row=3, column=0, padx=10, pady=5, sticky="w")
        muscles_choices = ["1", "2", "3", "4"]
        self.muscles_dropdown = ttk.Combobox(
            self.root, values=muscles_choices, textvariable=self.muscles_var)
        self.muscles_dropdown.grid(
            row=3, column=1, padx=10, pady=5, sticky="w")

        side_label = ttk.Label(self.root, text="Side:")
        side_label.grid(row=4, column=0, padx=10, pady=5, sticky="w")
        side_choices = ["Left", "Right"]
        self.side_dropdown = ttk.Combobox(
            self.root, values=side_choices, textvariable=self.side_var)
        self.side_dropdown.grid(row=4, column=1, padx=10, pady=5, sticky="w")

        remove_channels_label = ttk.Label(
            self.root, text="Channels to Remove (comma-separated):")
        remove_channels_label.grid(
            row=5, column=0, padx=10, pady=5, sticky="w")
        self.remove_channels_var = tk.StringVar()
        remove_channels_entry = ttk.Entry(
            self.root, textvariable=self.remove_channels_var)
        remove_channels_entry.grid(
            row=5, column=1, padx=10, pady=5, sticky="w")

        run_button = ttk.Button(
            self.root, text="Run", command=self.start_process)
        run_button.grid(row=6, columnspan=2, pady=10)

        exit_button = ttk.Button(
            self.root, text="Exit", command=self.root.destroy)
        exit_button.grid(row=7, columnspan=2, pady=10)

    def start_process(self):
        selected_device = self.device_var.get()
        selected_method = self.method_var.get()
        if self.analyzer_var.get() == "On":
            selected_analyzer = True
        else:
            selected_analyzer = False
        selected_side = self.side_var.get()
        selected_muscles = int(self.muscles_var.get())
        channels_to_remove = self.remove_channels_var.get()

        rospy.set_param("/device", selected_device)
        rospy.set_param("/method", selected_method)
        rospy.set_param("/latency_analyzer", selected_analyzer)
        rospy.set_param("/side", selected_side)
        rospy.set_param("/muscle_count", selected_muscles)
        rospy.set_param("/channels_to_remove", channels_to_remove)

        print("Device:", selected_device)
        print("Method:", selected_method)
        print("Analyzer:", selected_analyzer)
        print("Muscles:", selected_muscles)
        print("Removed Channels:", channels_to_remove)

        rospy.set_param("/startup_gui_completed", True)

    def shutdown(self):
        self.root.destroy()


if __name__ == "__main__":
    rospy.init_node('startup_gui_node', anonymous=True)
    root = tk.Tk()
    app = GUIApp(root)
    root.mainloop()
    rospy.spin()
