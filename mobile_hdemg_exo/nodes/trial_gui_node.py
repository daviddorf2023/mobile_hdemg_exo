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
        self.side_var = tk.StringVar()
        self.muscles_var = tk.StringVar()
        self.root.geometry("300x320")
        self.create_widgets()

    def create_widgets(self):
        device_label = ttk.Label(self.root, text="Trial Type:")
        device_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        device_choices = ["Flat", "Trapezoid",
                          "Sinusoid", "Bidirectional Sinusoid"]
        self.device_dropdown = ttk.Combobox(
            self.root, values=device_choices, textvariable=self.device_var)
        self.device_dropdown.grid(row=0, column=1, padx=10, pady=5, sticky="w")

        trials_label = ttk.Label(self.root, text="Number of Trials [1 to 10]:")
        trials_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.trials_var = tk.StringVar()
        self.trials_var.set("1")
        trials_entry = ttk.Entry(self.root, textvariable=self.trials_var)
        trials_entry.grid(row=1, column=1, padx=10, pady=5, sticky="w")

        run_button = ttk.Button(
            self.root, text="Run", command=self.start_process)
        run_button.grid(row=2, columnspan=2, pady=10)

        exit_button = ttk.Button(
            self.root, text="Exit", command=self.root.destroy)
        exit_button.grid(row=3, columnspan=2, pady=10)

    def start_process(self):
        rospy.set_param("/trial_gui_completed", True)


if __name__ == "__main__":
    rospy.init_node("trial_gui_node", anonymous=True)
    root = tk.Tk()
    app = GUIApp(root)
    root.mainloop()
    rospy.spin()
