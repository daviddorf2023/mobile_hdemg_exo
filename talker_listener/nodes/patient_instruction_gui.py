import tkinter as tk
import time
import pyttsx3
import rospy

def patient_node():
    # Create a tkinter window
    window = tk.Tk()

    # Set the window title
    window.title("Shirley Ryan AbilityLab - Patient Instruction for Ankle Exoskeleton")

    # Set the window to fullscreen
    window.attributes('-fullscreen', True)

    # Create a label to display the messages
    message_label = tk.Label(window, font=("Calibri", 100), pady=20)
    message_label.pack(expand=True)

    # Define the messages and their durations in seconds
    messages = [("Ready",2), ("Set",2), ("Go",2), ("Press your foot down", 5), ("Relax your foot", 3), ("Lift your foot up", 5), ("Relax your foot", 3), ("Great job!", 1)]

    # Initialize the text-to-speech engine
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)
    engine.setProperty('volume', 1.0)
    voices = engine.getProperty('voices')
    engine.setProperty('voice', voices[2].id)

    # Function to speak the given text
    def speak_text(text):
        engine.say(text)
        engine.runAndWait()

    # Function to update the message on the screen
    def update_message():
        for message, duration in messages:
            message_label.config(text=message)
            message_label.update()
            window.update()
            speak_text(message)
            time.sleep(duration)

    # Start the countdown
    update_message()

    # Close the window when finished
    window.destroy()

    # Start the tkinter event loop
    window.mainloop()

if __name__ == '__main__':
    try:
        patient_node()
    except rospy.ROSInterruptException:
        pass
