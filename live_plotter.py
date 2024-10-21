import matplotlib.pyplot as plt
import matplotlib.animation as animation

class LivePlot:
    def __init__(self):
        self.x_axis_counters = []
        self.roll_history = []
        self.pitch_history = []
        self.roll_setpoint_history = []
        self.pitch_setpoint_history = []

        self.heading_history = []
        self.altitude_history = []
        self.airspeed_history = []

        self.heading_setpoint_history = []
        self.altitude_setpoint_history = []
        self.airspeed_setpoint_history = []

        self.plot_array_max_length = 300  # Max number of data points
        self.i = 1  # Initialize x_axis counter

        # Create subplots
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4, self.ax5) = plt.subplots(5, 1, figsize=(10, 8))
        self.fig.suptitle("XPlane Autopilot System Control", fontsize=16)

        # Set titles for each subplot
        self.ax1.set_title("Roll")
        self.ax2.set_title("Pitch")
        self.ax3.set_title("Heading")
        self.ax4.set_title("Altitude")
        self.ax5.set_title("Airspeed")

        # Set up gridlines
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)
        self.ax4.grid(True)
        self.ax5.grid(True)

    def update_data(self, current_roll, desired_roll, current_pitch, desired_pitch,
                    current_heading, desired_heading, current_altitude, desired_altitude,
                    current_airspeed, desired_airspeed):

        if len(self.x_axis_counters) >= self.plot_array_max_length:
            # Remove oldest data
            self.x_axis_counters.pop(0)
            self.roll_history.pop(0)
            self.roll_setpoint_history.pop(0)
            self.pitch_history.pop(0)
            self.pitch_setpoint_history.pop(0)
            self.heading_history.pop(0)
            self.heading_setpoint_history.pop(0)
            self.altitude_history.pop(0)
            self.altitude_setpoint_history.pop(0)
            self.airspeed_history.pop(0)
            self.airspeed_setpoint_history.pop(0)

        # Append new data
        self.x_axis_counters.append(self.i)
        self.roll_history.append(current_roll)
        self.roll_setpoint_history.append(desired_roll)
        self.pitch_history.append(current_pitch)
        self.pitch_setpoint_history.append(desired_pitch)
        self.heading_history.append(current_heading)
        self.heading_setpoint_history.append(desired_heading)
        self.altitude_history.append(current_altitude)
        self.altitude_setpoint_history.append(desired_altitude)
        self.airspeed_history.append(current_airspeed)
        self.airspeed_setpoint_history.append(desired_airspeed)

        self.i += 1

    def animate(self, frame, *data):
        current_roll, desired_roll, current_pitch, desired_pitch, current_heading, desired_heading, current_altitude, desired_altitude, current_airspeed, desired_airspeed = data

        # Update data
        self.update_data(current_roll, desired_roll, current_pitch, desired_pitch, 
                         current_heading, desired_heading, current_altitude, 
                         desired_altitude, current_airspeed, desired_airspeed)

        # Clear plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        self.ax5.clear()

        # Re-plot the data with updated values
        self.ax1.plot(self.x_axis_counters, self.roll_history, label="Roll")
        self.ax1.plot(self.x_axis_counters, self.roll_setpoint_history, label="Roll Setpoint")
        self.ax1.legend()

        self.ax2.plot(self.x_axis_counters, self.pitch_history, label="Pitch")
        self.ax2.plot(self.x_axis_counters, self.pitch_setpoint_history, label="Pitch Setpoint")
        self.ax2.legend()

        self.ax3.plot(self.x_axis_counters, self.heading_history, label="Heading")
        self.ax3.plot(self.x_axis_counters, self.heading_setpoint_history, label="Heading Setpoint")
        self.ax3.legend()

        self.ax4.plot(self.x_axis_counters, self.altitude_history, label="Altitude")
        self.ax4.plot(self.x_axis_counters, self.altitude_setpoint_history, label="Altitude Setpoint")
        self.ax4.legend()

        self.ax5.plot(self.x_axis_counters, self.airspeed_history, label="Airspeed")
        self.ax5.plot(self.x_axis_counters, self.airspeed_setpoint_history, label="Airspeed Setpoint")
        self.ax5.legend()

        # Set grid lines again
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)
        self.ax4.grid(True)
        self.ax5.grid(True)

    def run(self, data_stream):
        # Call the animation function periodically to update the plots
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(data_stream), interval=100)

        # Display the plot
        plt.tight_layout()
        plt.show()

# Example usage:
# Create an instance of LivePlot and pass in the current data stream from your system
# data_stream is a placeholder for the actual values for current_roll, desired_roll, etc.
# live_plot = LivePlot()
# live_plot.run(data_stream)
