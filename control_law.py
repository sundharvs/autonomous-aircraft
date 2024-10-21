import PID
import xpc
import threading
from time import sleep
from math import exp
from live_plotter import LivePlot
from gps_horizontal_offset import horizontal_offset

class ControlLaw:
    DREFs = ["sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
        "sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot",
        "sim/flightmodel/failures/onground_any",
        "sim/flightmodel/misc/h_ind",
        "sim/cockpit/radios/nav1_hdef_dot",
        "sim/cockpit/radios/nav1_vdef_dot",
    ]
    
    def __init__(self):
        self.roll_PID = PID.PID(0.05, 0.01, 0.005)
        self.pitch_PID = PID.PID(0.06, 0.02, 0.014)

        self.altitude_PID = PID.PID(0.2, 0.04, 0.03)
        self.heading_PID = PID.PID(0.5,0.01,0.05,angle_wrap=True)
        self.speed_PID = PID.PID(0.2, 0.04, 0.09)

        self.loc_PID = PID.PID(-8.0,0.0,-0.02)
        self.gs_PID = PID.PID(3.0,0.3,0.05)
        self.flare_time = 0.0
        self.localizer_course = 59.6
        
        self.running = True
        self.mode = 1
        self.client = xpc.XPlaneConnect()
        
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_heading = 0.0
        self.current_altitude = 0.0
        self.current_asi = 0.0
        
        self.live_plot = LivePlot()

        
    def update_outer_loop(self):
        posi = self.client.getPOSI()
        multi_DREFs = self.client.getDREFs(self.DREFs)
        
        # Get signals
        self.current_hdg = multi_DREFs[1][0]
        self.current_altitude = multi_DREFs[3][0]
        
        # Feedback signals to PID controllers
        self.heading_PID.update(self.current_hdg) 
        self.altitude_PID.update(self.current_altitude)
                
    def update_inner_loop(self):
        posi = self.client.getPOSI()
        multi_DREFs = self.client.getDREFs(self.DREFs)

        # Get signals
        self.current_roll = posi[4]
        self.current_pitch = posi[3]
        self.current_asi = multi_DREFs[0][0]
        
        # Feedback signals to PID controllers
        self.roll_PID.update(self.current_roll)
        self.pitch_PID.update(self.current_pitch)
        self.speed_PID.update(self.current_asi)
        
    def update_ils_control(self):
        multi_DREFs = self.client.getDREFs(self.DREFs)
        current_loc_deviation = multi_DREFs[4][0]*1
        current_gs_deviation = multi_DREFs[5][0]*0.35
        self.gs_PID.update(current_gs_deviation)
        self.loc_PID.update(current_loc_deviation)
        
    def update_gps_control(self):
        multi_DREFs = self.client.getDREFs(self.DREFs)
        current_lat = multi_DREFs[6][0]
        current_long = multi_DREFs[7][0]
        horizontal_error = horizontal_offset(current_lat, current_long)
        self.gps_PID.update(horizontal_error)
        
    def mode_input(self):
        while self.running:
            user_input = input("Enter new mode: ")
            if user_input == "exit":
                self.running = False
            else:
                self.mode = float(user_input)
                
    def normalize(self, value, min=-1, max=1):
        if (value > max):
            return max
        elif (value < min):
            return min
        else:
            return value
    
    def mode_switch(self):
        posi = self.client.getPOSI()
        multi_DREFs = self.client.getDREFs(self.DREFs)
        
        current_asi = multi_DREFs[0][0]
        onground = multi_DREFs[2][0]
        current_altitude = multi_DREFs[3][0]
        
        mode = self.mode
        
        if mode == 1 and current_asi >= 70.0:
            #When you hit 70 kts (V1) takeoff
            mode = 2
            print("Switched to Mode 2")
            return mode
        if mode == 2 and current_altitude >= 500.0:
            # When altitude hits 500 ft, turn to crosswind leg
            mode = 3
            print("Switched to Mode 3")
            return mode
        if mode == 3 and current_altitude >= 1000.0:
            # When altitude hits 1000 ft, turn to downwind leg
            mode = 4
            print("Switched to Mode 4")
            return mode
        if mode == 6 and current_altitude <= 70.0:
            # When in approach mode and altitude hits 70 ft, enter flare mode
            mode = 7
            print("Switched to Mode 7")
            return mode
        
        if (mode == 6 or mode == 7) and onground:
            # When in approach mode or flare mode and weight on wheels is True, come to a stop
            mode = 8
            print("Switched to Mode 8")
            return mode
        else:
            return self.mode
                
    def main_loop(self):
        while self.running:
            self.mode = self.mode_switch()
            
            if(self.mode == 1):
                self.speed_PID.SetPoint = 70
                self.heading_PID.SetPoint = 59.6
            
            if(self.mode == 2):
                self.speed_PID.SetPoint = 90
                self.altitude_PID.SetPoint = 500
            
            if(self.mode == 3):
                self.heading_PID.SetPoint = 330
                self.altitude_PID.SetPoint = 1000
            
            if(self.mode == 4):
                self.heading_PID.SetPoint = 240
                
            if(self.mode == 5):
                self.heading_PID.SetPoint = 125 # Localizer intercept heading
            
            if(self.mode == 6):
                # ILS tracking
                self.speed_PID.SetPoint = 70
                self.loc_PID.SetPoint = 0
                self.gs_PID.SetPoint = 0
                self.update_ils_control()
                self.heading_PID.SetPoint = self.localizer_course + self.loc_PID.output
            
            if(self.mode == 7):
                self.heading_PID.SetPoint = self.localizer_course + self.loc_PID.output
                self.altitude_PID.SetPoint = 23*exp(-self.flare_time/3) + 47
                self.speed_PID.SetPoint = -1
                self.flare_time += 0.05
            
            if(self.mode == 8):
                self.speed_PID.SetPoint = -1
                self.altitude_PID.SetPoint = -1
                self.heading_PID.SetPoint = 59.6
            
            self.update_outer_loop()
            
            if(self.mode == 6):
                self.pitch_PID.SetPoint = self.normalize(self.gs_PID.output, min=-5, max=5)
            else:
                self.pitch_PID.SetPoint = self.normalize(self.altitude_PID.output, min=-10, max=10)
                
            self.roll_PID.SetPoint = self.normalize(self.heading_PID.output, min=-20, max=20)
            
            self.update_inner_loop()
            
            new_ail_ctrl = self.normalize(self.roll_PID.output)
            new_ele_ctrl = self.normalize(self.pitch_PID.output)
            new_throttle_ctrl = self.normalize(self.speed_PID.output,min=0,max=1)
            new_rud_ctrl = self.normalize(self.roll_PID.output*5)
            
            if(self.mode == 1 or self.mode == 8):
                ctrl = [new_ele_ctrl, 0.0, -998, new_throttle_ctrl] # ele, ail, rud, thr. -998 means don't change
            else:
                ctrl = [new_ele_ctrl, new_ail_ctrl, 0.0, new_throttle_ctrl]
                            
            self.client.sendCTRL(ctrl)
            self.live_plot.update(self.current_roll, self.roll_PID.SetPoint, self.current_pitch, self.pitch_PID.SetPoint, self.current_hdg, self.heading_PID.SetPoint, self.current_altitude, self.altitude_PID.SetPoint, self.current_asi, self.speed_PID.SetPoint)
            sleep(0.05)
            
if __name__ == "__main__":
    control_law = ControlLaw()
    
    loop_thread = threading.Thread(target=control_law.main_loop)
    input_thread = threading.Thread(target=control_law.mode_input)

    loop_thread.start()
    input_thread.start()

    # Wait for threads to finish
    loop_thread.join()
    input_thread.join()
