from time import sleep
import xpc
import PID

def total_energy_control(h_sp, V_sp, h, V, hdot, Vdot):
    g = 32.2 # ft/s^2

    hdot_min = -17 # ft/s
    hdot_max = 17 # ft/s
    vdot_min = -10 # ft/s^2
    vdot_max = 10 # ft/s^2
    
    airspeed_error_gain = 5.0
    altitude_error_gain = 5.0
    Vdot_sp = min(max((V_sp - V)*airspeed_error_gain, vdot_min), vdot_max)
    hdot_sp = min(max((h_sp - h)*altitude_error_gain, hdot_min), hdot_max)

    SKE = 0.5*V*V
    SPE = h*g
    
    SPEdot = hdot * g
    SKEdot = V * Vdot
    
    # Specific energy setpoints
    SPE_sp = h_sp*g
    SKE_sp = 0.5*V_sp*V_sp
    
    # Specific energy rate setpoints (m**2/s**3)
    SKEdot_sp = V_sp * Vdot_sp
    SPEdot_sp = hdot_sp * g
    
    # PITCH STUFF
    # Specific energy balance
    pitch_speed_weight = 1
    SPE_weight = 2 - pitch_speed_weight
    SKE_weight = pitch_speed_weight
    
    SEBdot = SPEdot*SPE_weight - SKEdot*SKE_weight
    SEBdot_sp = SPEdot_sp*SPE_weight - SKEdot_sp*SKE_weight

    climb_angle_to_SEB_rate = V * g
    pitch_integ_state = 0
    
    pitch_damping_gain = 0.1
    seb_rate_ff = 1.0
    
    SEBdot_correction = (SEBdot_sp - SEBdot) * pitch_damping_gain + seb_rate_ff *SEBdot_sp
    pitch_setpoint_unc = SEBdot_correction / climb_angle_to_SEB_rate + pitch_integ_state

    # THROTTLE STUFF
    # Specific total energy
    STEdot = SPEdot + SKEdot    
    STEdot_sp = SPEdot_sp + SKEdot_sp
    
    # // Adjust the demanded total energy rate to compensate for induced drag rise in turns.
	# // Assume induced drag scales linearly with normal load factor.
	# // The additional normal load factor is given by (1/cos(bank angle) - 1)
    # STEdot_sp = STEdot_sp + load_factor_correction * (load_factor - 1.0)
    
    max_climb_rate = 16.67 # ft/s (1000 fpm)
    min_sink_rate = -16.67 # ft/s (1000 fpm)
    STEdot_max = max_climb_rate*g
    STEdot_min = min_sink_rate*g
    STEdot_to_throttle = 1/(STEdot_max - STEdot_min)
    
    throttle_damping_gain = 0.05
    throttle_trim = 0.7
    throttle_above_trim_per_ste_rate = (1 - throttle_trim) / STEdot_max
    throttle_below_trim_per_ste_rate = throttle_trim / STEdot_max
    
    if (STEdot_sp >= 0):
        throttle_ff = throttle_trim + STEdot_sp * throttle_above_trim_per_ste_rate
    else:
        throttle_ff = throttle_trim - STEdot_sp * throttle_below_trim_per_ste_rate

    throttle_setpoint = ((STEdot_sp - STEdot) * throttle_damping_gain) * STEdot_to_throttle + throttle_ff
    
    return pitch_setpoint_unc, throttle_setpoint

if __name__ == "__main__":
    
    DREFs = ["sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
        "sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot",
        "sim/flightmodel/failures/onground_any",
        "sim/flightmodel/misc/h_ind",
        "sim/cockpit2/gauges/indicators/vvi_fpm_pilot",
        "sim/cockpit2/gauges/indicators/airspeed_acceleration_kts_sec_pilot"
        ]
    
    h_sp = 1500 # ft
    V_sp = 160 # ft/s
    
    pitch_PID = PID.PID(0.06, 0.02, 0.014)
    
    with xpc.XPlaneConnect() as client:
        while(True):
            posi = client.getPOSI()
            ctrl = client.getCTRL()
            multi_DREFs = client.getDREFs(DREFs)

            current_roll = posi[4]
            current_pitch = posi[3]
            current_hdg = multi_DREFs[1][0]
            current_altitude = multi_DREFs[3][0]
            current_asi = multi_DREFs[0][0] * 1.688
            onground = multi_DREFs[2][0]
            current_vs = multi_DREFs[4][0] / 60.0
            current_vdot = multi_DREFs[5][0] * 1.688
            
            print(current_vdot)
            
            (pitch_setpoint, throttle_setpoint) = total_energy_control(h_sp,V_sp,current_altitude,current_asi,current_vs,current_vdot)
            
            pitch_PID.SetPoint = max(min(pitch_setpoint, 10), -15)
            pitch_PID.update(current_pitch)
            
            new_ele_ctrl = max(min(pitch_PID.output,1),-1)
            new_throttle_ctrl = max(min(throttle_setpoint,1),-1)
            
            ctrl = [new_ele_ctrl, -998, 0.0, new_throttle_ctrl] # ele, ail, rud, thr. -998 means don't change
            client.sendCTRL(ctrl)
            
            print("Elev: " + str(new_ele_ctrl) + " Throttle: " + str(new_throttle_ctrl) + " Pitch setpoint: " + str(pitch_setpoint) + "Throttle setpoint: " + str(throttle_setpoint))
            
            sleep(0.05)