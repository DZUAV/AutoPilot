#ifndef __NOTICE__H__
#define __NOTICE__H__

#include "sys.h"
#include "stdbool.h"
typedef struct 
{
        bool initialising;        // true if initialising and the vehicle should not be moved
        uint8_t gps_status;       // see the GPS_0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint8_t gps_num_sats;     // number of sats
        uint8_t flight_mode;      // flight mode
        bool armed;               // 0 = disarmed, 1 = armed
        bool pre_arm_check;       // true if passing pre arm checks
        bool pre_arm_gps_check;   // true if passing pre arm gps checks
        bool save_trim;           // true if gathering trim data
        bool esc_calibration;     // true if calibrating escs
        bool failsafe_radio;      // true if radio failsafe
        bool failsafe_battery;    // true if battery failsafe
        bool parachute_release;   // true if parachute is being released
        bool ekf_bad;             // true if ekf is reporting problems
        bool autopilot_mode;      // true if vehicle is in an autopilot flight mode (only used by OreoLEDs)
        bool firmware_update;     // true just before vehicle firmware is updated
        bool compass_cal_running; // true if a compass calibration is running
        bool leak_detected;       // true if leak detected
        bool gps_fusion;          // true if the GPS is in use by EKF, usable for flight
        bool gps_glitching;       // true f the GPS is believed to be glitching is affecting navigation accuracy
        bool have_pos_abs;        // true if absolute position is available
        bool vehicle_lost;        // true when lost copter tone is requested (normally only used for copter)
        bool waiting_for_throw;   // true when copter is in THROW mode and waiting to detect the user hand launch
        bool powering_off;        // true when the vehicle is powering off
        bool video_recording;     // true when the vehicle is recording video

		bool imu_flag_icm20602;
		bool imu_flag_icm20689;
		bool imu_flag_mpu6000;
		bool imu_flag_ms5611;
	    bool imu_flag_ist8310;


}NOTICE_flags_and_values_type;


extern NOTICE_flags_and_values_type notice;


#endif
