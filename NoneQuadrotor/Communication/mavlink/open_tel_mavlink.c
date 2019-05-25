#include "open_tel_mavlink.h"//作者：恒久力行  qq:624668529

#include "define.h"
#include "stdint.h"
////Add By BigW
typedef uint8_t bool;
//typedef struct {
//    char c;
//} prog_char_t;
//	
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
mavlink_channel_t           chan;
//uint16_t                    packet_drops;
mavlink_heartbeat_t         heartbeat;
mavlink_attitude_t          attitude;
mavlink_global_position_int_t position;
//mavlink_ahrs_t              ahrs;
uint8_t buf[100]={12,13,1,2,3,18,10,0};
////End Add By BigW
//// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;
// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false
void handleMessage(mavlink_message_t* msg);
/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */
static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }
    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif
    // we are armed if we are not initialising
    if (0){//motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}
static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    mavlink_msg_attitude_send(
        chan,
	      buf[1],//millis(),
        buf[2],//ahrs.roll,
        buf[3],//ahrs.pitch,
        buf[4],//ahrs.yaw,
        buf[5],//omega.x,
        buf[6],//omega.y,
        buf[7]);//omega.z);
//        ++buf[1],//millis(),
//        ++buf[2],//ahrs.roll,
//        ++buf[3],//ahrs.pitch,
//        ++buf[4],//ahrs.yaw,
//        ++buf[5],//omega.x,
//        ++buf[6],//omega.y,
//        ++buf[7]);//omega.z);
}
 
static void NOINLINE send_location(mavlink_channel_t chan)
{
    //Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
    mavlink_msg_global_position_int_send(
        chan,
        1,//millis(),
        2,//current_loc.lat,                // in 1E7 degrees
        3,//current_loc.lng,                // in 1E7 degrees
        4,//g_gps->altitude * 10,             // millimeters above sea level
        5,//(current_loc.alt - home.alt) * 10,           // millimeters above ground
        6,//g_gps->ground_speed * rot.a.x,  // X speed cm/s
        7,//g_gps->ground_speed * rot.b.x,  // Y speed cm/s
        8,//g_gps->ground_speed * rot.c.x,
        9);//g_gps->ground_course);          // course in 1/100 degree
}
//static void NOINLINE send_ahrs(mavlink_channel_t chan)
//{
//    //Vector3f omega_I = ahrs.get_gyro_drift();
//    mavlink_msg_ahrs_send(
//        chan,
//        ++buf[8],//omega_I.x,
//        ++buf[9],//omega_I.y,
//        ++buf[10],//omega_I.z,
//        1,
//        0,
//        ++buf[11],//ahrs.get_error_rp(),
//        ++buf[12]);//ahrs.get_error_yaw());
//}
static void NOINLINE send_statustext(mavlink_channel_t chan)
{
}
// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    return false;
}
// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = serial_free();
    if (telemetry_delayed(chan)) {
        return false;
    }
    switch(id) {
      case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        break;
      case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;
      case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;
		
//      case MSG_AHRS:
//        CHECK_PAYLOAD_SIZE(AHRS);
//        send_ahrs(chan);
//        break;
      case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;
		  default:
			  break;
    }
    return true;
}



#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];
// send a message using mavlink
void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];
    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }
    if (id == MSG_RETRY_DEFERRED) {
        return;
    }
    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }
    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}
void mavlink_send_text(mavlink_channel_t chan, enum gcs_severity severity, char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }
    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        mav_array_memcpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(
            chan,
            severity,
            str);
    }
}
void update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;
    // process received bytes
    while(serial_available())
    {
        uint8_t c = serial_read_ch(); 
        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            mavlink_active = true;
            
					  //printf("%c",c);
					  printf("Received message with ID %d, sequence: %d from component %d of system %d",\
										msg.msgid, msg.seq, msg.compid, msg.sysid);
					  handleMessage(&msg);
        }
    }
}
void handleMessage(mavlink_message_t* msg)
{
    //struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
					  mavlink_msg_heartbeat_decode(msg, &heartbeat);  					
            break;
        }
				
				case MAVLINK_MSG_ID_ATTITUDE: {
					  mavlink_msg_attitude_decode(msg, &attitude);
					  break;
				}
				
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
					  mavlink_msg_global_position_int_decode(msg, &position);
					  break;
				}
//				
//				case MAVLINK_MSG_ID_AHRS: {
//					  mavlink_msg_ahrs_decode(msg, &ahrs); 
//					  break;
//				}
				
				default:
					  break;
    }     // end switch
		
} // end handle mavlink

