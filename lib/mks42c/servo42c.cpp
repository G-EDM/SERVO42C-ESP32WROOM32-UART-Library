//###############################################################
//  _______  _        _______    _______  _______  _______           _______     ___  _______  _______ 
// (       )| \    /\(  ____ \  (  ____ \(  ____ \(  ____ )|\     /|(  ___  )   /   )/ ___   )(  ____ \
// | () () ||  \  / /| (    \/  | (    \/| (    \/| (    )|| )   ( || (   ) |  / /) |\/   )  || (    \/
// | || || ||  (_/ / | (_____   | (_____ | (__    | (____)|| |   | || |   | | / (_) (_   /   )| |      
// | |(_)| ||   _ (  (_____  )  (_____  )|  __)   |     __)( (   ) )| |   | |(____   _)_/   / | |      
// | |   | ||  ( \ \       ) |        ) || (      | (\ (    \ \_/ / | |   | |     ) ( /   _/  | |      
// | )   ( ||  /  \ \/\____) |  /\____) || (____/\| ) \ \__  \   /  | (___) |     | |(   (__/\| (____/\
// |/     \||_/    \/\_______)  \_______)(_______/|/   \__/   \_/   (_______)     (_)\_______/(_______/
//                                                                                                    
// Library to control the Makerbase Servo42C driver
//###############################################################

//####################################################################
// 
// Connect the ESP TX (Transmit) Pin to the SERVO42C RX (Receive) Pin
// Connect the ESP RX (Receive) Pin to the SERVO42C TX (Transmit) Pin
// Connect 12v +- to the power input pins on the MKS42C 
//
// With this wiring there is no need to supply 3.3v to the UART 
// terminal on the 42C driver. The ESP and the 12v PSU need to share
// gnd to make this configuration work. If GND is already shared
// it is not needed to connect the 42C UART GND neither, making the wiring
// with the ESP possible with only 4 wires.
//
// In the Menu on the 42C display set the baudrate of the driver to 
// 38400 and set the work mode to CR_UART.
//
// It should now be possible to establish communication between ESP
// and the Servo42C driver
//
//####################################################################
#include "servo42c.h"
#include <iostream>

static const int MAX_POS_TORQUE  = 1200;
static const int MAX_POS_CURRENT = 3000;

// read commands
#define CMD_GET_ENCODER_VALUES             0x30
#define CMD_GET_NUMPULSES_RECEIVED         0x33 
//#define CMD_GET_MOTOR_ANGLE 0x36 // not documented in the manual
#define CMD_GET_SHAFT_ANGLE_ERROR          0x39 
#define CMD_GET_ENABLE_PIN_STATE           0x3A 
#define CMD_RELEASE_SHAFT_LOCK_PROTECTION  0x3D 
#define CMD_GET_SHAFT_LOCK_STATE           0x3E 

// write commands
#define CMD_ENCODER_CALIBRATE              0x80
#define CMD_SET_MOTOR_TYPE                 0x81
#define CMD_SET_WORK_MODE                  0x82
#define CMD_SET_CURRENT                    0x83
#define CMD_SET_SUBDIVISION                0x84
#define CMD_SET_ENABLE_PIN_ACTIVE_MODE     0x85
#define CMD_SET_MOTOR_DIRECTION            0x86
#define CMD_SET_AUTO_SCREEN_OFF            0x87
#define CMD_SET_SHAFT_LOCK_PROTECTION      0x88
#define CMD_SET_SUBDIVISON_INTERPOLATION   0x89
#define CMD_SET_BAUDRATE                   0x8A
#define CMD_SET_SLAVE_ADDRESS              0x8B
#define CMD_SET_RESTORE_DEFAULT            0x3F
#define CMD_SET_ZEROMODE_MODE              0x90
#define CMD_SET_ZEROMODE_ZERO              0x91
#define CMD_SET_ZEROMODE_SPEED             0x92
#define CMD_SET_ZEROMODE_DIR               0x93
#define CMD_SET_ZEROMODE_GOTO_ZERO         0x94
#define CMD_SET_PID_KP_POS                 0xA1
#define CMD_SET_PID_KI_POS                 0xA2
#define CMD_SET_PID_KD_POS                 0xA3
#define CMD_SET_ACCELERATION               0xA4
#define CMD_SET_MAX_TORQUE                 0xA5
#define CMD_SET_ENABLE_STATE               0xF3
#define CMD_SET_RUN_CONTINUOUS             0xF6
#define CMD_SET_STOP_MOTOR                 0xF7
#define CMD_SET_SAVE_CLEAR_CONTINUOUS      0xFF
#define CMD_SET_RUN_BY_STEPNUM             0xFD


void log_to_console(const uint8_t* array, size_t length) {
  for (size_t i = 0; i < length; i++) {
    // Print each byte as a two-digit hex value
    if (array[i] < 0x10) {
      Serial.print('0'); // Leading zero for single digit hex
    }
    Serial.print(array[i], HEX); // Print in hex
    if (i < length - 1) {
      Serial.print(' '); // Separator between bytes
    }
  }
  Serial.println(); // Newline at the end
}

SERVO42C::SERVO42C(){}


bool SERVO42C::init( HardwareSerial &serial ){
    _serial = &serial;
    return true;
}



//#########################################################################
// Return the uint8_t status from return messages that are supposes to
// return a status message. Documentations shows status values 0,1,2
//#########################################################################
uint8_t SERVO42C::extract_status( const uint8_t response[] ) {
    uint8_t status_byte = response[1];
    status_byte &= 0xFF;
    //log_to_console( response, 3 );
    return status_byte;
}

//#########################################################################
// Calculates the checksum of all hex blocks
//#########################################################################
uint8_t SERVO42C::create_checksum( uint8_t *hex_blocks, int block_num ){
    int sum = 0;
    for( int i = 0; i < block_num; ++i ) {
        sum += hex_blocks[i];
    }
    return sum & 0xFF; // Mask to 8 bits 
}

//#########################################################################
// Sends the bytes and waits for the response
// if the response is invalid or timed out it will retry multiple times
// the number of retries is defined with MKS_MAX_SEND_RETRIES and defaults
// to 3. The timeout defaults to 3 seconds and is defined with
// MKS_WAIT_TIMEOUT. Both are found in servo42c.h
// if there is a connection error that will lead to a timeout and it
// retries 3 times this would make 9 seconds of blocking
//#########################################################################
bool SERVO42C::send( uint8_t *hex_block_set, size_t hex_block_size, uint8_t *response, uint8_t receive_length ){
    uint8_t retry     = 0;
    bool    success = false;
    do{
        //log_to_console( hex_block_set, hex_block_size );
        _serial->flush();
        _serial->write( hex_block_set, hex_block_size ); // E0, A5, 00, 01, 0x86
        success = receive( response, receive_length );
    } while ( ( !success ) && ( ++retry < MKS_MAX_SEND_RETRIES ) );
    if( !success ){
        //Serial.println("Error");
    } else {
        //log_to_console( response, receive_length );
    }
    return success;
}

//#########################################################################
// Blocking function that waits for the response
// returns false on error or timeout and true on success
// uses the checksum to validate the received data
// does not check for the correct function code in the response...
// todo: pass function code and ensure the response belongs to the send
// command. Not a big issue for now.
//#########################################################################
bool SERVO42C::receive( uint8_t* response, uint8_t receive_length ){
    unsigned long start_time = millis();
    unsigned long time       = start_time;
    bool          success    = false;
    uint16_t      bytes_received = 0;
    uint8_t       received_byte;
    while(1){
        if( _serial->available() > 0 ){
            received_byte = _serial->read();
            if( bytes_received != 0 || received_byte == slave_address ){
                response[ bytes_received++ ] = received_byte;
            }
            start_time = time; // if something comes in let's get it
        }
        if( bytes_received == receive_length ){
            uint8_t computed_checksum = create_checksum( response, receive_length - 1 );
            uint8_t received_checksum = response[ receive_length - 1 ];
            if (received_checksum == computed_checksum) {
                success = true;
                //Serial.println("Checksum OK");
                break;
           } else {
                bytes_received = 0;
                //Serial.println("Checksum not OK");
            }
        }
        time = millis();
        if( ( time - start_time ) > MKS_WAIT_TIMEOUT ){
            //Serial.println("Timed out");
            break;
        }
    }
    return success;
}








//#########################################################################
// Get the current enable status
//
// UART return:
// status 1 = enabled - status 2 = disabled
//
// Function return:
// true = enabled, false = disabled
//#########################################################################
bool SERVO42C::get_enable_state(){
    uint8_t status = send_raw_cmd_status( CMD_GET_ENABLE_PIN_STATE );
    if( status == 0 ){ 
        // unhandled error
    }
    return status == 1 ? true : false;
}

//#########################################################################
// Release shaft lock protection
//
// UART return:
// status 1 = success - status 0 = fail
//
// Function return:
// true = success, false = fail
//#########################################################################
bool SERVO42C::release_shaft_lock_protection(){
    uint8_t status = send_raw_cmd_status( CMD_RELEASE_SHAFT_LOCK_PROTECTION );
    return status == 1 ? true : false;
}

//#########################################################################
// Read shaft lock protection state
//
// UART return:
// status 1 = protected - status 2 = not protected
//
// Function return:
// true = protected, false = not proteced
//#########################################################################
bool SERVO42C::get_shaft_lock_protection_state(){
    uint8_t status = send_raw_cmd_status( CMD_GET_SHAFT_LOCK_STATE );
    if( status == 0 ){ 
        // unhandled error
    }
    return status == 1 ? true : false;
}

float SERVO42C::get_shaft_angle_error(){
    int16_t value = send_raw_cmd_get_16bit( CMD_GET_SHAFT_ANGLE_ERROR, 4 );
    return (static_cast<float>(value) / 0xFFFF)*360.0f;
}
int32_t SERVO42C::get_pulses_received(){
    int32_t value = send_raw_cmd_get_32bit( CMD_GET_NUMPULSES_RECEIVED, 6 );
    return value;
}
int64_t SERVO42C::get_encoder_value(){
    uint8_t receive_length = 8;
    uint8_t hex_block_set[3] = {0};
    uint8_t response[receive_length];
    hex_block_set[0] = slave_address;
    hex_block_set[1] = CMD_GET_ENCODER_VALUES;
    hex_block_set[2] = create_checksum( hex_block_set, 2 );
    if( send( hex_block_set, 3, response, receive_length ) ){
        // first hex value E0 is the slave address and the last 01 is the checksum
        // hex value response[1] to respones[4] form a int32_t holding the carrier and response[5] response[6] form a int16_t holding the value
        // carrier is the number of total shaft turns done and value the position in the current rotation
        // Parse carrier
        int32_t carrier = 0; // carrier holds the revolutions done by the motor
        carrier |= (static_cast<int32_t>(response[1]) << 24);
        carrier |= (static_cast<int32_t>(response[2]) << 16);
        carrier |= (static_cast<int32_t>(response[3]) << 8);
        carrier |= static_cast<int32_t>(response[4]);
        // Parse value
        uint16_t value = (uint16_t)(((uint16_t)response[5] << 8) | response[6]); // the current position in the actual revolution
        return ((int64_t)carrier * 65536LL) + (int64_t)value;
    } else {
        // error not handled. Just returns 0...
        return 0;
    }
    return 0;
}

//###########################################################
// Sends a raw command and returns a int16_t
// returns 0 on error.... unhandled
//###########################################################
int16_t SERVO42C::send_raw_cmd_get_16bit( uint8_t cmd, uint8_t receive_length ){
    uint8_t hex_block_set[3] = {0};
    uint8_t response[receive_length];
    hex_block_set[0] = slave_address;
    hex_block_set[1] = cmd;
    hex_block_set[2] = create_checksum( hex_block_set, 2 );
    if( send( hex_block_set, 3, response, receive_length ) ){
        // looks good
        return (int16_t)((response[1] << 8) | response[2]); // big endian
    } else {
        // not so good
        return 0;
    }
}

//###########################################################
// Sends a raw command and returns a int32_t
// returns 0 on error.... unhandled
//###########################################################
int32_t SERVO42C::send_raw_cmd_get_32bit( uint8_t cmd, uint8_t receive_length ){
    uint8_t hex_block_set[3] = {0};
    uint8_t response[receive_length];
    hex_block_set[0] = slave_address;
    hex_block_set[1] = cmd;
    hex_block_set[2] = create_checksum( hex_block_set, 2 );
    if( send( hex_block_set, 3, response, receive_length ) ){
        // looks good
        int32_t result = 0;
        result |= ((int32_t)response[1] << 24); // big endian
        result |= ((int32_t)response[2] << 16);
        result |= ((int32_t)response[3] << 8);
        result |= ((int32_t)response[4]);
        return result;
    } else {
        // not so good
        return 0;
    }
}

//#########################################################################
// Run motor by dir, speed, steps
// dir: 0 = forward /CW??, 1 = reverse /CCW?? Docs only say forward/reverse
// speed: 0-127?
// steps: number of steps to move
//
// UART return:
// status 0 = run failed - 1 run starting…. - status 2 = run done
// set_move_steps( 0, 10, 2000, true/false )
//#########################################################################
bool SERVO42C::set_move_steps( uint8_t dir, uint8_t speed, uint32_t steps, bool blocking ){
    if( speed > 127 ){ speed = 127; }
    speed &= 0x7F; // redundant
    uint8_t data = (dir==1 ? 0x80 : 0x00) | speed; // direction and speed is packed into a single byte, first bit is dir, last 7 bits speed, padded with leading zeros if needed
    uint8_t status = send_8bit_32bit_status( CMD_SET_RUN_BY_STEPNUM, data, steps );
    if( status == 0 ){
        //Serial.println("Run failed");
        return false;
    }

    if( blocking && status == 1 ){
        // would be nice to have it blocking until done
        // status 2 does not seem to indicate the stepper
        // arrived at the wanted position but just that it started running
        // so this blocking part is somehow almost useless
        // except for ensuring that the function exits after the stepper started
        // moving... 
        unsigned long start_time = millis();
        unsigned long time       = start_time;
        unsigned long timeout    = steps * 100; // 100ms step delay should be ok for a timeout?
        uint8_t response[MKS_DEFAULT_RECEIVE_LENGTH];
        while( status == 1 ){
            vTaskDelay(10);
            if( receive( response, MKS_DEFAULT_RECEIVE_LENGTH ) ){
                status = extract_status( response );
            }
            time = millis();
            if( ( time - start_time ) > timeout ){
                break;
            }
        }
    }

    if( status == 2 ){
        //Serial.println("Motor started...");
    }


    return true;
}






//##################################################################
// Restore default values
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_restore_defaults(){
    uint8_t status = send_raw_cmd_status( CMD_SET_RESTORE_DEFAULT );
    return status == 1 ? true : false;
}

//##################################################################
// Stop motor
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_stop_motor(){
    //Serial.println("Stopping motor");
    uint8_t status = send_raw_cmd_status( CMD_SET_STOP_MOTOR );
    return status == 1 ? true : false;
}

//##############################################################
// Start calibration
//
// UART return: 
// status 1 = Calibrate success - status 2 = Calibrating failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_calibrate(){
    uint8_t status = send_8bit_status( CMD_ENCODER_CALIBRATE, 0x00 );
    return status == 1 ? true : false;
}

//##############################################################
// Set the motor type
// 0 = 0.9° step angle, 1 = 1.8" step angle (more common)
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_motor_type( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_MOTOR_TYPE, value );
    return status == 1 ? true : false;
}

//##############################################################
// Set the operation mode
// 0 = CR_OPEN, 1 = CR_vFOC, 2 = CR_UART
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_work_mode( uint8_t value ){
    if( value > 2 ){ value = 2; }
    uint8_t status = send_8bit_status( CMD_SET_WORK_MODE, value );
    return status == 1 ? true : false;
}

//##############################################################
// Set the max current in mA
// Current range: 0 - 3000
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_max_current( uint16_t current_ma ){
    if( current_ma > MAX_POS_CURRENT ){ current_ma = MAX_POS_CURRENT; }
    // current is send as integer from 0-15 that is then
    // multiplied by 200 on the MKS
    uint8_t value  = round( current_ma / 200 );
    uint8_t status = send_8bit_status( CMD_SET_CURRENT, value );
    return status == 1 ? true : false;
}

//##############################################################
// Set the subdivision / microsteps
// Range: 0 - 255 
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_subdivision( uint8_t value ){
    if( value > 255 ){ value = 255; }
    uint8_t status = send_8bit_status( CMD_SET_SUBDIVISION, value );
    return status == 1 ? true : false;
}

//##############################################################
// Set enable pin active mode
// 0 = active low, 1 = active high, 2 = always enabled 
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_enable_mode( uint8_t value ){
    if( value > 2 ){ value = 2; }
    uint8_t status = send_8bit_status( CMD_SET_ENABLE_PIN_ACTIVE_MODE, value );
    return status == 1 ? true : false;
}

//##############################################################
// Set motor dir
// 0 = CW, 1 = CCW
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_motor_dir( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_MOTOR_DIRECTION, value );
    return status == 1 ? true : false;
}

//##############################################################
// Set auto screen off (turn the screen off after some time)
// 0 = disabled, 1 = enabled
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_screen_auto_off( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_AUTO_SCREEN_OFF, value );
    return status == 1 ? true : false;
}

//##############################################################
// Enable/disable motor shaft locked-rotor protection
// 0 = disable, 1 = enable
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_shaft_lock_protection( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_SHAFT_LOCK_PROTECTION, value );
    return status == 1 ? true : false;
}

//##############################################################
// Enable/disable subdivision interpolation
// Guess this enables/disables microstepping
// 0 = disable, 1 = enable
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_subdivision_interpolation( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_SUBDIVISON_INTERPOLATION, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set UART Baudrate
// 1 = 9600.
// 2 = 19200.
// 3 = 25000.
// 4 = 38400.
// 5 = 57600.
// 6 = 115200
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_baudrate( uint8_t value ){
    if( value > 6 ){ value = 6; } else if( value < 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_BAUDRATE, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set UART slave address
// Range: 0 - 9
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_slave_address( uint8_t value ){
    if( value > 9 ){ value = 9; }
    slave_address = 0xE0 + value; // set internal address
    uint8_t status = send_8bit_status( CMD_SET_SLAVE_ADDRESS, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set zero mode
// 0 = disabled, 1 = DirMode, 2 = NearMode
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_zero_mode( uint8_t value ){
    if( value > 2 ){ value = 2; }
    uint8_t status = send_8bit_status( CMD_SET_ZEROMODE_MODE, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set zero mode zero position
// sets current position as zero position to return to in zero mode
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_zero_position(){
    uint8_t status = send_8bit_status( CMD_SET_ZEROMODE_ZERO, 0x00 );
    return status == 1 ? true : false;
}

//##################################################################
// Set zero mode speed
// Range: 0 - 4
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_zero_mode_speed( uint8_t value ){
    if( value > 4 ){ value = 4; }
    uint8_t status = send_8bit_status( CMD_SET_ZEROMODE_SPEED, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set zero mode direction
// 0 = CW, 1 = CCW
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_zero_mode_direction( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_ZEROMODE_DIR, value );
    return status == 1 ? true : false;
}

//##################################################################
// Move to zero position
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = error
//##################################################################
bool SERVO42C::set_goto_zero(){
    uint8_t status = send_8bit_status( CMD_SET_ZEROMODE_GOTO_ZERO, 0x00 );
    return status == 1 ? true : false;
}








//##################################################################
// Set PID Kp
// no max value provided in the docs
// default value is 1616
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_pid_kp( uint16_t value ){
    uint8_t status = send_16bit_status( CMD_SET_PID_KP_POS, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set PID Ki
// no max value provided in the docs
// default value is 1
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_pid_ki( uint16_t value ){
    uint8_t status = send_16bit_status( CMD_SET_PID_KI_POS, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set PID Kd
// no max value provided in the docs
// default value is 1616
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_pid_kd( uint16_t value ){
    uint8_t status = send_16bit_status( CMD_SET_PID_KD_POS, value );
    return status == 1 ? true : false;
}

//##################################################################
// Set acceleration ACC
// no max value provided in the docs
// default value is 286
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_acc( uint16_t value ){
    uint8_t status = send_16bit_status( CMD_SET_ACCELERATION, value );
    return status == 1 ? true : false;
}

//##############################################################
// Write functions to set params on the 42C driver
// Max torque range: 0 - 1200
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_max_torque( uint16_t torque ){
    if( torque > MAX_POS_TORQUE ){ torque = MAX_POS_TORQUE; }    
    uint8_t status = send_16bit_status( CMD_SET_MAX_TORQUE, torque );
    return status == 1 ? true : false;
}









//##############################################################
// Set motor enable state
// 0 = disable, 1 = enable
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_enable( uint8_t value ){
    if( value > 1 ){ value = 1; }
    uint8_t status = send_8bit_status( CMD_SET_ENABLE_STATE, value );
    return status == 1 ? true : false;
}

//#########################################################################
// Run continuous
// dir: 0 = forward /CW??, 1 = reverse /CCW?? Docs only say forward/reverse
// speed: 1 - 127
// Vrpm = (Speed × 30000)/(Mstep × 200)(RPM) (1.8 degree motor)
// Vrpm = (Speed × 30000)/(Mstep × 400)(RPM) (0.9 degree motor)
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_run_continuous( uint8_t dir, uint8_t speed ){
    if( speed > 127 ){ speed = 127; }
    speed &= 0x7F;
    uint8_t value = (dir==1 ? 0x80 : 0x00) | speed;
    uint8_t status = send_8bit_status( CMD_SET_RUN_CONTINUOUS, value );
    return status == 1 ? true : false;
}

//#####################################################################
// Save/Clear status
// For continuous mode. Motor will start going after power is turned on
// if the state is saved
// 0 = clear / 0xCA, 1 = save / 0xC8
//
// UART return:
// status 1 = Set success - status 0 = Set failed
//
// Function return:
// true = success, false = failed
//##################################################################
bool SERVO42C::set_save_clear_state( uint8_t value ){
    uint8_t data = 0xC8;
    if( value == 0 ){ data = 0xCA; }
    uint8_t status = send_8bit_status( CMD_SET_SAVE_CLEAR_CONTINUOUS, data );
    return status == 1 ? true : false;
}










void SERVO42C::get_8bit_hexblocks( uint8_t cmd, uint8_t value, uint8_t * hex_block_set ){
    hex_block_set[0] = slave_address;
    hex_block_set[1] = cmd;
    hex_block_set[2] = value & 0xFF;
    hex_block_set[3] = create_checksum( hex_block_set, 3 );
}

void SERVO42C::get_16bit_hexblocks( uint8_t cmd, uint16_t value, uint8_t * hex_block_set ){
    hex_block_set[0] = slave_address;
    hex_block_set[1] = cmd;
    hex_block_set[2] = (value >> 8) & 0xFF;
    hex_block_set[3] = value & 0xFF;
    hex_block_set[4] = create_checksum( hex_block_set, 4 );
}

//###########################################################
// Sends a command without data and returns a uint_8 (status)
//###########################################################
uint8_t SERVO42C::send_raw_cmd_status( uint8_t cmd, uint8_t receive_length ){
    uint8_t hex_block_set[3] = {0};
    uint8_t response[receive_length];
    hex_block_set[0] = slave_address;
    hex_block_set[1] = cmd;
    hex_block_set[2] = create_checksum( hex_block_set, 2 );
    if( send( hex_block_set, 3, response, receive_length ) ){
        // looks good
        return extract_status( response );
    } else {
        // not so good
        return 0;
    }
}

//###########################################################
// Sends an 8bit value and returns a uint_8 (status)
//###########################################################
uint8_t SERVO42C::send_8bit_status( uint8_t cmd, uint8_t value, uint8_t receive_length ){
    uint8_t hex_block_set[4] = {0};
    uint8_t response[receive_length];
    get_8bit_hexblocks( cmd, value, hex_block_set );
    if( send( hex_block_set, 4, response, receive_length ) ){
        // looks good
        return extract_status( response );
    } else {
        // not so good
        return 0;
    }
}

//###########################################################
// Sends a 16bit value and returns a uint_8 (status)
//###########################################################
uint8_t SERVO42C::send_16bit_status( uint8_t cmd, uint16_t value, uint8_t receive_length ){
    uint8_t hex_block_set[5] = {0};
    uint8_t response[receive_length];
    get_16bit_hexblocks( cmd, value, hex_block_set );
    if( send( hex_block_set, 5, response, receive_length ) ){
        // looks good
        return extract_status( response );
    } else {
        // not so good
        return 0;
    }
}

uint8_t SERVO42C::send_8bit_32bit_status( uint8_t cmd, uint8_t value_a, uint32_t value_b, uint8_t receive_length ){
    uint8_t hex_block_set[8] = {0};
    uint8_t response[receive_length];
    hex_block_set[0] = slave_address;
    hex_block_set[1] = cmd;
    hex_block_set[2] = value_a & 0xFF;
    hex_block_set[3] = (value_b >> 24) & 0xFF;
    hex_block_set[4] = (value_b >> 16) & 0xFF;
    hex_block_set[5] = (value_b >> 8) & 0xFF;
    hex_block_set[6] = value_b & 0xFF;
    hex_block_set[7] = create_checksum( hex_block_set, 7 );
    //log_to_console( hex_block_set, sizeof( hex_block_set) / sizeof( hex_block_set[0] ) + 1 );
    if( send( hex_block_set, 8, response, receive_length ) ){
        // looks good
        return extract_status( response );
    } else {
        // not so good
        return 0;
    }
}










