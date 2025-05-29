#pragma once

#ifndef SERVO42C_MKS
#define SERVO42C_MKS

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

#include "stdint.h"
#include <string>
#include <map>
#include <HardwareSerial.h>

static const uint8_t  MKS_MAX_SEND_RETRIES       = 3;
static const uint32_t MKS_WAIT_TIMEOUT           = 3000;
static const uint32_t MKS_DEFAULT_RECEIVE_LENGTH = 3;

class SERVO42C {

    protected:
        uint16_t microsteps;


    private:

        HardwareSerial *_serial;
        bool locked;
        int slave_address;

        // could make those methods static..
        static uint8_t create_checksum( uint8_t *hex_blocks, int block_num );
        static uint8_t extract_status( const uint8_t response[] );

        void    get_8bit_hexblocks( uint8_t cmd, uint8_t value, uint8_t * hex_block_set );
        void    get_16bit_hexblocks( uint8_t cmd, uint16_t value, uint8_t * hex_block_set);

        uint8_t send_8bit_status( uint8_t cmd, uint8_t value, uint8_t receive_length = MKS_DEFAULT_RECEIVE_LENGTH );
        uint8_t send_16bit_status( uint8_t cmd, uint16_t value, uint8_t receive_length = MKS_DEFAULT_RECEIVE_LENGTH );
        uint8_t send_8bit_32bit_status( uint8_t cmd, uint8_t value_a, uint32_t value_b, uint8_t receive_length = MKS_DEFAULT_RECEIVE_LENGTH );
        uint8_t send_raw_cmd_status( uint8_t cmd, uint8_t receive_length = MKS_DEFAULT_RECEIVE_LENGTH );
        int16_t send_raw_cmd_get_16bit( uint8_t cmd, uint8_t receive_length );
        int32_t send_raw_cmd_get_32bit( uint8_t cmd, uint8_t receive_length );

        bool    send( uint8_t *hex_block_set, size_t hex_block_size, uint8_t *response, uint8_t receive_length = MKS_DEFAULT_RECEIVE_LENGTH );
        bool    receive( uint8_t* response, uint8_t receive_length = MKS_DEFAULT_RECEIVE_LENGTH );
        
    public:
        SERVO42C();
        ~SERVO42C();
        bool    init( HardwareSerial &serial  );
        bool    set_calibrate( void );
        bool    set_motor_type( uint8_t motor_type = 1 );
        bool    set_work_mode( uint8_t mode = 1 );
        bool    set_max_current( uint16_t current = 1200 );
        bool    set_subdivision( uint8_t subdivision = 32 );
        bool    set_enable_mode( uint8_t mode = 0 );
        bool    set_motor_dir( uint8_t dir = 0 );
        bool    set_screen_auto_off( uint8_t value = 0 );
        bool    set_shaft_lock_protection( uint8_t value = 0 );
        bool    set_subdivision_interpolation( uint8_t value = 1 );
        bool    set_baudrate( uint8_t value = 3 );
        bool    set_slave_address( uint8_t address_num = 0 );
        bool    set_restore_defaults( void );
        bool    set_zero_mode( uint8_t value = 0 );
        bool    set_zero_position( void );
        bool    set_zero_mode_speed( uint8_t value = 1 );
        bool    set_zero_mode_direction( uint8_t value = 0 );
        bool    set_goto_zero( void );
        bool    set_pid_kp( uint16_t value = 1616 );
        bool    set_pid_ki( uint16_t value = 1 );
        bool    set_pid_kd( uint16_t value = 1616 );
        bool    set_acc( uint16_t value = 286 );
        bool    set_max_torque( uint16_t torque = 0 );
        bool    set_enable( uint8_t value = 1 );
        bool    set_run_continuous( uint8_t dir, uint8_t speed );
        bool    set_stop_motor( void );
        bool    set_save_clear_state( uint8_t value );
        bool    set_move_steps( uint8_t dir, uint8_t speed, uint32_t steps, bool blocking = true );
        bool    get_enable_state( void );
        bool    release_shaft_lock_protection( void );
        bool    get_shaft_lock_protection_state( void );
        int64_t get_encoder_value( void );
        int32_t get_pulses_received( void );
        float   get_shaft_angle_error( void );

};




#endif