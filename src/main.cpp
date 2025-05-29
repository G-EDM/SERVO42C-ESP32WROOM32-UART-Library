//#############################################################################################################
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
//
// The platform.ini file is configured for the
// AZdelivery ESP32 Wroom32 NodeMCU
// For a different board please adjust the platform.ini file as needed
//#############################################################################################################



//#############################################################################################################
// Main includes
//#############################################################################################################
#include <Arduino.h>

#include "main.h"
#include "servo42c.h"

//#include <SoftwareSerial.h>

SERVO42C *servo_stepper;

HardwareSerial mks_serial(2);


void setup() {

  Serial.begin(9600,SERIAL_8N1,RX,TX);
  mks_serial.begin(MKS42C_BAUDRATE,SERIAL_8N1,RX_PIN,TX_PIN);
  delay(2000);

  // create and configure the 42C servo stepper
  servo_stepper = new SERVO42C();
  servo_stepper->init( mks_serial );
  servo_stepper->set_slave_address( MKS42C_ADDRESS_DEFAULT );  // set the drivers slave address (this needs to be the same as set on the stepper driver itself)
  servo_stepper->set_max_current( MKS42C_MAXCURRENT_DEFAULT ); // set max current
  servo_stepper->set_max_torque( MKS42C_MAXTORQUE_DEFAULT );   // set max torque
  servo_stepper->set_enable_mode( MKS42C_ENABLEMODE_DEFAULT ); // set enable mode to active low (enable pin low = motor enabled )
  servo_stepper->set_subdivision( MKS42C_MICROSTEPS_DEFAULT ); // set microsteps
  servo_stepper->set_subdivision_interpolation( MKS42C_ENABLEMICROSTEPS_DEFAULT ); // enable microstepping I guess
  vTaskDelay(50);
  //servo_stepper->set_move_steps( 0, 80, 6000 ); // dir, speed, steps
  //vTaskDelay(1000); // let it run a little
  servo_stepper->set_stop_motor(); // enforce motor to stop
}

void loop(){ 

  Serial.println("");
  float aerr = servo_stepper->get_shaft_angle_error();
  int   prec = servo_stepper->get_pulses_received();
  int   encv = servo_stepper->get_encoder_value();
  Serial.print( "  Shaft error: " );
  Serial.print(aerr);
  Serial.print( "  Pulses: " );
  Serial.print(prec);
  Serial.print( "  Encoder: " );
  Serial.print(encv);
  Serial.print("  ");
  vTaskDelay(50);

}

