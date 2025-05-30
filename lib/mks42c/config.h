#pragma once

#ifndef WIRE_CONTROLLER_CONFIG
#define WIRE_CONTROLLER_CONFIG

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
//#define RX_PIN 20
//#define TX_PIN 21


#define RX_PIN 16
#define TX_PIN 0

#define MKS42C_MICROSTEPS_DEFAULT       128
#define MKS42C_ENABLEMICROSTEPS_DEFAULT 1
#define MKS42C_MAXCURRENT_DEFAULT       800 //mA in 200 step from 0,200,400,600.....
#define MKS42C_MAXTORQUE_DEFAULT        40  // no idea about the unit. Max is 1200
#define MKS42C_ADDRESS_DEFAULT          0   // default device slave address (0-9)
#define MKS42C_ENABLEMODE_DEFAULT       0   // active low enable pin

#define MKS42C_BAUDRATE 38400

#endif
