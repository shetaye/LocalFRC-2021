#ifndef _DS_PROTOCOL_H_
#define _DS_PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>

#define PACKET_START_CHAR '!'
#define CHECKSUM_LENGTH 2
#define TERMINATOR_LENGTH 2

//
// DS -> Robot Packets
//

// DS Control Packet - !c[control word][switch word][checksum][cr][lf]
#define MSGID_DS_CONTROL 'c'
#define DS_CONTROL_LENGTH               10
                                          // !c
#define DS_CONTROL_CONTROL_WORD_INDEX   2 // [control word]
#define DS_CONTROL_SWITCH_WORD_INDEX    4 // [switch word]
#define DS_CONTROL_CHECKSUM_INDEX       6 // [checksum]
#define DS_CONTROL_CHECKSUM_TERMINATOR  8 // [cr][lf]


// Joystick 1 Packet - !j[button word][leftXAxis][leftYAxis][rightXAxis][rightYAxis][triggerLeftAxis][triggerRightAxis][checksum][cr][lf]
// Joystick 2 Packet - !J[button word][leftXAxis][leftYAxis][rightXAxis][rightYAxis][triggerLeftAxis][triggerRightAxis][checksum][cr][lf]
#define MSGID_DS_JOYSTICK_1 'j'
#define MSGID_DS_JOYSTICK_2 'J'
#define DS_JOYSTICK_LENGTH                22  
#define DS_JOYSTICK_BUTTON_WORD_INDEX     2
#define DS_JOYSTICK_LEFT_X_AXIS_INDEX     6
#define DS_JOYSTICK_LEFT_Y_AXIS_INDEX     8
#define DS_JOYSTICK_RIGHT_X_AXIS_INDEX    10
#define DS_JOYSTICK_RIGHT_Y_AXIS_INDEX    12
#define DS_JOYSTICK_TRIGGER_X_AXIS_INDEX  14
#define DS_JOYSTICK_TRIGGER_Y_AXIS_INDEX  16
#define DS_JOYSTICK_CHECKSUM_INDEX        18
#define DS_JOYSTICK_CHECKSUM_TERMINATOR   20

// Whole 
#define MSDG

//
// Robot -> DS Packets
//

// Robot Status Packet - !S[status word][protocol version][cr][lf]
#define MSGID_ROBOT_STATUS 'S'
#define ROBOT_STATUS_LENGTH                 10
#define ROBOT_STATUS_SATATUS_WORD_INDEX     2
#define ROBOT_STATUS_PROTOCOL_VERSION_INDEX 4
#define ROBOT_STATUS_CHECKSUM_INDEX         6
#define ROBOT_STATUS_CHECKSUM_TERMINATOR    8

// Log Message Packet - Any line that does not start with !S

#define MIN_UART_MESSAGE_LENGTH DS_CONTROL_LENGTH

class DSProtocol
{

public:
  static int decodeDSControlPacket(char *buffer, int length, bool& estopped, bool& enabled, uint8_t& mode, uint8_t& switchState) {
    // https://github.com/kauailabs/navxmxp/blob/master/stm32/navx-mxp/AHRSProtocol.h#L366-L379
    // DO NOT MODIFY The passed in args, unless this is a real packet

    if (length < DS_CONTROL_LENGTH) return 0;
    if (buffer[0] == PACKET_START_CHAR  && buffer[1] == MSGID_DS_CONTROL) {
      if(!verifyChecksum(buffer, DS_CONTROL_CHECKSUM_INDEX)) return 0;

      // Data
      uint8_t controlWord = decodeUint8(&buffer[DS_CONTROL_CONTROL_WORD_INDEX]);
      switchState = decodeUint8(&buffer[DS_CONTROL_SWITCH_WORD_INDEX]);

      mode      = controlWord & 0x0f;         // Bits 0->4
      enabled   = (controlWord & (1<< 4)) > 0;  // Bit 5
      estopped  = (controlWord & (1<< 5)) > 0;  // Bit 6

      // printf("control word: %02x\n", controlWord);
      // printf("switch state: %02x\n", switchState);

      // printf("mode:     %d\n", mode);
      // printf("enabled:  %d\n", enabled);
      // printf("estopped: %d\n", estopped);

      return DS_CONTROL_LENGTH;
    }

     return 0;
  }

  static int decodeJoystick1Packet(char *buffer, int length, uint16_t& buttonWord, int8_t axis[]) {
    // Joystick 1 Packet - !j[button word][leftXAxis][leftYAxis][rightXAxis][rightYAxis][triggerLeftAxis][triggerRightAxis][checksum][cr][lf]

    if (length < DS_JOYSTICK_LENGTH) return 0;
    if (buffer[0] == PACKET_START_CHAR  && buffer[1] == MSGID_DS_JOYSTICK_1) {
      if(!verifyChecksum(buffer, DS_JOYSTICK_CHECKSUM_INDEX)) {
        printf("Failed checksum\n");
        return 0;
      }

      // Data
      buttonWord = decodeUint16(&buffer[DS_JOYSTICK_BUTTON_WORD_INDEX]);
      axis[0] = decodeUint8(&buffer[DS_JOYSTICK_LEFT_X_AXIS_INDEX]);
      axis[1] = decodeUint8(&buffer[DS_JOYSTICK_LEFT_Y_AXIS_INDEX]);
      axis[2] = decodeUint8(&buffer[DS_JOYSTICK_RIGHT_X_AXIS_INDEX]);
      axis[3] = decodeUint8(&buffer[DS_JOYSTICK_RIGHT_Y_AXIS_INDEX]);
      axis[4] = decodeUint8(&buffer[DS_JOYSTICK_TRIGGER_X_AXIS_INDEX]);
      axis[5] = decodeUint8(&buffer[DS_JOYSTICK_TRIGGER_Y_AXIS_INDEX]);

      return DS_JOYSTICK_LENGTH;
    }

     return 0;
  }

  static int decodeJoystick2Packet(char *buffer, int length, uint16_t& buttonWord, int8_t axis[]) {
    // Joystick 2 Packet - !J[button word][leftXAxis][leftYAxis][rightXAxis][rightYAxis][triggerLeftAxis][triggerRightAxis][checksum][cr][lf]
    if (length < DS_JOYSTICK_LENGTH) return 0;
    if (buffer[0] == PACKET_START_CHAR  && buffer[1] == MSGID_DS_JOYSTICK_2) {
      if(!verifyChecksum(buffer, DS_JOYSTICK_CHECKSUM_INDEX)) return 0;

      // Data
      buttonWord = decodeUint16(&buffer[DS_JOYSTICK_BUTTON_WORD_INDEX]);
      axis[0] = decodeUint8(&buffer[DS_JOYSTICK_LEFT_X_AXIS_INDEX]);
      axis[1] = decodeUint8(&buffer[DS_JOYSTICK_LEFT_Y_AXIS_INDEX]);
      axis[2] = decodeUint8(&buffer[DS_JOYSTICK_RIGHT_X_AXIS_INDEX]);
      axis[3] = decodeUint8(&buffer[DS_JOYSTICK_RIGHT_Y_AXIS_INDEX]);
      axis[4] = decodeUint8(&buffer[DS_JOYSTICK_TRIGGER_X_AXIS_INDEX]);
      axis[5] = decodeUint8(&buffer[DS_JOYSTICK_TRIGGER_Y_AXIS_INDEX]);

      return DS_JOYSTICK_LENGTH;
    }

     return 0;
  }



  static int encodeRobotStatus(char *protocol_buffer, bool estopped, bool enabled, uint8_t mode, uint8_t protocol_version) {
    // Header
    protocol_buffer[0] = PACKET_START_CHAR;
    protocol_buffer[1] = MSGID_ROBOT_STATUS;

    // Data
    uint8_t controlWord = 0;
    controlWord |= (mode & 0x03);    // Bits 0,1
    controlWord |= (enabled << 2);   // Bit 2
    controlWord |= (estopped << 3);  // Bit 3
    
    encodeUint8(controlWord,      &protocol_buffer[ROBOT_STATUS_SATATUS_WORD_INDEX]);
    encodeUint8(protocol_version, &protocol_buffer[ROBOT_STATUS_PROTOCOL_VERSION_INDEX]);

    // Footer
    encodeTermination(protocol_buffer, ROBOT_STATUS_LENGTH, ROBOT_STATUS_LENGTH - 4);
    return ROBOT_STATUS_LENGTH;

  }
protected:

  static void encodeTermination( char *buffer, int total_length, int content_length ) {
    if ( ( total_length >= (CHECKSUM_LENGTH + TERMINATOR_LENGTH) ) && ( total_length >= content_length + (CHECKSUM_LENGTH + TERMINATOR_LENGTH) ) )
    {
      // Checksum 
      unsigned char checksum = 0;
      for ( int i = 0; i < content_length; i++ )
      {
        checksum += buffer[i];
      }
      // convert checksum to two ascii bytes
      sprintf(&buffer[content_length], "%02X", checksum);
      // Message Terminator
      sprintf(&buffer[content_length + CHECKSUM_LENGTH], "%s","\r\n");
    }
  }
  
  static bool verifyChecksum( char *buffer, int content_length ) {
      // Calculate Checksum
      unsigned char checksum = 0;
      for ( int i = 0; i < content_length; i++ )
      {
        checksum += buffer[i];
      }

      // printf("content length %d\n", content_length);
      // printf("expected checksum %x\n", checksum);
  
      // Decode Checksum
      unsigned char decoded_checksum = decodeUint8( &buffer[content_length] );
      
      return ( checksum == decoded_checksum );
  }


  //
  // Primative encode/decode functions
  //
  
  static void encodeUint16( uint16_t value, char* buff ) {
    sprintf(&buff[0],"%04X", value );
  }

  static uint16_t decodeUint16( char *uint16_string ) {
    uint16_t decoded_uint16 = 0;
    unsigned int shift_left = 12;
    for ( int i = 0; i < 4; i++ ) 
    {
      unsigned char digit = uint16_string[i] <= '9' ? uint16_string[i] - '0' : ((uint16_string[i] - 'A') + 10);
      decoded_uint16 += (((uint16_t)digit) << shift_left);
      shift_left -= 4;
    }
    return decoded_uint16;  
  }  

  static void encodeUint8( uint8_t value, char* buff ) {
    sprintf(&buff[0],"%02X", value );
  }
  
  static unsigned char decodeUint8( char *checksum ) {
    unsigned char first_digit = checksum[0] <= '9' ? checksum[0] - '0' : ((checksum[0] - 'A') + 10);
    unsigned char second_digit = checksum[1] <= '9' ? checksum[1] - '0' : ((checksum[1] - 'A') + 10);
    unsigned char decoded_checksum = (first_digit * 16) + second_digit;
    return decoded_checksum;  
  }  

};


#endif // _DS_PROTOCOL_H_
