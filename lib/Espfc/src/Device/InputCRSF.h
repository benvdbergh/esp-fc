#ifndef _INPUT_DEVICE_INPUT_CRSF_H_
#define _INPUT_DEVICE_INPUT_CRSF_H_

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Math/Utils.h"

namespace Espfc {

namespace Device {

struct CrsfData
{
  uint8_t syncByte;
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
  uint8_t flags;
  /**
   * The endByte is 0x00 on FrSky and some futaba RX's, on Some CRSF2 RX's the value indicates the telemetry byte that is sent after every 4th crsf frame.
   * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
   * and https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
   */
  uint8_t endByte;
} __attribute__ ((__packed__));

#define CRSF_FLAG_SIGNAL_LOSS       (1 << 2)
#define CRSF_FLAG_FAILSAFE_ACTIVE   (1 << 3)

#pragma region CRSF Defines

#define CRSF_SIGNAL_OK          0x00
#define CRSF_SIGNAL_LOST        0x01
// Basic setup
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
 // Device address & type, The address means the destination of the data packet, so for decoder, the destination is the FC.
#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
// Baud ELRS receiver baud 420000 bit/s v2
#define SERIAL_BAUDRATE 115200 
#define ADDR_RADIO                     0xEA  //  Radio Transmitter
#define TYPE_SETTINGS_WRITE            0x2D
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8  // Flight Controler

//Define channel input limite
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 991
#define CRSF_CHANNEL_MAX 1811

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz
#define CRSF_PACKET_TIMEOUT_US 100000
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX   60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_FRAME_LENGTH 24  // length of type + payload + crc
#pragma endregion

static uint8_t crsf_crc8tab[256] = {
    	0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    	0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    	0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    	0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    	0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    	0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
   		0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    	0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    	0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    	0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    	0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    	0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    	0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    	0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    	0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    	0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

class InputCRSF: public InputDevice
{
  public:
    enum CrsfState {
      CRSF_START,
      CRSF_DATA,
      CRSF_END
    };

    
    // Custom Check function
    uint8_t inBuffer[CRSF_PACKET_SIZE];
	uint8_t crsfData[CRSF_PACKET_SIZE];
	int16_t channels[CRSF_MAX_CHANNEL];
	uint8_t failsafe_status;
	uint8_t frameLenth;
	int crsf_passthrough;
	int toChannels;

    InputCRSF(): _serial(NULL), _state(CRSF_START), _idx(0), _new_data(false) {}

    int begin(Device::SerialDevice * serial)
    {
      _serial = serial;
      for(size_t i = 0; i < CRSF_FRAME_SIZE; i++)
      {
        _data[i] = 0;
        if(i < CHANNELS) _channels[i] = 0;
      }
      return 1;
    }



    InputStatus update() override
    {
      if(!_serial) return INPUT_IDLE;

      if((*_serial).available() >= (int)CRSF_FRAME_SIZE)
      {
        //(*_serial).swap();
        bufferIndex=0;
        while((*_serial).available())
        {
          parse((*_serial).read());
        }
        //(*_serial).swap();
      }

      if(_new_data)
      {
        _new_data = false;
        if(_flags & CRSF_FLAG_FAILSAFE_ACTIVE) return INPUT_FAILSAFE;
        if(_flags & CRSF_FLAG_SIGNAL_LOSS) return INPUT_LOST;
        return INPUT_RECEIVED;
      }

      return INPUT_IDLE;
    }

    uint16_t get(uint8_t i) const override
    {
      return _channels[i];
    }


    size_t getChannelCount() const override { return CHANNELS; }

    bool needAverage() const override { return false; }

    void print(char c) const
    {
      //Serial.write(c);
    }

  private:
    uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
        u8 crc = 0;
        for (u8 i=0; i < len; i++) {
            crc = crsf_crc8tab[crc ^ *ptr++];
        }
        return crc;
    }

    void parse(int d)
    {
      if (bufferIndex==0){
        if(d == CRSF_ADDRESS_FLIGHT_CONTROLLER){
          inBuffer[bufferIndex++] = d;
          d = (*_serial).read();
          frameLength = d;
          inBuffer[bufferIndex++] = d;
        }else{
          bufferIndex=0;
        }
      }else if(bufferIndex >1 && bufferIndex < frameLength + 1){
          inBuffer[bufferIndex++] = d;
      }else if(bufferIndex == frameLength + 1){
        //calculate received packet crc
        inBuffer[bufferIndex++] = d;
        //uint8_t inCrc=inBuffer[CRSF_FRAME_LENGTH-1];
        uint8_t crc=crsf_crc8(&inBuffer[2],inBuffer[1]-1);
        inBuffer[24]=crc;
        //If crc is correct copy buffer data to crsfData
        if( frameLength==CRSF_FRAME_LENGTH && inBuffer[0]== CRSF_ADDRESS_FLIGHT_CONTROLLER){
          if(crc == inBuffer[25]){
            memcpy(crsfData,inBuffer,CRSF_PACKET_SIZE);
            failsafe_status = CRSF_SIGNAL_OK;
          }else{
            failsafe_status = CRSF_SIGNAL_LOST;
          }
        }
        bufferIndex = 0;
      }
    }

    void apply()
    {
        if(crsfData[1] == 24){
            channels[0]  = ((crsfData[3]|crsfData[4]<< 8) & 0x07FF);
            channels[1]  = ((crsfData[4]>>3|crsfData[5]<<5) & 0x07FF);
            channels[2]  = ((crsfData[5]>>6|crsfData[6]<<2|crsfData[7]<<10) & 0x07FF);
            channels[3]  = ((crsfData[7]>>1|crsfData[8]<<7) & 0x07FF);
            channels[4]  = ((crsfData[8]>>4|crsfData[9]<<4) & 0x07FF);
            channels[5]  = ((crsfData[9]>>7|crsfData[10]<<1|crsfData[11]<<9) & 0x07FF);
            channels[6]  = ((crsfData[11]>>2|crsfData[12]<<6) & 0x07FF);
            channels[7]  = ((crsfData[12]>>5|crsfData[13]<<3) & 0x07FF); // & the other 8 + 2 channels if you need them
            channels[8]  = ((crsfData[14]|crsfData[15]<< 8) & 0x07FF);
            channels[9]  = ((crsfData[15]>>3|crsfData[16]<<5) & 0x07FF);
            channels[10] = ((crsfData[16]>>6|crsfData[17]<<2|crsfData[18]<<10) & 0x07FF);
            channels[11] = ((crsfData[18]>>1|crsfData[19]<<7) & 0x07FF);
            channels[12] = ((crsfData[19]>>4|crsfData[20]<<4) & 0x07FF);
            channels[13] = ((crsfData[20]>>7|crsfData[21]<<1|crsfData[22]<<9) & 0x07FF);
            channels[14] = ((crsfData[22]>>2|crsfData[23]<<6) & 0x07FF);
            channels[15] = ((crsfData[23]>>5|crsfData[24]<<3) & 0x07FF);
        }
        _new_data = true;
    }   

    inline uint16_t convert(int v)
    {
      return Math::clamp(((v * 5) / 8) + 880, 800, 2200);
    }

    const static size_t CRSF_FRAME_SIZE = sizeof(CrsfData);

    Device::SerialDevice * _serial;
    CrsfState _state;
    uint8_t _idx = 0;
    bool _new_data;

    static const size_t CHANNELS = 16;

    uint8_t _data[CRSF_FRAME_SIZE];
    uint16_t _channels[CHANNELS];
    uint8_t _flags;


    // Custom Check function
    uint8_t crc;
    uint8_t byte_in_crsf;
	uint8_t bit_in_crsf;
	uint8_t ch;
	uint8_t bit_in_channel;
	uint8_t bit_in_servo;
	int bufferIndex;
	uint8_t inData;
	int feedState;
	uint32_t currentMicros;
	int frameLength;
};

}

}

#endif
