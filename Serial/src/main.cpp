#include <Arduino.h>
#include "SX127x.h"
#include "pins.h"

//#define _DEBUG_

#define FW_VERSION                                  "0.0.1"
#define SK_NAME                                     "lrrl"

#define SK_RESET                                0x00
#define SK_GET_VERSION                          0x01
#define SK_GET_NAME                             0x02
#define SK_GET_PIN                              0x10
#define SK_SET_PIN                              0x11
#define SK_GET_PINS                             0x14
#define DEVICE_READ                             0x80
#define DEVICE_WRITE                            0x81
#define DEVICE_INIT                             0x88
#define DEVICE_RESET                            0x89
#define SK_CMD_NONE                             0xFF

int RC = 0, oldRC=0;
int lastAddrRead=0;
uint8_t lastReq;
uint8_t requestBuffer[64];

typedef struct sHidCommand
{
    uint8_t Cmd;
    uint8_t CmdOpt;
    uint8_t CmdDataSize;
    uint8_t *CmdData;
} tHidCommand;


typedef enum
{
    SX_OK,
    SX_ERROR,
    SX_BUSY,
    SX_EMPTY,
    SX_DONE,
    SX_TIMEOUT,
    SX_UNSUPPORTED,
    SX_WAIT,
    SX_CLOSE,
    SX_YES,
    SX_NO,          
} tReturnCodes;


void DecodeCommand( uint8_t *hidReport, tHidCommand *cmd )
{
    cmd->Cmd = hidReport[0];
    cmd->CmdOpt = hidReport[1];
    cmd->CmdDataSize = hidReport[2];
    cmd->CmdData = hidReport + 3;
}

void EncodeCommandAns( uint8_t cmd, uint8_t stat, uint8_t dataSize, uint8_t *data )
{
    uint8_t buffer[64];
    buffer[0] =  cmd;
    buffer[1] =  stat;

    // TimeStamp
    memset( buffer + 2, 0, 62 );
    

    buffer[10] =  dataSize;
    memcpy( buffer + 11, ( const void* )data, dataSize );
    
    //send(buffer);
    //if (dataSize!=0){
        Serial.write(buffer, 64);
        Serial.flush();
    //}
}

void CmdProcess(void)
{
    uint8_t stat = SX_OK;
    uint8_t size = 0;
    uint8_t dataBuffer[64];
    tHidCommand cmd = { SK_CMD_NONE, 0, 0, NULL };
    DecodeCommand(requestBuffer, &cmd);
    
    switch (cmd.Cmd) {
        case DEVICE_RESET:
            SX127x_reset();
            break;
        case SK_RESET:
        case DEVICE_INIT:
            ///radio.hw_reset();
            SX127x_reset();
            ///radio.init();   //SX1272Init( );
            // Set FSK modem ON
            ///radio.set_opmode(RF_OPMODE_SLEEP);  
            ///radio.RegOpMode.bits.LongRangeMode = 0;
            ///radio.write_reg(REG_OPMODE, radio.RegOpMode.octet);            
            //radio.SetLoRaOn( false ); //SX1272SetLoRaOn( false ); // Default radio setting
            // Default answer settings
            break;
        case SK_GET_VERSION:
            strcpy( ( char* )dataBuffer, FW_VERSION );
            size = strlen( FW_VERSION );
            break;
        case DEVICE_READ:
            // cmd.CmdData[0] = size
            // cmd.CmdData[1] = address
            size = cmd.CmdData[0];
            lastAddrRead = cmd.CmdData[1];
            SX127x_read_buffer( cmd.CmdData[1], dataBuffer, size );
            stat = SX_OK;
            break;
        case SK_GET_PINS:
            dataBuffer[0] = 0;
            if (digitalRead(SX127x_IRQ)==HIGH)
                dataBuffer[0] |= 0x01;
///            if (radio.dio1)
///                dataBuffer[0] |= 0x02;
            /*dataBuffer[0] |= DIO1 << 1;
            dataBuffer[0] |= DIO2 << 2;
            dataBuffer[0] |= DIO3 << 3;
            dataBuffer[0] |= DIO4 << 4;
            dataBuffer[0] |= DIO5 << 5;*/
            size = 1;
            break;
        case SK_GET_PIN:
            // cmd.CmdData[0] = Pin id
            switch( cmd.CmdData[0] ) {
                case 11:    // FEM_CPS_PIN
                    // not existing on shield board -- dataBuffer[0] = radio.femcps;
                    break;
                case 12:    // FEM_CTX_PIN
                    // not existing on shield board --  dataBuffer[0] = radio.femctx;
                    break;
                default:
                    dataBuffer[0] = 0xFF; // Signal ID error
                    break;
            } // ...switch( cmd.CmdData[0] )
            break;
        case SK_SET_PIN:
            // cmd.CmdData[0] = Pin id
            // cmd.CmdData[1] = Pin state
            switch( cmd.CmdData[0] ) {
                case 6:
                case 7:
                case 8:
                    // ignore LEDs
                    break;
                case 11:    // FEM_CPS_PIN
                    // not existing on shield board -- radio.femcps = cmd.CmdData[1];
                    break;                    
                case 12:    // FEM_CTX_PIN
                    // not existing on shield board -- radio.femctx = cmd.CmdData[1];
                    break;
                default:
                    stat = SX_UNSUPPORTED;
                    break;
            } // ...switch( cmd.CmdData[0] )
            
            break;
        case DEVICE_WRITE:
            // cmd.CmdData[0] = size
            // cmd.CmdData[1] = address
            // cmd.CmdData[2] = Buffer first byte
            // cmd.CmdData[2+(size-1)] = Buffer last byte
            ///radio.WriteBuffer( cmd.CmdData[1], cmd.CmdData + 2, cmd.CmdData[0] );
            SX127x_write_buffer( cmd.CmdData[1], cmd.CmdData + 2, cmd.CmdData[0] );
                //cmd.CmdData[1], dataBuffer, size );
            stat = SX_OK;
            break;
        case SK_GET_NAME:
            strcpy( ( char* )dataBuffer, SK_NAME );
            size = strlen( SK_NAME );
            break;            
        default:
            ///pc.printf("%d: ", recv_report.length);
            ///for(int i = 0; i < recv_report.length; i++) {
           ///     pc.printf("%02x ", recv_report.data[i]);
            ///}
            ///pc.printf("\r\n");
            stat = SX_UNSUPPORTED;
        break;
    } // ...switch (cmd.Cmd)
    
    EncodeCommandAns( cmd.Cmd, stat, size, dataBuffer);
}

#define LED 15 

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);    
  pinMode(SX127x_IRQ, INPUT);    
  //Serial.setTimeout(10);
  //delay(3000);
  //Serial.print("Cpu freq:");
  //Serial.println(getCpuFrequencyMhz());
  //Serial.setDebugOutput(true);
  SX127x_begin();
}

void loop() {
  if (Serial.available()>0){
    digitalWrite(LED, digitalRead(LED)^1);
    Serial.readBytes(requestBuffer,64);
    CmdProcess();
    digitalWrite(LED, digitalRead(LED)^1);
  }

}