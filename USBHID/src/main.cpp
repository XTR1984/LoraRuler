#if ARDUINO_USB_MODE
#warning This sketch should be used when USB is in OTG mode
void setup(){}
void loop(){}
#else
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "HIDTypes.h"
#include "SX127x.h"
USBHID HID;

//#define _DEBUG_

#define FW_VERSION                                  "3.1.0"
#define SK_NAME                                     "mbed"

#define HID_SK_RESET                                0x00
#define HID_SK_GET_VERSION                          0x01
#define HID_SK_GET_NAME                             0x02
#define HID_SK_GET_PIN                              0x10
#define HID_SK_SET_PIN                              0x11
#define HID_SK_GET_PINS                             0x14
#define HID_DEVICE_READ                             0x80
#define HID_DEVICE_WRITE                            0x81
#define HID_DEVICE_INIT                             0x88
#define HID_DEVICE_RESET                            0x89
#define HID_SK_CMD_NONE                             0xFF

int RC = 0, oldRC=0;
int lastAddrRead=0;
uint8_t lastReq;
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



//TUD_HID_REPORT_DESC_GENERIC_INOUT
static const uint8_t report_descriptor[] = {
     // report descriptor for general input/output
        0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
        0x09, 0x01,        // Usage (0x01)
        0xA1, 0x01,        // Collection (Application)
        0x09, 0x02,        //   Usage (0x02)
        0x15, 0x00,        //   Logical Minimum (0)
        0x25, 0xFF,        //   Logical Maximum (255)
        0x75, 0x08,        //   Report Size (8)
        0x95, 0x40,        //   Report Count (64)
        0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x09, 0x03,        //   Usage (0x03)
        0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0xC0               // End Collection
};




class CustomHIDDevice: public USBHIDDevice {
public:
  bool flag;
  bool verbose=false;
  char request[65];
  char response[65];
  HID_REPORT send_report;
  HID_REPORT recv_report;

  CustomHIDDevice(void){
    static bool initialized = false;
    if(!initialized){
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
    flag = false;
    
  }
  
  void begin(void){
    HID.begin();
  }
    
  uint16_t _onGetDescriptor(uint8_t* buffer){
    Serial.print("on_get_descriptor");
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }


  void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len)
  {
    RC+=1;
    lastReq = buffer[0];
    if (len==64){
      memcpy(recv_report.data,buffer,64);
      HidCmdProcess();
      
    }
    //send((uint8_t *) response);
  }

void HidDecodeCommand( uint8_t *hidReport, tHidCommand *cmd )
{
    cmd->Cmd = hidReport[0];
    cmd->CmdOpt = hidReport[1];
    cmd->CmdDataSize = hidReport[2];
    cmd->CmdData = hidReport + 3;
}

void HidEncodeCommandAns( uint8_t cmd, uint8_t stat, uint8_t dataSize, uint8_t *data )
{
    send_report.data[0] =  cmd;
    send_report.data[1] =  stat;

    // TimeStamp
    memset( send_report.data + 2, 0, 8 );

    send_report.data[10] =  dataSize;
    memcpy( send_report.data + 11, ( const void* )data, dataSize );
    
    //send_report.length = 11 + dataSize;
    send_report.length = 64;
    send(send_report.data);
}

void HidCmdProcess(void)
{
    uint8_t stat = SX_OK;
    uint8_t size = 0;
    uint8_t dataBuffer[64];
    tHidCommand cmd = { HID_SK_CMD_NONE, 0, 0, NULL };
    #ifdef _DEBUG_
    int i;
    #endif /* _DEBUG_ */

    HidDecodeCommand(recv_report.data, &cmd);
    
    switch (cmd.Cmd) {
        case HID_DEVICE_RESET:
            SX127x_reset();
            break;
        case HID_SK_RESET:
        case HID_DEVICE_INIT:
            ///radio.hw_reset();
            SX127x_reset();
            #ifdef _DEBUG_
            if (verbose)
                printf("reset-init\r\n");
            #endif /* _DEBUG_ */
            ///radio.init();   //SX1272Init( );
            // Set FSK modem ON
            ///radio.set_opmode(RF_OPMODE_SLEEP);  
            ///radio.RegOpMode.bits.LongRangeMode = 0;
            ///radio.write_reg(REG_OPMODE, radio.RegOpMode.octet);            
            //radio.SetLoRaOn( false ); //SX1272SetLoRaOn( false ); // Default radio setting
            // Default answer settings
            break;
        case HID_SK_GET_VERSION:
            strcpy( ( char* )dataBuffer, FW_VERSION );
            size = strlen( FW_VERSION );
            break;
        case HID_DEVICE_READ:
            // cmd.CmdData[0] = size
            // cmd.CmdData[1] = address
            size = cmd.CmdData[0];
            lastAddrRead = cmd.CmdData[1];
            SX127x_read_buffer( cmd.CmdData[1], dataBuffer, size );
            #ifdef _DEBUG_
            if (verbose) {
                ///pc.printf("read %d bytes from %02x: ", size, cmd.CmdData[1]);
                ///for (i = 0; i < size; i++)
                ///    pc.printf("%02x ", dataBuffer[i]);
                ///pc.printf("\r\n");
            }
            #endif /* _DEBUG_ */
            stat = SX_OK;
            break;
        case HID_SK_GET_PINS:
            dataBuffer[0] = 0;
            ///if (radio.dio0)
///                dataBuffer[0] |= 0x01;
///            if (radio.dio1)
///                dataBuffer[0] |= 0x02;
            #ifdef _DEBUG_
            ///if (verbose && dataBuffer[0] != 0)
            ///    printf("HID_SK_GET_PINS:%02x\r\n", dataBuffer[0]);
            #endif /* _DEBUG_ */
            /*dataBuffer[0] |= DIO1 << 1;
            dataBuffer[0] |= DIO2 << 2;
            dataBuffer[0] |= DIO3 << 3;
            dataBuffer[0] |= DIO4 << 4;
            dataBuffer[0] |= DIO5 << 5;*/
            size = 1;
            break;
        case HID_SK_GET_PIN:
            // cmd.CmdData[0] = Pin id
            switch( cmd.CmdData[0] ) {
                case 11:    // FEM_CPS_PIN
                    // not existing on shield board -- dataBuffer[0] = radio.femcps;
                    #ifdef _DEBUG_
                    if (verbose)
                        ("HID_SK_GET_PIN femcps:%02x\r\n", dataBuffer[0]);
                    #endif /* _DEBUG_ */
                    break;
                case 12:    // FEM_CTX_PIN
                    // not existing on shield board --  dataBuffer[0] = radio.femctx;
                    #ifdef _DEBUG_
                    if (verbose)
                        printf("HID_SK_GET_PIN femctx:%02x\r\n", dataBuffer[0]);
                    #endif /* _DEBUG_ */
                    break;
                default:
                    dataBuffer[0] = 0xFF; // Signal ID error
                    #ifdef _DEBUG_
                    printf("HID_SK_GET_PIN %d\r\n", cmd.CmdData[0]);
                    #endif /* _DEBUG_ */
                    break;
            } // ...switch( cmd.CmdData[0] )
            break;
        case HID_SK_SET_PIN:
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
                    #ifdef _DEBUG_
                    if (verbose)
                        printf("HID_SK_SET_PIN femcps:%d\r\n", (int)radio.femcps);
                    #endif /* _DEBUG_ */
                    break;                    
                case 12:    // FEM_CTX_PIN
                    // not existing on shield board -- radio.femctx = cmd.CmdData[1];
                    #ifdef _DEBUG_
                    if (verbose)
                        printf("HID_SK_SET_PIN femctx:%d\r\n", (int)radio.femctx); 
                    #endif /* _DEBUG_ */
                    break;
                default:
                    stat = SX_UNSUPPORTED;
                    #ifdef _DEBUG_
                    pc.printf("HID_SK_SET_PIN %d %d\r\n", cmd.CmdData[0], cmd.CmdData[1]);
                    #endif /* _DEBUG_ */
                    break;
            } // ...switch( cmd.CmdData[0] )
            
            break;
        case HID_DEVICE_WRITE:
            // cmd.CmdData[0] = size
            // cmd.CmdData[1] = address
            // cmd.CmdData[2] = Buffer first byte
            // cmd.CmdData[2+(size-1)] = Buffer last byte
            ///radio.WriteBuffer( cmd.CmdData[1], cmd.CmdData + 2, cmd.CmdData[0] );
            #ifdef _DEBUG_
            if (verbose) {
                pc.printf("write %d bytes to %02x: ", cmd.CmdData[0], cmd.CmdData[1]);
                for (i = 0; i < cmd.CmdData[0]; i++)
                    pc.printf("%02x ", cmd.CmdData[2+i]);
                pc.printf("\r\n");
            }
            #endif /* _DEBUG_ */
            stat = SX_OK;
            break;
        case HID_SK_GET_NAME:
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
    
    HidEncodeCommandAns( cmd.Cmd, stat, size, dataBuffer);
}

  bool send(uint8_t * value){
    return HID.SendReport(0, value, 64);
  }
};

CustomHIDDevice Device;
#endif /* ARDUINO_USB_MODE */




void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.print("Cpu freq:");
  Serial.println(getCpuFrequencyMhz());
  //Serial.setDebugOutput(true);
  Device.begin();
  USB.begin();
  SX127x_begin();
}

void loop() {
  if (RC!=oldRC){
      Serial.print(RC);
      Serial.print(" " + String(lastReq) );
      Serial.println(" " + String(lastAddrRead) );
      Serial.flush();
      oldRC=RC;
    }    
    delay(30);
}