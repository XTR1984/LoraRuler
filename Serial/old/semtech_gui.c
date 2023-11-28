//from https://os.mbed.com/users/modtronix/code/SX1276_Semtech_GUI/ 
#include "mbed.h"
#include "USBHID.h"
#include "sx127x.h"

//#define _DEBUG_

/*
 * This program is a modified version of the "hid_test" application by Wayne Roberts. All credits go
 * to Wayne for writing it, I just did some minor modiciations!
 *
 * The program was tested with the FRDM-KL25Z board, with a Modtronix inAir9 SX1276 board mounted in
 * a Modtronix SHD3I shield. The inAir9 module is mounted in iMod port 3 of the SHD3I module. The
 * SHD3I shield is mounted on the FRDM-KL25Z board.
 * It enables the use of the Semtech "SX1276 Starter Kit A" application to be used with this board.
 * At the time of writing this, this app was available on Semtech's SX1276 page, in the "Docs & Resources"
 * tab, via the link "PC GUI (1.0.0Beta5)".
 * This app can be used for sending and receiving data via the SX1276. To use it:
 * - Plug the FRDM-KL25Z into the PC via the USB marked "USB KL25Z"
 * - Start Semtech "SX1276 Starter Kit A" app on PC
 * - App should connect to board, enabling it to control board.
 *
 * ========== Firmware for KL25Z ==========
 * The default firmware listed on mbed.org puts a clock pulse on pin A5. To fix this, use alternative
 * firmware (or unsolder resistor R24).
 * - Download "OpenSDA Firmware" from http://www.pemicro.com/opensda/
 * - Put KL25Z into bootloader mode (hold down button while plugging in SDA USB).
 * - Drag "MSD-DEBUG-FRDM-KL25Z_Pemicro_v114.SDA" file onto it.
 * - Reset KL25Z.
 * - If the USB drivers do not automatically install, download and run "Windows USB Drivers" from
 *   www.pemicro.com/opensda. After this, when you plug in the KL25Z next time, all drivers should
 *   install correctly.
 */

#define TARGET_KL25Z_SHD3I_INAIR8           /* FRDM-KL25Z + Modtronix inAir9 SX1276 board + SHD3I daughter board */

//Defines for the FRDM-KL25Z board, with Modtronix SHD3I with an inAir9 SX1276 module mounted in it.
#ifdef TARGET_KL25Z_SHD3I_INAIR8
//SCLK=D13, MISO=D12, MOSI=D11
//CS=D7, Reset=A5
//DIO0=D2, DIO1=D8, DIO2=D4, DIO3=A4, DIO4=N.C., DIO5=D3
//           mosi, miso, sclk, cs,  rst, dio0, dio1
SX127x radio(D11,  D12,  D13,  D7,  A5,  D2,   D8);
#else
//  pin:      3     8     1    7     10    12      5
//           mosi, miso, sclk, cs,   rst,  dio0,  dio1
//           D11   D12   D13   D10   D9    D8     D2
SX127x radio(PTD2, PTD3, PTD1, PTD0, PTD5, PTA13, PTD4);
#endif

#ifdef _DEBUG_
    #include "sx127x_lora.h"
    SX127x_lora lora(radio);
#endif /* _DEBUG_ */

//We declare a USBHID device. By default input and output reports are 64 bytes long.
USBHID hid(64, 64, 0x47a, 0x0b);
//USBHID (uint8_t output_report_length=64, uint8_t input_report_length=64, uint16_t vendor_id=0x1234, uint16_t product_id=0x0006, uint16_t product_release=0x0001, bool connect=true)

Serial pc(USBTX, USBRX);

//This report will contain data to be sent
HID_REPORT send_report;
HID_REPORT recv_report;

DigitalOut l1(LED1);

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

#ifdef _DEBUG_
char verbose = 0;
#endif /* _DEBUG_ */

//Following code is from V7
//DigitalOut rfsw1(PTC8);
//DigitalOut rfsw2(PTC9);

void rfsw_callback()
{
    /*
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {  // start of transmission
        if (radio.HF) {
            if (radio.RegPaConfig.bits.PaSelect) { // if PA_BOOST
                rfsw2 = 0;
                rfsw1 = 1;
            } else { // RFO to power amp
                rfsw2 = 1;
                rfsw1 = 0;
            }
        } else {
            // todo: sx1276
        }
    } else if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || radio.RegOpMode.bits.Mode == RF_OPMODE_CAD) { // start of reception
        if (radio.HF) {
            rfsw2 = 1;
            rfsw1 = 1;
        } else {
            // todo: sx1276
        }
    } else { // RF switch shutdown
        rfsw2 = 0;
        rfsw1 = 0;
    }
    */
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
    hid.send(&send_report);
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
            radio.hw_reset();
            break;
        case HID_SK_RESET:
        case HID_DEVICE_INIT:
            radio.hw_reset();
            #ifdef _DEBUG_
            if (verbose)
                printf("reset-init\r\n");
            #endif /* _DEBUG_ */
            radio.init();   //SX1272Init( );
            // Set FSK modem ON
            radio.set_opmode(RF_OPMODE_SLEEP);
            radio.RegOpMode.bits.LongRangeMode = 0;
            radio.write_reg(REG_OPMODE, radio.RegOpMode.octet);
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
            radio.ReadBuffer( cmd.CmdData[1], dataBuffer, size );
            #ifdef _DEBUG_
            if (verbose) {
                pc.printf("read %d bytes from %02x: ", size, cmd.CmdData[1]);
                for (i = 0; i < size; i++)
                    pc.printf("%02x ", dataBuffer[i]);
                pc.printf("\r\n");
            }
            #endif /* _DEBUG_ */
            stat = SX_OK;
            break;
        case HID_SK_GET_PINS:
            dataBuffer[0] = 0;
            if (radio.dio0)
                dataBuffer[0] |= 0x01;
            if (radio.dio1)
                dataBuffer[0] |= 0x02;
            #ifdef _DEBUG_
            if (verbose && dataBuffer[0] != 0)
                printf("HID_SK_GET_PINS:%02x\r\n", dataBuffer[0]);
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
                    //MODTRONIX modified, we do not have a femcps pin
                    //dataBuffer[0] = radio.femcps;
                    dataBuffer[0] = 0;
                    #ifdef _DEBUG_
                    if (verbose)
                        printf("HID_SK_GET_PIN femcps:%02x\r\n", dataBuffer[0]);
                    #endif /* _DEBUG_ */
                    break;
                case 12:    // FEM_CTX_PIN
                    //MODTRONIX modified, we do not have a femctx pin
                    //dataBuffer[0] = radio.femctx;
                    dataBuffer[0] = 0;
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
                    //MODTRONIX modified, we do not have a femcps pin
                    //radio.femcps = cmd.CmdData[1];
                    #ifdef _DEBUG_
                    if (verbose)
                        printf("HID_SK_SET_PIN femcps:%d\r\n", (int)radio.femcps);
                    #endif /* _DEBUG_ */
                    break;
                case 12:    // FEM_CTX_PIN
                    //MODTRONIX modified, we do not have a femctx pin
                    //radio.femctx = cmd.CmdData[1];
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
            radio.WriteBuffer( cmd.CmdData[1], cmd.CmdData + 2, cmd.CmdData[0] );
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
            pc.printf("%d: ", recv_report.length);
            for(int i = 0; i < recv_report.length; i++) {
                pc.printf("%02x ", recv_report.data[i]);
            }
            pc.printf("\r\n");
            stat = SX_UNSUPPORTED;
        break;
    } // ...switch (cmd.Cmd)

    HidEncodeCommandAns( cmd.Cmd, stat, size, dataBuffer);
}

#ifdef _DEBUG_
void printOpMode()
{
    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    switch (radio.RegOpMode.bits.Mode) {
        case RF_OPMODE_SLEEP: printf(" [7msleep [0m"); break;
        case RF_OPMODE_STANDBY: printf(" [7mstby [0m"); break;
        case RF_OPMODE_SYNTHESIZER_TX: printf(" [33mfstx [0m"); break;
        case RF_OPMODE_TRANSMITTER: printf(" [31mtx [0m"); break;
        case RF_OPMODE_SYNTHESIZER_RX: printf(" [33mfsrx [0m"); break;
        case RF_OPMODE_RECEIVER: printf(" [32mrx [0m"); break;
        case 6:
            if (radio.RegOpMode.bits.LongRangeMode)
                printf(" [42mrxs [0m");
            else
                printf("-6-");
            break;  // todo: different lora/fsk
        case 7:
            if (radio.RegOpMode.bits.LongRangeMode)
                printf(" [45mcad [0m");
            else
                printf("-7-");
            break;  // todo: different lora/fsk
    }
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void
printPa()
{
    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);
    if (radio.RegPaConfig.bits.PaSelect) {
        float output_dBm = 17 - (15-radio.RegPaConfig.bits.OutputPower);
        printf(" PABOOST OutputPower=%.1fdBm", output_dBm);
    } else {
        float pmax = (0.6*radio.RegPaConfig.bits.MaxPower) + 10.8;
        float output_dBm = pmax - (15-radio.RegPaConfig.bits.OutputPower);
        printf(" RFO pmax=%.1fdBm OutputPower=%.1fdBm", pmax, output_dBm);
    }
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void /* things always present, whether lora or fsk */
common_print_status()
{
    printf("version:0x%02x %.3fMHz ", radio.read_reg(REG_VERSION), radio.get_frf_MHz());
    printOpMode();

    printPa();

    radio.RegOcp.octet = radio.read_reg(REG_OCP);
    if (radio.RegOcp.bits.OcpOn) {
        int imax = 0;
        if (radio.RegOcp.bits.OcpTrim < 16)
            imax = 45 + (5 * radio.RegOcp.bits.OcpTrim);
        else if (radio.RegOcp.bits.OcpTrim < 28)
            imax = -30 + (10 * radio.RegOcp.bits.OcpTrim);
        else
            imax = 240;
        printf(" OcpOn %dmA ", imax);
    } else
        printf(" OcpOFF ");

    printf("\r\n");

}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void lora_print_dio()
{
   radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    printf("DIO5:");
    switch (radio.RegDioMapping2.bits.Dio5Mapping) {
        case 0: printf("ModeReady"); break;
        case 1: printf("ClkOut"); break;
        case 2: printf("ClkOut"); break;
    }
    printf(" DIO4:");
    switch (radio.RegDioMapping2.bits.Dio4Mapping) {
        case 0: printf("CadDetected"); break;
        case 1: printf("PllLock"); break;
        case 2: printf("PllLock"); break;
    }
    radio.RegDioMapping1.octet = radio.read_reg(REG_DIOMAPPING1);
    printf(" DIO3:");
    switch (radio.RegDioMapping1.bits.Dio3Mapping) {
        case 0: printf("CadDone"); break;
        case 1: printf("ValidHeader"); break;
        case 2: printf("PayloadCrcError"); break;
    }
    printf(" DIO2:");
    switch (radio.RegDioMapping1.bits.Dio2Mapping) {
        case 0:
        case 1:
        case 2:
            printf("FhssChangeChannel");
            break;
    }
    printf(" DIO1:");
    switch (radio.RegDioMapping1.bits.Dio1Mapping) {
        case 0: printf("RxTimeout"); break;
        case 1: printf("FhssChangeChannel"); break;
        case 2: printf("CadDetected"); break;
    }
    printf(" DIO0:");
    switch (radio.RegDioMapping1.bits.Dio0Mapping) {
        case 0: printf("RxDone"); break;
        case 1: printf("TxDone"); break;
        case 2: printf("CadDone"); break;
    }

    printf("\r\n");
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void
printCodingRate(bool from_rx)
{
    uint8_t d = lora.getCodingRate(from_rx);
    printf("CodingRate:");
    switch (d) {
        case 1: printf("4/5 "); break;
        case 2: printf("4/6 "); break;
        case 3: printf("4/7 "); break;
        case 4: printf("4/8 "); break;
        default:
            printf("%d ", d);
            break;
    }
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printHeaderMode()
{
    if (lora.getHeaderMode())
        printf("implicit ");
    else
        printf("explicit ");
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printBw()
{
    uint8_t bw = lora.getBw();

    printf("Bw:");
    if (radio.type == SX1276) {
        switch (lora.RegModemConfig.sx1276bits.Bw) {
            case 0: printf("7.8KHz "); break;
            case 1: printf("10.4KHz "); break;
            case 2: printf("15.6KHz "); break;
            case 3: printf("20.8KHz "); break;
            case 4: printf("31.25KHz "); break;
            case 5: printf("41.7KHz "); break;
            case 6: printf("62.5KHz "); break;
            case 7: printf("125KHz "); break;
            case 8: printf("250KHz "); break;
            case 9: printf("500KHz "); break;
            default: printf("%x ", lora.RegModemConfig.sx1276bits.Bw); break;
        }
    } else if (radio.type == SX1272) {
        switch (lora.RegModemConfig.sx1272bits.Bw) {
            case 0: printf("125KHz "); break;
            case 1: printf("250KHz "); break;
            case 2: printf("500KHz "); break;
            case 3: printf("11b "); break;
        }
    }
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printSf()
{
    // spreading factor same between sx127[26]
    printf("sf:%d ", lora.getSf());
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printTxContinuousMode()
{
    printf("TxContinuousMode:%d ", lora.RegModemConfig2.sx1276bits.TxContinuousMode);    // same for sx1272 and sx1276
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printAgcAutoOn()
{
    printf("AgcAutoOn:%d", lora.getAgcAutoOn());
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printRxPayloadCrcOn()
{
    bool on = lora.getRxPayloadCrcOn();
    //printf("RxPayloadCrcOn:%s ", on ? "on" : "off");
    if (on)
        printf("RxPayloadCrcOn:1 = Tx CRC Enabled\r\n");
    else
        printf("RxPayloadCrcOn:1 = no Tx CRC\r\n");
}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void printLoraIrqs_(bool clear)
{
    //in radio class -- RegIrqFlags_t RegIrqFlags;

    //already read RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    printf("\r\nIrqFlags:");
    if (lora.RegIrqFlags.bits.CadDetected)
        printf("CadDetected ");
    if (lora.RegIrqFlags.bits.FhssChangeChannel) {
        //radio.RegHopChannel.octet = radio.read_reg(REG_LR_HOPCHANNEL);
        printf("FhssChangeChannel:%d ", lora.RegHopChannel.bits.FhssPresentChannel);
    }
    if (lora.RegIrqFlags.bits.CadDone)
        printf("CadDone ");
    if (lora.RegIrqFlags.bits.TxDone)
        printf("TxDone ");
    if (lora.RegIrqFlags.bits.ValidHeader)
        printf(" [42mValidHeader [0m ");
    if (lora.RegIrqFlags.bits.PayloadCrcError)
        printf(" [41mPayloadCrcError [0m ");
    if (lora.RegIrqFlags.bits.RxDone)
        printf(" [42mRxDone [0m ");
    if (lora.RegIrqFlags.bits.RxTimeout)
        printf("RxTimeout ");

    printf("\r\n");

    if (clear)
        radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);

}
#endif /* _DEBUG_ */

#ifdef _DEBUG_
void lora_print_status()
{
    uint8_t d;

    if (radio.type == SX1276)
        printf("\r\nSX1276 ");
    else if (radio.type == SX1272)
        printf("\r\nSX1272 ");

    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (!radio.RegOpMode.bits.LongRangeMode) {
        printf("FSK\r\n");
        return;
    }

    lora_print_dio();
    printf("LoRa ");

    // printing LoRa registers at 0x0d -> 0x3f

    lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
    lora.RegModemConfig2.octet = radio.read_reg(REG_LR_MODEMCONFIG2);

    printCodingRate(false); // false: transmitted coding rate
    printHeaderMode();
    printBw();
    printSf();
    printRxPayloadCrcOn();
    // RegModemStat
    printf("ModemStat:0x%02x\r\n", radio.read_reg(REG_LR_MODEMSTAT));

    // fifo ptrs:
    lora.RegPayloadLength = radio.read_reg(REG_LR_PAYLOADLENGTH);
    lora.RegRxMaxPayloadLength = radio.read_reg(REG_LR_RX_MAX_PAYLOADLENGTH);
    printf("fifoptr=0x%02x txbase=0x%02x rxbase=0x%02x payloadLength=0x%02x maxlen=0x%02x",
        radio.read_reg(REG_LR_FIFOADDRPTR),
        radio.read_reg(REG_LR_FIFOTXBASEADDR),
        radio.read_reg(REG_LR_FIFORXBASEADDR),
        lora.RegPayloadLength,
        lora.RegRxMaxPayloadLength
    );

    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    printLoraIrqs_(false);

    lora.RegHopPeriod = radio.read_reg(REG_LR_HOPPERIOD);
    if (lora.RegHopPeriod != 0) {
        printf("\r\nHopPeriod:0x%02x\r\n", lora.RegHopPeriod);
    }

    printf("SymbTimeout:0x%03x ", radio.read_u16(REG_LR_MODEMCONFIG2) & 0x3ff);

    lora.RegPreamble = radio.read_u16(REG_LR_PREAMBLEMSB);
    printf("PreambleLength:0x%03x ", lora.RegPreamble);


    if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE) {
        d = radio.read_reg(REG_LR_RSSIVALUE);
        printf("rssi:%ddBm ", d-120);
    }

    printTxContinuousMode();

    printf("\r\n");
    printAgcAutoOn();
    if (radio.type == SX1272) {
        printf(" LowDataRateOptimize:%d\r\n", lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
    }

    printf("\r\nHeaderCount:%d PacketCount:%d, ",
        radio.read_u16(REG_LR_RXHEADERCNTVALUE_MSB), radio.read_u16(REG_LR_RXPACKETCNTVALUE_MSB));

    printf("Lora detection threshold:%02x\r\n", radio.read_reg(REG_LR_DETECTION_THRESHOLD));
    lora.RegTest31.octet = radio.read_reg(REG_LR_TEST31);
    printf("detect_trig_same_peaks_nb:%d\r\n", lora.RegTest31.bits.detect_trig_same_peaks_nb);

    if (radio.type == SX1272) {
        lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
        printf("LowDataRateOptimize:%d\r\n", lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
    } else if (radio.type == SX1276) {
        lora.RegModemConfig3.octet = radio.read_reg(REG_LR_MODEMCONFIG3);
        printf("LowDataRateOptimize:%d\r\n", lora.RegModemConfig3.sx1276bits.LowDataRateOptimize);
    }

    printf("\r\n");
    //printf("A %02x\r\n", radio.RegModemConfig2.octet);
}
#endif /* _DEBUG_ */

/*void
service_radio()
{
    service_action_e act = radio.service();

    switch (act) {
        case SERVICE_READ_FIFO:
            printf("SERVICE_READ_FIFO\r\n");
            // clear Irq flags
            radio.write_reg(REG_LR_IRQFLAGS, radio.RegIrqFlags.octet);
            break;
        case SERVICE_TX_DONE:
            printf("SERVICE_TX_DONE\r\n");
            break;
        case SERVICE_ERROR:
            printf("error\r\n");
            break;
    } // ...switch (act)
}*/

int
main(void)
{
    #ifdef _DEBUG_
    pc.baud(57600);
    pc.printf("\r\nstart\r\n");
    #endif /* _DEBUG_ */

    //Required for V7 of SX127x library, but V7 doesn't work for this program!
    radio.rf_switch.attach(rfsw_callback);

    while (1) {
        //try to read a msg
        if (hid.readNB(&recv_report)) {
            HidCmdProcess();
        }

        #ifdef _DEBUG_
        if (pc.readable()) {
            char c = pc.getc();
            if (c == 'v') {
                pc.printf("verbose ");
                if (verbose) {
                    verbose = 0;
                    pc.printf("off");
                } else {
                    verbose = 1;
                    pc.printf("on");
                }
                pc.printf("\r\n");
            } else if (c == '.') {
                common_print_status();
                if (radio.RegOpMode.bits.LongRangeMode)
                    lora_print_status();
                else
                    printf("FSK\r\n");
            } else if (c == 't') {
                int i;
                printf("tx\r\n");
                radio.set_opmode(RF_OPMODE_TRANSMITTER);
                for (i = 0; i < 20; i++) {
                    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
                    printf("opmode:%02x\r\n", radio.RegOpMode.octet);
                }
            } else if (c == 'T') {
                printf("start_tx\r\n");
                lora.RegPayloadLength = 8;
                radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
                lora.start_tx(8);
            } /*else if (c == 'e') {
                printf("service_radio\r\n");
                service_radio();
            }*/ else if (c == 's') {
                radio.set_opmode(RF_OPMODE_STANDBY);
                printf("standby\r\n");
            } else if (c == 'h') {
                printf("hwreset\r\n");
                radio.hw_reset();
                radio.init();   //SX1272Init( );
            } /*else if (c == 'l') {
                radio.SetLoRaOn(!radio.RegOpMode.bits.LongRangeMode);
                printf("LongRangeMode:%d\r\n", radio.RegOpMode.bits.LongRangeMode);
            }*/ else if (c == '?') {
                printf("s   standby\r\n");
                printf("T   lora_start_tx(8)\r\n");
                printf(".   print status\r\n");
                printf("v   toggle verbose\r\n");
                printf("t   tx mode test\r\n");
                printf("e   manualy service radio once\r\n");
                printf("h   hwreset, init\r\n");
                printf("l   toggle lora mode\r\n");
            }
        } // ...if (pc.readable())
        #endif /* _DEBUG_ */

    } // ...while (1)
}
