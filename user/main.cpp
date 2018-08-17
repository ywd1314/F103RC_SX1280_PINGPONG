/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2017 Semtech

Description: Main program

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include "mbed.h"
#include "radio.h"
#include "sx1280-hal.h"


/*!
 * \brief Used to display firmware version on RS232
 */
#define FIRMWARE_VERSION ( ( char* )"Firmware Version: 170314A" )

/*!
 * Use the Ping Ping in uncommented Mode
 */
//#define MODE_BLE
#define MODE_LORA
//#define MODE_GENERIC
//#define MODE_FLRC


/*!
 * \brief Defines the nominal frequency
 */
#define RF_FREQUENCY                                2400000000UL // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define TX_OUTPUT_POWER                             13



extern serial_t    stdio_uart;
extern int         stdio_uart_inited;



/*!
 * \brief Defines the states of the application
 */
typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
}AppStates_t;

/*!
 * \brief Defines the buffer size, i.e. the payload size
 */
#define BUFFER_SIZE                                 16

/*!
 * \brief Define the possible message type for this application
 */
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

/*!
 * \brief Defines the size of the token defining message type in the payload
 */
#define PINGPONGSIZE                    4

/*!
 * \brief The size of the buffer
 */
uint8_t BufferSize = BUFFER_SIZE;

/*!
 * \brief The buffer
 */
uint8_t Buffer[BUFFER_SIZE];

/*!
 * \brief The State of the application
 */
AppStates_t AppState = APP_LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    NULL,             // rangingDone
    NULL,             // cadDone
};

// mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks...
SX1280Hal Radio( PA_7, PA_6, PA_5, PA_4, PC_4, PC_5, NC, NC, PB_0, &callbacks );

DigitalOut TxLed( PD_2 );
DigitalOut RxLed( PD_2 );



/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            100 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            100 // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
Serial s( USBTX, USBRX );
/*!
 * \brief Specify serial datarate for UART debug output
 */
void baud( int baudrate )
{
    

    s.baud( baudrate );
}

int main( )
{
    bool isMaster = true;
    ModulationParams_t modulationParams;

  
		serial_init(&stdio_uart,PA_2,PA_3);
	  stdio_uart_inited = 1;
	
	
	  baud( 115200 );


    RxLed  = 1;
    TxLed  = 1;
    

    wait_ms( 500 ); // wait for on board DC/DC start-up time

    Radio.Init( );
    Radio.SetRegulatorMode( USE_DCDC ); // Can also be set in LDO mode but consume more power

    memset( &Buffer, 0x00, BufferSize );

    s.printf( "\n\n\r     SX1280 Ping Pong Demo Application (%s)\n\n\r", FIRMWARE_VERSION );

#if defined( MODE_BLE )

    printf( "\nPing Pong running in BLE mode\n\r");
    modulationParams.PacketType                   = PACKET_TYPE_BLE;
    modulationParams.Params.Ble.BitrateBandwidth  = GEN_BLE_BR_0_125_BW_0_3;
    modulationParams.Params.Ble.ModulationIndex   = GEN_BLE_MOD_IND_1_00;
    modulationParams.Params.Ble.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType                 = PACKET_TYPE_BLE;
    packetParams.Params.Ble.BlePacketType   = BLE_EYELONG_1_0;
    packetParams.Params.Ble.ConnectionState = BLE_MASTER_SLAVE;
    packetParams.Params.Ble.CrcField        = BLE_CRC_3B;
    packetParams.Params.Ble.Whitening       = RADIO_WHITENING_OFF;

#elif defined( MODE_GENERIC )

    s.printf( "\nPing Pong running in GENERIC mode\n\r");
    modulationParams.PacketType                       = PACKET_TYPE_GENERIC;
    modulationParams.Params.Generic.BitrateBandwidth  = GEN_BLE_BR_0_125_BW_0_3;
    modulationParams.Params.Generic.ModulationIndex   = GEN_BLE_MOD_IND_1_00;
    modulationParams.Params.Generic.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType                    = PACKET_TYPE_GENERIC;
    packetParams.Params.Generic.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Generic.SyncWordLength = GEN_SYNCWORD_LENGTH_5_BYTE;
    packetParams.Params.Generic.SyncWordMatch  = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Generic.HeaderType     = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Generic.PayloadLength  = 15;
    packetParams.Params.Generic.CrcLength      = RADIO_CRC_3_BYTES;
    packetParams.Params.Generic.Whitening      = RADIO_WHITENING_ON;

#elif defined( MODE_LORA )

    s.printf( "\nPing Pong running in LORA mode\n\r");
    modulationParams.PacketType                  = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
    modulationParams.Params.LoRa.Bandwidth       = LORA_BW_0200;
    modulationParams.Params.LoRa.CodingRate      = LORA_CR_4_5;

    PacketParams.PacketType                 = PACKET_TYPE_LORA;
    PacketParams.Params.LoRa.PreambleLength = 0x08;
    PacketParams.Params.LoRa.HeaderType     = LORA_PACKET_VARIABLE_LENGTH;
    PacketParams.Params.LoRa.PayloadLength  = 15;
    PacketParams.Params.LoRa.Crc       = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ       = LORA_IQ_INVERTED;

#elif defined( MODE_FLRC )

    s.printf( "\nPing Pong running in FLRC mode\n\r");
    modulationParams.PacketType                    = PACKET_TYPE_FLRC;
    modulationParams.Params.Flrc.BitrateBandwidth  = FLRC_BR_0_260_BW_0_3;
    modulationParams.Params.Flrc.CodingRate        = FLRC_CR_1_2;
    modulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType                 = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch  = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType     = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Flrc.PayloadLength  = 15;
    packetParams.Params.Flrc.CrcLength      = RADIO_CRC_3_BYTES;
    packetParams.Params.Flrc.Whitening      = RADIO_WHITENING_OFF;

#else
#error "Please select the mode of operation for the Ping Ping demo"
#endif

    Radio.SetStandby( STDBY_RC );
    Radio.SetPacketType( modulationParams.PacketType );
    Radio.SetModulationParams( &modulationParams );
    Radio.SetPacketParams( &PacketParams );

    Radio.SetRfFrequency( RF_FREQUENCY );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_20_US );

    // only used in GENERIC and BLE mode
    Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );

    RxLed = 0;
    TxLed = 0;

    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
    AppState = APP_LOWPOWER;

    while( 1 )
    {
        switch( AppState )
        {
            case APP_RX:
                AppState = APP_LOWPOWER;
                RxLed = !RxLed;
                Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
								Radio.GetPacketStatus(&PacketStatus);
								RssiValue = PacketStatus.LoRa.RssiPkt;
								SnrValue = PacketStatus.LoRa.SnrPkt;
								printf("rssi: %d;snr: %d\n\r", RssiValue, SnrValue );
                if( isMaster == true )
                {
                    if( BufferSize > 0 )
                    {
                        if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, PINGPONGSIZE ) == 0 )//两个字符串相等返回0
                        {
                            s.printf( "...Pong\r\n" );
                            memcpy( Buffer, PingMsg, PINGPONGSIZE );
                            Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                            Radio.SendPayload( Buffer, BufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                        }
                        else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, PINGPONGSIZE ) == 0 )
                        {
                            // A master already exists then become a slave
                            s.printf( "...Ping\r\n" );
                            isMaster = false;
                            memcpy( Buffer, PongMsg, PINGPONGSIZE );
                            Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                            Radio.SendPayload( Buffer, BufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                        }
                        else // valid reception but neither a PING or a PONG message
                        {    // Set device as master ans start again
                            isMaster = true;
                            Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                        }
                    }
                }
                else
                {
                    if( BufferSize > 0 )
                    {
                        if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, PINGPONGSIZE ) == 0 )
                        {
                            s.printf( "...Ping\r\n" );
                            memcpy( Buffer, PongMsg, 4 );
                            Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                            Radio.SendPayload( Buffer, BufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                        }
                        else // valid reception but not a PING as expected
                        {
                            isMaster = true;
                            Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                        }
                    }
                }
                break;

            case APP_TX:
                AppState = APP_LOWPOWER;
                TxLed = !TxLed;
                if( isMaster == true )
                {
                    s.printf( "Ping...\r\n" );
                }
                else
                {
                    s.printf( "Pong...\r\n" );
                }
                Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                break;

            case APP_RX_TIMEOUT:
                AppState = APP_LOWPOWER;
                if( isMaster == true )
                {
                    // Send the next PING frame
                    memcpy( Buffer, PingMsg, PINGPONGSIZE );
                    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SendPayload( Buffer, BufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                }
                else
                {
                    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                }
                break;

            case APP_RX_ERROR:
                AppState = APP_LOWPOWER;
                // We have received a Packet with a CRC error, send reply as if packet was correct
                if( isMaster == true )
                {
                    // Send the next PING frame
                    memcpy( Buffer, PingMsg, PINGPONGSIZE );
                    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SendPayload( Buffer, BufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                }
                else
                {
                    // Send the next PONG frame
                    memcpy( Buffer, PongMsg, PINGPONGSIZE );
                    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SendPayload( Buffer, BufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
                }
                break;

            case APP_TX_TIMEOUT:
                AppState = APP_LOWPOWER;
                Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                break;

            case APP_LOWPOWER:
                break;

            default:
                // Set low power
                break;
        }
    }
}

void OnTxDone( void )
{
    AppState = APP_TX;
}

void OnRxDone( void )
{
    AppState = APP_RX;
//    Radio.GetPacketStatus(&PacketStatus);
//#if ( defined( MODE_BLE ) || defined( MODE_GENERIC ) )
//    RssiValue = PacketStatus.Ble.RssiSync;
//    s.printf("rssi: %d\n\r", RssiValue);
//#elif defined( MODE_LORA )
//    RssiValue = PacketStatus.LoRa.RssiPkt;
//    SnrValue = PacketStatus.LoRa.SnrPkt;
//    printf("rssi: %d\n\r", RssiValue);
//#endif
}

void OnTxTimeout( void )
{
    AppState = APP_TX_TIMEOUT;
    s.printf( "<>>>>>>>>TXE\r\n" );
}

void OnRxTimeout( void )
{
    AppState = APP_RX_TIMEOUT;
}

void OnRxError( IrqErrorCode_t errorCode )
{
    AppState = APP_RX_ERROR;
    s.printf( "RXE<>>>>>>>>\r\n" );
}

void OnRangingDone( IrqRangingCode_t val )
{
}

void OnCadDone( bool channelActivityDetected )
{
}
