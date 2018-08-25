package liblorago

import (
	"fmt"
	"log"
	"os"
	"time"
)

var SX125x_32MHz_FRAC = uint32(15625)

type lgw_sx127x_rxbw_e int

const (
	LGW_SX127X_RXBW_2K6_HZ lgw_sx127x_rxbw_e = iota
	LGW_SX127X_RXBW_3K1_HZ
	LGW_SX127X_RXBW_3K9_HZ
	LGW_SX127X_RXBW_5K2_HZ
	LGW_SX127X_RXBW_6K3_HZ
	LGW_SX127X_RXBW_7K8_HZ
	LGW_SX127X_RXBW_10K4_HZ
	LGW_SX127X_RXBW_12K5_HZ
	LGW_SX127X_RXBW_15K6_HZ
	LGW_SX127X_RXBW_20K8_HZ
	LGW_SX127X_RXBW_25K_HZ
	LGW_SX127X_RXBW_31K3_HZ
	LGW_SX127X_RXBW_41K7_HZ
	LGW_SX127X_RXBW_50K_HZ
	LGW_SX127X_RXBW_62K5_HZ
	LGW_SX127X_RXBW_83K3_HZ
	LGW_SX127X_RXBW_100K_HZ
	LGW_SX127X_RXBW_125K_HZ
	LGW_SX127X_RXBW_166K7_HZ
	LGW_SX127X_RXBW_200K_HZ
	LGW_SX127X_RXBW_250K_HZ
)

//sx125x
var SX125x_TX_DAC_CLK_SEL = 1   /* 0:int, 1:ext */
var SX125x_TX_DAC_GAIN = 2      /* 3:0, 2:-3, 1:-6, 0:-9 dBFS (default 2) */
var SX125x_TX_MIX_GAIN = 14     /* -38 + 2*TxMixGain dB (default 14) */
var SX125x_TX_PLL_BW = 1        /* 0:75, 1:150, 2:225, 3:300 kHz (default 3) */
var SX125x_TX_ANA_BW = 0        /* 17.5 / 2*(41-TxAnaBw) MHz (default 0) */
var SX125x_TX_DAC_BW = 5        /* 24 + 8*TxDacBw Nb FIR taps (default 2) */
var SX125x_RX_LNA_GAIN = 1      /* 1 to 6, 1 highest gain */
var SX125x_RX_BB_GAIN = 12      /* 0 to 15 , 15 highest gain */
var SX125x_LNA_ZIN = 1          /* 0:50, 1:200 Ohms (default 1) */
var SX125x_RX_ADC_BW = 7        /* 0 to 7, 2:100<BW<200, 5:200<BW<400,7:400<BW kHz SSB (default 7) */
var SX125x_RX_ADC_TRIM = 6      /* 0 to 7, 6 for 32MHz ref, 5 for 36MHz ref */
var SX125x_RX_BB_BW = 0         /* 0:750, 1:500, 2:375; 3:250 kHz SSB (default 1, max 3) */
var SX125x_RX_PLL_BW = 0        /* 0:75, 1:150, 2:225, 3:300 kHz (default 3, max 3) */
var SX125x_ADC_TEMP = 0         /* ADC temperature measurement mode (default 0) */
var SX125x_XOSC_GM_STARTUP = 13 /* (default 13) */
var SX125x_XOSC_DISABLE = 2     /* Disable of Xtal Oscillator blocks bit0:regulator, bit1:core(gm), bit2:amplifier */

//sx1272_fsk
const (
	/*!
	 * ============================================================================
	 * SX1272 Internal registers Address
	 * ============================================================================
	 */
	SX1272_REG_FIFO = 0x00
	// Common settings
	SX1272_REG_OPMODE     = 0x01
	SX1272_REG_BITRATEMSB = 0x02
	SX1272_REG_BITRATELSB = 0x03
	SX1272_REG_FDEVMSB    = 0x04
	SX1272_REG_FDEVLSB    = 0x05
	SX1272_REG_FRFMSB     = 0x06
	SX1272_REG_FRFMID     = 0x07
	SX1272_REG_FRFLSB     = 0x08
	// Tx settings
	SX1272_REG_PACONFIG = 0x09
	SX1272_REG_PARAMP   = 0x0A
	SX1272_REG_OCP      = 0x0B
	// Rx settings
	SX1272_REG_LNA            = 0x0C
	SX1272_REG_RXCONFIG       = 0x0D
	SX1272_REG_RSSICONFIG     = 0x0E
	SX1272_REG_RSSICOLLISION  = 0x0F
	SX1272_REG_RSSITHRESH     = 0x10
	SX1272_REG_RSSIVALUE      = 0x11
	SX1272_REG_RXBW           = 0x12
	SX1272_REG_AFCBW          = 0x13
	SX1272_REG_OOKPEAK        = 0x14
	SX1272_REG_OOKFIX         = 0x15
	SX1272_REG_OOKAVG         = 0x16
	SX1272_REG_RES17          = 0x17
	SX1272_REG_RES18          = 0x18
	SX1272_REG_RES19          = 0x19
	SX1272_REG_AFCFEI         = 0x1A
	SX1272_REG_AFCMSB         = 0x1B
	SX1272_REG_AFCLSB         = 0x1C
	SX1272_REG_FEIMSB         = 0x1D
	SX1272_REG_FEILSB         = 0x1E
	SX1272_REG_PREAMBLEDETECT = 0x1F
	SX1272_REG_RXTIMEOUT1     = 0x20
	SX1272_REG_RXTIMEOUT2     = 0x21
	SX1272_REG_RXTIMEOUT3     = 0x22
	SX1272_REG_RXDELAY        = 0x23
	// Oscillator settings
	SX1272_REG_OSC = 0x24
	// Packet handler settings
	SX1272_REG_PREAMBLEMSB   = 0x25
	SX1272_REG_PREAMBLELSB   = 0x26
	SX1272_REG_SYNCCONFIG    = 0x27
	SX1272_REG_SYNCVALUE1    = 0x28
	SX1272_REG_SYNCVALUE2    = 0x29
	SX1272_REG_SYNCVALUE3    = 0x2A
	SX1272_REG_SYNCVALUE4    = 0x2B
	SX1272_REG_SYNCVALUE5    = 0x2C
	SX1272_REG_SYNCVALUE6    = 0x2D
	SX1272_REG_SYNCVALUE7    = 0x2E
	SX1272_REG_SYNCVALUE8    = 0x2F
	SX1272_REG_PACKETCONFIG1 = 0x30
	SX1272_REG_PACKETCONFIG2 = 0x31
	SX1272_REG_PAYLOADLENGTH = 0x32
	SX1272_REG_NODEADRS      = 0x33
	SX1272_REG_BROADCASTADRS = 0x34
	SX1272_REG_FIFOTHRESH    = 0x35
	// SM settings
	SX1272_REG_SEQCONFIG1 = 0x36
	SX1272_REG_SEQCONFIG2 = 0x37
	SX1272_REG_TIMERRESOL = 0x38
	SX1272_REG_TIMER1COEF = 0x39
	SX1272_REG_TIMER2COEF = 0x3A
	// Service settings
	SX1272_REG_IMAGECAL = 0x3B
	SX1272_REG_TEMP     = 0x3C
	SX1272_REG_LOWBAT   = 0x3D
	// Status
	SX1272_REG_IRQFLAGS1 = 0x3E
	SX1272_REG_IRQFLAGS2 = 0x3F
	// I/O settings
	SX1272_REG_DIOMAPPING1 = 0x40
	SX1272_REG_DIOMAPPING2 = 0x41
	// Version
	SX1272_REG_VERSION = 0x42
	// Additional settings
	SX1272_REG_AGCREF      = 0x43
	SX1272_REG_AGCTHRESH1  = 0x44
	SX1272_REG_AGCTHRESH2  = 0x45
	SX1272_REG_AGCTHRESH3  = 0x46
	SX1272_REG_PLLHOP      = 0x4B
	SX1272_REG_TCXO        = 0x58
	SX1272_REG_PADAC       = 0x5A
	SX1272_REG_PLL         = 0x5C
	SX1272_REG_PLLLOWPN    = 0x5E
	SX1272_REG_FORMERTEMP  = 0x6C
	SX1272_REG_BITRATEFRAC = 0x70
)

//sx1272_lora
const (
	SX1272_REG_LR_FIFO = 0x00
	// Common settings
	SX1272_REG_LR_OPMODE = 0x01
	SX1272_REG_LR_FRFMSB = 0x06
	SX1272_REG_LR_FRFMID = 0x07
	SX1272_REG_LR_FRFLSB = 0x08
	// Tx settings
	SX1272_REG_LR_PACONFIG = 0x09
	SX1272_REG_LR_PARAMP   = 0x0A
	SX1272_REG_LR_OCP      = 0x0B
	// Rx settings
	SX1272_REG_LR_LNA = 0x0C
	// LoRa registers
	SX1272_REG_LR_FIFOADDRPTR         = 0x0D
	SX1272_REG_LR_FIFOTXBASEADDR      = 0x0E
	SX1272_REG_LR_FIFORXBASEADDR      = 0x0F
	SX1272_REG_LR_FIFORXCURRENTADDR   = 0x10
	SX1272_REG_LR_IRQFLAGSMASK        = 0x11
	SX1272_REG_LR_IRQFLAGS            = 0x12
	SX1272_REG_LR_RXNBBYTES           = 0x13
	SX1272_REG_LR_RXHEADERCNTVALUEMSB = 0x14
	SX1272_REG_LR_RXHEADERCNTVALUELSB = 0x15
	SX1272_REG_LR_RXPACKETCNTVALUEMSB = 0x16
	SX1272_REG_LR_RXPACKETCNTVALUELSB = 0x17
	SX1272_REG_LR_MODEMSTAT           = 0x18
	SX1272_REG_LR_PKTSNRVALUE         = 0x19
	SX1272_REG_LR_PKTRSSIVALUE        = 0x1A
	SX1272_REG_LR_RSSIVALUE           = 0x1B
	SX1272_REG_LR_HOPCHANNEL          = 0x1C
	SX1272_REG_LR_MODEMCONFIG1        = 0x1D
	SX1272_REG_LR_MODEMCONFIG2        = 0x1E
	SX1272_REG_LR_SYMBTIMEOUTLSB      = 0x1F
	SX1272_REG_LR_PREAMBLEMSB         = 0x20
	SX1272_REG_LR_PREAMBLELSB         = 0x21
	SX1272_REG_LR_PAYLOADLENGTH       = 0x22
	SX1272_REG_LR_PAYLOADMAXLENGTH    = 0x23
	SX1272_REG_LR_HOPPERIOD           = 0x24
	SX1272_REG_LR_FIFORXBYTEADDR      = 0x25
	SX1272_REG_LR_FEIMSB              = 0x28
	SX1272_REG_LR_FEIMID              = 0x29
	SX1272_REG_LR_FEILSB              = 0x2A
	SX1272_REG_LR_RSSIWIDEBAND        = 0x2C
	SX1272_REG_LR_DETECTOPTIMIZE      = 0x31
	SX1272_REG_LR_INVERTIQ            = 0x33
	SX1272_REG_LR_DETECTIONTHRESHOLD  = 0x37
	SX1272_REG_LR_SYNCWORD            = 0x39
	SX1272_REG_LR_INVERTIQ2           = 0x3B

	// end of documented register in datasheet
	// I/O settings
	SX1272_REG_LR_DIOMAPPING1 = 0x40
	SX1272_REG_LR_DIOMAPPING2 = 0x41
	// Version
	SX1272_REG_LR_VERSION = 0x42
	// Additional settings
	SX1272_REG_LR_AGCREF     = 0x43
	SX1272_REG_LR_AGCTHRESH1 = 0x44
	SX1272_REG_LR_AGCTHRESH2 = 0x45
	SX1272_REG_LR_AGCTHRESH3 = 0x46
	SX1272_REG_LR_PLLHOP     = 0x4B
	SX1272_REG_LR_TCXO       = 0x58
	SX1272_REG_LR_PADAC      = 0x5A
	SX1272_REG_LR_PLL        = 0x5C
	SX1272_REG_LR_PLLLOWPN   = 0x5E
	SX1272_REG_LR_FORMERTEMP = 0x6C
)

//sc1276_fsk
const (
	/*!
	 * ============================================================================
	 * SX1276 Internal registers Address
	 * ============================================================================
	 */
	SX1276_REG_FIFO = 0x00
	// Common settings
	SX1276_REG_OPMODE     = 0x01
	SX1276_REG_BITRATEMSB = 0x02
	SX1276_REG_BITRATELSB = 0x03
	SX1276_REG_FDEVMSB    = 0x04
	SX1276_REG_FDEVLSB    = 0x05
	SX1276_REG_FRFMSB     = 0x06
	SX1276_REG_FRFMID     = 0x07
	SX1276_REG_FRFLSB     = 0x08
	// Tx settings
	SX1276_REG_PACONFIG = 0x09
	SX1276_REG_PARAMP   = 0x0A
	SX1276_REG_OCP      = 0x0B
	// Rx settings
	SX1276_REG_LNA            = 0x0C
	SX1276_REG_RXCONFIG       = 0x0D
	SX1276_REG_RSSICONFIG     = 0x0E
	SX1276_REG_RSSICOLLISION  = 0x0F
	SX1276_REG_RSSITHRESH     = 0x10
	SX1276_REG_RSSIVALUE      = 0x11
	SX1276_REG_RXBW           = 0x12
	SX1276_REG_AFCBW          = 0x13
	SX1276_REG_OOKPEAK        = 0x14
	SX1276_REG_OOKFIX         = 0x15
	SX1276_REG_OOKAVG         = 0x16
	SX1276_REG_RES17          = 0x17
	SX1276_REG_RES18          = 0x18
	SX1276_REG_RES19          = 0x19
	SX1276_REG_AFCFEI         = 0x1A
	SX1276_REG_AFCMSB         = 0x1B
	SX1276_REG_AFCLSB         = 0x1C
	SX1276_REG_FEIMSB         = 0x1D
	SX1276_REG_FEILSB         = 0x1E
	SX1276_REG_PREAMBLEDETECT = 0x1F
	SX1276_REG_RXTIMEOUT1     = 0x20
	SX1276_REG_RXTIMEOUT2     = 0x21
	SX1276_REG_RXTIMEOUT3     = 0x22
	SX1276_REG_RXDELAY        = 0x23
	// Oscillator settings
	SX1276_REG_OSC = 0x24
	// Packet handler settings
	SX1276_REG_PREAMBLEMSB   = 0x25
	SX1276_REG_PREAMBLELSB   = 0x26
	SX1276_REG_SYNCCONFIG    = 0x27
	SX1276_REG_SYNCVALUE1    = 0x28
	SX1276_REG_SYNCVALUE2    = 0x29
	SX1276_REG_SYNCVALUE3    = 0x2A
	SX1276_REG_SYNCVALUE4    = 0x2B
	SX1276_REG_SYNCVALUE5    = 0x2C
	SX1276_REG_SYNCVALUE6    = 0x2D
	SX1276_REG_SYNCVALUE7    = 0x2E
	SX1276_REG_SYNCVALUE8    = 0x2F
	SX1276_REG_PACKETCONFIG1 = 0x30
	SX1276_REG_PACKETCONFIG2 = 0x31
	SX1276_REG_PAYLOADLENGTH = 0x32
	SX1276_REG_NODEADRS      = 0x33
	SX1276_REG_BROADCASTADRS = 0x34
	SX1276_REG_FIFOTHRESH    = 0x35
	// SM settings
	SX1276_REG_SEQCONFIG1 = 0x36
	SX1276_REG_SEQCONFIG2 = 0x37
	SX1276_REG_TIMERRESOL = 0x38
	SX1276_REG_TIMER1COEF = 0x39
	SX1276_REG_TIMER2COEF = 0x3A
	// Service settings
	SX1276_REG_IMAGECAL = 0x3B
	SX1276_REG_TEMP     = 0x3C
	SX1276_REG_LOWBAT   = 0x3D
	// Status
	SX1276_REG_IRQFLAGS1 = 0x3E
	SX1276_REG_IRQFLAGS2 = 0x3F
	// I/O settings
	SX1276_REG_DIOMAPPING1 = 0x40
	SX1276_REG_DIOMAPPING2 = 0x41
	// Version
	SX1276_REG_VERSION = 0x42
	// Additional settings
	SX1276_REG_PLLHOP      = 0x44
	SX1276_REG_TCXO        = 0x4B
	SX1276_REG_PADAC       = 0x4D
	SX1276_REG_FORMERTEMP  = 0x5B
	SX1276_REG_BITRATEFRAC = 0x5D
	SX1276_REG_AGCREF      = 0x61
	SX1276_REG_AGCTHRESH1  = 0x62
	SX1276_REG_AGCTHRESH2  = 0x63
	SX1276_REG_AGCTHRESH3  = 0x64
	SX1276_REG_PLL         = 0x70
)

//sx1276_lora
const (
	/*!
	 * ============================================================================
	 * SX1276 Internal registers Address
	 * ============================================================================
	 */
	SX1276_REG_LR_FIFO = 0x00
	// Common settings
	SX1276_REG_LR_OPMODE = 0x01
	SX1276_REG_LR_FRFMSB = 0x06
	SX1276_REG_LR_FRFMID = 0x07
	SX1276_REG_LR_FRFLSB = 0x08
	// Tx settings
	SX1276_REG_LR_PACONFIG = 0x09
	SX1276_REG_LR_PARAMP   = 0x0A
	SX1276_REG_LR_OCP      = 0x0B
	// Rx settings
	SX1276_REG_LR_LNA = 0x0C
	// LoRa registers
	SX1276_REG_LR_FIFOADDRPTR         = 0x0D
	SX1276_REG_LR_FIFOTXBASEADDR      = 0x0E
	SX1276_REG_LR_FIFORXBASEADDR      = 0x0F
	SX1276_REG_LR_FIFORXCURRENTADDR   = 0x10
	SX1276_REG_LR_IRQFLAGSMASK        = 0x11
	SX1276_REG_LR_IRQFLAGS            = 0x12
	SX1276_REG_LR_RXNBBYTES           = 0x13
	SX1276_REG_LR_RXHEADERCNTVALUEMSB = 0x14
	SX1276_REG_LR_RXHEADERCNTVALUELSB = 0x15
	SX1276_REG_LR_RXPACKETCNTVALUEMSB = 0x16
	SX1276_REG_LR_RXPACKETCNTVALUELSB = 0x17
	SX1276_REG_LR_MODEMSTAT           = 0x18
	SX1276_REG_LR_PKTSNRVALUE         = 0x19
	SX1276_REG_LR_PKTRSSIVALUE        = 0x1A
	SX1276_REG_LR_RSSIVALUE           = 0x1B
	SX1276_REG_LR_HOPCHANNEL          = 0x1C
	SX1276_REG_LR_MODEMCONFIG1        = 0x1D
	SX1276_REG_LR_MODEMCONFIG2        = 0x1E
	SX1276_REG_LR_SYMBTIMEOUTLSB      = 0x1F
	SX1276_REG_LR_PREAMBLEMSB         = 0x20
	SX1276_REG_LR_PREAMBLELSB         = 0x21
	SX1276_REG_LR_PAYLOADLENGTH       = 0x22
	SX1276_REG_LR_PAYLOADMAXLENGTH    = 0x23
	SX1276_REG_LR_HOPPERIOD           = 0x24
	SX1276_REG_LR_FIFORXBYTEADDR      = 0x25
	SX1276_REG_LR_MODEMCONFIG3        = 0x26
	SX1276_REG_LR_FEIMSB              = 0x28
	SX1276_REG_LR_FEIMID              = 0x29
	SX1276_REG_LR_FEILSB              = 0x2A
	SX1276_REG_LR_RSSIWIDEBAND        = 0x2C
	SX1276_REG_LR_TEST2F              = 0x2F
	SX1276_REG_LR_TEST30              = 0x30
	SX1276_REG_LR_DETECTOPTIMIZE      = 0x31
	SX1276_REG_LR_INVERTIQ            = 0x33
	SX1276_REG_LR_TEST36              = 0x36
	SX1276_REG_LR_DETECTIONTHRESHOLD  = 0x37
	SX1276_REG_LR_SYNCWORD            = 0x39
	SX1276_REG_LR_TEST3A              = 0x3A
	SX1276_REG_LR_INVERTIQ2           = 0x3B

	// end of documented register in datasheet
	// I/O settings
	SX1276_REG_LR_DIOMAPPING1 = 0x40
	SX1276_REG_LR_DIOMAPPING2 = 0x41
	// Version
	SX1276_REG_LR_VERSION = 0x42
	// Additional settings
	SX1276_REG_LR_PLLHOP      = 0x44
	SX1276_REG_LR_TCXO        = 0x4B
	SX1276_REG_LR_PADAC       = 0x4D
	SX1276_REG_LR_FORMERTEMP  = 0x5B
	SX1276_REG_LR_BITRATEFRAC = 0x5D
	SX1276_REG_LR_AGCREF      = 0x61
	SX1276_REG_LR_AGCTHRESH1  = 0x62
	SX1276_REG_LR_AGCTHRESH2  = 0x63
	SX1276_REG_LR_AGCTHRESH3  = 0x64
	SX1276_REG_LR_PLL         = 0x70
)

/**
@struct lgw_radio_FSK_bandwidth_s
@brief Associate a bandwidth in kHz with its corresponding register values
*/
type lgw_sx127x_FSK_bandwidth_s struct {
	RxBwKHz  uint32
	RxBwMant uint8
	RxBwExp  uint8
}

/**
@struct lgw_radio_type_version_s
@brief Associate a radio type with its corresponding expected version value
        read in the radio version register.
*/
type lgw_radio_type_version_s struct {
	typ         lgw_radio_type_e
	reg_version uint8
}

const (
	PLL_LOCK_MAX_ATTEMPTS = 5
)

var sx127x_FskBandwidths = [...]lgw_sx127x_FSK_bandwidth_s{
	{2600, 2, 7}, /* LGW_SX127X_RXBW_2K6_HZ */
	{3100, 1, 7}, /* LGW_SX127X_RXBW_3K1_HZ */
	{3900, 0, 7}, /* ... */
	{5200, 2, 6},
	{6300, 1, 6},
	{7800, 0, 6},
	{10400, 2, 5},
	{12500, 1, 5},
	{15600, 0, 5},
	{20800, 2, 4},
	{25000, 1, 4}, /* ... */
	{31300, 0, 4},
	{41700, 2, 3},
	{50000, 1, 3},
	{62500, 0, 3},
	{83333, 2, 2},
	{100000, 1, 2},
	{125000, 0, 2},
	{166700, 2, 1},
	{200000, 1, 1}, /* ... */
	{250000, 0, 1}, /* LGW_SX127X_RXBW_250K_HZ */
}

func sx125x_write(c *os.File, channel, spi_mux_mode, spi_mux_target byte, addr, data uint8) error {
	var reg_add, reg_dat, reg_cs uint16

	/* checking input parameters */
	if channel >= LGW_RF_CHAIN_NB {
		return fmt.Errorf("ERROR: INVALID RF_CHAIN")
	}
	if addr >= 0x7F {
		return fmt.Errorf("ERROR: ADDRESS OUT OF RANGE")
	}

	/* selecting the target radio */
	switch channel {
	case 0:
		reg_add = LGW_SPI_RADIO_A__ADDR
		reg_dat = LGW_SPI_RADIO_A__DATA
		reg_cs = LGW_SPI_RADIO_A__CS
	case 1:
		reg_add = LGW_SPI_RADIO_B__ADDR
		reg_dat = LGW_SPI_RADIO_B__DATA
		reg_cs = LGW_SPI_RADIO_B__CS
	default:
		return fmt.Errorf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT", channel)
	}

	/* SPI master data write procedure */
	err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_cs, 0)
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_add, int32(0x80|addr)) /* MSB at 1 for write operation */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_dat, int32(data))
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_cs, 1)
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_cs, 0)
	if err != nil {
		return err
	}

	return nil
}

func sx125x_read(c *os.File, spi_mux_mode, spi_mux_target byte, channel, addr byte) (byte, error) {
	var reg_add, reg_dat, reg_cs, reg_rb uint16

	/* checking input parameters */
	if channel >= LGW_RF_CHAIN_NB {
		return 0, fmt.Errorf("ERROR: INVALID RF_CHAIN")
	}
	if addr >= 0x7F {
		return 0, fmt.Errorf("ERROR: ADDRESS OUT OF RANGE")
	}

	/* selecting the target radio */
	switch channel {
	case 0:
		reg_add = LGW_SPI_RADIO_A__ADDR
		reg_dat = LGW_SPI_RADIO_A__DATA
		reg_cs = LGW_SPI_RADIO_A__CS
		reg_rb = LGW_SPI_RADIO_A__DATA_READBACK
		break

	case 1:
		reg_add = LGW_SPI_RADIO_B__ADDR
		reg_dat = LGW_SPI_RADIO_B__DATA
		reg_cs = LGW_SPI_RADIO_B__CS
		reg_rb = LGW_SPI_RADIO_B__DATA_READBACK
		break

	default:
		return 0, fmt.Errorf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT", channel)
	}

	/* SPI master data read procedure */
	err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_cs, 0)
	if err != nil {
		return 0, err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_add, int32(addr)) /* MSB at 0 for read operation */
	if err != nil {
		return 0, err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_dat, 0)
	if err != nil {
		return 0, err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_cs, 1)
	if err != nil {
		return 0, err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_cs, 0)
	if err != nil {
		return 0, err
	}
	read_value, err := Lgw_reg_r(c, spi_mux_mode, spi_mux_target, reg_rb)
	if err != nil {
		return 0, err
	}

	return byte(read_value), nil
}

func setup_sx1272_FSK(f *os.File, frequency uint32, rxbw_khz lgw_sx127x_rxbw_e, rssi_offset int8) error {
	ModulationShaping := byte(0)
	PllHop := byte(1)
	LnaGain := byte(1)
	LnaBoost := byte(3)
	AdcBwAuto := byte(0)
	AdcBw := byte(7)
	AdcLowPwr := byte(0)
	AdcTrim := byte(6)
	AdcTest := byte(0)
	RxBwExp := sx127x_FskBandwidths[rxbw_khz].RxBwExp
	RxBwMant := sx127x_FskBandwidths[rxbw_khz].RxBwMant
	RssiSmoothing := int8(5)

	/* Set in FSK mode */
	err := Lgw_sx127x_reg_w(f, SX1272_REG_OPMODE, 0)
	if err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	err = Lgw_sx127x_reg_w(f, SX1272_REG_OPMODE, 0|(ModulationShaping<<3)) /* Sleep mode, no FSK shaping */
	if err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	err = Lgw_sx127x_reg_w(f, SX1272_REG_OPMODE, 1|(ModulationShaping<<3)) /* Standby mode, no FSK shaping */
	if err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)

	/* Set RF carrier frequency */
	err = Lgw_sx127x_reg_w(f, SX1272_REG_PLLHOP, PllHop<<7)
	if err != nil {
		return err
	}
	freq_reg := uint64(frequency<<19) / 32000000
	err = Lgw_sx127x_reg_w(f, SX1272_REG_FRFMSB, byte(freq_reg>>16))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_FRFMID, byte(freq_reg>>8))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_FRFLSB, byte(freq_reg>>0))
	if err != nil {
		return err
	}

	/* Config */
	err = Lgw_sx127x_reg_w(f, SX1272_REG_LNA, LnaBoost|(LnaGain<<5)) /* Improved sensitivity, highest gain */
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, 0x68, AdcBw|(AdcBwAuto<<3))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, 0x69, AdcTest|(AdcTrim<<4)|(AdcLowPwr<<7))
	if err != nil {
		return err
	}

	/* set BR and FDEV for 200 kHz bandwidth*/
	err = Lgw_sx127x_reg_w(f, SX1272_REG_BITRATEMSB, 125)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_BITRATELSB, 0)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_FDEVMSB, 2)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_FDEVLSB, 225)
	if err != nil {
		return err
	}

	/* Config continues... */
	err = Lgw_sx127x_reg_w(f, SX1272_REG_RXCONFIG, 0) /* Disable AGC */
	if err != nil {
		return err
	}
	RssiOffsetReg := rssi_offset
	if rssi_offset < 0 {
		RssiOffsetReg = (^((-rssi_offset) + 1)) /* 2's complement */
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_RSSICONFIG, byte(RssiSmoothing|(RssiOffsetReg<<3))) /* Set RSSI smoothing to 64 samples, RSSI offset to given value */
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_RXBW, RxBwExp|(RxBwMant<<3))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_RXDELAY, 2)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1272_REG_PLL, 0x10) /* PLL BW set to 75 KHz */
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, 0x47, 1) /* optimize PLL start-up time */
	if err != nil {
		return err
	}

	/* set Rx continuous mode */
	err = Lgw_sx127x_reg_w(f, SX1272_REG_OPMODE, 5|(ModulationShaping<<3)) /* Receiver Mode, no FSK shaping */
	if err != nil {
		return err
	}
	time.Sleep(500 * time.Millisecond)
	reg_val, err := Lgw_sx127x_reg_r(f, SX1272_REG_IRQFLAGS1)
	if err != nil {
		return err
	}
	/* Check if RxReady and ModeReady */
	if (TAKE_N_BITS_FROM(reg_val, 6, 1) == 0) || (TAKE_N_BITS_FROM(reg_val, 7, 1) == 0) {
		return fmt.Errorf("ERROR: SX1272 failed to enter RX continuous mode")
	}
	time.Sleep(500 * time.Millisecond)

	log.Printf("INFO: Successfully configured SX1272 for FSK modulation (rxbw=%d)\n", rxbw_khz)
	return nil
}

func setup_sx1276_FSK(f *os.File, frequency uint32, rxbw_khz lgw_sx127x_rxbw_e, rssi_offset int8) error {
	ModulationShaping := byte(0)
	PllHop := byte(1)
	LnaGain := byte(1)
	LnaBoost := byte(3)
	AdcBwAuto := byte(0)
	AdcBw := byte(7)
	AdcLowPwr := byte(0)
	AdcTrim := byte(6)
	AdcTest := byte(0)
	RxBwExp := sx127x_FskBandwidths[rxbw_khz].RxBwExp
	RxBwMant := sx127x_FskBandwidths[rxbw_khz].RxBwMant
	RssiSmoothing := int8(5)

	/* Set in FSK mode */
	err := Lgw_sx127x_reg_w(f, SX1276_REG_OPMODE, 0)
	if err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	err = Lgw_sx127x_reg_w(f, SX1276_REG_OPMODE, 0|(ModulationShaping<<3)) /* Sleep mode, no FSK shaping */
	if err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	err = Lgw_sx127x_reg_w(f, SX1276_REG_OPMODE, 1|(ModulationShaping<<3)) /* Standby mode, no FSK shaping */
	if err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)

	/* Set RF carrier frequency */
	err = Lgw_sx127x_reg_w(f, SX1276_REG_PLLHOP, PllHop<<7)
	if err != nil {
		return err
	}
	freq_reg := uint64(frequency<<19) / 32000000
	err = Lgw_sx127x_reg_w(f, SX1276_REG_FRFMSB, byte(freq_reg>>16))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_FRFMID, byte(freq_reg>>8))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_FRFLSB, byte(freq_reg>>0))
	if err != nil {
		return err
	}

	/* Config */
	err = Lgw_sx127x_reg_w(f, SX1276_REG_LNA, LnaBoost|(LnaGain<<5)) /* Improved sensitivity, highest gain */
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, 0x57, AdcBw|(AdcBwAuto<<3))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, 0x58, AdcTest|(AdcTrim<<4)|(AdcLowPwr<<7))
	if err != nil {
		return err
	}

	/* set BR and FDEV for 200 kHz bandwidth*/
	err = Lgw_sx127x_reg_w(f, SX1276_REG_BITRATEMSB, 125)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_BITRATELSB, 0)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_FDEVMSB, 2)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_FDEVLSB, 225)
	if err != nil {
		return err
	}

	/* Config continues... */
	err = Lgw_sx127x_reg_w(f, SX1276_REG_RXCONFIG, 0) /* Disable AGC */
	if err != nil {
		return err
	}
	RssiOffsetReg := rssi_offset
	if rssi_offset < 0 {
		RssiOffsetReg = (^((-rssi_offset) + 1)) /* 2's complement */
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_RSSICONFIG, byte(RssiSmoothing|(RssiOffsetReg<<3))) /* Set RSSI smoothing to 64 samples, RSSI offset to given value */
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_RXBW, RxBwExp|(RxBwMant<<3))
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_RXDELAY, 2)
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, SX1276_REG_PLL, 0x10) /* PLL BW set to 75 KHz */
	if err != nil {
		return err
	}
	err = Lgw_sx127x_reg_w(f, 0x43, 1) /* optimize PLL start-up time */
	if err != nil {
		return err
	}

	/* set Rx continuous mode */
	err = Lgw_sx127x_reg_w(f, SX1276_REG_OPMODE, 5|(ModulationShaping<<3)) /* Receiver Mode, no FSK shaping */
	if err != nil {
		return err
	}
	time.Sleep(500 * time.Millisecond)
	reg_val, err := Lgw_sx127x_reg_r(f, SX1276_REG_IRQFLAGS1)
	if err != nil {
		return err
	}
	/* Check if RxReady and ModeReady */
	if (TAKE_N_BITS_FROM(reg_val, 6, 1) == 0) || (TAKE_N_BITS_FROM(reg_val, 7, 1) == 0) {
		return fmt.Errorf("ERROR: SX1276 failed to enter RX continuous mode")
	}
	time.Sleep(500 * time.Millisecond)

	log.Printf("INFO: Successfully configured SX1276 for FSK modulation (rxbw=%d)\n", rxbw_khz)
	return nil
}

func reset_sx127x(f *os.File, radio_type lgw_radio_type_e) error {
	switch radio_type {
	case LGW_RADIO_TYPE_SX1276:
		err := Lgw_fpga_reg_w(f, LGW_FPGA_CTRL_RADIO_RESET, 0)
		if err != nil {
			return err
		}
		err = Lgw_fpga_reg_w(f, LGW_FPGA_CTRL_RADIO_RESET, 1)
		if err != nil {
			return err
		}
	case LGW_RADIO_TYPE_SX1272:
		err := Lgw_fpga_reg_w(f, LGW_FPGA_CTRL_RADIO_RESET, 1)
		if err != nil {
			return err
		}
		err = Lgw_fpga_reg_w(f, LGW_FPGA_CTRL_RADIO_RESET, 0)
		if err != nil {
			return err
		}
	default:
		return fmt.Errorf("ERROR: Failed to reset sx127x, not supported (%d)", radio_type)
	}
	return nil
}

func Lgw_setup_sx125x(c *os.File, lgw_spi_mux_mode, spi_mux_target, rf_chain, rf_clkout byte, rf_enable bool, rf_radio_type lgw_radio_type_e, freq_hz uint32) error {
	if rf_chain >= LGW_RF_CHAIN_NB {
		return fmt.Errorf("ERROR: INVALID RF_CHAIN")
	}

	/* Get version to identify SX1255/57 silicon revision */
	b, err := sx125x_read(c, lgw_spi_mux_mode, spi_mux_target, rf_chain, 0x07)
	if err != nil {
		return err
	}
	fmt.Print("Note: SX125x #%d version register returned 0x%02X\n", rf_chain, b)

	/* General radio setup */
	if rf_clkout == rf_chain {
		err := sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x10, uint8(SX125x_TX_DAC_CLK_SEL+2))
		if err != nil {
			return err
		}
	} else {
		err := sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x10, uint8(SX125x_TX_DAC_CLK_SEL))
		if err != nil {
			return err
		}
	}

	switch rf_radio_type {
	case LGW_RADIO_TYPE_SX1255:
		err := sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x28, uint8(SX125x_XOSC_GM_STARTUP+SX125x_XOSC_DISABLE*16))
		if err != nil {
			return err
		}
	case LGW_RADIO_TYPE_SX1257:
		err := sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x26, uint8(SX125x_XOSC_GM_STARTUP+SX125x_XOSC_DISABLE*16))
		if err != nil {
			return err
		}
	default:
		return fmt.Errorf("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE", rf_radio_type)
	}
	if rf_enable {

		err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x08, uint8(SX125x_TX_MIX_GAIN+SX125x_TX_DAC_GAIN*16))
		if err != nil {
			return err
		}
		err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x0A, uint8(SX125x_TX_ANA_BW+SX125x_TX_PLL_BW*32))
		if err != nil {
			return err
		}
		err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x0B, uint8(SX125x_TX_DAC_BW))
		if err != nil {
			return err
		}

		/* Rx gain and trim */
		err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x0C, uint8(SX125x_LNA_ZIN+SX125x_RX_BB_GAIN*2+SX125x_RX_LNA_GAIN*32))
		if err != nil {
			return err
		}
		err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x0D, uint8(SX125x_RX_BB_BW+SX125x_RX_ADC_TRIM*4+SX125x_RX_ADC_BW*32))
		if err != nil {
			return err
		}
		err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x0E, uint8(SX125x_ADC_TEMP+SX125x_RX_PLL_BW*2))
		if err != nil {
			return err
		}

		/* set RX PLL frequency */
		switch rf_radio_type {
		case LGW_RADIO_TYPE_SX1255:
			part_int := freq_hz / (SX125x_32MHz_FRAC << 7)                               /* integer part, gives the MSB */
			part_frac := ((freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC /* fractional part, gives middle part and LSB */

			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x01, 0xFF&uint8(part_int)) /* Most Significant Byte */
			if err != nil {
				return err
			}
			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x02, 0xFF&uint8(part_frac>>8)) /* middle byte */
			if err != nil {
				return err
			}
			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x03, 0xFF&uint8(part_frac)) /* Least Significant Byte */
			if err != nil {
				return err
			}
		case LGW_RADIO_TYPE_SX1257:
			part_int := freq_hz / (SX125x_32MHz_FRAC << 8)                                                /* integer part, gives the MSB */
			part_frac := ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC                  /* fractional part, gives middle part and LSB */
			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x01, 0xFF&uint8(part_int)) /* Most Significant Byte */
			if err != nil {
				return err
			}
			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x02, 0xFF&uint8(part_frac>>8)) /* middle byte */
			if err != nil {
				return err
			}
			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x03, 0xFF&uint8(part_frac)) /* Least Significant Byte */
			if err != nil {
				return err
			}
		default:
			return fmt.Errorf("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE", rf_radio_type)
		}
		/* start and PLL lock */
		for cpt_attempts := 0; cpt_attempts < PLL_LOCK_MAX_ATTEMPTS; cpt_attempts++ {
			if cpt_attempts >= PLL_LOCK_MAX_ATTEMPTS {
				return fmt.Errorf("ERROR: FAIL TO LOCK PLL")
			}
			err := sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x00, 1) /* enable Xtal oscillator */
			if err != nil {
				return err
			}
			err = sx125x_write(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x00, 3) /* Enable RX (PLL+FE) */
			if err != nil {
				return err
			}
			time.Sleep(1 * time.Millisecond)
			val, err := sx125x_read(c, rf_chain, lgw_spi_mux_mode, spi_mux_target, 0x11)
			if err != nil {
				return err
			}
			if (val & 0x02) != 0 {
				return err
			}
		}
	} else {
		log.Printf("Note: SX125x #%d kept in standby mode\n", rf_chain)
	}

	return nil
}

func Lgw_sx127x_reg_w(f *os.File, address byte, reg_value byte) error {
	return Lgw_spi_w(f, LGW_SPI_MUX_MODE1, LGW_SPI_MUX_TARGET_SX127X, address, reg_value)
}

func Lgw_sx127x_reg_r(f *os.File, address byte) (byte, error) {
	return Lgw_spi_r(f, LGW_SPI_MUX_MODE1, LGW_SPI_MUX_TARGET_SX127X, address)
}

func Lgw_setup_sx127x(f *os.File, frequency uint32, modulation byte, rxbw_khz lgw_sx127x_rxbw_e, rssi_offset int8) error {
	radio_type := LGW_RADIO_TYPE_NONE
	supported_radio_type := [2]lgw_radio_type_version_s{
		{LGW_RADIO_TYPE_SX1272, 0x22},
		{LGW_RADIO_TYPE_SX1276, 0x12},
	}

	/* Check parameters */
	if modulation != MOD_FSK {
		return fmt.Errorf("ERROR: modulation not supported for SX127x (%d)", modulation)
	}
	if rxbw_khz > LGW_SX127X_RXBW_250K_HZ {
		return fmt.Errorf("ERROR: RX bandwidth not supported for SX127x (%d)", rxbw_khz)
	}

	/* Probing radio type */
	for i := 0; i < len(supported_radio_type); i++ {
		/* Reset the radio */
		err := reset_sx127x(f, supported_radio_type[i].typ)
		if err != nil {
			return fmt.Errorf("ERROR: Failed to reset sx127x")
		}
		/* Read version register */
		version, err := Lgw_sx127x_reg_r(f, 0x42)
		if err != nil {
			return fmt.Errorf("ERROR: Failed to read sx127x version register")
		}
		/* Check if we got the expected version */
		if version != supported_radio_type[i].reg_version {
			log.Printf("INFO: sx127x version register - read:0x%X, expected:0x%X\n", version, supported_radio_type[i].reg_version)
			continue
		} else {
			log.Printf("INFO: sx127x radio has been found (type:%d, version:0x%X)\n", supported_radio_type[i].typ, version)
			radio_type = supported_radio_type[i].typ
			break
		}
	}
	if radio_type == LGW_RADIO_TYPE_NONE {
		return fmt.Errorf("ERROR: sx127x radio has not been found")
	}

	/* Setup the radio */
	switch modulation {
	case MOD_FSK:
		if radio_type == LGW_RADIO_TYPE_SX1272 {
			err := setup_sx1272_FSK(f, frequency, rxbw_khz, rssi_offset)
			if err != nil {
				return err
			}
		} else {
			err := setup_sx1276_FSK(f, frequency, rxbw_khz, rssi_offset)
			if err != nil {
				return err
			}
		}
		break
	default:
		/* Should not happen */
		break
	}
	return nil
}
