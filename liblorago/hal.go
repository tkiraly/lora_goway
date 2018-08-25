package liblorago

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"math"
	"os"
	"reflect"
	"time"
)

func IS_LORA_BW(bw byte) bool { return ((bw == BW_125KHZ) || (bw == BW_250KHZ) || (bw == BW_500KHZ)) }
func IS_LORA_STD_DR(dr byte) bool {
	return ((dr == DR_LORA_SF7) || (dr == DR_LORA_SF8) || (dr == DR_LORA_SF9) || (dr == DR_LORA_SF10) || (dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))
}
func IS_LORA_MULTI_DR(dr byte) bool {
	return ((dr & ^DR_LORA_MULTI) == 0) /* ones outside of DR_LORA_MULTI bitmask -> not a combination of LoRa datarates */
}
func IS_LORA_CR(cr byte) bool {
	return ((cr == CR_LORA_4_5) || (cr == CR_LORA_4_6) || (cr == CR_LORA_4_7) || (cr == CR_LORA_4_8))
}

func IS_FSK_BW(bw byte) bool { return ((bw >= 1) && (bw <= 7)) }
func IS_FSK_DR(dr int) bool  { return ((dr >= DR_FSK_MIN) && (dr <= DR_FSK_MAX)) }

func IS_TX_MODE(mode byte) bool {
	return ((mode == IMMEDIATE) || (mode == TIMESTAMPED) || (mode == ON_GPS))
}

const (
	LGW_LBT_ISSUE = 1
	/* radio-specific parameters */
	LGW_XTAL_FREQU  = 32000000 /* frequency of the RF reference oscillator */
	LGW_RF_CHAIN_NB = 2        /* number of RF chains */

	/* type of if_chain + modem */
	IF_UNDEFINED  = 0
	IF_LORA_STD   = 0x10 /* if + standard single-SF LoRa modem */
	IF_LORA_MULTI = 0x11 /* if + LoRa receiver with multi-SF capability */
	IF_FSK_STD    = 0x20 /* if + standard FSK modem */

	/* concentrator chipset-specific parameters */
	/* to use array parameters, declare a local const and use 'if_chain' as index */
	LGW_IF_CHAIN_NB   = 10     /* number of IF+modem RX chains */
	LGW_PKT_FIFO_SIZE = 16     /* depth of the RX packet FIFO */
	LGW_DATABUFF_SIZE = 1024   /* size in bytes of the RX data buffer (contains payload & metadata) */
	LGW_REF_BW        = 125000 /* typical bandwidth of data channel */
	LGW_MULTI_NB      = 8      /* number of LoRa 'multi SF' chains */

	/* values available for the 'modulation' parameters */
	/* NOTE: arbitrary values */
	MOD_UNDEFINED byte = 0
	MOD_LORA      byte = 0x10
	MOD_FSK       byte = 0x20

	/* values available for the 'bandwidth' parameters (LoRa & FSK) */
	/* NOTE: directly encode FSK RX bandwidth, do not change */
	BW_UNDEFINED byte = 0
	BW_500KHZ    byte = 0x01
	BW_250KHZ    byte = 0x02
	BW_125KHZ    byte = 0x03
	BW_62K5HZ    byte = 0x04
	BW_31K2HZ    byte = 0x05
	BW_15K6HZ    byte = 0x06
	BW_7K8HZ     byte = 0x07

	/* values available for the 'datarate' parameters */
	/* NOTE: LoRa values used directly to code SF bitmask in 'multi' modem, do not change */
	DR_UNDEFINED  byte = 0
	DR_LORA_SF7   byte = 0x02
	DR_LORA_SF8   byte = 0x04
	DR_LORA_SF9   byte = 0x08
	DR_LORA_SF10  byte = 0x10
	DR_LORA_SF11  byte = 0x20
	DR_LORA_SF12  byte = 0x40
	DR_LORA_MULTI byte = 0x7E
	/* NOTE: for FSK directly use baudrate between 500 bauds and 250 kbauds */
	DR_FSK_MIN = 500
	DR_FSK_MAX = 250000

	/* values available for the 'coderate' parameters (LoRa only) */
	/* NOTE: arbitrary values */
	CR_UNDEFINED byte = 0
	CR_LORA_4_5  byte = 0x01
	CR_LORA_4_6  byte = 0x02
	CR_LORA_4_7  byte = 0x03
	CR_LORA_4_8  byte = 0x04

	/* values available for the 'status' parameter */
	/* NOTE: values according to hardware specification */
	STAT_UNDEFINED = 0x00
	STAT_NO_CRC    = 0x01
	STAT_CRC_BAD   = 0x11
	STAT_CRC_OK    = 0x10

	/* values available for the 'tx_mode' parameter */
	IMMEDIATE   = 0
	TIMESTAMPED = 1
	ON_GPS      = 2
	// ON_EVENT      =3
	// GPS_DELAYED   =4
	// EVENT_DELAYED =5

	/* values available for 'select' in the status function */
	TX_STATUS = 1
	RX_STATUS = 2

	/* status code for TX_STATUS */
	/* NOTE: arbitrary values */
	TX_STATUS_UNKNOWN = 0
	TX_OFF            = 1 /* TX modem disabled, it will ignore commands */
	TX_FREE           = 2 /* TX modem is free, ready to receive a command */
	TX_SCHEDULED      = 3 /* TX modem is loaded, ready to send the packet after an event and/or delay */
	TX_EMITTING       = 4 /* TX modem is emitting */

	/* status code for RX_STATUS */
	/* NOTE: arbitrary values */
	RX_STATUS_UNKNOWN = 0
	RX_OFF            = 1 /* RX modem is disabled, it will ignore commands  */
	RX_ON             = 2 /* RX modem is receiving */
	RX_SUSPENDED      = 3 /* RX is suspended while a TX is ongoing */

	/* Maximum size of Tx gain LUT */
	TX_GAIN_LUT_SIZE_MAX = 16

	/* LBT constants */
	LBT_CHANNEL_FREQ_NB = 8 /* Number of LBT channels */
)

var LGW_RF_RX_BANDWIDTH = [...]int{1000000, 1000000} /* bandwidth of the radios */

var LGW_IFMODEM_CONFIG = [...]byte{
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_MULTI,
	IF_LORA_STD,
	IF_FSK_STD,
} /* configuration of available IF chains and modems on the hardware */

type lgw_radio_type_e byte

const (
	LGW_RADIO_TYPE_NONE lgw_radio_type_e = iota
	LGW_RADIO_TYPE_SX1255
	LGW_RADIO_TYPE_SX1257
	LGW_RADIO_TYPE_SX1272
	LGW_RADIO_TYPE_SX1276
)

/**
@struct lgw_pkt_rx_s
@brief Structure containing the metadata of a packet that was received and a pointer to the payload
*/
type Lgw_pkt_rx_s struct {
	Freq_hz    uint32  /*!> central frequency of the IF chain */
	If_chain   byte    /*!> by which IF chain was packet received */
	Status     byte    /*!> status of the received packet */
	Count_us   uint32  /*!> internal concentrator counter for timestamping, 1 microsecond resolution */
	Rf_chain   byte    /*!> through which RF chain the packet was received */
	Modulation byte    /*!> modulation used by the packet */
	Bandwidth  byte    /*!> modulation bandwidth (LoRa only) */
	Datarate   uint32  /*!> RX datarate of the packet (SF for LoRa) */
	Coderate   byte    /*!> error-correcting code of the packet (LoRa only) */
	Rssi       float64 /*!> average packet RSSI in dB */
	Snr        float64 /*!> average packet SNR, in dB (LoRa only) */
	Snr_min    float64 /*!> minimum packet SNR, in dB (LoRa only) */
	Snr_max    float64 /*!> maximum packet SNR, in dB (LoRa only) */
	Crc        uint16  /*!> CRC that was received in the payload */
	Size       uint16  /*!> payload size in bytes */
	Payload    []byte  /*!> buffer containing the payload */
}

/**
@struct lgw_pkt_tx_s
@brief Structure containing the configuration of a packet to send and a pointer to the payload
*/
type Lgw_pkt_tx_s struct {
	freq_hz    uint32    /*!> center frequency of TX */
	tx_mode    byte      /*!> select on what event/time the TX is triggered */
	count_us   uint32    /*!> timestamp or delay in microseconds for TX trigger */
	rf_chain   byte      /*!> through which RF chain will the packet be sent */
	rf_power   int8      /*!> TX power, in dBm */
	modulation byte      /*!> modulation to use for the packet */
	bandwidth  byte      /*!> modulation bandwidth (LoRa only) */
	datarate   uint32    /*!> TX datarate (baudrate for FSK, SF for LoRa) */
	coderate   byte      /*!> error-correcting code of the packet (LoRa only) */
	invert_pol bool      /*!> invert signal polarity, for orthogonal downlinks (LoRa only) */
	f_dev      byte      /*!> frequency deviation, in kHz (FSK only) */
	preamble   uint16    /*!> set the preamble length, 0 for default */
	no_crc     bool      /*!> if true, do not send a CRC in the packet */
	no_header  bool      /*!> if true, enable implicit header mode (LoRa), fixed length (FSK) */
	size       uint16    /*!> payload size in bytes */
	payload    [256]byte /*!> buffer containing the payload */
}

/**
@struct lgw_tx_gain_s
@brief Structure containing all gains of Tx chain
*/
type Lgw_tx_gain_s struct {
	dig_gain byte /*!> 2 bits, control of the digital gain of SX1301 */
	pa_gain  byte /*!> 2 bits, control of the external PA (SX1301 I/O) */
	dac_gain byte /*!> 2 bits, control of the radio DAC */
	mix_gain byte /*!> 4 bits, control of the radio mixer */
	rf_power int8 /*!> measured TX power at the board connector, in dBm */
}

/**
@struct lgw_tx_gain_lut_s
@brief Structure defining the Tx gain LUT
*/
type Lgw_tx_gain_lut_s struct {
	lut  [TX_GAIN_LUT_SIZE_MAX]Lgw_tx_gain_s /*!> Array of Tx gain struct */
	size byte                                /*!> Number of LUT indexes */
}

func SET_PPM_ON(bw, dr byte) bool {
	return (((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))
}
func IF_HZ_TO_REG(f int32) int32 { return (f << 5) / 15625 }

const (
	MCU_ARB         = 0
	MCU_AGC         = 1
	MCU_ARB_FW_BYTE = 8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
	MCU_AGC_FW_BYTE = 8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
	FW_VERSION_ADDR = 0x20 /* Address of firmware version in data memory */
	FW_VERSION_CAL  = 2    /* Expected version of calibration firmware */
	FW_VERSION_AGC  = 4    /* Expected version of AGC firmware */
	FW_VERSION_ARB  = 1    /* Expected version of arbiter firmware */

	TX_METADATA_NB = 16
	RX_METADATA_NB = 16

	AGC_CMD_WAIT  = 16
	AGC_CMD_ABORT = 17

	MIN_LORA_PREAMBLE = 6
	STD_LORA_PREAMBLE = 8
	MIN_FSK_PREAMBLE  = 3
	STD_FSK_PREAMBLE  = 5

	RSSI_MULTI_BIAS = -35 /* difference between "multi" modem RSSI offset and "stand-alone" modem RSSI offset */
	RSSI_FSK_POLY_0 = 60  /* polynomiam coefficients to linearize FSK RSSI */
	RSSI_FSK_POLY_1 = 1.5351
	RSSI_FSK_POLY_2 = 0.003

	/* Useful bandwidth of SX125x radios to consider depending on channel bandwidth */
	/* Note: the below values come from lab measurements. For any question, please contact Semtech support */
	LGW_RF_RX_BANDWIDTH_125KHZ = 925000  /* for 125KHz channels */
	LGW_RF_RX_BANDWIDTH_250KHZ = 1000000 /* for 250KHz channels */
	LGW_RF_RX_BANDWIDTH_500KHZ = 1100000 /* for 500KHz channels */

	TX_START_DELAY_DEFAULT = 1497 /* Calibrated value for 500KHz BW and notch filter disabled */
)

var ifmod_config = LGW_IFMODEM_CONFIG

func load_firmware(c *os.File, target int, spi_mux_mode, spi_mux_target byte, firmware []byte) error {
	var reg_rst uint16
	var reg_sel uint16

	if target == MCU_ARB {
		if len(firmware) != MCU_ARB_FW_BYTE {
			return fmt.Errorf("ERROR: NOT A VALID SIZE FOR MCU ARG FIRMWARE")
		}
		reg_rst = LGW_MCU_RST_0
		reg_sel = LGW_MCU_SELECT_MUX_0
	} else if target == MCU_AGC {
		if len(firmware) != MCU_AGC_FW_BYTE {
			return fmt.Errorf("ERROR: NOT A VALID SIZE FOR MCU AGC FIRMWARE")
		}
		reg_rst = LGW_MCU_RST_1
		reg_sel = LGW_MCU_SELECT_MUX_1
	} else {
		return fmt.Errorf("ERROR: NOT A VALID TARGET FOR LOADING FIRMWARE")
	}

	/* reset the targeted MCU */
	err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_rst, 1)
	if err != nil {
		return err
	}

	/* set mux to access MCU program RAM and set address to 0 */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_sel, 0)
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_MCU_PROM_ADDR, 0)
	if err != nil {
		return err
	}

	/* write the program in one burst */
	err = Lgw_reg_wb(c, spi_mux_mode, spi_mux_target, LGW_MCU_PROM_DATA, firmware)
	if err != nil {
		return err
	}
	/* Read back firmware code for check */
	_, err = Lgw_reg_r(c, spi_mux_mode, spi_mux_target, LGW_MCU_PROM_DATA) /* bug workaround */
	if err != nil {
		return err
	}

	fw_check, err := Lgw_reg_rb(c, spi_mux_mode, spi_mux_target, LGW_MCU_PROM_DATA, uint16(len(firmware)))
	if err != nil {
		return err
	}

	if reflect.DeepEqual(firmware, fw_check) != true {
		return fmt.Errorf("ERROR: Failed to load fw %d", target)
	}

	/* give back control of the MCU program ram to the MCU */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, reg_sel, 1)
	if err != nil {
		return err
	}

	return nil
}

func lgw_constant_adjust(c *os.File, spi_mux_mode, spi_mux_target byte, s *State) error {

	/* I/Q path setup */
	// Lgw_reg_w(LGW_RX_INVERT_IQ,0); /* default 0 */
	// Lgw_reg_w(LGW_MODEM_INVERT_IQ,1); /* default 1 */
	// Lgw_reg_w(LGW_CHIRP_INVERT_RX,1); /* default 1 */
	// Lgw_reg_w(LGW_RX_EDGE_SELECT,0); /* default 0 */
	// Lgw_reg_w(LGW_MBWSSF_MODEM_INVERT_IQ,0); /* default 0 */
	// Lgw_reg_w(LGW_DC_NOTCH_EN,1); /* default 1 */
	err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RSSI_BB_FILTER_ALPHA, 6) /* default 7 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RSSI_DEC_FILTER_ALPHA, 7) /* default 5 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RSSI_CHANN_FILTER_ALPHA, 7) /* default 8 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RSSI_BB_DEFAULT_VALUE, 23) /* default 32 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RSSI_CHANN_DEFAULT_VALUE, 85) /* default 100 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RSSI_DEC_DEFAULT_VALUE, 66) /* default 100 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_DEC_GAIN_OFFSET, 7) /* default 8 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_CHAN_GAIN_OFFSET, 6) /* default 7 */
	if err != nil {
		return err
	}

	/* Correlator setup */
	// Lgw_reg_w(LGW_CORR_DETECT_EN,126); /* default 126 */
	// Lgw_reg_w(LGW_CORR_NUM_SAME_PEAK,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_MAC_GAIN,5); /* default 5 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF6,0); /* default 0 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF7,1); /* default 1 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF8,1); /* default 1 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF9,1); /* default 1 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF10,1); /* default 1 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF11,1); /* default 1 */
	// Lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF12,1); /* default 1 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF6,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF7,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF8,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF9,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF10,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF11,4); /* default 4 */
	// Lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF12,4); /* default 4 */

	/* LoRa 'multi' demodulators setup */
	// Lgw_reg_w(LGW_PREAMBLE_SYMB1_NB,10); /* default 10 */
	// Lgw_reg_w(LGW_FREQ_TO_TIME_INVERT,29); /* default 29 */
	// Lgw_reg_w(LGW_FRAME_SYNCH_GAIN,1); /* default 1 */
	// Lgw_reg_w(LGW_SYNCH_DETECT_TH,1); /* default 1 */
	// Lgw_reg_w(LGW_ZERO_PAD,0); /* default 0 */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_SNR_AVG_CST, 3) /* default 2 */
	if err != nil {
		return err
	}
	if s.lorawan_public { /* LoRa network */
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FRAME_SYNCH_PEAK1_POS, 3) /* default 1 */
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FRAME_SYNCH_PEAK2_POS, 4) /* default 2 */
		if err != nil {
			return err
		}
	} else { /* private network */
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FRAME_SYNCH_PEAK1_POS, 1) /* default 1 */
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FRAME_SYNCH_PEAK2_POS, 2) /* default 2 */
		if err != nil {
			return err
		}
	}

	// Lgw_reg_w(LGW_PREAMBLE_FINE_TIMING_GAIN,1); /* default 1 */
	// Lgw_reg_w(LGW_ONLY_CRC_EN,1); /* default 1 */
	// Lgw_reg_w(LGW_PAYLOAD_FINE_TIMING_GAIN,2); /* default 2 */
	// Lgw_reg_w(LGW_TRACKING_INTEGRAL,0); /* default 0 */
	// Lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_RDX8,0); /* default 0 */
	// Lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_SF12_RDX4,4092); /* default 4092 */
	// Lgw_reg_w(LGW_MAX_PAYLOAD_LEN,255); /* default 255 */

	/* LoRa standalone 'MBWSSF' demodulator setup */
	// Lgw_reg_w(LGW_MBWSSF_PREAMBLE_SYMB1_NB,10); /* default 10 */
	// Lgw_reg_w(LGW_MBWSSF_FREQ_TO_TIME_INVERT,29); /* default 29 */
	// Lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_GAIN,1); /* default 1 */
	// Lgw_reg_w(LGW_MBWSSF_SYNCH_DETECT_TH,1); /* default 1 */
	// Lgw_reg_w(LGW_MBWSSF_ZERO_PAD,0); /* default 0 */
	if s.lorawan_public { /* LoRa network */
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS, 3) /* default 1 */
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS, 4) /* default 2 */
		if err != nil {
			return err
		}
	} else {
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS, 1) /* default 1 */
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS, 2) /* default 2 */
		if err != nil {
			return err
		}
	}
	// Lgw_reg_w(LGW_MBWSSF_ONLY_CRC_EN,1); /* default 1 */
	// Lgw_reg_w(LGW_MBWSSF_PAYLOAD_FINE_TIMING_GAIN,2); /* default 2 */
	// Lgw_reg_w(LGW_MBWSSF_PREAMBLE_FINE_TIMING_GAIN,1); /* default 1 */
	// Lgw_reg_w(LGW_MBWSSF_TRACKING_INTEGRAL,0); /* default 0 */
	// Lgw_reg_w(LGW_MBWSSF_AGC_FREEZE_ON_DETECT,1); /* default 1 */

	/* Improvement of reference clock frequency error tolerance */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_ADJUST_MODEM_START_OFFSET_RDX4, 1) /* default 0 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_ADJUST_MODEM_START_OFFSET_SF12_RDX4, 4094) /* default 4092 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_CORR_MAC_GAIN, 7) /* default 5 */
	if err != nil {
		return err
	}

	/* FSK datapath setup */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_RX_INVERT, 1) /* default 0 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_MODEM_INVERT_IQ, 1) /* default 0 */
	if err != nil {
		return err
	}

	/* FSK demodulator setup */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_RSSI_LENGTH, 4) /* default 0 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_PKT_MODE, 1) /* variable length, default 0 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_CRC_EN, 1) /* default 0 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_DCFREE_ENC, 2) /* default 0 */
	if err != nil {
		return err
	}
	// Lgw_reg_w(LGW_FSK_CRC_IBM,0); /* default 0 */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_ERROR_OSR_TOL, 10) /* default 0 */
	if err != nil {
		return err
	}
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_PKT_LENGTH, 255) /* max packet length in variable length mode */
	if err != nil {
		return err
	}
	// Lgw_reg_w(LGW_FSK_NODE_ADRS,0); /* default 0 */
	// Lgw_reg_w(LGW_FSK_BROADCAST,0); /* default 0 */
	// Lgw_reg_w(LGW_FSK_AUTO_AFC_ON,0); /* default 0 */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_PATTERN_TIMEOUT_CFG, 128) /* sync timeout (allow 8 bytes preamble + 8 bytes sync word, default 0 */
	if err != nil {
		return err
	}

	/* TX general parameters */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_START_DELAY, TX_START_DELAY_DEFAULT) /* default 0 */
	if err != nil {
		return err
	}

	/* TX LoRa */
	// Lgw_reg_w(LGW_TX_MODE,0); /* default 0 */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_SWAP_IQ, 1) /* "normal" polarity; default 0 */
	if err != nil {
		return err
	}
	if s.lorawan_public { /* LoRa network */
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_FRAME_SYNCH_PEAK1_POS, 3) /* default 1 */
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_FRAME_SYNCH_PEAK2_POS, 4) /* default 2 */
		if err != nil {
			return err
		}
	} else { /* Private network */
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_FRAME_SYNCH_PEAK1_POS, 1) /* default 1 */
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_FRAME_SYNCH_PEAK2_POS, 2) /* default 2 */
		if err != nil {
			return err
		}
	}

	/* TX FSK */
	// Lgw_reg_w(LGW_FSK_TX_GAUSSIAN_EN,1); /* default 1 */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_FSK_TX_GAUSSIAN_SELECT_BT, 2) /* Gaussian filter always on TX, default 0 */
	if err != nil {
		return err
	}
	// Lgw_reg_w(LGW_FSK_TX_PATTERN_EN,1); /* default 1 */
	// Lgw_reg_w(LGW_FSK_TX_PREAMBLE_SEQ,0); /* default 0 */

	return nil
}

func lgw_bw_getval(x byte) int32 {
	switch x {
	case BW_500KHZ:
		return 500000
	case BW_250KHZ:
		return 250000
	case BW_125KHZ:
		return 125000
	case BW_62K5HZ:
		return 62500
	case BW_31K2HZ:
		return 31200
	case BW_15K6HZ:
		return 15600
	case BW_7K8HZ:
		return 7800
	default:
		return -1
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

func lgw_sf_getval(x byte) int32 {
	switch x {
	case DR_LORA_SF7:
		return 7
	case DR_LORA_SF8:
		return 8
	case DR_LORA_SF9:
		return 9
	case DR_LORA_SF10:
		return 10
	case DR_LORA_SF11:
		return 11
	case DR_LORA_SF12:
		return 12
	default:
		return -1
	}
}

func lgw_get_tx_start_delay(tx_notch_support, tx_notch_enable bool, tx_notch_offset byte, bw uint8) uint16 {
	bw_delay_us := float64(0)
	notch_delay_us := float64(0)
	/* Notch filtering performed by FPGA adds a constant delay (group delay) that we need to compensate */
	if tx_notch_enable {
		notch_delay_us = lgw_fpga_get_tx_notch_delay(tx_notch_support, tx_notch_offset)
	}

	/* Calibrated delay brought by SX1301 depending on signal bandwidth */
	switch bw {
	case BW_125KHZ:
		bw_delay_us = 1.5
	case BW_500KHZ:
		/* Intended fall-through: it is the calibrated reference */
		fallthrough
	default:
	}

	tx_start_delay := float64(TX_START_DELAY_DEFAULT) - bw_delay_us - notch_delay_us

	log.Printf("INFO: tx_start_delay=%d (%f) - (%d, bw_delay=%f, notch_delay=%f)\n", uint16(tx_start_delay), tx_start_delay, TX_START_DELAY_DEFAULT, bw_delay_us, notch_delay_us)

	return uint16(tx_start_delay) /* keep truncating instead of rounding: better behaviour measured */
}

func Lgw_start(path string, s *State) (*os.File, byte, byte, error) {
	e := s.rf_tx_enable[1]
	index := 0
	if e {
		index = 1
	}
	f, lgw_spi_mux_mode, spi_mux_target, tx_notch_support, spectral_scan_support, lbt_support, tx_notch_offset, err := Lgw_connect(path, false, s.rf_tx_notch_freq[index])
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: FAIL TO CONNECT BOARD")
	}

	/* reset the registers (also shuts the radios down) */
	err = Lgw_soft_reset(f, lgw_spi_mux_mode)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* gate clocks */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_GLOBAL_EN, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CLK32M_EN, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* switch on and reset the radios (also starts the 32 MHz XTAL) */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_A_EN, 1)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_B_EN, 1)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(500 * time.Millisecond) /* TODO: optimize */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_RST, 1)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(5 * time.Millisecond)
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_RST, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* setup the radios */
	err = Lgw_setup_sx125x(f, lgw_spi_mux_mode, spi_mux_target, 0, s.rf_clkout, s.rf_enable[0], s.rf_radio_type[0], s.rf_rx_freq[0])
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: Failed to setup sx125x radio for RF chain 0")
	}
	err = Lgw_setup_sx125x(f, lgw_spi_mux_mode, spi_mux_target, 1, s.rf_clkout, s.rf_enable[1], s.rf_radio_type[1], s.rf_rx_freq[1])
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: Failed to setup sx125x radio for RF chain 1")
	}

	/* gives AGC control of GPIOs to enable Tx external digital filter */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_GPIO_MODE, 31) /* Set all GPIOs as output */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_GPIO_SELECT_OUTPUT, 2)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	//TODO: ezt befejezni
	///* Configure LBT */
	//if s.lbt_enabled == true {
	//	Lgw_reg_w(LGW_CLK32M_EN, 1)
	//	i = lbt_setup()
	//	if i != LGW_LBT_SUCCESS {
	//		DEBUG_MSG("ERROR: lbt_setup() did not return SUCCESS\n")
	//		return nil, 0, 0, LGW_HAL_ERROR
	//	}

	//	/* Start SX1301 counter and LBT FSM at the same time to be in sync */
	//	Lgw_reg_w(LGW_CLK32M_EN, 0)
	//	i = lbt_start()
	//	if i != LGW_LBT_SUCCESS {
	//		DEBUG_MSG("ERROR: lbt_start() did not return SUCCESS\n")
	//		return nil, 0, 0, LGW_HAL_ERROR
	//	}
	//}

	/* Enable clocks */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_GLOBAL_EN, 1)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CLK32M_EN, 1)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* GPIOs table :
	   DGPIO0 -> N/A
	   DGPIO1 -> N/A
	   DGPIO2 -> N/A
	   DGPIO3 -> TX digital filter ON
	   DGPIO4 -> TX ON
	*/

	/* select calibration command */
	cal_cmd := 0
	if s.rf_enable[0] {
		cal_cmd |= 0x01 /* Bit 0: Calibrate Rx IQ mismatch compensation on radio A */
	}
	if s.rf_enable[1] {
		cal_cmd |= 0x02 /* Bit 1: Calibrate Rx IQ mismatch compensation on radio B */
	}
	if s.rf_enable[0] && s.rf_tx_enable[0] {
		cal_cmd |= 0x04 /* Bit 2: Calibrate Tx DC offset on radio A */
	}
	if s.rf_enable[1] && s.rf_tx_enable[1] {
		cal_cmd |= 0x08 /* Bit 3: Calibrate Tx DC offset on radio B */
	}
	cal_cmd |= 0x10 /* Bit 4: 0: calibrate with DAC gain=2, 1: with DAC gain=3 (use 3) */

	switch s.rf_radio_type[0] { /* we assume that there is only one radio type on the board */
	case LGW_RADIO_TYPE_SX1255:
		cal_cmd |= 0x20 /* Bit 5: 0: SX1257, 1: SX1255 */
	case LGW_RADIO_TYPE_SX1257:
		cal_cmd |= 0x00 /* Bit 5: 0: SX1257, 1: SX1255 */
	default:
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE", s.rf_radio_type[0])
	}

	cal_cmd |= 0x00  /* Bit 6-7: Board type 0: ref, 1: FPGA, 3: board X */
	cal_time := 2300 /* measured between 2.1 and 2.2 sec, because 1 TX only */

	/* Load the calibration firmware  */
	err = load_firmware(f, MCU_AGC, lgw_spi_mux_mode, spi_mux_target, cal_firmware)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FORCE_HOST_RADIO_CTRL, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	} /* gives to AGC MCU the control of the radios */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, int32(cal_cmd)) /* send calibration configuration word */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_RST_1, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* Check firmware version */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_ADDR, FW_VERSION_ADDR)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	read_val, err := Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_DATA)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	fw_version := uint8(read_val)
	if fw_version != FW_VERSION_CAL {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: Version of calibration firmware not expected, actual:%d expected:%d", fw_version, FW_VERSION_CAL)
	}

	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_PAGE_REG, 3) /* Calibration will start on this condition as soon as MCU can talk to concentrator registers */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_EMERGENCY_FORCE_HOST_CTRL, 0) /* Give control of concentrator registers to MCU */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* Wait for calibration to end */
	fmt.Printf("Note: calibration started (time: %d ms)\n", cal_time)
	time.Sleep(time.Duration(cal_time) * time.Millisecond)                                 /* Wait for end of calibration */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_EMERGENCY_FORCE_HOST_CTRL, 1) /* Take back control */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* Get calibration status */
	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	cal_status := uint8(read_val)
	/*
	   bit 7: calibration finished
	   bit 0: could access SX1301 registers
	   bit 1: could access radio A registers
	   bit 2: could access radio B registers
	   bit 3: radio A RX image rejection successful
	   bit 4: radio B RX image rejection successful
	   bit 5: radio A TX DC Offset correction successful
	   bit 6: radio B TX DC Offset correction successful
	*/
	if (cal_status & 0x81) != 0x81 {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: CALIBRATION FAILURE (STATUS = %d)", cal_status)
	} else {
		fmt.Printf("Note: calibration finished (status = %d)\n", cal_status)
	}
	if s.rf_enable[0] && ((cal_status & 0x02) == 0) {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("WARNING: calibration could not access radio A")
	}
	if s.rf_enable[1] && ((cal_status & 0x04) == 0) {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("WARNING: calibration could not access radio B")
	}
	if s.rf_enable[0] && ((cal_status & 0x08) == 0) {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("WARNING: problem in calibration of radio A for image rejection")
	}
	if s.rf_enable[1] && ((cal_status & 0x10) == 0) {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("WARNING: problem in calibration of radio B for image rejection")
	}
	if s.rf_enable[0] && s.rf_tx_enable[0] && ((cal_status & 0x20) == 0) {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("WARNING: problem in calibration of radio A for TX DC offset")
	}
	if s.rf_enable[1] && s.rf_tx_enable[1] && ((cal_status & 0x40) == 0) {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("WARNING: problem in calibration of radio B for TX DC offset")
	}

	/* Get TX DC offset values */
	for i := 0; i <= 7; i++ {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_ADDR, int32(0xA0+i))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_DATA)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		s.cal_offset_a_i[i] = int8(read_val)
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_ADDR, int32(0xA8+i))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_DATA)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		s.cal_offset_a_q[i] = int8(read_val)
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_ADDR, int32(0xB0+i))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_DATA)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		s.cal_offset_b_i[i] = int8(read_val)
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_ADDR, int32(0xB8+i))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_DATA)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		s.cal_offset_b_q[i] = int8(read_val)
	}

	/* load adjusted parameters */
	err = lgw_constant_adjust(f, lgw_spi_mux_mode, spi_mux_target, s)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* Sanity check for RX frequency */
	if s.rf_rx_freq[0] == 0 {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: wrong configuration, rf_rx_freq[0] is not set")
	}

	/* Freq-to-time-drift calculation */
	x := 4096000000 / (s.rf_rx_freq[0] >> 1) /* dividend: (4*2048*1000000) >> 1, rescaled to avoid 32b overflow */
	if x > 63 {
		x = 63 /* saturation */
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FREQ_TO_TIME_DRIFT, int32(x)) /* default 9 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	x = 4096000000 / (s.rf_rx_freq[0] >> 3) /* dividend: (16*2048*1000000) >> 3, rescaled to avoid 32b overflow */
	if x > 63 {
		x = 63 /* saturation */
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_FREQ_TO_TIME_DRIFT, int32(x)) /* default 36 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* configure LoRa 'multi' demodulators aka. LoRa 'sensor' channels (IF0-3) */
	radio_select := 0 /* IF mapping to radio A/B (per bit, 0=A, 1=B) */
	for i := 0; i < LGW_MULTI_NB; i++ {
		if s.if_rf_chain[i] == 1 {
			radio_select += 1 << uint(i) /* transform bool array into binary word */
		}
	}
	/*
	   Lgw_reg_w(LGW_RADIO_SELECT, radio_select);

	   LGW_RADIO_SELECT is used for communication with the firmware, "radio_select"
	   will be loaded in LGW_RADIO_SELECT at the end of start procedure.
	*/

	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_0, IF_HZ_TO_REG(s.if_freq[0])) /* default -384 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_1, IF_HZ_TO_REG(s.if_freq[1])) /* default -128 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_2, IF_HZ_TO_REG(s.if_freq[2])) /* default 128 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_3, IF_HZ_TO_REG(s.if_freq[3])) /* default 384 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_4, IF_HZ_TO_REG(s.if_freq[4])) /* default -384 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_5, IF_HZ_TO_REG(s.if_freq[5])) /* default -128 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_6, IF_HZ_TO_REG(s.if_freq[6])) /* default 128 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_7, IF_HZ_TO_REG(s.if_freq[7])) /* default 384 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	var corr int32
	if s.if_enable[0] {
		corr = int32(s.lora_multi_sfmask[0])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR0_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[1] {
		corr = int32(s.lora_multi_sfmask[1])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR1_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[2] {
		corr = int32(s.lora_multi_sfmask[2])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR2_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[3] {
		corr = int32(s.lora_multi_sfmask[3])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR3_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[4] {
		corr = int32(s.lora_multi_sfmask[4])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR4_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[5] {
		corr = int32(s.lora_multi_sfmask[5])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR5_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[6] {
		corr = int32(s.lora_multi_sfmask[6])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR6_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[7] {
		corr = int32(s.lora_multi_sfmask[7])
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CORR7_DETECT_EN, corr) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_PPM_OFFSET, 0x60) /* as the threshold is 16ms, use 0x60 to enable ppm_offset for SF12 and SF11 @125kHz*/
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_CONCENTRATOR_MODEM_ENABLE, 1) /* default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* configure LoRa 'stand-alone' modem (IF8) */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_8, IF_HZ_TO_REG(s.if_freq[8])) /* MBWSSF modem (default 0) */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[8] == true {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RADIO_SELECT, int32(s.if_rf_chain[8]))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		switch s.lora_rx_bw {
		case BW_125KHZ:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_MODEM_BW, 0)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case BW_250KHZ:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_MODEM_BW, 1)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case BW_500KHZ:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_MODEM_BW, 2)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		default:
			return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT", s.lora_rx_bw)
		}
		switch s.lora_rx_sf {
		case DR_LORA_SF7:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RATE_SF, 7)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case DR_LORA_SF8:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RATE_SF, 8)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case DR_LORA_SF9:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RATE_SF, 9)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case DR_LORA_SF10:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RATE_SF, 10)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case DR_LORA_SF11:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RATE_SF, 11)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		case DR_LORA_SF12:
			err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_RATE_SF, 12)
			if err != nil {
				return nil, lgw_spi_mux_mode, spi_mux_target, err
			}
		default:
			return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT", s.lora_rx_sf)
		}
		var offset int32
		if s.lora_rx_ppm_offset {
			offset = 1
		}
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_PPM_OFFSET, offset) /* default 0 */
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_MODEM_ENABLE, 1) /* default 0 */
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
	} else {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MBWSSF_MODEM_ENABLE, 0)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
	}

	/* configure FSK modem (IF9) */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_IF_FREQ_9, IF_HZ_TO_REG(s.if_freq[9])) /* FSK modem, default 0 */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_PSIZE, int32(s.fsk_sync_word_size-1))
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_TX_PSIZE, int32(s.fsk_sync_word_size-1))
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	fsk_sync_word_reg := s.fsk_sync_word << (8 * (8 - s.fsk_sync_word_size))
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_REF_PATTERN_LSB, int32(0xFFFFFFFF&fsk_sync_word_reg))
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_REF_PATTERN_MSB, int32(0xFFFFFFFF&(fsk_sync_word_reg>>32)))
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if s.if_enable[9] {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_RADIO_SELECT, int32(s.if_rf_chain[9]))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_BR_RATIO, int32(LGW_XTAL_FREQU/s.fsk_rx_dr)) /* setting the dividing ratio for datarate */
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_CH_BW_EXPO, int32(s.fsk_rx_bw))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_MODEM_ENABLE, 1) /* default 0 */
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
	} else {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FSK_MODEM_ENABLE, 0)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
	}

	/* Load firmware */
	err = load_firmware(f, MCU_ARB, lgw_spi_mux_mode, spi_mux_target, arb_firmware)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = load_firmware(f, MCU_AGC, lgw_spi_mux_mode, spi_mux_target, agc_firmware)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* gives the AGC MCU control over radio, RF front-end and filter gain */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FORCE_HOST_RADIO_CTRL, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FORCE_HOST_FE_CTRL, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_FORCE_DEC_FILTER_GAIN, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* Get MCUs out of reset */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, 0) /* MUST not be = to 1 or 2 at firmware init */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_RST_0, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_RST_1, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* Check firmware version */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_ADDR, FW_VERSION_ADDR)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_AGC_MCU_RAM_DATA)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	fw_version = uint8(read_val)
	if fw_version != FW_VERSION_AGC {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: Version of AGC firmware not expected, actual:%d expected:%d", fw_version, FW_VERSION_AGC)
	}
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_ARB_MCU_RAM_ADDR, FW_VERSION_ADDR)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_DBG_ARB_MCU_RAM_DATA)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	fw_version = uint8(read_val)
	if fw_version != FW_VERSION_ARB {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: Version of arbiter firmware not expected, actual:%d expected:%d", fw_version, FW_VERSION_ARB)
	}

	log.Printf("Info: Initialising AGC firmware...\n")
	time.Sleep(1 * time.Millisecond)

	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if read_val != 0x10 {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%X", uint8(read_val))
	}

	/* Update Tx gain LUT and start AGC */
	for i := uint8(0); i < s.txgain_lut.size; i++ {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, AGC_CMD_WAIT) /* start a transaction */
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		time.Sleep(1 * time.Millisecond)
		load_val := s.txgain_lut.lut[i].mix_gain + (16 * s.txgain_lut.lut[i].dac_gain) + (64 * s.txgain_lut.lut[i].pa_gain)
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, int32(load_val))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		time.Sleep(1 * time.Millisecond)
		read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		if read_val != (0x30 + int32(i)) {
			return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%X", uint8(read_val))
		}
	}
	/* As the AGC fw is waiting for 16 entries, we need to abort the transaction if we get less entries */
	if s.txgain_lut.size < TX_GAIN_LUT_SIZE_MAX {
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, AGC_CMD_WAIT)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		time.Sleep(1 * time.Millisecond)
		load_val := AGC_CMD_ABORT
		err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, int32(load_val))
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		time.Sleep(1 * time.Millisecond)
		read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
		if err != nil {
			return nil, lgw_spi_mux_mode, spi_mux_target, err
		}
		if read_val != 0x30 {
			return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%X", uint8(read_val))
		}
	}

	/* Load Tx freq MSBs (always 3 if f > 768 for SX1257 or f > 384 for SX1255 */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, AGC_CMD_WAIT)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(1 * time.Millisecond)
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, 3)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(1 * time.Millisecond)
	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if read_val != 0x33 {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%X", uint8(read_val))
	}

	/* Load chan_select firmware option */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, AGC_CMD_WAIT)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(1 * time.Millisecond)
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, 0)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(1 * time.Millisecond)
	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if read_val != 0x30 {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%X", uint8(read_val))
	}

	/* End AGC firmware init and check status */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, AGC_CMD_WAIT)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(1 * time.Millisecond)
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_RADIO_SELECT, int32(radio_select)) /* Load intended value of RADIO_SELECT */
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	time.Sleep(1 * time.Millisecond)
	log.Printf("Info: putting back original RADIO_SELECT value\n")
	read_val, err = Lgw_reg_r(f, lgw_spi_mux_mode, spi_mux_target, LGW_MCU_AGC_STATUS)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}
	if read_val != 0x40 {
		return nil, lgw_spi_mux_mode, spi_mux_target, fmt.Errorf("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%X", uint8(read_val))
	}

	/* enable GPS event capture */
	err = Lgw_reg_w(f, lgw_spi_mux_mode, spi_mux_target, LGW_GPS_EN, 1)
	if err != nil {
		return nil, lgw_spi_mux_mode, spi_mux_target, err
	}

	/* */
	//TODO lbt, stupid
	//if lbt_is_enabled() == true {
	//	printf("INFO: Configuring LBT, this may take few seconds, please wait...\n")
	//	wait_ms(8400)
	//}

	s.tx_notch_support = tx_notch_support
	s.spectral_scan_support = spectral_scan_support
	s.lbt_support = lbt_support
	s.tx_notch_offset = tx_notch_offset

	return f, lgw_spi_mux_mode, spi_mux_target, nil
}

func Lgw_stop(f *os.File, lgw_spi_mux_mode byte) error {
	err := Lgw_soft_reset(f, lgw_spi_mux_mode)
	if err != nil {
		return err
	}
	err = Lgw_disconnect(f)
	if err != nil {
		return err
	}
	return nil
}

func Lgw_receive(c *os.File, spi_mux_mode, spi_mux_target byte, max_pkt uint8, s *State) ([]Lgw_pkt_rx_s, error) {
	//int nb_pkt_fetch; /* loop variable and return value */
	//struct lgw_pkt_rx_s *p; /* pointer to the current structure in the struct array */
	//uint8_t buff[255+RX_METADATA_NB]; /* buffer to store the result of SPI read bursts */
	//unsigned sz; /* size of the payload, uses to address metadata */
	//int ifmod; /* type of if_chain/modem a packet was received by */
	//int stat_fifo; /* the packet status as indicated in the FIFO */
	//uint32_t raw_timestamp; /* timestamp when internal 'RX finished' was triggered */
	//uint32_t delay_x, delay_y, delay_z; /* temporary variable for timestamp offset calculation */
	//uint32_t timestamp_correction; /* correction to account for processing delay */
	//uint32_t sf, cr, bw_pow, crc_en, ppm; /* used to calculate timestamp correction */

	if max_pkt < 0 || max_pkt > LGW_PKT_FIFO_SIZE {
		return nil, fmt.Errorf("ERROR: %d = INVALID MAX NUMBER OF PACKETS TO FETCH", max_pkt)
	}

	pkt_data := make([]Lgw_pkt_rx_s, 16)

	/* iterate max_pkt times at most */
	for nb_pkt_fetch := 0; nb_pkt_fetch < 16; nb_pkt_fetch++ {

		/* fetch all the RX FIFO data */
		buff, err := Lgw_reg_rb(c, spi_mux_mode, spi_mux_target, LGW_RX_PACKET_DATA_FIFO_NUM_STORED, 5)
		if err != nil {
			return nil, err
		}
		/* 0:   number of packets available in RX data buffer */
		/* 1,2: start address of the current packet in RX data buffer */
		/* 3:   CRC status of the current packet */
		/* 4:   size of the current packet payload in byte */

		/* how many packets are in the RX buffer ? Break if zero */
		if buff[0] == 0 {
			break /* no more packets to fetch, exit out of FOR loop */
		}

		/* sanity check */
		if buff[0] > LGW_PKT_FIFO_SIZE {
			return nil, fmt.Errorf("WARNING: %d = INVALID NUMBER OF PACKETS TO FETCH, ABORTING", buff[0])
		}

		//fmt.Printf("FIFO content: %d %d %d %d %d\n", buff[0], buff[1], buff[2], buff[3], buff[4])
		pkt_data[nb_pkt_fetch].Size = uint16(buff[4])
		sz := pkt_data[nb_pkt_fetch].Size
		stat_fifo := buff[3] /* will be used later, need to save it before overwriting buff */

		/* get payload + metadata */
		buff, err = Lgw_reg_rb(c, spi_mux_mode, spi_mux_target, LGW_RX_DATA_BUF_DATA, sz+RX_METADATA_NB)
		if err != nil {
			return nil, err
		}

		/* copy payload to result struct */
		//memcpy((void *)p->payload, (void *)buff, sz);
		pkt_data[nb_pkt_fetch].Payload = make([]byte, sz)
		copy(pkt_data[nb_pkt_fetch].Payload, buff)

		/* process metadata */
		pkt_data[nb_pkt_fetch].If_chain = buff[sz+0]
		if pkt_data[nb_pkt_fetch].If_chain >= LGW_IF_CHAIN_NB {
			return nil, fmt.Errorf("WARNING: %d NOT A VALID IF_CHAIN NUMBER, ABORTING", pkt_data[nb_pkt_fetch].If_chain)
		}
		ifmod := ifmod_config[pkt_data[nb_pkt_fetch].If_chain]

		pkt_data[nb_pkt_fetch].Rf_chain = s.if_rf_chain[pkt_data[nb_pkt_fetch].If_chain]
		pkt_data[nb_pkt_fetch].Freq_hz = uint32(int32(s.rf_rx_freq[pkt_data[nb_pkt_fetch].Rf_chain]) + s.if_freq[pkt_data[nb_pkt_fetch].If_chain])
		pkt_data[nb_pkt_fetch].Rssi = float64(float64(buff[sz+5]) + s.rf_rssi_offset[pkt_data[nb_pkt_fetch].Rf_chain])
		crc_en := 0
		var timestamp_correction int
		if (ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD) {
			switch stat_fifo & 0x07 {
			case 5:
				pkt_data[nb_pkt_fetch].Status = STAT_CRC_OK
				crc_en = 1
			case 7:
				pkt_data[nb_pkt_fetch].Status = STAT_CRC_BAD
				crc_en = 1
			case 1:
				pkt_data[nb_pkt_fetch].Status = STAT_NO_CRC
				crc_en = 0
			default:
				pkt_data[nb_pkt_fetch].Status = STAT_UNDEFINED
				crc_en = 0
			}
			pkt_data[nb_pkt_fetch].Modulation = MOD_LORA
			pkt_data[nb_pkt_fetch].Snr = (float64(int8(buff[sz+2]))) / 4
			pkt_data[nb_pkt_fetch].Snr_min = (float64(int8(buff[sz+3]))) / 4
			pkt_data[nb_pkt_fetch].Snr_max = (float64(int8(buff[sz+4]))) / 4
			if ifmod == IF_LORA_MULTI {
				pkt_data[nb_pkt_fetch].Bandwidth = BW_125KHZ /* fixed in hardware */
			} else {
				pkt_data[nb_pkt_fetch].Bandwidth = s.lora_rx_bw /* get the parameter from the config variable */
			}
			sf := (buff[sz+1] >> 4) & 0x0F
			switch sf {
			case 7:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_LORA_SF7)
			case 8:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_LORA_SF8)
			case 9:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_LORA_SF9)
			case 10:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_LORA_SF10)
			case 11:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_LORA_SF11)
			case 12:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_LORA_SF12)
			default:
				pkt_data[nb_pkt_fetch].Datarate = uint32(DR_UNDEFINED)
			}
			cr := (buff[sz+1] >> 1) & 0x07
			switch cr {
			case 1:
				pkt_data[nb_pkt_fetch].Coderate = CR_LORA_4_5
				break
			case 2:
				pkt_data[nb_pkt_fetch].Coderate = CR_LORA_4_6
				break
			case 3:
				pkt_data[nb_pkt_fetch].Coderate = CR_LORA_4_7
				break
			case 4:
				pkt_data[nb_pkt_fetch].Coderate = CR_LORA_4_8
				break
			default:
				pkt_data[nb_pkt_fetch].Coderate = CR_UNDEFINED
			}
			var ppm byte
			/* determine if 'PPM mode' is on, needed for timestamp correction */
			if SET_PPM_ON(pkt_data[nb_pkt_fetch].Bandwidth, byte(pkt_data[nb_pkt_fetch].Datarate)) {
				ppm = 1
			}

			/* timestamp correction code, base delay */

			delay_x := 0
			bw_pow := 0
			/* timestamp correction code, base delay */
			if ifmod == IF_LORA_STD { /* if packet was received on the stand-alone LoRa modem */
				switch s.lora_rx_bw {
				case BW_125KHZ:
					delay_x = 64
					bw_pow = 1
					break
				case BW_250KHZ:
					delay_x = 32
					bw_pow = 2
					break
				case BW_500KHZ:
					delay_x = 16
					bw_pow = 4
					break
				default:
					return nil, fmt.Errorf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data[nb_pkt_fetch].Bandwidth)
					delay_x = 0
					bw_pow = 0
				}
			} else { /* packet was received on one of the sensor channels = 125kHz */
				delay_x = 114
				bw_pow = 1
			}
			delay_y := 0
			delay_z := 0
			/* timestamp correction code, variable delay */
			if (sf >= 6) && (sf <= 12) && (bw_pow > 0) {
				if (2*(int(sz)+2*crc_en) - (int(sf) - 7)) <= 0 { /* payload fits entirely in first 8 symbols */
					delay_y = int((((1 << (sf - 1)) * (sf + 1)) + (3 * (1 << (sf - 4)))) / byte(bw_pow))
					delay_z = 32 * (2*(int(sz)+2*crc_en) + 5) / bw_pow
				} else {
					delay_y = int((((1 << (sf - 1)) * (sf + 1)) + ((4 - ppm) * (1 << (sf - 4)))) / byte(bw_pow))
					delay_z = int((16 + 4*int(cr)) * (((2*(int(sz)+2*crc_en) - int(sf) + 6) % (int(sf) - 2*int(ppm))) + 1) / bw_pow)
				}
				timestamp_correction = int(delay_x + delay_y + delay_z)
			}

			/* RSSI correction */
			if ifmod == IF_LORA_MULTI {
				pkt_data[nb_pkt_fetch].Rssi -= RSSI_MULTI_BIAS
			}

		} else if ifmod == IF_FSK_STD {
			switch stat_fifo & 0x07 {
			case 5:
				pkt_data[nb_pkt_fetch].Status = STAT_CRC_OK
				break
			case 7:
				pkt_data[nb_pkt_fetch].Status = STAT_CRC_BAD
				break
			case 1:
				pkt_data[nb_pkt_fetch].Status = STAT_NO_CRC
				break
			default:
				pkt_data[nb_pkt_fetch].Status = STAT_UNDEFINED
				break
			}
			pkt_data[nb_pkt_fetch].Modulation = MOD_FSK
			pkt_data[nb_pkt_fetch].Snr = -128.0
			pkt_data[nb_pkt_fetch].Snr_min = -128.0
			pkt_data[nb_pkt_fetch].Snr_max = -128.0
			pkt_data[nb_pkt_fetch].Bandwidth = BW_125KHZ
			pkt_data[nb_pkt_fetch].Datarate = 50000
			pkt_data[nb_pkt_fetch].Coderate = CR_UNDEFINED
			timestamp_correction = (680000 / 50000) - 20

			/* RSSI correction */
			pkt_data[nb_pkt_fetch].Rssi = RSSI_FSK_POLY_0 + RSSI_FSK_POLY_1*pkt_data[nb_pkt_fetch].Rssi + RSSI_FSK_POLY_2*math.Pow(pkt_data[nb_pkt_fetch].Rssi, 2)
		} else {
			pkt_data[nb_pkt_fetch].Status = STAT_UNDEFINED
			pkt_data[nb_pkt_fetch].Modulation = MOD_UNDEFINED
			pkt_data[nb_pkt_fetch].Rssi = -128.0
			pkt_data[nb_pkt_fetch].Snr = -128.0
			pkt_data[nb_pkt_fetch].Snr_min = -128.0
			pkt_data[nb_pkt_fetch].Snr_max = -128.0
			pkt_data[nb_pkt_fetch].Bandwidth = BW_UNDEFINED
			pkt_data[nb_pkt_fetch].Datarate = uint32(DR_UNDEFINED)
			pkt_data[nb_pkt_fetch].Coderate = CR_UNDEFINED
			timestamp_correction = 0
		}

		raw_timestamp := (uint32(buff[sz+6])) + (uint32(buff[sz+7]) << 8) + (uint32(buff[sz+8]) << 16) + (uint32(buff[sz+9]) << 24)
		pkt_data[nb_pkt_fetch].Count_us = uint32(int(raw_timestamp) - timestamp_correction)
		pkt_data[nb_pkt_fetch].Crc = uint16(buff[sz+10]) + (uint16(buff[sz+11]) << 8)

		/* advance packet FIFO */
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_RX_PACKET_DATA_FIFO_NUM_STORED, 0)
		if err != nil {
			return nil, err
		}
	}

	return pkt_data, nil
}

func Lgw_send(c *os.File, spi_mux_mode, spi_mux_target byte, pkt_data Lgw_pkt_tx_s, s *State) error {
	buff := make([]byte, 256+TX_METADATA_NB)
	//int i, x;
	//uint8_t buff[256+TX_METADATA_NB]; /* buffer to prepare the packet to send + metadata before SPI write burst */
	//uint32_t part_int = 0; /* integer part for PLL register value calculation */
	//uint32_t part_frac = 0; /* fractional part for PLL register value calculation */
	//uint16_t fsk_dr_div; /* divider to configure for target datarate */
	//int transfer_size = 0; /* data to transfer from host to TX databuffer */
	//int payload_offset = 0; /* start of the payload content in the databuffer */
	//uint8_t pow_index = 0; /* 4-bit value to set the firmware TX power */
	//uint8_t target_mix_gain = 0; /* used to select the proper I/Q offset correction */
	//uint32_t count_trig = 0; /* timestamp value in trigger mode corrected for TX start delay */
	//bool tx_allowed = false;
	//uint16_t tx_start_delay;
	//bool tx_notch_enable = false;

	/* check input range (segfault prevention) */
	if pkt_data.rf_chain >= LGW_RF_CHAIN_NB {
		return fmt.Errorf("ERROR: INVALID RF_CHAIN TO SEND PACKETS")
	}

	/* check input variables */
	if s.rf_tx_enable[pkt_data.rf_chain] == false {
		return fmt.Errorf("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD")
	}
	if s.rf_enable[pkt_data.rf_chain] == false {
		return fmt.Errorf("ERROR: SELECTED RF_CHAIN IS DISABLED")
	}
	if !IS_TX_MODE(pkt_data.tx_mode) {
		return fmt.Errorf("ERROR: TX_MODE NOT SUPPORTED")
	}
	if pkt_data.modulation == MOD_LORA {
		if !IS_LORA_BW(pkt_data.bandwidth) {
			return fmt.Errorf("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX")
		}
		if !IS_LORA_STD_DR(byte(pkt_data.datarate)) {
			return fmt.Errorf("ERROR: DATARATE NOT SUPPORTED BY LORA TX")
		}
		if !IS_LORA_CR(pkt_data.coderate) {
			return fmt.Errorf("ERROR: CODERATE NOT SUPPORTED BY LORA TX")
		}
		if pkt_data.size > 255 {
			return fmt.Errorf("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX")
		}
	} else if pkt_data.modulation == MOD_FSK {
		if (pkt_data.f_dev < 1) || (pkt_data.f_dev > 200) {
			return fmt.Errorf("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE")
		}
		if !IS_FSK_DR(int(pkt_data.datarate)) {
			return fmt.Errorf("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN")
		}
		if pkt_data.size > 255 {
			return fmt.Errorf("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX")
		}
	} else {
		return fmt.Errorf("ERROR: INVALID TX MODULATION")
	}

	/* Enable notch filter for LoRa 125kHz */
	if (pkt_data.modulation == MOD_LORA) && (pkt_data.bandwidth == BW_125KHZ) {
		s.tx_notch_enable = true
	}

	/* Get the TX start delay to be applied for this TX */
	tx_start_delay := lgw_get_tx_start_delay(s.tx_notch_support, s.tx_notch_enable, s.tx_notch_offset, pkt_data.bandwidth)

	/* interpretation of TX power */
	for pow_index := s.txgain_lut.size - 1; pow_index > 0; pow_index-- {
		if s.txgain_lut.lut[pow_index].rf_power <= pkt_data.rf_power {
			break
		}
	}
	pow_index := 0
	/* loading TX imbalance correction */
	target_mix_gain := s.txgain_lut.lut[pow_index].mix_gain
	if pkt_data.rf_chain == 0 { /* use radio A calibration table */
		err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_OFFSET_I, int32(s.cal_offset_a_i[target_mix_gain-8]))
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_OFFSET_Q, int32(s.cal_offset_a_q[target_mix_gain-8]))
		if err != nil {
			return err
		}
	} else { /* use radio B calibration table */
		err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_OFFSET_I, int32(s.cal_offset_b_i[target_mix_gain-8]))
		if err != nil {
			return err
		}
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_OFFSET_Q, int32(s.cal_offset_b_q[target_mix_gain-8]))
		if err != nil {
			return err
		}
	}

	/* Set digital gain from LUT */
	err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_GAIN, int32(s.txgain_lut.lut[pow_index].dig_gain))
	if err != nil {
		return err
	}

	/* fixed metadata, useful payload and misc metadata compositing */
	transfer_size := TX_METADATA_NB + pkt_data.size /*  */
	payload_offset := TX_METADATA_NB                /* start the payload just after the metadata */

	part_int := uint32(0)
	part_frac := uint32(0)

	/* metadata 0 to 2, TX PLL frequency */
	switch s.rf_radio_type[0] { /* we assume that there is only one radio type on the board */
	case LGW_RADIO_TYPE_SX1255:
		part_int = pkt_data.freq_hz / (SX125x_32MHz_FRAC << 7)                               /* integer part, gives the MSB */
		part_frac = ((pkt_data.freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC /* fractional part, gives middle part and LSB */
	case LGW_RADIO_TYPE_SX1257:
		part_int = pkt_data.freq_hz / (SX125x_32MHz_FRAC << 8)                               /* integer part, gives the MSB */
		part_frac = ((pkt_data.freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC /* fractional part, gives middle part and LSB */
	default:
		return fmt.Errorf("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE", s.rf_radio_type[0])
	}

	buff[0] = byte(part_int)       /* Most Significant Byte */
	buff[1] = byte(part_frac >> 8) /* middle byte */
	buff[2] = byte(part_frac)      /* Least Significant Byte */

	/* metadata 3 to 6, timestamp trigger value */
	/* TX state machine must be triggered at (T0 - lgw_i_tx_start_delay_us) for packet to start being emitted at T0 */
	if pkt_data.tx_mode == TIMESTAMPED {
		count_trig := pkt_data.count_us - uint32(tx_start_delay)
		buff[3] = byte(count_trig >> 24)
		buff[4] = byte(count_trig >> 16)
		buff[5] = byte(count_trig >> 8)
		buff[6] = byte(count_trig)
	}

	/* parameters depending on modulation  */
	if pkt_data.modulation == MOD_LORA {
		/* metadata 7, modulation type, radio chain selection and TX power */
		buff[7] = (0x20 & (pkt_data.rf_chain << 5)) | byte(0x0F&pow_index) /* bit 4 is 0 -> LoRa modulation */

		buff[8] = 0 /* metadata 8, not used */

		/* metadata 9, CRC, LoRa CR & SF */
		switch byte(pkt_data.datarate) {
		case DR_LORA_SF7:
			buff[9] = 7
			break
		case DR_LORA_SF8:
			buff[9] = 8
			break
		case DR_LORA_SF9:
			buff[9] = 9
			break
		case DR_LORA_SF10:
			buff[9] = 10
			break
		case DR_LORA_SF11:
			buff[9] = 11
			break
		case DR_LORA_SF12:
			buff[9] = 12
			break
		default:
			log.Printf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.datarate)
		}
		switch pkt_data.coderate {
		case CR_LORA_4_5:
			buff[9] |= 1 << 4
			break
		case CR_LORA_4_6:
			buff[9] |= 2 << 4
			break
		case CR_LORA_4_7:
			buff[9] |= 3 << 4
			break
		case CR_LORA_4_8:
			buff[9] |= 4 << 4
			break
		default:
			log.Printf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.coderate)
		}
		if pkt_data.no_crc == false {
			buff[9] |= 0x80 /* set 'CRC enable' bit */
		} else {
			log.Printf("Info: packet will be sent without CRC\n")
		}

		/* metadata 10, payload size */
		buff[10] = byte(pkt_data.size)

		/* metadata 11, implicit header, modulation bandwidth, PPM offset & polarity */
		switch pkt_data.bandwidth {
		case BW_125KHZ:
			buff[11] = 0
			break
		case BW_250KHZ:
			buff[11] = 1
			break
		case BW_500KHZ:
			buff[11] = 2
			break
		default:
			log.Printf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.bandwidth)
		}
		if pkt_data.no_header == true {
			buff[11] |= 0x04 /* set 'implicit header' bit */
		}
		if SET_PPM_ON(pkt_data.bandwidth, byte(pkt_data.datarate)) {
			buff[11] |= 0x08 /* set 'PPM offset' bit at 1 */
		}
		if pkt_data.invert_pol == true {
			buff[11] |= 0x10 /* set 'TX polarity' bit at 1 */
		}

		/* metadata 12 & 13, LoRa preamble size */
		if pkt_data.preamble == 0 { /* if not explicit, use recommended LoRa preamble size */
			pkt_data.preamble = STD_LORA_PREAMBLE
		} else if pkt_data.preamble < MIN_LORA_PREAMBLE { /* enforce minimum preamble size */
			pkt_data.preamble = MIN_LORA_PREAMBLE
			log.Printf("Note: preamble length adjusted to respect minimum LoRa preamble size\n")
		}
		buff[12] = byte(pkt_data.preamble >> 8)
		buff[13] = byte(pkt_data.preamble)

		/* metadata 14 & 15, not used */
		buff[14] = 0
		buff[15] = 0

		/* MSB of RF frequency is now used in AGC firmware to implement large/narrow filtering in SX1257/55 */
		buff[0] &= 0x3F /* Unset 2 MSBs of frequency code */
		if pkt_data.bandwidth == BW_500KHZ {
			buff[0] |= 0x80 /* Set MSB bit to enlarge analog filter for 500kHz BW */
		}

		/* Set MSB-1 bit to enable digital filter if required */
		if s.tx_notch_enable == true {
			log.Printf("INFO: Enabling TX notch filter\n")
			buff[0] |= 0x40
		}
	} else if pkt_data.modulation == MOD_FSK {
		/* metadata 7, modulation type, radio chain selection and TX power */
		buff[7] = (0x20 & (pkt_data.rf_chain << 5)) | 0x10 | byte(0x0F&pow_index) /* bit 4 is 1 -> FSK modulation */

		buff[8] = 0 /* metadata 8, not used */

		/* metadata 9, frequency deviation */
		buff[9] = pkt_data.f_dev

		/* metadata 10, payload size */
		buff[10] = byte(pkt_data.size)
		/* TODO: how to handle 255 bytes packets ?!? */

		/* metadata 11, packet mode, CRC, encoding */
		temp := byte(0x02)
		if pkt_data.no_crc {
			temp = 0
		}
		buff[11] = 0x01 | temp | (0x02 << 2) /* always in variable length packet mode, whitening, and CCITT CRC if CRC is not disabled  */

		/* metadata 12 & 13, FSK preamble size */
		if pkt_data.preamble == 0 { /* if not explicit, use LoRa MAC preamble size */
			pkt_data.preamble = STD_FSK_PREAMBLE
		} else if pkt_data.preamble < MIN_FSK_PREAMBLE { /* enforce minimum preamble size */
			pkt_data.preamble = MIN_FSK_PREAMBLE
			log.Printf("Note: preamble length adjusted to respect minimum FSK preamble size\n")
		}
		buff[12] = byte(pkt_data.preamble >> 8)
		buff[13] = byte(pkt_data.preamble)

		/* metadata 14 & 15, FSK baudrate */
		fsk_dr_div := uint16(uint32(LGW_XTAL_FREQU) / pkt_data.datarate) /* Ok for datarate between 500bps and 250kbps */
		buff[14] = byte(fsk_dr_div >> 8)
		buff[15] = byte(fsk_dr_div)

		/* insert payload size in the packet for variable mode */
		buff[16] = byte(pkt_data.size)
		transfer_size++  /* one more byte to transfer to the TX modem */
		payload_offset++ /* start the payload with one more byte of offset */

		/* MSB of RF frequency is now used in AGC firmware to implement large/narrow filtering in SX1257/55 */
		buff[0] &= 0x7F /* Always use narrow band for FSK (force MSB to 0) */

	} else {
		return fmt.Errorf("ERROR: INVALID TX MODULATION..")
	}

	/* Configure TX start delay based on TX notch filter */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_START_DELAY, int32(tx_start_delay))
	if err != nil {
		return err
	}

	/* copy payload from user struct to buffer containing metadata */
	copy(buff[payload_offset:], pkt_data.payload[:pkt_data.size])
	buff = buff[:payload_offset+int(pkt_data.size)]
	//memcpy((void *)(buff + payload_offset), (void *)(pkt_data.payload), pkt_data.size);

	/* reset TX command flags */
	err = Lgw_abort_tx(c, spi_mux_mode, spi_mux_target)
	if err != nil {
		return err
	}

	/* put metadata + payload in the TX data buffer */
	err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_DATA_BUF_ADDR, 0)
	if err != nil {
		return err
	}
	err = Lgw_reg_wb(c, spi_mux_mode, spi_mux_target, LGW_TX_DATA_BUF_DATA, buff)
	if err != nil {
		return err
	}

	//TODO: LBT
	//x = lbt_is_channel_free(&pkt_data, tx_start_delay, &tx_allowed)
	//if x != LGW_LBT_SUCCESS {
	//	DEBUG_MSG("ERROR: Failed to check channel availability for TX\n")
	//	return LGW_HAL_ERROR
	//}
	//if tx_allowed == true {
	switch pkt_data.tx_mode {
	case IMMEDIATE:
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_TRIG_IMMEDIATE, 1)
		if err != nil {
			return err
		}
		break

	case TIMESTAMPED:
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_TRIG_DELAYED, 1)
		if err != nil {
			return err
		}
		break

	case ON_GPS:
		err = Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_TRIG_GPS, 1)
		if err != nil {
			return err
		}
		break

	default:
		return fmt.Errorf("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT", pkt_data.tx_mode)
	}
	//TODO: LBT
	//}	 else {
	//	DEBUG_MSG("ERROR: Cannot send packet, channel is busy (LBT)\n")
	//	return LGW_LBT_ISSUE
	//}

	return nil
}

func Lgw_abort_tx(c *os.File, spi_mux_mode, spi_mux_target byte) error {
	err := Lgw_reg_w(c, spi_mux_mode, spi_mux_target, LGW_TX_TRIG_ALL, 0)
	if err != nil {
		return err
	}
	return nil
}

//NOTE: completely custom code. Original libloragw have a lot of static fields which store the internal state, all of them are inside this struct
type State struct {
	rf_tx_notch_freq  [LGW_RF_CHAIN_NB]uint32
	rf_tx_enable      [LGW_RF_CHAIN_NB]bool
	rf_enable         [LGW_RF_CHAIN_NB]bool
	rf_rx_freq        [LGW_RF_CHAIN_NB]uint32 /* absolute, in Hz */
	rf_rssi_offset    [LGW_RF_CHAIN_NB]float64
	rf_radio_type     [LGW_RF_CHAIN_NB]lgw_radio_type_e
	if_enable         [LGW_IF_CHAIN_NB]bool
	if_rf_chain       [LGW_IF_CHAIN_NB]byte  /* for each IF, 0 -> radio A, 1 -> radio B */
	if_freq           [LGW_IF_CHAIN_NB]int32 /* relative to radio frequency, +/- in Hz */
	lora_multi_sfmask [LGW_MULTI_NB]byte     /* enables SF for LoRa 'multi' modems */

	lora_rx_bw         byte /* bandwidth setting for LoRa standalone modem */
	lora_rx_sf         byte /* spreading factor setting for LoRa standalone modem */
	lora_rx_ppm_offset bool

	fsk_rx_bw          byte   /* bandwidth setting of FSK modem */
	fsk_rx_dr          uint32 /* FSK modem datarate in bauds */
	fsk_sync_word_size byte   /* default number of bytes for FSK sync word */
	fsk_sync_word      uint64 /* default FSK sync word (ALIGNED RIGHT, MSbit first) */

	lorawan_public bool
	rf_clkout      byte

	/* TX I/Q imbalance coefficients for mixer gain = 8 to 15 */
	cal_offset_a_i [8]int8 /* TX I offset for radio A */
	cal_offset_a_q [8]int8 /* TX Q offset for radio A */
	cal_offset_b_i [8]int8 /* TX I offset for radio B */
	cal_offset_b_q [8]int8 /* TX Q offset for radio B */

	txgain_lut Lgw_tx_gain_lut_s

	lbt_enabled bool

	tx_notch_enable bool
	tx_notch_offset byte

	tx_notch_support      bool
	spectral_scan_support bool
	lbt_support           bool
}

type Config struct {
	SX1301Conf struct {
		LorawanPublic bool `json:"lorawan_public"`
		Clksrc        byte `json:"clksrc"`
		Radio0        struct {
			Enable     bool    `json:"enable"`
			Type       string  `json:"type"`
			Freq       uint32  `json:"freq"`
			RssiOffset float64 `json:"rssi_offset"`
			TxEnable   bool    `json:"tx_enable"`
		} `json:"radio_0"`
		Radio1 struct {
			Enable     bool    `json:"enable"`
			Type       string  `json:"type"`
			Freq       uint32  `json:"freq"`
			RssiOffset float64 `json:"rssi_offset"`
			TxEnable   bool    `json:"tx_enable"`
		} `json:"radio_1"`
		ChanMultiSF0 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_0"`
		ChanMultiSF1 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_1"`
		ChanMultiSF2 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_2"`
		ChanMultiSF3 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_3"`
		ChanMultiSF4 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_4"`
		ChanMultiSF5 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_5"`
		ChanMultiSF6 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_6"`
		ChanMultiSF7 struct {
			Enable bool  `json:"enable"`
			Radio  byte  `json:"radio"`
			If     int32 `json:"if"`
		} `json:"chan_multiSF_7"`
		ChanLoraStd struct {
			Enable       bool  `json:"enable"`
			Radio        byte  `json:"radio"`
			If           int32 `json:"if"`
			Bandwidth    int   `json:"bandwidth"`
			SpreadFactor int   `json:"spread_factor"`
		} `json:"chan_Lora_std"`
		ChanFSK struct {
			Enable    bool   `json:"enable"`
			Radio     byte   `json:"radio"`
			If        int32  `json:"if"`
			Bandwidth int    `json:"bandwidth"`
			Datarate  uint32 `json:"datarate"`
		} `json:"chan_FSK"`
	} `json:"SX1301_conf"`
	GatewayConf struct {
		GatewayID string `json:"gateway_ID"`
	} `json:"gateway_conf"`
}

var internalstates = make(map[string]State)

func ParseConfig(configpath string) (*State, error) {
	state := State{}
	state.txgain_lut.size = 2
	state.txgain_lut.lut = [TX_GAIN_LUT_SIZE_MAX]Lgw_tx_gain_s{}
	state.txgain_lut.lut[0] = Lgw_tx_gain_s{
		dig_gain: 0,
		pa_gain:  2,
		dac_gain: 3,
		mix_gain: 10,
		rf_power: 14,
	}
	state.txgain_lut.lut[1] = Lgw_tx_gain_s{
		dig_gain: 0,
		pa_gain:  3,
		dac_gain: 3,
		mix_gain: 14,
		rf_power: 27,
	}
	f, err := ioutil.ReadFile(configpath)
	if err != nil {
		return nil, err
	}
	var config Config
	err = json.Unmarshal(f, &config)
	if err != nil {
		return nil, err
	}
	state.lorawan_public = config.SX1301Conf.LorawanPublic
	state.rf_clkout = config.SX1301Conf.Clksrc
	state.rf_enable[0] = config.SX1301Conf.Radio0.Enable
	state.rf_rx_freq[0] = config.SX1301Conf.Radio0.Freq
	state.rf_rssi_offset[0] = config.SX1301Conf.Radio0.RssiOffset
	state.rf_tx_enable[0] = config.SX1301Conf.Radio0.TxEnable
	switch config.SX1301Conf.Radio0.Type {
	case "SX1257":
		state.rf_radio_type[0] = LGW_RADIO_TYPE_SX1257
	case "SX1255":
		state.rf_radio_type[0] = LGW_RADIO_TYPE_SX1255
	default:
		return nil, fmt.Errorf("ERROR: NOT A VALID RADIO TYPE")
	}
	state.rf_enable[1] = config.SX1301Conf.Radio1.Enable
	state.rf_rx_freq[1] = config.SX1301Conf.Radio1.Freq
	state.rf_rssi_offset[1] = config.SX1301Conf.Radio1.RssiOffset
	state.rf_tx_enable[1] = config.SX1301Conf.Radio1.TxEnable
	switch config.SX1301Conf.Radio1.Type {
	case "SX1257":
		state.rf_radio_type[1] = LGW_RADIO_TYPE_SX1257
	case "SX1255":
		state.rf_radio_type[1] = LGW_RADIO_TYPE_SX1255
	default:
		return nil, fmt.Errorf("ERROR: NOT A VALID RADIO TYPE")
	}
	state.if_enable[0] = config.SX1301Conf.ChanMultiSF0.Enable
	state.if_rf_chain[0] = config.SX1301Conf.ChanMultiSF0.Radio
	state.if_freq[0] = config.SX1301Conf.ChanMultiSF0.If
	state.lora_multi_sfmask[0] = DR_LORA_MULTI //multisf only
	state.if_enable[1] = config.SX1301Conf.ChanMultiSF1.Enable
	state.if_rf_chain[1] = config.SX1301Conf.ChanMultiSF1.Radio
	state.if_freq[1] = config.SX1301Conf.ChanMultiSF1.If
	state.lora_multi_sfmask[1] = DR_LORA_MULTI //multisf only
	state.if_enable[2] = config.SX1301Conf.ChanMultiSF2.Enable
	state.if_rf_chain[2] = config.SX1301Conf.ChanMultiSF2.Radio
	state.if_freq[2] = config.SX1301Conf.ChanMultiSF2.If
	state.lora_multi_sfmask[2] = DR_LORA_MULTI //multisf only
	state.if_enable[3] = config.SX1301Conf.ChanMultiSF3.Enable
	state.if_rf_chain[3] = config.SX1301Conf.ChanMultiSF3.Radio
	state.if_freq[3] = config.SX1301Conf.ChanMultiSF3.If
	state.lora_multi_sfmask[3] = DR_LORA_MULTI //multisf only
	state.if_enable[4] = config.SX1301Conf.ChanMultiSF4.Enable
	state.if_rf_chain[4] = config.SX1301Conf.ChanMultiSF4.Radio
	state.if_freq[4] = config.SX1301Conf.ChanMultiSF4.If
	state.lora_multi_sfmask[4] = DR_LORA_MULTI //multisf only
	state.if_enable[5] = config.SX1301Conf.ChanMultiSF5.Enable
	state.if_rf_chain[5] = config.SX1301Conf.ChanMultiSF5.Radio
	state.if_freq[5] = config.SX1301Conf.ChanMultiSF5.If
	state.lora_multi_sfmask[5] = DR_LORA_MULTI //multisf only
	state.if_enable[6] = config.SX1301Conf.ChanMultiSF6.Enable
	state.if_rf_chain[6] = config.SX1301Conf.ChanMultiSF6.Radio
	state.if_freq[6] = config.SX1301Conf.ChanMultiSF6.If
	state.lora_multi_sfmask[6] = DR_LORA_MULTI //multisf only
	state.if_enable[7] = config.SX1301Conf.ChanMultiSF7.Enable
	state.if_rf_chain[7] = config.SX1301Conf.ChanMultiSF7.Radio
	state.if_freq[7] = config.SX1301Conf.ChanMultiSF7.If
	state.lora_multi_sfmask[7] = DR_LORA_MULTI //multisf only
	state.if_enable[8] = config.SX1301Conf.ChanLoraStd.Enable
	state.if_rf_chain[8] = config.SX1301Conf.ChanLoraStd.Radio
	state.if_freq[8] = config.SX1301Conf.ChanLoraStd.If
	switch config.SX1301Conf.ChanLoraStd.Bandwidth {
	case 500000:
		state.lora_rx_bw = BW_500KHZ
	case 250000:
		state.lora_rx_bw = BW_250KHZ
	case 125000:
		state.lora_rx_bw = BW_125KHZ
	case 62500:
		state.lora_rx_bw = BW_62K5HZ
	case 31200:
		state.lora_rx_bw = BW_31K2HZ
	case 15600:
		state.lora_rx_bw = BW_15K6HZ
	case 7800:
		state.lora_rx_bw = BW_7K8HZ
	}
	switch config.SX1301Conf.ChanLoraStd.SpreadFactor {
	case 7:
		state.lora_rx_sf = DR_LORA_SF7
	case 8:
		state.lora_rx_sf = DR_LORA_SF8
	case 9:
		state.lora_rx_sf = DR_LORA_SF9
	case 10:
		state.lora_rx_sf = DR_LORA_SF10
	case 11:
		state.lora_rx_sf = DR_LORA_SF11
	case 12:
		state.lora_rx_sf = DR_LORA_SF12
	}

	state.lora_rx_ppm_offset = SET_PPM_ON(state.lora_rx_bw, state.lora_rx_sf)

	state.if_enable[9] = config.SX1301Conf.ChanFSK.Enable
	state.if_rf_chain[9] = config.SX1301Conf.ChanFSK.Radio
	state.if_freq[9] = config.SX1301Conf.ChanFSK.If
	switch config.SX1301Conf.ChanFSK.Bandwidth {
	case 500000:
		state.fsk_rx_bw = BW_500KHZ
	case 250000:
		state.fsk_rx_bw = BW_250KHZ
	case 125000:
		state.fsk_rx_bw = BW_125KHZ
	case 62500:
		state.fsk_rx_bw = BW_62K5HZ
	case 31200:
		state.fsk_rx_bw = BW_31K2HZ
	case 15600:
		state.fsk_rx_bw = BW_15K6HZ
	case 7800:
		state.fsk_rx_bw = BW_7K8HZ
	}
	state.fsk_rx_dr = config.SX1301Conf.ChanFSK.Datarate
	state.fsk_sync_word_size = 3
	state.fsk_sync_word = 0xC194C1
	return &state, nil
}
