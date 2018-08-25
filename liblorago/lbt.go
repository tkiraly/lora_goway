package liblorago

import (
	"fmt"
	"log"
	"os"
)

const (
	LBT_TIMESTAMP_MASK = 0x007FF000 /* 11-bits timestamp */
)

func lbt_setup(f *os.File, s *State) error {
	lbt_start_freq := uint32(0)
	/* Check if LBT feature is supported by FPGA */
	val, err := Lgw_fpga_reg_r(f, LGW_FPGA_FEATURE)
	if err != nil {
		return fmt.Errorf("ERROR: Failed to read FPGA Features register")
	}
	if TAKE_N_BITS_FROM(uint8(val), 2, 1) != 1 {
		return fmt.Errorf("ERROR: No support for LBT in FPGA")
	}

	/* Get FPGA lowest frequency for LBT channels */
	val, err = Lgw_fpga_reg_r(f, LGW_FPGA_LBT_INITIAL_FREQ)
	if err != nil {
		return fmt.Errorf("ERROR: Failed to read LBT initial frequency from FPGA")
	}
	switch val {
	case 0:
		lbt_start_freq = 915000000
		break
	case 1:
		lbt_start_freq = 863000000
		break
	default:
		return fmt.Errorf("ERROR: LBT start frequency %d is not supported", val)
	}

	/* Configure SX127x for FSK */
	err = Lgw_setup_sx127x(f, lbt_start_freq, MOD_FSK, LGW_SX127X_RXBW_100K_HZ, s.lbt_rssi_offset_dB) /* 200KHz LBT channels */
	if err != nil {
		return fmt.Errorf("ERROR: Failed to configure SX127x for LBT")
	}

	/* Configure FPGA for LBT */
	val = int32(-2 * s.lbt_rssi_target_dBm) /* Convert RSSI target in dBm to FPGA register format */
	err = Lgw_fpga_reg_w(f, LGW_FPGA_RSSI_TARGET, val)
	if err != nil {
		return fmt.Errorf("ERROR: Failed to configure FPGA for LBT")
	}
	/* Set default values for non-active LBT channels */
	for i := s.lbt_nb_active_channel; i < LBT_CHANNEL_FREQ_NB; i++ {
		s.lbt_channel_cfg[i].freq_hz = lbt_start_freq
		s.lbt_channel_cfg[i].scan_time_us = 128 /* fastest scan for non-active channels */
	}
	/* Configure FPGA for both active and non-active LBT channels */
	for i := uint16(0); i < LBT_CHANNEL_FREQ_NB; i++ {
		/* Check input parameters */
		if s.lbt_channel_cfg[i].freq_hz < lbt_start_freq {
			return fmt.Errorf("ERROR: LBT channel frequency is out of range (%d)", s.lbt_channel_cfg[i].freq_hz)
		}
		if (s.lbt_channel_cfg[i].scan_time_us != 128) && (s.lbt_channel_cfg[i].scan_time_us != 5000) {
			return fmt.Errorf("ERROR: LBT channel scan time is not supported (%d)", s.lbt_channel_cfg[i].scan_time_us)
		}
		/* Configure */
		freq_offset := (s.lbt_channel_cfg[i].freq_hz - lbt_start_freq) / 100E3 /* 100kHz unit */
		err := Lgw_fpga_reg_w(f, LGW_FPGA_LBT_CH0_FREQ_OFFSET+i, int32(freq_offset))
		if err != nil {
			return fmt.Errorf("ERROR: Failed to configure FPGA for LBT channel %d (freq offset)", i)
		}
		if s.lbt_channel_cfg[i].scan_time_us == 5000 { /* configured to 128 by default */
			err := Lgw_fpga_reg_w(f, LGW_FPGA_LBT_SCAN_TIME_CH0+i, 1)
			if err != nil {
				return fmt.Errorf("ERROR: Failed to configure FPGA for LBT channel %d (freq offset)", i)
			}
		}
	}
	return nil
}

func lbt_start(f *os.File) error {
	err := Lgw_fpga_reg_w(f, LGW_FPGA_CTRL_FEATURE_START, 1)
	if err != nil {
		return fmt.Errorf("ERROR: Failed to start LBT FSM")
	}
	return nil
}

func lbt_is_channel_free(c *os.File, spi_mux_mode, spi_mux_target byte, pkt_data Lgw_pkt_tx_s, tx_start_delay uint16, tx_allowed bool, s *State) error {
	//int i;
	//int32_t val;
	tx_start_time := uint32(0)
	tx_max_time := uint32(0)
	lbt_time := uint32(0)
	lbt_time1 := uint32(0)
	lbt_time2 := uint32(0)
	//uint32_t tx_end_time = 0;
	//uint32_t delta_time = 0;
	//uint32_t sx1301_time = 0;
	lbt_channel_decod_1 := -1
	lbt_channel_decod_2 := -1
	//uint32_t packet_duration = 0;

	/* Check if TX is allowed */
	if s.lbt_enable == true {
		/* TX allowed for LoRa only */
		if pkt_data.modulation != MOD_LORA {
			tx_allowed = false
			log.Printf("INFO: TX is not allowed for this modulation (%X)\n", pkt_data.modulation)
			return nil
		}

		/* Get SX1301 time at last PPS */
		sx1301_time, err := Lgw_get_trigcnt(c, spi_mux_mode, spi_mux_target)
		if err != nil {
			return err
		}
		switch pkt_data.tx_mode {
		case TIMESTAMPED:
			log.Printf("tx_mode                    = TIMESTAMPED\n")
			tx_start_time = pkt_data.count_us & LBT_TIMESTAMP_MASK
			break
		case ON_GPS:
			log.Printf("tx_mode                    = ON_GPS\n")
			tx_start_time = (sx1301_time + uint32(tx_start_delay) + 1000000) & LBT_TIMESTAMP_MASK
			break
		case IMMEDIATE:
			log.Printf("ERROR: tx_mode IMMEDIATE is not supported when LBT is enabled\n")
			fallthrough
			/* FALLTHROUGH  */
		default:
			return fmt.Errorf("error, sorry")
		}

		/* Select LBT Channel corresponding to required TX frequency */
		lbt_channel_decod_1 = -1
		lbt_channel_decod_2 = -1
		if pkt_data.bandwidth == BW_125KHZ {
			for i := byte(0); i < s.lbt_nb_active_channel; i++ {
				if pkt_data.freq_hz == s.lbt_channel_cfg[i].freq_hz {
					log.Printf("LBT: select channel %d (%d Hz)\n", i, s.lbt_channel_cfg[i].freq_hz)
					lbt_channel_decod_1 = int(i)
					lbt_channel_decod_2 = int(i)
					if s.lbt_channel_cfg[i].scan_time_us == 5000 {
						tx_max_time = 4000000 /* 4 seconds */
					} else { /* scan_time_us = 128 */
						tx_max_time = 400000 /* 400 milliseconds */
					}
					break
				}
			}
		} else if pkt_data.bandwidth == BW_250KHZ {
			/* In case of 250KHz, the TX freq has to be in between 2 consecutive channels of 200KHz BW.
			   The TX can only be over 2 channels, not more */
			for i := byte(0); i < (s.lbt_nb_active_channel - 1); i++ {
				if (pkt_data.freq_hz == (s.lbt_channel_cfg[i].freq_hz+s.lbt_channel_cfg[i+1].freq_hz)/2) &&
					((s.lbt_channel_cfg[i+1].freq_hz - s.lbt_channel_cfg[i].freq_hz) == 200E3) {
					log.Printf("LBT: select channels %d,%d (%d Hz)\n", i, i+1, (s.lbt_channel_cfg[i].freq_hz+s.lbt_channel_cfg[i+1].freq_hz)/2)
					lbt_channel_decod_1 = int(i)
					lbt_channel_decod_2 = int(i) + 1
					if s.lbt_channel_cfg[i].scan_time_us == 5000 {
						tx_max_time = 4000000 /* 4 seconds */
					} else { /* scan_time_us = 128 */
						tx_max_time = 200000 /* 200 milliseconds */
					}
					break
				}
			}
		} else {
			/* Nothing to do for now */
		}

		/* Get last time when selected channel was free */
		if (lbt_channel_decod_1 >= 0) && (lbt_channel_decod_2 >= 0) {
			err := Lgw_fpga_reg_w(c, LGW_FPGA_LBT_TIMESTAMP_SELECT_CH, int32(lbt_channel_decod_1))
			if err != nil {
				return err
			}
			val, err := Lgw_fpga_reg_r(c, LGW_FPGA_LBT_TIMESTAMP_CH)
			if err != nil {
				return err
			}
			lbt_time = uint32(val&0x0000FFFF) * 256  /* 16bits (1LSB = 256µs) */
			lbt_time1 = uint32(val&0x0000FFFF) * 256 /* 16bits (1LSB = 256µs) */

			if lbt_channel_decod_1 != lbt_channel_decod_2 {
				err := Lgw_fpga_reg_w(c, LGW_FPGA_LBT_TIMESTAMP_SELECT_CH, int32(lbt_channel_decod_2))
				if err != nil {
					return err
				}
				val, err := Lgw_fpga_reg_r(c, LGW_FPGA_LBT_TIMESTAMP_CH)
				if err != nil {
					return err
				}
				lbt_time2 = uint32(val&0x0000FFFF) * 256 /* 16bits (1LSB = 256µs) */

				if lbt_time2 < lbt_time1 {
					lbt_time = lbt_time2
				}
			}
		} else {
			lbt_time = 0
		}

		packet_duration, err := Lgw_time_on_air(pkt_data, s)
		packet_duration *= 1000
		tx_end_time := (tx_start_time + packet_duration) & LBT_TIMESTAMP_MASK
		delta_time := uint32(0)
		if lbt_time < tx_end_time {
			delta_time = tx_end_time - lbt_time
		} else {
			/* It means LBT counter has wrapped */
			log.Printf("LBT: lbt counter has wrapped\n")
			delta_time = (LBT_TIMESTAMP_MASK - lbt_time) + tx_end_time
		}

		/* send data if allowed */
		/* lbt_time: last time when channel was free */
		/* tx_max_time: maximum time allowed to send packet since last free time */
		/* 2048: some margin */
		if (delta_time < (tx_max_time - 2048)) && (lbt_time != 0) {
			tx_allowed = true
		} else {
			log.Printf("ERROR: TX request rejected (LBT)\n")
			tx_allowed = false
		}
	} else {
		/* Always allow if LBT is disabled */
		tx_allowed = true
	}

	return nil
}
