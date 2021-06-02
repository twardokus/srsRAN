/**
 * Copyright 2013-2021 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <stdbool.h>

#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/phch/sci.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"
#include "srsran/phy/utils/random.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/phy/ue/ue_sync.h"

// create a radio variable for RF
static srsran_rf_t radio;

static bool keep_running = true;

srsran_cell_sl_t cell = {.nof_prb = 50, .N_sl_id = 0, .tm = SRSRAN_SIDELINK_TM4, .cp = SRSRAN_CP_NORM};
static srsran_random_t random_gen    = NULL;

uint32_t prb_start_idx = 0;
/*
void usage(char* prog)
{
	printf("Usage: %s [cdeipt]\n", prog);
	printf("\t-p nof_prb [Default %d]\n", cell.nof_prb);
	printf("\t-c N_sl_id [Default %d]\n", cell.N_sl_id);
	printf("\t-t Sidelink transmission mode {1,2,3,4} [Default %d]\n", (cell.tm + 1));
	printf("\t-v [set srsran_verbose to debug, default none]\n");
}

void parse_args(int argc, char** argv)
{
	int opt;
	while ((opt = getopt(argc, argv, "ceiptv")) != -1) {
		switch (opt) {
		case 'c':
			cell.N_sl_id = (int32_t)strtol(argv[optind], NULL, 10);
			break;
		case 'p':
			cell.nof_prb = (uint32_t)strtol(argv[optind], NULL, 10);
			break;
		case 't':
			switch (strtol(argv[optind], NULL, 10)) {
			case 1:
				cell.tm = SRSRAN_SIDELINK_TM1;
				break;
			case 2:
				cell.tm = SRSRAN_SIDELINK_TM2;
				break;
			case 3:
				cell.tm = SRSRAN_SIDELINK_TM3;
				break;
			case 4:
				cell.tm = SRSRAN_SIDELINK_TM4;
				break;
			default:
				usage(argv[0]);
				exit(-1);
				break;
			}
			break;
			case 'v':
				srsran_verbose++;
				break;
			default:
				usage(argv[0]);
				exit(-1);
		}
	}
	if (cell.cp == SRSRAN_CP_EXT && cell.tm >= SRSRAN_SIDELINK_TM3) {
		ERROR("Selected TM does not support extended CP");
		usage(argv[0]);
		exit(-1);
	}
}*/

void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    keep_running = false;
  } else if (signo == SIGSEGV) {
    exit(1);
  }
}

typedef struct {
	bool     use_standard_lte_rates;
	bool     disable_plots;
	char*    input_file_name;
	uint32_t file_start_sf_idx;
	uint32_t nof_rx_antennas;
	char*    rf_dev;
	char*    rf_args;
	double   rf_freq;
	float    rf_gain;

	// Sidelink specific args
	uint32_t size_sub_channel;
	uint32_t num_sub_channel;
} prog_args_t;

static prog_args_t prog_args;

void args_default(prog_args_t* args)
{
	args->disable_plots          = true;
	args->use_standard_lte_rates = false;
	args->input_file_name        = NULL;
	args->file_start_sf_idx      = 0;
	args->nof_rx_antennas        = 1;
	args->rf_dev                 = "";
	args->rf_dev                 = "";
	args->rf_args                = "";
	args->rf_freq                = 5.92e9;
	args->rf_gain                = 50;
	args->size_sub_channel       = 10;
	args->num_sub_channel        = 5;
}

void usage(prog_args_t* args, char* prog)
{
	printf("Usage: %s [agrnmv] -f rx_frequency_hz\n", prog);
	printf("\t-a RF args [Default %s]\n", args->rf_args);
	printf("\t-d RF devicename [Default %s]\n", args->rf_dev);
	printf("\t-i input_file_name\n");
	printf("\t-m Start subframe_idx [Default %d]\n", args->file_start_sf_idx);
	printf("\t-g RF Gain [Default %.2f dB]\n", args->rf_gain);
	printf("\t-A nof_rx_antennas [Default %d]\n", args->nof_rx_antennas);
	printf("\t-c N_sl_id [Default %d]\n", cell.N_sl_id);
	printf("\t-p nof_prb [Default %d]\n", cell.nof_prb);
	printf("\t-s size_sub_channel [Default for 50 prbs %d]\n", args->size_sub_channel);
	printf("\t-n num_sub_channel [Default for 50 prbs %d]\n", args->num_sub_channel);
	printf("\t-t Sidelink transmission mode {1,2,3,4} [Default %d]\n", (cell.tm + 1));
	printf("\t-r use_standard_lte_rates [Default %i]\n", args->use_standard_lte_rates);
#ifdef ENABLE_GUI
	printf("\t-w disable plots [Default enabled]\n");
#endif
	printf("\t-v srsran_verbose\n");
}

int srsran_rf_recv_wrapper(void* h, cf_t* data[SRSRAN_MAX_PORTS], uint32_t nsamples, srsran_timestamp_t* t)
{
  DEBUG(" ----  Receive %d samples  ----", nsamples);
  void* ptr[SRSRAN_MAX_PORTS];
  for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
    ptr[i] = data[i];
  }
  return srsran_rf_recv_with_time_multi(h, ptr, nsamples, true, &t->full_secs, &t->frac_secs);
}

void parse_args(prog_args_t* args, int argc, char** argv)
{
	int opt;
	args_default(args);

	while ((opt = getopt(argc, argv, "acdimgpvwrxfA")) != -1) {
		switch (opt) {
		case 'a':
			args->rf_args = argv[optind];
			break;
		case 'c':
			cell.N_sl_id = (int32_t)strtol(argv[optind], NULL, 10);
			break;
		case 'd':
			args->rf_dev = argv[optind];
			break;
		case 'i':
			args->input_file_name = argv[optind];
			break;
		case 'm':
			args->file_start_sf_idx = (uint32_t)strtol(argv[optind], NULL, 10);
			break;
		case 'g':
			args->rf_gain = strtof(argv[optind], NULL);
			break;
		case 'p':
			cell.nof_prb = (int32_t)strtol(argv[optind], NULL, 10);
			break;
		case 'f':
			args->rf_freq = strtof(argv[optind], NULL);
			break;
		case 'A':
			args->nof_rx_antennas = (int32_t)strtol(argv[optind], NULL, 10);
			break;
		case 'v':
			srsran_verbose++;
			break;
		case 'w':
			args->disable_plots = true;
			break;
		case 'r':
			args->use_standard_lte_rates = true;
			break;
		default:
			usage(args, argv[0]);
			exit(-1);
		}
	}
	if (args->rf_freq < 0 && args->input_file_name == NULL) {
		usage(args, argv[0]);
		exit(-1);
	}
}

int main(int argc, char** argv)
{

	parse_args(&prog_args, argc, argv);

	// variables

	/* sf_len defines the length of the subframe in kHz
	 *
	 * with 50 PRB there are 768 subcarriers per OFDM symbol
	 * each subcarrier has a 15 kHz subcarrier spacing
	 * 768 subcarriers * 15 kHz -> 11520 kHz (11.52 MHz), which becomes the sampling rate
	 */
	// #define SRSRAN_SF_LEN_PRB(nof_prb) ((uint32_t)SRSRAN_SF_LEN(srsran_symbol_sz(nof_prb)))
	// SRSRAN_SF_LEN_PRB(50) ((uint32_t)SRSRAN_SF_LEN(srsran_symbol_sz(50)))
	// SRSRAN_SF_LEN_PRB(50) ((uint32_t)SRSRAN_SF_LEN(768))
	// SRSRAN_SF_LEN_PRB(50) -> 11520
	uint32_t sf_len = SRSRAN_SF_LEN_PRB(cell.nof_prb);

	/* sampling rate is 15kHz * symbol size (which is 768 subcarriers)
	 * which yields sample rate of 11.52 MHz
	 */
	int srate = srsran_sampling_freq_hz(cell.nof_prb);

	// configure the signal interrupt to exit cleanly
	signal(SIGINT, sig_int_handler);
	sigset_t sigset;
	sigemptyset(&sigset);
	sigaddset(&sigset, SIGINT);
	sigprocmask(SIG_UNBLOCK, &sigset, NULL);

	// open the radio device for use
	if (srsran_rf_open_devname(&radio, prog_args.rf_dev, prog_args.rf_args, /*# of channels*/ 1)) {
		ERROR("Error opening rf");
		exit(-1);
	}

	// configure radio parameters
	srsran_rf_set_tx_gain(&radio, prog_args.rf_gain);
	srsran_rf_set_tx_srate(&radio, (double)srate);
	srsran_rf_set_tx_freq(&radio, /*# of channels*/ 1, prog_args.rf_freq);

	// report radio parameters
	printf("Tx gain:\t%.2f dB\n", prog_args.rf_gain);
	printf("Sample rate:\t%.2f MHz\n", (float)srate/1000000);
	printf("RF freq:\t%.6f MHz\n", (float) prog_args.rf_freq / 1000000);

	for(int i = 0; i < 10; i++) {
		//-----------------------------------------------------------------
		// copied from pscch_test.c and then modified as needed

		/* Create a sidelink resource pool
		 *
		 * This sets up the resource pool object to assign the size and number of
		 * subchannels (for 50 PRB there are 5 subchannels of 10 PRB each)
		 */
		srsran_sl_comm_resource_pool_t sl_comm_resource_pool;
		if (srsran_sl_comm_resource_pool_get_default_config(&sl_comm_resource_pool, cell) != SRSRAN_SUCCESS) {
			ERROR("Error initializing sl_comm_resource_pool");
			return SRSRAN_ERROR;
		}

		/* The number of resource elements in a subframe
		 *
		 * A resource element (RE) is the smallest physical channel unit, addressable
		 * by subcarrier index k and symbol index l within a PRB
		 *
		 * As there are 50 PRBs with 12 subcarriers over 7 OFDM symbols (under normal CP)
		 * per slot (1/2 subframe), then 2 slots * 50 PRBs * 12 subcarriers * 7 symbols
		 * gives the total number of resource elements in a subframe, 8400.
		 */
		uint32_t sf_n_re   = SRSRAN_SF_LEN_RE(cell.nof_prb, cell.cp);

		/* A "vector" of complex floats of size equal to the number of resource elements
		 * in a subframe. This buffer will be sent to the USRP to be transmitted.
		 */
		cf_t*    sf_buffer = srsran_vec_cf_malloc(sf_n_re);
		srsran_vec_cf_zero(sf_buffer, sf_n_re);

		// SCI (Sidelink Control Information)

		// SCI structure from srsRAN library
		srsran_sci_t sci;

		/* Initialize the SCI structure
		 *
		 * This fills in SCI information from the sidelink cell and resource pool such
		 * as the number of PRBs and transmission mode, as well as setting the SCI Format
		 * to format 1 (for Rel. 14, as opposed to format 0 for Rel. 12 D2D which is an
		 * antecedent of LTE-V2X
		 */
		srsran_sci_init(&sci, &cell, &sl_comm_resource_pool);

		// MCS index 2 is QPSK, which is always used for PSCCH. 16-QAM is supported for PSSCH only.
		sci.mcs_idx = 2;

		// PSCCH (Physical Sidelink Control CHannel)

		// PSCCH structure from the srsRAN library
		srsran_pscch_t pscch;

		/* Initialize the PSCCH structure
		 *
		 * This configures the PSCCH by allocating memory for all subcomponents (e.g., codeword
		 * buffer) and prepares the PSCCH to be loaded with a payload. It also sets up elements
		 * like the CRC polynomial and coding parameters for use in encoding (and similar)
		 * operations.
		 */
		if (srsran_pscch_init(&pscch, SRSRAN_MAX_PRB) != SRSRAN_SUCCESS) {
			ERROR("Error in PSCCH init");
			return SRSRAN_ERROR;
		}

		/* This sets parameters of the PSCCH according to the sidelink cell; for example, the
		 * number of symbols and SCI length are set based on sidelink transmission mode (1-4)
		 */
		if (srsran_pscch_set_cell(&pscch, cell) != SRSRAN_SUCCESS) {
			ERROR("Error in PSCCH init");
			return SRSRAN_ERROR;
		}

		// Create a byte array to hold the SCI bits which will be transmitted
		uint8_t sci_tx[SRSRAN_SCI_MAX_LEN] = {};

		// Pack the SCI message as format 1 into the sci_tx array of bits
		if (srsran_sci_format1_pack(&sci, sci_tx) != SRSRAN_SUCCESS) {
			printf("Error packing sci format 1\n");
			return SRSRAN_ERROR;
		}

		printf("Tx payload: ");
		srsran_vec_fprint_hex(stdout, sci_tx, sci.sci_len);

		/* Encode the formatted SCI message as PSCCH and place in the subframe buffer at
		 * prb_start_idx, which is zero here.
		 */
		srsran_pscch_encode(&pscch, sci_tx, sf_buffer, prb_start_idx);

		// PSSCH (Physical Sidelink Shared CHannel)

		srsran_pssch_t pssch = {};

		/* Initialize the PSSCH with channel-specific parameters, as above for PSCCH */
		if (srsran_pssch_init(&pssch, &cell, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
			ERROR("Error initializing PSSCH");
			return SRSRAN_ERROR;
		}

		// the largest number of PRBs allowed for DFT pre-coding
		uint32_t nof_prb_pssch = srsran_dft_precoding_get_valid_prb(cell.nof_prb);

		/* NXID is the 16-bit CRC of the PSCCH SCI message, used as the PSSCH scrambling identity
		 */
		uint32_t N_x_id = 255;

		// array to hold the bit representation of the transport block (TB) to be sent via PSSCH
		uint8_t tb[SRSRAN_SL_SCH_MAX_TB_LEN] = {};

		// configure the PSSCH structure with appropriate parameters
		srsran_pssch_cfg_t pssch_cfg = {
				prb_start_idx + pscch.pscch_nof_prb, /* PRB start index for PSSCH immediately after PSCCH */
				nof_prb_pssch,
				N_x_id,
				4, /*mcs_idx - MCS 4 is 16-QAM*/
				0, /*Resource Indication Value (RIV) index*/
				0  /*subframe index*/
		};

		// configure the PSSCH using the configuration structure created above
		if (srsran_pssch_set_cfg(&pssch, pssch_cfg) != SRSRAN_SUCCESS) {
			ERROR("Error configuring PSSCH");
			exit(-1);
		}

		// Randomize data to fill the transport block
		struct timeval tv;
		gettimeofday(&tv, NULL);
		random_gen = srsran_random_init(tv.tv_usec);
		for (int i = 0; i < pssch.sl_sch_tb_len; i++) {
			tb[i] = srsran_random_uniform_int_dist(random_gen, 0, 1);
		}

		/* Encode and place TB in PSSCH RBs of sf_buffer
		 *
		 */
		if (srsran_pssch_encode(&pssch, tb, pssch.sl_sch_tb_len, sf_buffer) != SRSRAN_SUCCESS) {
			ERROR("Error encoding PSSCH");
			exit(-1);
		}

		// int srsran_rf_send(srsran_rf_t* rf, void* data, uint32_t nsamples, bool blocking)
		srsran_rf_send(&radio, sf_buffer, sf_len, true);

		printf("Sent data to USRP!\n");

		free(sf_buffer);
		srsran_sci_free(&sci);
		srsran_pscch_free(&pscch);
		srsran_pssch_free(&pssch);
	}

	srsran_rf_close(&radio);
}
