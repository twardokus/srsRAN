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

#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/phch/sci.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"
#include "srsran/phy/utils/random.h"
#include "srsran/phy/rf/rf.h"

// create a radio variable for RF
static srsran_rf_t radio;

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
  args->disable_plots          = false;
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

	//-----------------------------------------------------------------
	// copied from pscch_test.c and then modified as needed

	int ret = SRSRAN_ERROR;

	parse_args(&prog_args, argc, argv);

	srsran_sl_comm_resource_pool_t sl_comm_resource_pool;
	if (srsran_sl_comm_resource_pool_get_default_config(&sl_comm_resource_pool, cell) != SRSRAN_SUCCESS) {
		ERROR("Error initializing sl_comm_resource_pool");
		return SRSRAN_ERROR;
	}

	uint32_t sf_n_re   = SRSRAN_SF_LEN_RE(cell.nof_prb, cell.cp);
	cf_t*    sf_buffer = srsran_vec_cf_malloc(sf_n_re);

	// SCI
	srsran_sci_t sci;
	srsran_sci_init(&sci, &cell, &sl_comm_resource_pool);
	sci.mcs_idx = 2;

	// PSCCH
	srsran_pscch_t pscch;
	if (srsran_pscch_init(&pscch, SRSRAN_MAX_PRB) != SRSRAN_SUCCESS) {
		ERROR("Error in PSCCH init");
		return SRSRAN_ERROR;
	}

	if (srsran_pscch_set_cell(&pscch, cell) != SRSRAN_SUCCESS) {
		ERROR("Error in PSCCH init");
		return SRSRAN_ERROR;
	}

	// SCI message bits
	uint8_t sci_tx[SRSRAN_SCI_MAX_LEN] = {};
	if (sci.format == SRSRAN_SCI_FORMAT0) {
		if (srsran_sci_format0_pack(&sci, sci_tx) != SRSRAN_SUCCESS) {
			printf("Error packing sci format 0\n");
			return SRSRAN_ERROR;
		}
	} else if (sci.format == SRSRAN_SCI_FORMAT1) {
		if (srsran_sci_format1_pack(&sci, sci_tx) != SRSRAN_SUCCESS) {
			printf("Error packing sci format 1\n");
			return SRSRAN_ERROR;
		}
	}

	printf("Tx payload: ");
	srsran_vec_fprint_hex(stdout, sci_tx, sci.sci_len);

	// Put SCI into PSCCH
	srsran_pscch_encode(&pscch, sci_tx, sf_buffer, prb_start_idx);

	//-----------------------------------------------------------------
	// copied from pssch_test.c and then modified as needed
	srsran_pssch_t pssch = {};
	if (srsran_pssch_init(&pssch, &cell, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
		ERROR("Error initializing PSSCH");
		return SRSRAN_ERROR;
	}

	uint32_t nof_prb_pssch = srsran_dft_precoding_get_valid_prb(cell.nof_prb);
	uint32_t N_x_id        = 255;
	cf_t*    sf_buffer_2     = srsran_vec_cf_malloc(sf_n_re);
	if (!sf_buffer_2) {
		ERROR("Error allocating memory");
		return SRSRAN_ERROR;
	}
	srsran_vec_cf_zero(sf_buffer_2, sf_n_re);

	// Transport block buffer
	uint8_t tb[SRSRAN_SL_SCH_MAX_TB_LEN] = {};

	srsran_pssch_cfg_t pssch_cfg = {prb_start_idx + pscch.pscch_nof_prb, nof_prb_pssch, N_x_id, /*mcs_idx*/ 4, 0, 0};
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

	// PSSCH encoding
	if (srsran_pssch_encode(&pssch, tb, pssch.sl_sch_tb_len, sf_buffer_2) != SRSRAN_SUCCESS) {
		ERROR("Error encoding PSSCH");
		exit(-1);
	}

	printf("Made it here\n");

	if (srsran_rf_open_devname(&radio, prog_args.rf_dev, prog_args.rf_args, prog_args.nof_rx_antennas)) {
		ERROR("Error opening rf");
		exit(-1);
	}

	uint32_t sf_len = SRSRAN_SF_LEN_PRB(cell.nof_prb);
	// int srsran_rf_send(srsran_rf_t* rf, void* data, uint32_t nsamples, bool blocking)
	srsran_rf_send(&radio, sf_buffer_2, sf_len, true);

	printf("PSCCH has %d symbols\n", pscch.nof_symbols);
	printf("PSSCH has %d symbols\n", pssch.nof_tx_symbols);
	printf("\n");
	printf("PSSCH has %d as scfdma_symbols_length\n", pssch.scfdma_symbols_len);

	free(sf_buffer);
	free(sf_buffer_2);
	srsran_sci_free(&sci);
	srsran_pscch_free(&pscch);
	srsran_pssch_free(&pssch);

	return ret;
}
