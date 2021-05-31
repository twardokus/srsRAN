#include <semaphore.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include "srsran/common/pcap.h"
#include "srsran/phy/ch_estimation/chest_sl.h"
#include "srsran/phy/common/phy_common_sl.h"
#include "srsran/phy/dft/ofdm.h"
#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/phch/ra_sl.h"
#include "srsran/phy/phch/sci.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/phy/ue/ue_sync.h"
#include "srsran/phy/utils/bit.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

static srsran_cell_sl_t cell_sl = {.nof_prb = 50, .tm = SRSRAN_SIDELINK_TM4, .cp = SRSRAN_CP_NORM, .N_sl_id = 0};

// used for looping as well as handling signal interrupts
static bool keep_running = true;

// struct for the program arguments
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

// function to initialize default arguments
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

// create a radio variable for RF
static srsran_rf_t radio;

// variable for program arguments
static prog_args_t prog_args;

// function to handle system interrupts
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    keep_running = false;
  } else if (signo == SIGSEGV) {
    exit(1);
  }
}

// usage output if parse_args errors out
void usage(prog_args_t* args, char* prog)
{
  printf("Usage: %s [agrnmv] -f tx_frequency_hz\n", prog);
  printf("\t-a RF args [Default %s]\n", args->rf_args);
  printf("\t-d RF devicename [Default %s]\n", args->rf_dev);
  printf("\t-i input_file_name\n");
  printf("\t-m Start subframe_idx [Default %d]\n", args->file_start_sf_idx);
  printf("\t-g RF Gain [Default %.2f dB]\n", args->rf_gain);
  printf("\t-A nof_rx_antennas [Default %d]\n", args->nof_rx_antennas);
  printf("\t-c N_sl_id [Default %d]\n", cell_sl.N_sl_id);
  printf("\t-p nof_prb [Default %d]\n", cell_sl.nof_prb);
  printf("\t-s size_sub_channel [Default for 50 prbs %d]\n", args->size_sub_channel);
  printf("\t-n num_sub_channel [Default for 50 prbs %d]\n", args->num_sub_channel);
  printf("\t-t Sidelink transmission mode {1,2,3,4} [Default %d]\n", (cell_sl.tm + 1));
  printf("\t-r use_standard_lte_rates [Default %i]\n", args->use_standard_lte_rates);
#ifdef ENABLE_GUI
  printf("\t-w disable plots [Default enabled]\n");
#endif
  printf("\t-v srsran_verbose\n");
}

// parses arguments from argv
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
        cell_sl.N_sl_id = (int32_t)strtol(argv[optind], NULL, 10);
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
        cell_sl.nof_prb = (int32_t)strtol(argv[optind], NULL, 10);
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

int main(int argc, char** argv) {

	// gracefully handle interrupts (like Ctrl+C)
	signal(SIGINT, sig_int_handler);
	sigset_t sigset;
	sigemptyset(&sigset);
	sigaddset(&sigset, SIGINT);
	sigprocmask(SIG_UNBLOCK, &sigset, NULL);

	// parse program arguments
	parse_args(&prog_args, argc, argv);

	srsran_use_standard_symbol_size(prog_args.use_standard_lte_rates);

	// set up the sidelink resource pool
	srsran_sl_comm_resource_pool_t sl_comm_resource_pool;
	if (srsran_sl_comm_resource_pool_get_default_config(&sl_comm_resource_pool, cell_sl) != SRSRAN_SUCCESS) {
		ERROR("Error initializing sl_comm_resource_pool");
		return SRSRAN_ERROR;
	}

	// configure the radio device
	printf("Opening RF device...\n");

	if (srsran_rf_open_devname(&radio, prog_args.rf_dev, prog_args.rf_args, prog_args.nof_rx_antennas)) {
		ERROR("Error opening rf");
		exit(-1);
	}

	srsran_rf_set_tx_gain(&radio, prog_args.rf_gain);

	printf("Set TX freq: %.6f MHz\n",
			srsran_rf_set_tx_freq(&radio, prog_args.nof_rx_antennas, prog_args.rf_freq) / 1e6);
	printf("Set TX gain: %.1f dB\n", prog_args.rf_gain);
	int srate = srsran_sampling_freq_hz(cell_sl.nof_prb);

	if (srate != -1) {
		printf("Setting sampling rate %.2f MHz\n", (float)srate / 1000000);
		float srate_rf = srsran_rf_set_tx_srate(&radio, (double)srate);
		if (srate_rf != srate) {
			ERROR("Could not set sampling rate");
			exit(-1);
		}
	} else {
		ERROR("Invalid number of PRB %d", cell_sl.nof_prb);
		exit(-1);
	}

	// create PSCCH signal

	// SCI
	srsran_sci_t sci;
	srsran_sci_init(&sci, &cell_sl, &sl_comm_resource_pool);

	uint8_t sci_tx[SRSRAN_SCI_MAX_LEN] = {};

	// int srsran_sci_format1_pack(srsran_sci_t* q, uint8_t* output)
	if(srsran_sci_format1_pack(&sci, sci_tx) != SRSRAN_SUCCESS) {
		ERROR("Error packing SCI");
		exit(-1);
	}
/*
	srsran_pscch_t pscch = {};
	srsran_pssch_t pssch = {};


	// Crap below here :)

	uint8_t sci_rx[SRSRAN_SCI_MAX_LEN]      = {};
	char	sci_msg[SRSRAN_SCI_MSG_MAX_LEN] = {};

	srsran_pscch_t pscch = {};
	srsran_pssch_t pssch = {};

	if(srsran_pscch_init(&pscch, SRSRAN_MAX_PRB) != SRSRAN_SUCCESS) {
		ERROR("Error in PSCCH init");
		return SRSRAN_ERROR;
	}
	if (srsran_pscch_set_cell(&pscch, cell_sl) != SRSRAN_SUCCESS) {
		ERROR("Error in PSCCH set cell");
		return SRSRAN_ERROR;
	}
	// PSCCH Channel estimation
	srsran_chest_sl_cfg_t pscch_chest_sl_cfg = {};
	srsran_chest_sl_t     pscch_chest        = {};
	if (srsran_chest_sl_init(&pscch_chest, SRSRAN_SIDELINK_PSCCH, cell_sl, sl_comm_resource_pool) != SRSRAN_SUCCESS) {
		ERROR("Error in chest PSCCH init");
		return SRSRAN_ERROR;
	}

	if (srsran_pssch_init(&pssch, &cell_sl, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
		ERROR("Error initializing PSSCH");
		return SRSRAN_ERROR;
	}

	srsran_chest_sl_cfg_t pssch_chest_sl_cfg = {};
	srsran_chest_sl_t     pssch_chest        = {};
	if (srsran_chest_sl_init(&pssch_chest, SRSRAN_SIDELINK_PSSCH, cell_sl, sl_comm_resource_pool) != SRSRAN_SUCCESS) {
		ERROR("Error in chest PSSCH init");
		return SRSRAN_ERROR;
	}

	uint8_t tb[SRSRAN_SL_SCH_MAX_TB_LEN]            = {};
	uint8_t packed_tb[SRSRAN_SL_SCH_MAX_TB_LEN / 8] = {};

	srsran_ue_sync_t ue_sync = {};
	if (!prog_args.input_file_name) {
		srsran_cell_t cell = {};
		cell.nof_prb       = cell_sl.nof_prb;
		cell.cp            = SRSRAN_CP_NORM;
		cell.nof_ports     = 1;

		if (srsran_ue_sync_init_multi_decim_mode(&ue_sync,
												 cell.nof_prb,
												 false,
												 srsran_rf_recv_wrapper,
												 prog_args.nof_rx_antennas,
												 (void*)&radio,
												 1,
												 SYNC_MODE_GNSS)) {
			fprintf(stderr, "Error initiating sync_gnss\n");
			exit(-1);
		}

		if (srsran_ue_sync_set_cell(&ue_sync, cell)) {
			ERROR("Error initiating ue_sync");
			exit(-1);
		}

	}

	// srsran_pscch_encode(srsran_pscch_t* q, uint8_t* sci, cf_t* sf_buffer, uint32_t prb_start_idx)
	/*if(srsran_pscch_encode(srs) == SRSRAN_ERROR) {
		ERROR("Error during PSCCH encode");
		return SRSRAN_ERROR;
	}*/

	/*
	// send crafted signal to RF device
	srsran_rf_send2(&radio,
						void*        data,
						uint32_t     nsamples,
						bool         blocking,
						bool         start_of_burst,
						bool         end_of_burst
						)
	*/





	printf("Completed without errors\n");

	return SRSRAN_SUCCESS;
}
