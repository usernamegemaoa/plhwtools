/*
  E-Paper test - plhwtools

  Copyright (C) 2011, 2012, 2013 Plastic Logic Limited

      Guillaume Tucker <guillaume.tucker@plasticlogic.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <assert.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <libplepaper.h>
#include "libplhw.h"

#define LOG_TAG "plhw"
#include "log.h"

static const char APP_NAME[] = "plhwtools";
static const char VERSION[] = "0.5";
static const char DESCRIPTION[] = "Plastic Logic hardware tools";
static const char LICENSE[] =
	"This program is distributed in the hope that it will be useful,\n"
	"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
	"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
	"GNU General Public License for more details.\n";
static const char COPYRIGHT[] =
	"Copyright (C) 2011, 2012, 2013 Plastic Logic Limited";

struct ctx {
	struct cpld *cpld;
	struct hvpmic *hvpmic;
	struct dac5820 *dac;
	struct adc11607 *adc;
	struct eeprom *eeprom;
	struct pbtn *pbtn;
	struct plep *plep;
};

struct command {
	const char *cmd;
	const char *help;
	int (*run) (struct ctx *, int argc, char **argv);
};

struct switch_id {
	const char *name;
	int id;
};

#define DAC_CH DAC5820_CH_A
#define DAC_ON DAC5820_POW_ON
#define DAC_OFF DAC5820_POW_OFF_100K

static int g_abort = 0;
static struct termios g_original_stdin_termios;
static enum { TERM_IN_BLANK, TERM_IN_ERROR, TERM_IN_SAVED, TERM_IN_EDITED }
	g_stdin_termios_state = TERM_IN_BLANK;
static const char *g_i2c_bus = PLHW_DEF_I2C_BUS;
static unsigned g_i2c_addr = 0;
static const char *g_opt = NULL;

/* Top-level */
static void print_help(const struct command *commands, const char *help_cmd);
static int run_cmd(struct ctx *ctx, const struct command *commands,

                   int argc, char **argv);
static void sigint_abort(int signum);

/* CPLD */
static const char help_cpld[];
static struct cpld *require_cpld(struct ctx *ctx);
static int run_cpld(struct ctx *ctx, int argc, char **argv);
static void dump_cpld_data(const struct cpld *cpld);

/* HVPMIC */
static const char help_hvpmic[];
static struct hvpmic *require_hvpmic(struct ctx *ctx);
static int run_hvpmic(struct ctx *ctx, int argc, char **argv);
static int set_hvpmic_timing(struct hvpmic *hvpmic, int argc, char **argv);
static int set_hvpmic_timings(struct hvpmic *hvpmic, int argc, char **argv);
static int set_hvpmic_vcom(struct hvpmic *hvpmic, int argc, char **argv);
static int get_hvpmic_fault(struct hvpmic *hvpmic);
static int dump_hvpmic_state(struct hvpmic *hvpmic);
static int dump_hvpmic_en(struct hvpmic *hvpmic, enum hvpmic_en_id id);
static int dump_hvpmic_timings(struct hvpmic *hvpmic);
static int dump_hvpmic_vcom(struct hvpmic *hvpmic);
static int dump_hvpmic_temperature(struct hvpmic *hvpmic);

static const char HVPMIC_TIMINGS_SEQ0[HVPMIC_NB_TIMINGS] = {
	8, 2, 11, 3, 0, 0, 0, 0
};

/* DAC */
static const char help_dac[];
static struct dac5820 *require_dac(struct ctx *ctx);
static int run_dac(struct ctx *ctx, int argc, char **argv);

/* ADC */
static const char help_adc[];
static int run_adc(struct ctx *ctx, int argc, char **argv);

/* GPIO (push buttons) */
static const char help_pbtn[];
static int run_pbtn(struct ctx *ctx, int argc, char **argv);
static int pbtn_abort_cb(void);

/* EEPROM */
static const char help_eeprom[];
static int run_eeprom(struct ctx *ctx, int argc, char **argv);
static int full_rw_eeprom(struct eeprom *eeprom);
static int rw_file_eeprom(struct eeprom *eeprom, int fd, int write_file);
static void log_eeprom_progress(size_t total, size_t rem, const char *msg);

/* Power */
static const char help_eeprom[];
static int run_power(struct ctx *ctx, int argc, char **argv);
static int power_on_seq0(struct ctx *ctx, char vcom);
static int power_off_seq0(struct ctx *ctx);

/* ePDC */
static const char help_epdc[];
static struct plep *require_epdc(struct ctx *ctx);
static int run_epdc(struct ctx *ctx, int argc, char **argv);

/* Utilities */
static const char help_power[];
static int switch_on_off(const struct switch_id *switches, void *ctx,
			 const char *sw_name, const char *on_off,
			 int (*get_sw) (void *ctx, int sw_id),
			 int (*set_sw) (void *ctx, int sw_id, int on));
static int save_stdin_termios(void);
static int restore_stdin_termios(void);
static int disable_stdin_buffering(void);
static int get_on_off_opt(const char *on_off);
static void dump_hex_data(const char *data, size_t size);

/* Power sequence configuration */

static const struct power_seq {
	const char *name;
	int (*on) (struct ctx *ctx, char vcom);
	int (*off) (struct ctx *ctx);
	char timing[HVPMIC_NB_TIMINGS];
} seqs[] = {
	{ .name = "seq0", .on = power_on_seq0, .off = power_off_seq0,
	  .timing = { 8, 2, 11, 3, 0, 0, 0, 0 } },
	{ .name = NULL },
};

static const struct power_seq *get_power_seq(int argc, char **argv);

/* ----------------------------------------------------------------------------
 * Top-level
 */

int main(int argc, char **argv)
{
#define CMD_STRUCT(CMD)						\
	{ .cmd = #CMD, .help = help_##CMD, .run = run_##CMD }

	const struct command commands[] = {
		CMD_STRUCT(cpld),
		CMD_STRUCT(hvpmic),
		CMD_STRUCT(dac),
		CMD_STRUCT(adc),
		CMD_STRUCT(pbtn),
		CMD_STRUCT(eeprom),
		CMD_STRUCT(power),
		CMD_STRUCT(epdc),
		{ .cmd = NULL, .help = NULL, .run = NULL }
	};

#undef CMD_STRUCT

	static const char *OPTIONS = "h::va:b:o:";
	struct ctx ctx = {
		.cpld = NULL,
		.hvpmic = NULL,
		.dac = NULL,
		.adc = NULL,
		.eeprom = NULL,
		.pbtn = NULL,
	};

	__sighandler_t original_sigint_handler;
	int ret = -1;
	int c;

	while ((c = getopt(argc, argv, OPTIONS)) != -1) {
		switch (c) {
		case 'h':
			print_help(commands, optarg);
			exit(EXIT_SUCCESS);
			break;

		case 'v':
			printf("%s v%s - %s\n%s\n%s\n", APP_NAME, VERSION,
			       DESCRIPTION, COPYRIGHT, LICENSE);
			exit(EXIT_SUCCESS);
			break;

		case 'a': {
			unsigned long addr;

			errno = 0;
			addr = strtoul(optarg, NULL, 16);

			if (errno) {
				LOG("Failed to parse I2C address");
				exit(EXIT_FAILURE);
			}

			g_i2c_addr = (unsigned) addr;
			break;
		}

		case 'b':
			g_i2c_bus = optarg;
			break;

		case 'o':
			g_opt = optarg;
			break;

		case '?':
		default:
			LOG("Invalid arguments");
			print_help(commands, NULL);
			exit(EXIT_FAILURE);
			break;
		}
	}

	if (save_stdin_termios() < 0)
		LOG("Warning: failed to save stdin termios");

	original_sigint_handler = signal(SIGINT, sigint_abort);

	/* -- command line arguments -- */

	if (optind == argc) {
		print_help(commands, NULL);
		exit(EXIT_SUCCESS);
	}

	ret = run_cmd(&ctx, commands, (argc - optind), &argv[optind]);

	/* -- clean-up --- */

	if (ctx.cpld != NULL)
		cpld_free(ctx.cpld);

	if (ctx.hvpmic != NULL)
		hvpmic_free(ctx.hvpmic);

	if (ctx.dac != NULL)
		dac5820_free(ctx.dac);

	if (ctx.adc != NULL)
		adc11607_free(ctx.adc);

	if (ctx.eeprom != NULL)
		eeprom_free(ctx.eeprom);

	if (ctx.pbtn != NULL)
		pbtn_free(ctx.pbtn);

	if (ctx.plep != NULL)
		plep_free(ctx.plep);

	if (restore_stdin_termios() < 0)
		LOG("Warning: failed to restore stdin termios");

	signal(SIGINT, original_sigint_handler);

	exit((ret < 0) ? EXIT_FAILURE : EXIT_SUCCESS);
}

static void print_help(const struct command *commands, const char *help_cmd)
{
	const struct command *cmd;

	if (help_cmd != NULL) {
		for (cmd = commands; cmd->cmd != NULL; ++cmd) {
			if (!strcmp(cmd->cmd, help_cmd)) {
				printf("Command: %s\n%s", cmd->cmd, cmd->help);
				return;
			}
		}
	}

	printf(
"Usage: %s <OPTIONS> <COMMAND_NAME> <COMMAND_ARGUMENTS>\n"
"\n"
"COMMAND_NAME:\n"
"    The following commands can be used (arguments are detailed separately):\n"
"\n"
"    cpld       Control Plasstic Logic CPLD over I2C register interface\n"
"    hvpmic     Control MAX17135 HV PMIC (timings, switches)\n"
"    dac        Control DAC power and register value\n"
"    adc        Read ADC values\n"
"    pbtn       Push button test procedure using I2C GPIO expander\n"
"    eeprom     Read/write/test display EEPROM\n"
"    power      Run full power on/off sequence using multiple devices\n"
"\n"
"OPTIONS:\n"
"  -h [COMMAND]\n"
"    Show this help message and exit.  If COMMAND is provided, only show the\n"
"    help message for the given command.\n"
"\n"
"  -v\n"
"    Show the version, copyright and license information and exit.\n"
"\n"
"  -b I2C_BUS_DEVICE\n"
"    Specify the I2C bus device to be used, typically /dev/i2c-X where X is\n"
"    the I2C bus number.\n"
"\n"
"  -a I2C_ADDRESS\n"
"    Specify the I2C address of the device to be used with the command.\n"
"    This only applies to commands that use a single I2C device.\n"
"\n"
"  -o COMMAND_OPTIONS\n"
"    Optional argument string which can be used by the command.  Please see\n"
"    each command help for more details.\n"
"\n", APP_NAME);

	for (cmd = commands; cmd->cmd != NULL; ++cmd)
		printf("Command: %s\n%s\n", cmd->cmd, cmd->help);
}

static int run_cmd(struct ctx *ctx, const struct command *commands,
                   int argc, char **argv)
{
	const char *cmd_str = argv[0];
	char **cmd_argv = &argv[1];
	int cmd_argc = argc - 1;
	const struct command *cmd;
	int ret = -1;

	for (cmd = commands; cmd->cmd != NULL; ++cmd) {
		if (!strcmp(cmd->cmd, cmd_str)) {
			ret = cmd->run(ctx, cmd_argc, cmd_argv);
			break;
		}
	}

	if (cmd->cmd == NULL) {
		LOG("invalid arguments");
		print_help(commands, NULL);
	} else if (ret < 0) {
		LOG("command failed");
	}

	return ret;
}

static void sigint_abort(int signum)
{
	if (signum == SIGINT) {
		LOG("abort!");
		g_abort = 1;
	}
}

/* ----------------------------------------------------------------------------
 * CPLD
 */

static struct cpld *require_cpld(struct ctx *ctx)
{
	if (ctx->cpld == NULL)
		ctx->cpld = cpld_init(g_i2c_bus, g_i2c_addr ?
				      g_i2c_addr : CPLD_DEF_I2C_ADDR);

	return ctx->cpld;
}

static int _cpld_set_switch(void *cpld, int sw, int on)
{
	return cpld_set_switch(cpld, sw, on);
}

static int _cpld_get_switch(void *cpld, int sw)
{
	return cpld_get_switch(cpld, sw);
}

static int run_cpld(struct ctx *ctx, int argc, char **argv)
{
	static const struct switch_id switches[] = {
		{ .name = "hv",           .id = CPLD_HVEN },
		{ .name = "vcom_en",      .id = CPLD_COM_SW_EN },
		{ .name = "vcom_close",   .id = CPLD_COM_SW_CLOSE },
		{ .name = "vcom_psu",     .id = CPLD_COM_PSU },
		{ .name = "bpcom_clamp",  .id = CPLD_BPCOM_CLAMP },
		{ .name = NULL,           .id = -1 }
	};

	const char *cmd;
	const char *arg;
	struct cpld *cpld = require_cpld(ctx);

	if (cpld == NULL)
		return -1;

	if (argc < 1) {
		LOG("CPLD v%i, board id: %i", cpld_get_version(cpld),
		    cpld_get_board_id(cpld));

		LOG_N("initial CPLD data: [");
		dump_cpld_data(cpld);
		fprintf(LOG_FILE, "]\n");

		return 0;
	}

	cmd = argv[0];
	arg = (argc > 1) ? argv[1] : NULL;

	if (!strcmp(cmd, "version")) {
		const int ver = cpld_get_version(cpld);

		if (ver < 0)
			return -1;

		printf("%i\n", ver);

		return 0;
	}

	return switch_on_off(switches, cpld, cmd, arg, _cpld_get_switch,
			     _cpld_set_switch);
}

static void dump_cpld_data(const struct cpld *cpld)
{
	size_t size = cpld_get_data_size(cpld);
	char *data = malloc(size);
	const char *end;
	const char *byte;
	int n;

	if (data == NULL) {
		LOG("failed to allocate buffer");
		return;
	}

	n = cpld_dump(cpld, data, size);
	end = &data[n];

	for (byte = data; byte != end; ++byte)
		fprintf(LOG_FILE, "%02X ", *byte);

	fprintf(LOG_FILE, "\b");
	free(data);
}

/* ----------------------------------------------------------------------------
 * HV PMIC
 */

static struct hvpmic *require_hvpmic(struct ctx *ctx)
{
	if (ctx->hvpmic == NULL)
		ctx->hvpmic = hvpmic_init(g_i2c_bus, g_i2c_addr ?
					  g_i2c_addr : HVPMIC_DEF_I2C_ADDR);

	return ctx->hvpmic;
}

static int run_hvpmic(struct ctx *ctx, int argc, char **argv)
{
	struct hvpmic *hvpmic = require_hvpmic(ctx);
	const char *cmd_str;
	int on;

	if (hvpmic == NULL)
		return -1;

	if (argc == 0)
		return dump_hvpmic_state(hvpmic);

	cmd_str = argv[0];

	if (!strcmp(cmd_str, "timing"))
		return set_hvpmic_timing(hvpmic, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "timings"))
		return set_hvpmic_timings(hvpmic, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "vcom"))
		return set_hvpmic_vcom(hvpmic, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "fault"))
		return get_hvpmic_fault(hvpmic);

	if (argc != 2) {
		LOG("invalid arguments");
		return -1;
	}

	on = get_on_off_opt(argv[1]);

	if (on < 0) {
		LOG("invalid on/off value");
		return -1;
	}

	if (!strcmp(cmd_str, "en"))
		return hvpmic_set_en(hvpmic, HVPMIC_EN_EN, on);

	if (!strcmp(cmd_str, "cen"))
		return hvpmic_set_en(hvpmic, HVPMIC_EN_CEN, on);

	if (!strcmp(cmd_str, "cen2"))
		return hvpmic_set_en(hvpmic, HVPMIC_EN_CEN2, on);

	LOG("invalid arguments");

	return -1;
}

static int set_hvpmic_timing(struct hvpmic *hvpmic, int argc, char **argv)
{
	int timing_no;
	int timing_ms;

	if (argc < 2) {
		LOG("invalid arguments");
		return -1;
	}

	timing_no = atoi(argv[0]);

	if ((timing_no < 0) || (timing_no >= HVPMIC_NB_TIMINGS)) {
		LOG("invalid timing number %i (valid: 0 - %i)",
		    timing_no, HVPMIC_NB_TIMINGS);
		return -1;
	}

	timing_ms = atoi(argv[1]);

	if ((timing_ms < 0) || (timing_ms > 255)) {
		LOG("invalid timing value %i (valid: 0 - 255)", timing_ms);
		return -1;
	}

	LOG("setting timing #%i to %i ms", timing_no, timing_ms);

	return hvpmic_set_timing(hvpmic, timing_no, timing_ms);
}

static int set_hvpmic_timings(struct hvpmic *hvpmic, int argc, char **argv)
{
	char timings[HVPMIC_NB_TIMINGS];
	int stat;
	int i;

	if (argc < 1) {
		stat = hvpmic_get_timings(hvpmic, timings, HVPMIC_NB_TIMINGS);

		if (stat < 0) {
			LOG("failed to get the HVPMIC timings");
			return stat;
		}

		if (stat != HVPMIC_NB_TIMINGS) {
			LOG("could only read %i timings", stat);
			return -1;
		}

		for (i = 0; i < HVPMIC_NB_TIMINGS; ++i)
			printf("%d: %d\n", i, timings[i]);
	} else {
		const struct power_seq *seq;
		int n_timings;

		if (argc == 1)
			seq = get_power_seq(argc, argv);
		else
			seq = NULL;

		if (seq != NULL) {
			LOG("Setting timings for %s:", seq->name);

			for (i = 0; i < HVPMIC_NB_TIMINGS; ++i) {
				LOG("%d: %d", i, seq->timing[i]);
				timings[i] = seq->timing[i];
			}

			n_timings = HVPMIC_NB_TIMINGS;
		} else {
			if (argc > HVPMIC_NB_TIMINGS) {
				n_timings = HVPMIC_NB_TIMINGS;
				LOG("warning: only using the %d first timings",
				    n_timings);
			} else {
				n_timings = argc;
			}

			for (i = 0; i < n_timings; ++i) {
				int t = atoi(argv[i]);

				if ((t < 0) || (t > 255)) {
					LOG("invalid timing %i "
					    "(valid: 0 - 255)", t);
					return -1;
				}

				timings[i] = t;
			}
		}

		stat = hvpmic_set_timings(hvpmic, timings, n_timings);

		if (stat) {
			LOG("failed to write the timings");
			return stat;
		}
	}

	return 0;
}

static int set_hvpmic_vcom(struct hvpmic *hvpmic, int argc, char **argv)
{
	int vcom_raw;

	if (argc < 1) {
		LOG("invalid arguments");
		return -1;
	}

	vcom_raw = atoi(argv[0]);

	if ((vcom_raw < 0) || (vcom_raw > 255)) {
		LOG("invalid VCOM value %i (valid: 0 - 255)", vcom_raw);
		return -1;
	}

	LOG("setting VCOM to %i (0x%02X)", vcom_raw, vcom_raw);

	return hvpmic_set_vcom(hvpmic, (char) vcom_raw);
}

#define HVPMIC_FAULT_CASE(id) \
	case id: fault_str = #id; break;

static int get_hvpmic_fault(struct hvpmic *hvpmic)
{
	int fault = hvpmic_get_fault(hvpmic);
	const char *fault_str = NULL;

	if (fault < 0) {
		LOG("failed to read HVPMIC fault id");
		return -1;
	}

	switch (fault) {
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_NONE)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_FBPG)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_HVINP)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_HVINN)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_FBNG)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_HVINPSC)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_HVINNSC)
	HVPMIC_FAULT_CASE(HVPMIC_FAULT_OT)
	}

	if (fault_str == NULL) {
		LOG("invalid HVPMIC fault id");
		return -1;
	}

	LOG("HVPMIC fault: %s", fault_str);

	return 0;
}

#undef HVPMIC_FAULT_CASE

static int dump_hvpmic_state(struct hvpmic *hvpmic)
{
	int ret = 0;

	LOG("HVPMIC id: 0x%02X, rev: 0x%02X",
	    hvpmic_get_prod_id(hvpmic), hvpmic_get_prod_rev(hvpmic));

	if (dump_hvpmic_en(hvpmic, HVPMIC_EN_EN) < 0)
		ret = -1;

	if (dump_hvpmic_en(hvpmic, HVPMIC_EN_CEN) < 0)
		ret = -1;

	if (dump_hvpmic_en(hvpmic, HVPMIC_EN_CEN2) < 0)
		ret = -1;

	if (dump_hvpmic_timings(hvpmic) < 0)
		ret = -1;

	if (dump_hvpmic_vcom(hvpmic) < 0)
		ret = -1;

	if (dump_hvpmic_temperature(hvpmic) < 0)
		ret = -1;

	return ret;
}

static int dump_hvpmic_en(struct hvpmic *hvpmic, enum hvpmic_en_id id)
{
	int en = hvpmic_get_en(hvpmic, id);
	const char *en_name = NULL;

	switch (id) {
	case HVPMIC_EN_EN:   en_name = "EN";   break;
	case HVPMIC_EN_CEN:  en_name = "CEN";  break;
	case HVPMIC_EN_CEN2: en_name = "CEN2"; break;
	}

	if (en < 0) {
		LOG("failed to get %s status", en_name);
		return -1;
	}

	LOG("%s status: %s", en_name, en ? "on" : "off");

	return 0;
}

static int dump_hvpmic_timings(struct hvpmic *hvpmic)
{
	char timings[HVPMIC_NB_TIMINGS];
	int ret = hvpmic_get_timings(hvpmic, timings, HVPMIC_NB_TIMINGS);

	if (ret < 0) {
		LOG("failed to get the timings");
	} else {
		unsigned i;

		for (i = 0; i < HVPMIC_NB_TIMINGS; ++i)
			LOG("timing #%i: %3i ms", i, timings[i]);
	}

	return ret;
}

static int dump_hvpmic_vcom(struct hvpmic *hvpmic)
{
	char vcom_raw;

	if (hvpmic_get_vcom(hvpmic, &vcom_raw) < 0) {
		LOG("failed to read VCOM");
		return -1;
	}

	LOG("VCOM: %i (0x%02X)", vcom_raw, vcom_raw);

	return 0;
}

static int dump_hvpmic_temperature(struct hvpmic *hvpmic)
{
	int sensor_en;
	short temp_i, temp_e;
	float temp_i_f, temp_e_f;
	int ret = 0;

	sensor_en = hvpmic_get_temp_sensor_en(hvpmic);

	if (sensor_en < 0) {
		LOG("failed to get the temperature sensor state");
		ret = -1;
	} else {
		LOG("temperature sensor enabled: %s", sensor_en ? "yes":"no");
	}

	if ((hvpmic_get_temperature(hvpmic, &temp_i, HVPMIC_TEMP_INT) < 0)
	    || (hvpmic_get_temperature(hvpmic, &temp_e, HVPMIC_TEMP_EXT) < 0)){
		LOG("failed to read temperatures");
		ret = -1;
	} else {
		temp_i_f = hvpmic_convert_temperature(hvpmic, temp_i);
		temp_e_f = hvpmic_convert_temperature(hvpmic, temp_e);
		LOG("internal temperature: %.1f C", temp_i_f);
		LOG("external temperature: %.1f C", temp_e_f);
	}

	return ret;
}

/* ----------------------------------------------------------------------------
 * DAC
 */

static struct dac5820 *require_dac(struct ctx *ctx)
{
	if (ctx->dac == NULL) {
		ctx->dac = dac5820_init(g_i2c_bus, g_i2c_addr ?
					g_i2c_addr : DAC5820_DEF_I2C_ADDR);
	}

	return ctx->dac;
}

static int run_dac(struct ctx *ctx, int argc, char **argv)
{
	struct dac5820 *dac = require_dac(ctx);
	const char *channel_str;
	const char *arg_str;
	enum dac5820_channel_id channel_id;
	int value;

	if (ctx->dac == NULL)
		return -1;

	dac = ctx->dac;

	if (argc < 2) {
		LOG("invalid arguments");
		return -1;
	}

	channel_str = argv[0];
	arg_str = argv[1];

	if (!strcmp(channel_str, "A")) {
		channel_id = DAC5820_CH_A;
	} else if (!strcmp(channel_str, "B")) {
		channel_id = DAC5820_CH_B;
	}  else {
		LOG("invalid channel identifier (A or B)");
		return -1;
	}

	if (!strcmp(arg_str, "on"))
		return dac5820_set_power(dac, channel_id, DAC5820_POW_ON);

	if (!strcmp(arg_str, "off"))
		return dac5820_set_power(dac, channel_id,
					 DAC5820_POW_OFF_FLOAT);

	if (!strcmp(arg_str, "off1k"))
		return dac5820_set_power(dac, channel_id,
					 DAC5820_POW_OFF_1K);

	if (!strcmp(arg_str, "off100k"))
		return dac5820_set_power(dac, channel_id,
					 DAC5820_POW_OFF_100K);

	value = atoi(arg_str);

	if ((value < 0) || (value > 0xFF)) {
		LOG("invalid value %i (valid: 0 - 255)", value);
		return -1;
	}

	return dac5820_output(dac, channel_id, value);
}

/* ----------------------------------------------------------------------------
 * ADC
 */

static int run_adc(struct ctx *ctx, int argc, char **argv)
{
	struct adc11607 *adc;
	enum adc11607_ref_id ref;
	adc11607_result_t result;
	int nb_chans;
	int chan;

	if (ctx->adc == NULL)
		ctx->adc = adc11607_init(g_i2c_bus, g_i2c_addr ?
					 g_i2c_addr : ADC11607_DEF_I2C_ADDR);

	if (ctx->adc == NULL)
		return -1;

	adc = ctx->adc;
	nb_chans = adc11607_get_nb_channels(adc);

	if (argc > 0) {
		const char *ref_str = argv[0];

		if (!strcmp(ref_str, "internal")) {
			ref = ADC11607_REF_INTERNAL;
		} else if (!strcmp(ref_str, "external")) {
			/* ToDo: set the external reference voltage value */
			ref = ADC11607_REF_EXTERNAL;
		} else if (!strcmp(ref_str, "vdd")) {
			ref = ADC11607_REF_VDD;
		} else {
			LOG("invalid reference voltage");
			return -1;
		}
	} else {
		ref = ADC11607_REF_INTERNAL;
	}

	if (adc11607_set_ref(adc, ref) < 0) {
		LOG("failed to select reference voltage");
		return -1;
	}

	if (adc11607_read_results(adc) < 0) {
		LOG("failed to read the ADC results");
		return -1;
	}

	if (argc > 1) {
		static const float VCOM_COEFF = 10.0;
		const char *chan_arg = argv[1];

		if (!strcmp(chan_arg, "vcom")) {
			result = adc11607_get_result(adc, 1);

			if (result == ADC11607_INVALID_RESULT) {
				LOG("invalid result");
				return -1;
			}

			printf("%f\n",
			       adc11607_get_volts(adc, result) * VCOM_COEFF);

			return 0;
		}

		chan = atoi(chan_arg);

		if ((chan < 0) || (chan >= nb_chans)) {
			LOG("invalid channel number (0-%i)", (nb_chans - 1));
			return -1;
		}

		result = adc11607_get_result(adc, chan);

		if (result == ADC11607_INVALID_RESULT) {
			LOG("invalid result");
			return -1;
		}

		printf("%f\n", adc11607_get_volts(adc, result));

		return 0;
	}

	for (chan = 0; chan < nb_chans; ++chan) {
		result = adc11607_get_result(adc, chan);

		if (result == ADC11607_INVALID_RESULT) {
			LOG("invalid result");
			return -1;
		}

		LOG("ch. %i, result: %i (%.3f V, %i mV)", chan, result,
		    adc11607_get_volts(adc, result),
		    adc11607_get_millivolts(adc, result));
	}

	return 0;
}

/* ----------------------------------------------------------------------------
 * GPIO Expanders
 */

static int run_pbtn(struct ctx *ctx, int argc, char **argv)
{
	struct pbtn *pbtn;
	int btn;
	int ret = 0;

	if (ctx->pbtn == NULL)
		ctx->pbtn = pbtn_init(g_i2c_bus, g_i2c_addr ?
				      g_i2c_addr : PBTN_DEF_I2C_ADDR);

	if (ctx->pbtn == NULL)
		return -1;

	pbtn = ctx->pbtn;
	pbtn_set_abort_cb(pbtn, pbtn_abort_cb);

	LOG("Type Ctrl-C to abort");

	LOG("waiting for button #7 on");
	btn = pbtn_wait(pbtn, PBTN_7, 1);
	LOG("result: %i", btn);

	if (btn < 0)
		ret = -1;

	LOG("waiting for button #7 off");
	btn = pbtn_wait(pbtn, PBTN_7, 0);
	LOG("result: %i", btn);

	if (btn < 0)
		ret = -1;

	LOG("waiting for button #9 on");
	btn = pbtn_wait(pbtn, PBTN_9, 1);
	LOG("result: %i", btn);

	if (btn < 0)
		ret = -1;

	LOG("please release all buttons now");
	btn = pbtn_wait(pbtn, PBTN_ALL, 0);
	LOG("thanks");

	if (btn < 0)
		ret = -1;

	LOG("waiting for any button on");
	btn = pbtn_wait_any(pbtn, PBTN_ALL, 1);
	LOG("result: 0x%02X", btn);

	if (btn < 0)
		ret = -1;

	pbtn_set_abort_cb(pbtn, NULL);

	return ret;
}

static int pbtn_abort_cb(void)
{
	return g_abort ? -1 : 0;
}

/* ----------------------------------------------------------------------------
 * EEPROM
 */

static int run_eeprom(struct ctx *ctx, int argc, char **argv)
{
	struct eeprom *eeprom;
	const char *cmd_str;
	int fd;
	const char *f_name;
	int write_file;
	int ret;

	if (ctx->eeprom == NULL)
		ctx->eeprom = eeprom_init(g_i2c_bus, g_i2c_addr ?
					  g_i2c_addr : EEPROM_DEF_I2C_ADDR);

	if (ctx->eeprom == NULL)
		return -1;

	if (argc < 1) {
		LOG("invalid arguments");
		return -1;
	}

	eeprom = ctx->eeprom;

	if (g_opt != NULL) {
		unsigned long block_size;

		errno = 0;
		block_size = strtoul(g_opt, NULL, 10);

		if (errno) {
			LOG("Failed to parse block size");
			return -1;
		}

		eeprom_set_block_size(eeprom, block_size);
	}

	cmd_str = argv[0];

	if (!strcmp(cmd_str, "full_rw")) {
		char c;

		if (disable_stdin_buffering() < 0)
			LOG("Warning: failed to disable input buffering");

		fprintf(LOG_FILE,
			"Warning: this will overwrite the EEPROM data.\n"
			"Continue ? [N/y] ");
		c = fgetc(stdin);
		putchar('\n');

		if (restore_stdin_termios() < 0)
			LOG("Warning: failed to restore input buffering");

		if (c == 'y') {
			return full_rw_eeprom(eeprom);
		} else {
			fprintf(LOG_FILE, "aborted\n");
			return -1;
		}
	}

	if (!strcmp(cmd_str, "e2f")) {
		write_file = 1;
	} else if (!strcmp(cmd_str, "f2e")) {
		write_file = 0;
	} else {
		LOG("invalid arguments");
		return -1;
	}

	if (argc < 2) {
		fd = write_file ? STDOUT_FILENO : STDIN_FILENO;
		f_name = NULL;
	} else {
		static const int write_flags = (O_WRONLY | O_CREAT | O_TRUNC);
		static const int read_flags = (O_RDONLY);
		const int flags = write_file ? write_flags : read_flags;

		f_name = argv[1];
		fd = open(f_name, flags);

		if (fd < 0) {
			LOG("failed to open the file (%s)", f_name);
			return -1;
		}
	}

	ret = rw_file_eeprom(eeprom, fd, write_file);

	if (f_name != NULL) {
		if (write_file) {
			if (fchmod(fd, 0444) < 0) {
				LOG("Warning: chmod failed");
			}
		}

		close(fd);
	}

	return ret;
}

static int full_rw_eeprom(struct eeprom *eeprom)
{
	const size_t size = eeprom_get_size(eeprom);
	char *data_w = malloc(size);
	char *data_r = malloc(size);
	char *it_r;
	char *it_w;
	unsigned i;
	int ret = 0;

	LOG("EEPROM size: %zu", size);
	LOG("I2C block size: %zu", eeprom_get_block_size(eeprom));

	if ((data_w == NULL) || (data_r == NULL)) {
		if (data_r == NULL)
			LOG("failed to allocate read buffer");
		else
			free(data_r);

		if (data_w == NULL)
			LOG("failed to allocate write buffer");
		else
			free(data_w);

		return -1;
	}

	LOG("preparing buffers ...");

	srand(time(0));

	for (i = 0, it_w = data_w; i < size; ++i)
		*it_w++ = (rand() & 0xFF);

	memset(data_r, 0, size);

	LOG("beginning of the data to be written:");
	dump_hex_data(data_w, 256);

	LOG("writing to EEPROM ...");

	eeprom_seek(eeprom, 0);

	if (eeprom_write(eeprom, data_w, size) < 0) {
		LOG("failed to write data");
		ret = -1;
	}

	LOG("reading the EEPROM ...");

	eeprom_seek(eeprom, 0);

	if (eeprom_read(eeprom, data_r, size) < 0) {
		LOG("failed to read data");
		ret = -1;
	}

	LOG("beginning of the data read back:");
	dump_hex_data(data_r, 256);

	LOG("comparing results ...");

	for (i = 0, it_r = data_r, it_w = data_w; i < size; ++i) {
		if (*it_r++ != *it_w++) {
			const int addr = (i > 128) ? (i - 128) : 0;

			LOG("error found at address 0x%04X", i);
			LOG("dump start at 0x%04X", addr);
			LOG("written:");
			dump_hex_data(&data_w[addr], 256);
			LOG("read:");
			dump_hex_data(&data_r[addr], 256);
			ret = -1;
			break;
		}
	}

	if (!ret)
		LOG("All good.");

	free(data_r);
	free(data_w);

	return ret;
}

static int rw_file_eeprom(struct eeprom *eeprom, int fd, int write_file)
{
	static const size_t buffer_size = 4096;
	const char *msg = write_file ? "Reading" : "Writing";
	char *buffer = malloc(buffer_size);
	const size_t esize = eeprom_get_size(eeprom);
	size_t left = esize;
	int ret;

	if (buffer == NULL) {
		LOG("failed to allocate buffer");
		return -1;
	}

	eeprom_seek(eeprom, 0);
	ret = 0;

	while (left && !ret) {
		const size_t rwsz =
			(left > buffer_size) ? buffer_size : left;

		if (write_file) {
			log_eeprom_progress(esize, left - rwsz, msg);

			if (eeprom_read(eeprom, buffer, rwsz) < 0)
				ret = -1;
			else if (write(fd, buffer, rwsz) < 0)
				ret = -1;

			left -= rwsz;
		} else {
			const ssize_t rdsz = read(fd, buffer, rwsz);

			if (rdsz >= 0)
				log_eeprom_progress(esize, left - rdsz, msg);

			if (rdsz < 0)
				ret = -1;
			else if (eeprom_write(eeprom, buffer, rdsz) < 0)
				ret = -1;
			else if ((size_t) rdsz == rwsz)
				left -= rwsz;
			else
				left = 0;
		}
	}

	fprintf(LOG_FILE, "\n");
	free(buffer);

	return ret;
}

static void log_eeprom_progress(size_t total, size_t rem, const char *msg)
{
	const size_t prog = total - rem;
	const int prog_percent = prog * 100 / total;

	fprintf(LOG_FILE, "\r%s EEPROM... %i%% (%zu)",
		msg, prog_percent, prog);
}

/* ----------------------------------------------------------------------------
 * Power
 */

static int run_power(struct ctx *ctx, int argc, char **argv)
{
	const struct power_seq *seq;
	int stat;
	int on;

	if (argc < 1) {
		LOG("invalid arguments");
		return -1;
	}

	on = get_on_off_opt(argv[0]);

	if (on < 0) {
		LOG("invalid arguments (possible values are `on' or `off')");
		return -1;
	}

	seq = get_power_seq(argc - 1, argv + 1);

	if (seq == NULL)
		return -1;

	if (on) {
		/* ToDo: get the VCOM as floating point in volts */
		char vcom = 128;

		if (argc > 2) {
			const int vcom_raw = atoi(argv[2]);

			if ((vcom_raw < 0) || (vcom_raw > 255))
				LOG("invalid vcom value (valid range: 0-255)");
			else
				vcom = (char) vcom_raw;
		}

		stat = seq->on(ctx, vcom);
	} else {
		stat = seq->off(ctx);
	}

	if (!stat)
		LOG("Power %s", on ? "on" : "off");

	return stat;
}

#define STEP(cmd, msg) do {			\
		const int res = (cmd);			\
		if (res < 0) {				\
			LOG(msg" failed (ERROR)");	\
			return res;			\
		} else {				\
			LOG(msg" ok");			\
		}					\
	} while (0)

static int power_on_seq0(struct ctx *ctx, char vcom)
{
	struct cpld *cpld = require_cpld(ctx);
	struct hvpmic *hvpmic = require_hvpmic(ctx);
	struct dac5820 *dac = require_dac(ctx);

	if ((cpld == NULL) || (hvpmic == NULL))
		return -1;

	STEP(cpld_set_switch(cpld, CPLD_BPCOM_CLAMP, 1), "BPCOM clamp");
	STEP(cpld_set_switch(cpld, CPLD_HVEN, 1), "HV enable");
	STEP(hvpmic_wait_for_pok(hvpmic), "wait for POK");
	STEP(cpld_set_switch(cpld, CPLD_COM_SW_CLOSE, 0), "COM open");
	STEP(cpld_set_switch(cpld, CPLD_COM_SW_EN, 1), "COM enable");
	STEP(cpld_set_switch(cpld, CPLD_COM_PSU, 1), "COM PSU on");
	STEP(dac5820_output(dac, DAC5820_CH_A, vcom), "VCOM DAC value");
	STEP(dac5820_set_power(dac, DAC_CH, DAC_ON), "DAC power on");
	STEP(cpld_set_switch(cpld, CPLD_COM_SW_CLOSE, 1), "COM close");

	return 0;
}

static int power_off_seq0(struct ctx *ctx)
{
	struct cpld *cpld = require_cpld(ctx);
	struct dac5820 *dac = require_dac(ctx);

	if (cpld == NULL)
		return -1;

	STEP(cpld_set_switch(cpld, CPLD_COM_SW_CLOSE, 0), "COM open");
	STEP(cpld_set_switch(cpld, CPLD_COM_SW_EN, 0), "COM disable");
	STEP(dac5820_set_power(dac, DAC_CH, DAC_OFF), "DAC power off");
	STEP(cpld_set_switch(cpld, CPLD_COM_PSU, 0), "COM PSU off");
	STEP(cpld_set_switch(cpld, CPLD_HVEN, 0), "HV disable");

	return 0;
}

#undef STEP

static const struct power_seq *get_power_seq(int argc, char **argv)
{
	const struct power_seq *seq;

	if (argc < 1) {
		seq = &seqs[0];
	} else {
		const char *seq_name = argv[0];


		for (seq = seqs; seq->name != NULL; ++seq)
			if (!strcmp(seq_name, seq->name))
				break;

		if (seq->name == NULL)
			LOG("Sequence not found: %s", seq_name);
	}

	if (seq->name == NULL)
		return NULL;

	return seq;
}

/* ----------------------------------------------------------------------------
 * ePDC
 */

static struct plep *require_epdc(struct ctx *ctx)
{
	if (ctx->plep == NULL)
		ctx->plep = plep_init(NULL, NULL); /* ToDo: parse options */

	return ctx->plep;
}

static int epdc_get_set_hw_opt(struct ctx *ctx, int argc, char **argv)
{
	static const char *opt_list[_PLEP_HW_OPT_N_] = {
		[PLEP_POWER_OFF_DELAY_MS] = "power_off_delay_ms",
		[PLEP_CLEAR_ON_EXIT] = "clear_on_exit",
	};
	const char *opt_str = argv[0];
	int opt;
	int value;

	for (opt = 0; opt < _PLEP_HW_OPT_N_; ++opt) {
		if (!strcmp(opt_list[opt], opt_str))
			break;
	}

	if (opt == _PLEP_HW_OPT_N_) {
		LOG("Invalid hardware option identifier: %s", opt_str);
		return -1;
	}

	if (argc == 1) {
		if (plep_get_hw_opt(ctx->plep, opt, &value))
			return -1;
	} else {
		value = atoi(argv[1]);

		if (plep_set_hw_opt(ctx->plep, opt, value))
			return -1;
	}

	return 0;
}

static int run_epdc(struct ctx *ctx, int argc, char **argv)
{
	struct plep *plep = require_epdc(ctx);
	const char *cmd;
	int stat;

	if (plep == NULL)
		return -1;

	if (argc < 2) {
		LOG("Invalid arguments");
		return -1;
	}

	cmd = argv[0];

	if (!strcmp(cmd, "opt")) {
		stat = epdc_get_set_hw_opt(ctx, (argc - 1), &argv[1]);
	} else {
		LOG("Unsupported command");
		stat = -1;
	}

	return stat;
}

/* ----------------------------------------------------------------------------
 * Utilities
 */

static int switch_on_off(const struct switch_id *switches, void *ctx,
			 const char *sw_name, const char *on_off,
			 int (*get_sw) (void *ctx, int sw_id),
			 int (*set_sw) (void *ctx, int sw_id, int on))
{
	const struct switch_id *sw;
	int stat;

	for (sw = switches; sw->name != NULL; ++sw)
		if (!strcmp(sw->name, sw_name))
			break;

	if (!sw->name)
		return -1;

	if (on_off == NULL) {
		const int on = get_sw(ctx, sw->id);

		if (on < 0) {
			stat = on;
		} else {
			LOG("%s: %s", sw->name, on ? "on" : "off");
			stat = 0;
		}
	} else {
		const int on = get_on_off_opt(on_off);

		if (on < 0) {
			LOG("invalid value: %s, expected [on off]", on_off);
			stat = on;
		} else {
			stat = set_sw(ctx, sw->id, on);
		}
	}

	return stat;
}

static int save_stdin_termios(void)
{
	if (g_stdin_termios_state != TERM_IN_BLANK)
		return -1;

	if (tcgetattr(STDIN_FILENO, &g_original_stdin_termios) < 0)
		return -1;

	g_stdin_termios_state = TERM_IN_SAVED;

	return 0;
}

static int restore_stdin_termios(void)
{
	if ((g_stdin_termios_state == TERM_IN_BLANK)
	    || (g_stdin_termios_state == TERM_IN_SAVED))
		return 0;

	assert((g_stdin_termios_state == TERM_IN_EDITED)
	       || (g_stdin_termios_state == TERM_IN_ERROR));

	if (tcsetattr(STDIN_FILENO, TCSANOW, &g_original_stdin_termios) < 0)
		return -1;

	g_stdin_termios_state = TERM_IN_SAVED;

	return 0;
}

static int disable_stdin_buffering(void)
{
	struct termios stdin_termios;

	if ((g_stdin_termios_state == TERM_IN_BLANK)
	    || (g_stdin_termios_state == TERM_IN_ERROR))
		return -1;

	if (tcgetattr(STDIN_FILENO, &stdin_termios) < 0)
		return -1;

	stdin_termios.c_lflag &= ~(ICANON | ECHO);

	if (tcsetattr(STDIN_FILENO, TCSANOW, &stdin_termios) < 0) {
		g_stdin_termios_state = TERM_IN_ERROR;
		return -1;
	}

	g_stdin_termios_state = TERM_IN_EDITED;

	return 0;
}

static int get_on_off_opt(const char *on_off)
{
	if (!strcmp(on_off, "on"))
		return 1;

	if (!strcmp(on_off, "off"))
		return 0;

	return -1;
}

static void dump_hex_data(const char *data, size_t size)
{
	size_t n_lines;
	size_t remaining = size;
	const char *byte;
	unsigned line;

	if (!size)
		return;

	n_lines = size / 16;

	if (size % 16)
		++n_lines;

	for (line = 0, byte = data; line < n_lines; ++line) {
		const int length = (remaining < 16) ? remaining : 16;
		int i;

		if (line && !(line % 16))
			fprintf(LOG_FILE, "\n");

		for (i = 0; i < length; ++i)
			fprintf(LOG_FILE, "%02X ", *byte++);

		fprintf(LOG_FILE, "\b\n");
		remaining -= length;
	}
}

/* ----------------------------------------------------------------------------
 * Help strings
 */

static const char help_cpld[] =
"  When called with no arguments, the CPLD firmware version and board id\n"
"  are displayed.\n"
"  All the following switches accept an optional extra argument to set their\n"
"  state to `on' or `off'.  If no extra argument is provided, then their\n"
"  current state is reported.\n"
"  Switches:\n"
"    hv:           HV enable\n"
"    gate:         gate drivers VDD enable\n"
"    vcom_close:   VCOM switch close (`off' to open, `on' to close)\n"
"    vcom_psu:     VCOM power supply enable\n"
"    bpcom_clamp:  BPCOM clamp enable\n"
"  Other:\n"
"    version:      Get the CPLD version number (plain decimal on stdout)\n";

static const char help_hvpmic[] =
"  With no arguments, all the HV PMIC status information is dumped.\n"
"  To set a timing value:\n"
"    timing TIMING_NUMBER TIMING_VALUE_MS\n"
"  To set the VCOM register value:\n"
"    vcom VCOM_REGISTER_VALUE\n"
"  To switch on/off the HV power supplies (en, cen, cen2):\n"
"    [en, cen, cen2] [on, off]\n";

static const char help_dac[] =
"  First argument: either A or B to select the channel.\n"
"  Second argument:\n"
"    on:       turn the power on\n"
"    off:      turn the power off and let the output floating\n"
"    off1k:    turn the power off and pull the output to GND with 1K\n"
"    off100k:  turn the power off and pull the output to GND with 100K\n"
"    value between 0 and 255: set the output of the given channel\n";

static const char help_adc[] =
"  With no arguments, the default reference voltage is used and all channels\n"
"  are converted to volts and displayed.  When a reference voltage is\n"
"  specified but no channel is selected, then all the channels are shown.\n"
"  When a channel is selected, then a plain floating point voltage is\n"
"  generated on stdout\n"
"  First optional argument: reference voltage\n"
"    internal:  use the internal reference voltage (default)\n"
"    external:  use the external reference voltage\n"
"    vdd:       use VDD as reference voltage\n"
"  Second optional argument: channel\n"
"    channel number starting from 0: select that channel\n"
"    vcom: read the VCOM value on its dedicated channel\n";

static const char help_pbtn[] =
"  No arguments, just a small procedure to manually test the buttons\n";

static const char help_eeprom[] =
"  Supported arguments:\n"
"    full_rw:        write random data, read it back and compare\n"
"    e2f FILE_NAME:  dump EEPROM contents to a file, or stdout by default\n"
"    f2e FILE_NAME:  dump file contents or stdin by default to EEPROM\n"
"  Options: -o I2C_BLOCK_SIZE\n"
"    The maximum I2C block transfer size in bytes can be passed as an\n"
"    option.  The default is 96, but it can be increased to 512 for example\n"
"    to speed-up the operation if the platform I2C driver supportes it.  It\n"
"    can also be useful to reduce the block size in case the platform I2C\n"
"    driver supports only smaller packets.\n";

static const char help_power[] =
"  Supported arguments:\n"
"    on [seq] [vcom]\n"
"      turn the power on, with optional sequence name (seq0 by default) and\n"
"      optional HVPMIC VCOM value (0-255)\n"
"    off [seq]\n"
"      turn the power off\n";

static const char help_epdc[] =
"  This command is used to access the low-level interface to electrophoretic\n"
"  display controllers (ePDC) via the PLSDK libplepaper library.\n"
"  Supported arguments:\n"
"    opt OPT [VALUE]: Set a hardware option OPT to the given numerical\n"
"                     VALUE or print its current value if none.  Supported\n"
"                     option identifiers for OPT are:\n"
"      power_off_delay_ms: delay in milliseconds between end of display\n"
"                          update and display HV power off\n"
"      clear_on_exit:      clear the screen when the ePDC is shut down\n";
