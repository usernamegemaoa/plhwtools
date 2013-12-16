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

#include <plsdk/plconfig.h>
#include <libplepaper.h>
#include <libplhw.h>

#define LOG_TAG "plhw"
#include <plsdk/log.h>

static const char APP_NAME[] = "plhwtools";
static const char VERSION[] = "1.3";
static const char DESCRIPTION[] = "Plastic Logic hardware tools";
static const char LICENSE[] =
	"This program is distributed in the hope that it will be useful,\n"
	"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
	"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
	"GNU General Public License for more details.\n";
static const char COPYRIGHT[] =
	"Copyright (C) 2011, 2012, 2013 Plastic Logic Limited";

struct ctx {
	struct plconfig *config;
	struct cpld *cpld;
	struct max17135 *max17135;
	struct tps65185 *tps65185;
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
static const char *g_i2c_bus = NULL;
static unsigned g_i2c_addr = PLHW_NO_I2C_ADDR;
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

/* MAX17135 */
static const char help_max17135[];
static struct max17135 *require_max17135(struct ctx *ctx);
static int run_max17135(struct ctx *ctx, int argc, char **argv);
static int set_max17135_timing(struct max17135 *p, int argc, char **argv);
static int set_max17135_timings(struct max17135 *p, int argc, char **argv);
static int set_max17135_vcom(struct max17135 *p, int argc, char **argv);
static int get_max17135_fault(struct max17135 *p);
static int dump_max17135_state(struct max17135 *p);
static int dump_max17135_en(struct max17135 *p, enum max17135_en_id id);
static int dump_max17135_timings(struct max17135 *p);
static int dump_max17135_vcom(struct max17135 *p);
static int dump_max17135_temperature(struct max17135 *p);

static const char MAX17135_TIMINGS_SEQ0[MAX17135_NB_TIMINGS] = {
	8, 2, 11, 3, 0, 0, 0, 0
};

/* TPS65185 */
static const char help_tps65185[];
static struct tps65185 *require_tps65185(struct ctx *ctx);
static int run_tps65185(struct ctx *ctx, int argc, char **argv);
static int run_tps65185_vcom(struct tps65185 *p, int argc, char **argv);
static int run_tps65185_seq(struct tps65185 *p, int argc, char **argv);
static int run_tps65185_en(struct tps65185 *p, int argc, char **argv);
static int dump_tps65185_state(struct tps65185 *p);
static void dump_tps65185_seq_item(const char *name,
				   enum tps65185_strobe strobe,
				   const struct tps65185_seq *seq);

static const char *tps65185_en_id_str[6] = {
	"vgneg", "vee", "vpos", "vddh", "vcom", "v3p3",
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
struct eeprom_opt {
	unsigned i2c_addr;
	size_t data_size;
	size_t skip;
	int zero_padding;
	unsigned long block_size;
	unsigned long page_size;
};
static const char help_eeprom[];
static int run_eeprom(struct ctx *ctx, int argc, char **argv);
static int full_rw_eeprom(struct eeprom *eeprom, const struct eeprom_opt *opt);
static int pad_eeprom(struct eeprom *eeprom, size_t left,
		      const struct eeprom_opt *opt);
static int rw_file_eeprom(struct eeprom *eeprom, int fd, int write_file,
			  const struct eeprom_opt *opt);
static int parse_eeprom_opt(struct ctx *ctx, struct eeprom_opt *eopt);
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
	char timing[MAX17135_NB_TIMINGS];
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
		CMD_STRUCT(max17135),
		CMD_STRUCT(tps65185),
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
		.config = NULL,
		.cpld = NULL,
		.max17135 = NULL,
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

	ctx.config = plconfig_init(NULL, "plhwtools");

	if (ctx.config == NULL)
		exit(EXIT_FAILURE);

	if (g_i2c_bus == NULL)
		g_i2c_bus = plconfig_get_str(ctx.config, "i2c-bus", NULL);

	ret = run_cmd(&ctx, commands, (argc - optind), &argv[optind]);

	/* -- clean-up --- */

	if (ctx.cpld != NULL)
		cpld_free(ctx.cpld);

	if (ctx.max17135 != NULL)
		max17135_free(ctx.max17135);

	if (ctx.tps65185 != NULL)
		tps65185_free(ctx.tps65185);

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

	plconfig_free(ctx.config);

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
"    max17135   Control MAX17135 HV PMIC (timings, switches)\n"
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
		ctx->cpld = cpld_init(g_i2c_bus, g_i2c_addr);

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
		LOG_PRINT("]\n");

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
		LOG_PRINT("%02X ", *byte);

	LOG_PRINT("\b");
	free(data);
}

/* ----------------------------------------------------------------------------
 * HV PMIC
 */

static struct max17135 *require_max17135(struct ctx *ctx)
{
	if (ctx->max17135 == NULL)
		ctx->max17135 = max17135_init(g_i2c_bus, g_i2c_addr);

	return ctx->max17135;
}

static int run_max17135(struct ctx *ctx, int argc, char **argv)
{
	struct max17135 *max17135 = require_max17135(ctx);
	const char *cmd_str;
	int on;

	if (max17135 == NULL)
		return -1;

	if (argc == 0)
		return dump_max17135_state(max17135);

	cmd_str = argv[0];

	if (!strcmp(cmd_str, "timing"))
		return set_max17135_timing(max17135, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "timings"))
		return set_max17135_timings(max17135, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "vcom"))
		return set_max17135_vcom(max17135, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "fault"))
		return get_max17135_fault(max17135);

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
		return max17135_set_en(max17135, MAX17135_EN_EN, on);

	if (!strcmp(cmd_str, "cen"))
		return max17135_set_en(max17135, MAX17135_EN_CEN, on);

	if (!strcmp(cmd_str, "cen2"))
		return max17135_set_en(max17135, MAX17135_EN_CEN2, on);

	LOG("invalid arguments");

	return -1;
}

static int set_max17135_timing(struct max17135 *p, int argc, char **argv)
{
	int timing_no;
	int timing_ms;

	if (argc < 2) {
		LOG("invalid arguments");
		return -1;
	}

	timing_no = atoi(argv[0]);

	if ((timing_no < 0) || (timing_no >= MAX17135_NB_TIMINGS)) {
		LOG("invalid timing number %i (valid: 0 - %i)",
		    timing_no, MAX17135_NB_TIMINGS);
		return -1;
	}

	timing_ms = atoi(argv[1]);

	if ((timing_ms < 0) || (timing_ms > 255)) {
		LOG("invalid timing value %i (valid: 0 - 255)", timing_ms);
		return -1;
	}

	LOG("setting timing #%i to %i ms", timing_no, timing_ms);

	return max17135_set_timing(p, timing_no, timing_ms);
}

static int set_max17135_timings(struct max17135 *p, int argc, char **argv)
{
	char timings[MAX17135_NB_TIMINGS];
	int stat;
	int i;

	if (argc < 1) {
		stat = max17135_get_timings(p, timings, MAX17135_NB_TIMINGS);

		if (stat < 0) {
			LOG("failed to get the MAX17135 timings");
			return stat;
		}

		if (stat != MAX17135_NB_TIMINGS) {
			LOG("could only read %i timings", stat);
			return -1;
		}

		for (i = 0; i < MAX17135_NB_TIMINGS; ++i)
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

			for (i = 0; i < MAX17135_NB_TIMINGS; ++i) {
				LOG("%d: %d", i, seq->timing[i]);
				timings[i] = seq->timing[i];
			}

			n_timings = MAX17135_NB_TIMINGS;
		} else {
			if (argc > MAX17135_NB_TIMINGS) {
				n_timings = MAX17135_NB_TIMINGS;
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

		stat = max17135_set_timings(p, timings, n_timings);

		if (stat) {
			LOG("failed to write the timings");
			return stat;
		}
	}

	return 0;
}

static int set_max17135_vcom(struct max17135 *p, int argc, char **argv)
{
	int vcom_raw;

	if (argc < 1) {
		char value;

		if (max17135_get_vcom(p, &value))
			return -1;

		printf("%d\n", value);

		return 0;
	}

	vcom_raw = atoi(argv[0]);

	if ((vcom_raw < 0) || (vcom_raw > 255)) {
		LOG("invalid VCOM value %i (valid: 0 - 255)", vcom_raw);
		return -1;
	}

	LOG("setting VCOM to %i (0x%02X)", vcom_raw, vcom_raw);

	return max17135_set_vcom(p, (char) vcom_raw);
}

#define MAX17135_FAULT_CASE(id) \
	case id: fault_str = #id; break;

static int get_max17135_fault(struct max17135 *p)
{
	const int fault = max17135_get_fault(p);
	const char *fault_str = NULL;

	if (fault < 0) {
		LOG("failed to read MAX17135 fault id");
		return -1;
	}

	switch (fault) {
	MAX17135_FAULT_CASE(MAX17135_FAULT_NONE)
	MAX17135_FAULT_CASE(MAX17135_FAULT_FBPG)
	MAX17135_FAULT_CASE(MAX17135_FAULT_HVINP)
	MAX17135_FAULT_CASE(MAX17135_FAULT_HVINN)
	MAX17135_FAULT_CASE(MAX17135_FAULT_FBNG)
	MAX17135_FAULT_CASE(MAX17135_FAULT_HVINPSC)
	MAX17135_FAULT_CASE(MAX17135_FAULT_HVINNSC)
	MAX17135_FAULT_CASE(MAX17135_FAULT_OT)
	}

	if (fault_str == NULL) {
		LOG("invalid MAX17135 fault id");
		return -1;
	}

	LOG("MAX17135 fault: %s", fault_str);

	return 0;
}

#undef MAX17135_FAULT_CASE

static int dump_max17135_state(struct max17135 *p)
{
	int ret = 0;

	LOG("MAX17135 id: 0x%02X, rev: 0x%02X",
	    max17135_get_prod_id(p), max17135_get_prod_rev(p));

	if (dump_max17135_en(p, MAX17135_EN_EN) < 0)
		ret = -1;

	if (dump_max17135_en(p, MAX17135_EN_CEN) < 0)
		ret = -1;

	if (dump_max17135_en(p, MAX17135_EN_CEN2) < 0)
		ret = -1;

	if (dump_max17135_timings(p) < 0)
		ret = -1;

	if (dump_max17135_vcom(p) < 0)
		ret = -1;

	if (dump_max17135_temperature(p) < 0)
		ret = -1;

	return ret;
}

static int dump_max17135_en(struct max17135 *p, enum max17135_en_id id)
{
	const int en = max17135_get_en(p, id);
	const char *en_name;

	switch (id) {
	case MAX17135_EN_EN:   en_name = "EN";   break;
	case MAX17135_EN_CEN:  en_name = "CEN";  break;
	case MAX17135_EN_CEN2: en_name = "CEN2"; break;
	default:
		assert(!"invalid power rail identifier");
		return -1;
	}

	if (en < 0) {
		LOG("failed to get %s status", en_name);
		return -1;
	}

	LOG("%s status: %s", en_name, en ? "on" : "off");

	return 0;
}

static int dump_max17135_timings(struct max17135 *p)
{
	char timings[MAX17135_NB_TIMINGS];
	int ret = max17135_get_timings(p, timings, MAX17135_NB_TIMINGS);

	if (ret < 0) {
		LOG("failed to get the timings");
	} else {
		unsigned i;

		for (i = 0; i < MAX17135_NB_TIMINGS; ++i)
			LOG("timing #%i: %3i ms", i, timings[i]);
	}

	return ret;
}

static int dump_max17135_vcom(struct max17135 *p)
{
	char vcom_raw;

	if (max17135_get_vcom(p, &vcom_raw) < 0) {
		LOG("failed to read VCOM");
		return -1;
	}

	LOG("VCOM: %i (0x%02X)", vcom_raw, vcom_raw);

	return 0;
}

static int dump_max17135_temperature(struct max17135 *p)
{
	int sensor_en;
	short temp_i, temp_e;
	float temp_i_f, temp_e_f;
	int ret = 0;

	sensor_en = max17135_get_temp_sensor_en(p);

	if (sensor_en < 0) {
		LOG("failed to get the temperature sensor state");
		ret = -1;
	} else {
		LOG("temperature sensor enabled: %s", sensor_en ? "yes":"no");
	}

	if ((max17135_get_temperature(p, &temp_i, MAX17135_TEMP_INT) < 0)
	    || (max17135_get_temperature(p, &temp_e, MAX17135_TEMP_EXT) < 0)){
		LOG("failed to read temperatures");
		ret = -1;
	} else {
		temp_i_f = max17135_convert_temperature(p, temp_i);
		temp_e_f = max17135_convert_temperature(p, temp_e);
		LOG("internal temperature: %.1f C", temp_i_f);
		LOG("external temperature: %.1f C", temp_e_f);
	}

	return ret;
}

/* ----------------------------------------------------------------------------
 * TPS65185
 */

static struct tps65185 *require_tps65185(struct ctx *ctx)
{
	if (ctx->tps65185 == NULL)
		ctx->tps65185 = tps65185_init(g_i2c_bus, g_i2c_addr);

	return ctx->tps65185;
}

static int run_tps65185(struct ctx *ctx, int argc, char **argv)
{
	struct tps65185 *tps65185 = require_tps65185(ctx);
	const char *cmd_str;

	if (tps65185 == NULL)
		return -1;

	if (argc == 0)
		return dump_tps65185_state(tps65185);

	cmd_str = argv[0];

	if (!strcmp(cmd_str, "vcom"))
		return run_tps65185_vcom(tps65185, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "seq"))
		return run_tps65185_seq(tps65185, argc - 1, &argv[1]);

	if (!strcmp(cmd_str, "active"))
		return tps65185_set_power(tps65185, TPS65185_ACTIVE);

	if (!strcmp(cmd_str, "standby"))
		return tps65185_set_power(tps65185, TPS65185_STANDBY);

	if (!strcmp(cmd_str, "en"))
		return run_tps65185_en(tps65185, argc - 1, &argv[1]);

	LOG("unsupported command: %s", cmd_str);

	return -1;
}

static int run_tps65185_vcom(struct tps65185 *p, int argc, char **argv)
{
	int vcom_raw;

	if (argc == 0) {
		uint16_t vcom;

		if (tps65185_get_vcom(p, &vcom))
			return -1;

		printf("%d\n", vcom);

		return 0;
	}

	vcom_raw = atoi(argv[0]);

	if ((vcom_raw < 0) || (vcom_raw > 0x1FF)) {
		LOG("invalid VCOM value %d (value: 0 - 511)", vcom_raw);
		return -1;
	}

	LOG("setting VCOM to %d (0x%04X)", vcom_raw, vcom_raw);

	return tps65185_set_vcom(p, (uint16_t)vcom_raw);
}

static int run_tps65185_seq(struct tps65185 *p, int argc, char **argv)
{
	struct tps65185_seq seq;
	const char *up_down_str;
	int up;
	int i;
	char **it;

	if (argc < 1) {
		LOG("invalid arguments");
		return -1;
	}

	up_down_str = argv[0];

	if (!strcmp(up_down_str, "up")) {
		up = 1;
	} else if (!strcmp(up_down_str, "down")) {
		up = 0;
	} else {
		LOG("invalid up/down identifier");
		return -1;
	}

	if (argc == 1) {
		if (tps65185_get_seq(p, &seq, up))
			return -1;

		dump_tps65185_seq_item("VDDH", seq.vddh, &seq);
		dump_tps65185_seq_item("VPOS", seq.vpos, &seq);
		dump_tps65185_seq_item("VEE", seq.vee, &seq);
		dump_tps65185_seq_item("VNEG", seq.vneg, &seq);

		return 0;
	}

	if (argc != 9) {
		LOG("invalid sequence arguments");
		return -1;
	}

	it = &argv[1];

	{
		enum tps65185_strobe * const strobes[4] = {
			&seq.vddh, &seq.vpos, &seq.vee, &seq.vneg
		};

		for (i = 0; i < 4; ++i) {
			const int value = atoi(*it++);

			if ((value < 1) || (value > 4)) {
				LOG("invalid strobe value: %d (1-4)", value);
				return -1;
			}

			*strobes[i] = value - 1;
		}
	}

	{
		enum tps65185_delay * const delays[4] = {
			&seq.strobe1, &seq.strobe2, &seq.strobe3, &seq.strobe4
		};

		for (i = 0; i < 4; ++i) {
			const int value = atoi(*it++);

			if ((value < 3) || (value > 12) || (value % 3)) {
				LOG("invalid strobe delay value: %d", value);
				return -1;
			}

			*delays[i] = (value / 3) - 1;
		}
	}

	return tps65185_set_seq(p, &seq, up);
}

static int run_tps65185_en(struct tps65185 *p, int argc, char **argv)
{
	const char *en_str;
	enum tps65185_en_id id;
	int on;

	if (argc == 0) {
		LOG("no power rail identifier provided");
		return -1;
	}

	en_str = argv[0];

	for (id = 0; id < 6; ++id)
		if (!strcmp(en_str, tps65185_en_id_str[id]))
			break;

	if (id == 6) {
		LOG("invalid power rail identifier: %s", en_str);
		return -1;
	}

	if (argc == 1) {
		on = tps65185_get_en(p, id);

		if (on < 0)
			return -1;

		LOG("%s: %s", en_str, on ? "on" : "off");

		return 0;
	}

	on = get_on_off_opt(argv[1]);

	if (on < 0)
		return -1;

	return tps65185_set_en(p, id, on);
}

static int dump_tps65185_state(struct tps65185 *p)
{
	struct tps65185_info info;
	struct tps65185_seq seq;
	uint16_t vcom;
	enum tps65185_en_id en_id;

	tps65185_get_info(p, &info);
	LOG("version: %d.%d.%d", info.version, info.major, info.minor);

	if (tps65185_get_vcom(p, &vcom)) {
		LOG("failed to read VCOM...");
		return -1;
	}

	LOG("VCOM: %d (0x%04X)", vcom, vcom);

	if (tps65185_get_seq(p, &seq, 1))
		return -1;

	LOG("Power up sequence:");
	dump_tps65185_seq_item("VDDH", seq.vddh, &seq);
	dump_tps65185_seq_item("VPOS", seq.vpos, &seq);
	dump_tps65185_seq_item("VEE", seq.vee, &seq);
	dump_tps65185_seq_item("VNEG", seq.vneg, &seq);

	if (tps65185_get_seq(p, &seq, 0))
		return -1;

	LOG("Power down sequence:");
	dump_tps65185_seq_item("VDDH", seq.vddh, &seq);
	dump_tps65185_seq_item("VPOS", seq.vpos, &seq);
	dump_tps65185_seq_item("VEE", seq.vee, &seq);
	dump_tps65185_seq_item("VNEG", seq.vneg, &seq);

	LOG("Power rail states:");
	for (en_id = 0; en_id < 6; ++en_id) {
		int en = tps65185_get_en(p, en_id);

		if (en < 0)
			return -1;

		LOG("%s: %s", tps65185_en_id_str[en_id], en ? "on" : "off");
	}

	return 0;
}

static void dump_tps65185_seq_item(const char *name,
				   enum tps65185_strobe strobe,
				   const struct tps65185_seq *seq)
{
	enum tps65185_delay delay;

	switch (strobe) {
	case TPS65185_STROBE1:
		delay = seq->strobe1;
		break;
	case TPS65185_STROBE2:
		delay = seq->strobe2;
		break;
	case TPS65185_STROBE3:
		delay = seq->strobe3;
		break;
	case TPS65185_STROBE4:
		delay = seq->strobe4;
		break;
	default:
		assert(!"invalid strobe identifier");
		return;
	}

	LOG("%5s: STROBE%d (%d ms)", name, (strobe + 1), ((delay + 1) * 3));
}

/* ----------------------------------------------------------------------------
 * DAC
 */

static struct dac5820 *require_dac(struct ctx *ctx)
{
	if (ctx->dac == NULL)
		ctx->dac = dac5820_init(g_i2c_bus, g_i2c_addr);

	return ctx->dac;
}

static int run_dac(struct ctx *ctx, int argc, char **argv)
{
	struct dac5820 *dac = require_dac(ctx);
	const char *channel_str;
	const char *arg_str;
	enum dac5820_channel_id channel_id;
	int value;

	if (dac == NULL)
		return -1;

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
		ctx->adc = adc11607_init(g_i2c_bus, g_i2c_addr);

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
		ctx->pbtn = pbtn_init(g_i2c_bus, g_i2c_addr);

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
	struct eeprom_opt eeprom_opt;
	const char *eeprom_mode;
	const char *cmd_str;
	int fd;
	unsigned i2c_addr;
	size_t esize;
	const char *f_name;
	int write_file;
	int ret;

	if (argc < 2) {
		LOG("invalid arguments");
		return -1;
	}

	eeprom_mode = argv[0];
	cmd_str = argv[1];

	i2c_addr = g_i2c_addr;
	eeprom_opt.i2c_addr = PLHW_NO_I2C_ADDR;
	eeprom_opt.data_size = 0;
	eeprom_opt.skip = 0;
	eeprom_opt.zero_padding = 0;
	eeprom_opt.block_size = 0;
	eeprom_opt.page_size = 0;

	if (g_opt != NULL)
		if (parse_eeprom_opt(ctx, &eeprom_opt))
			return -1;

	if (eeprom_opt.i2c_addr != PLHW_NO_I2C_ADDR)
		i2c_addr = eeprom_opt.i2c_addr;
	else
		i2c_addr = g_i2c_addr;

	if (ctx->eeprom == NULL)
		ctx->eeprom = eeprom_init(g_i2c_bus, i2c_addr, eeprom_mode);

	if (ctx->eeprom == NULL)
		return -1;

	eeprom = ctx->eeprom;

	esize = eeprom_get_size(eeprom);

	if (!eeprom_opt.data_size) {
		eeprom_opt.data_size = esize;
	} else if (eeprom_opt.data_size > esize) {
		LOG("data size bigger than EEPROM size");
		return -1;
	}

	if (eeprom_opt.block_size)
		eeprom_set_block_size(eeprom, eeprom_opt.block_size);

	if (eeprom_opt.page_size)
		eeprom_set_page_size(eeprom, eeprom_opt.page_size);

	if (!strcmp(cmd_str, "full_rw")) {
		char c;

		if (disable_stdin_buffering() < 0)
			LOG("Warning: failed to disable input buffering");

		LOG_PRINT("Warning: this will overwrite the EEPROM data.\n"
			  "Continue ? [N/y] ");
		c = fgetc(stdin);
		putchar('\n');

		if (restore_stdin_termios() < 0)
			LOG("Warning: failed to restore input buffering");

		if (c == 'y') {
			return full_rw_eeprom(eeprom, &eeprom_opt);
		} else {
			LOG_PRINT("aborted\n");
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

	if (argc < 3) {
		fd = write_file ? STDOUT_FILENO : STDIN_FILENO;
		f_name = NULL;
	} else {
		static const int write_flags = (O_WRONLY | O_CREAT | O_TRUNC);
		static const int read_flags = (O_RDONLY);
		const int flags = write_file ? write_flags : read_flags;

		f_name = argv[2];
		fd = open(f_name, flags);

		if (fd < 0) {
			LOG("failed to open the file (%s)", f_name);
			return -1;
		}
	}

	ret = rw_file_eeprom(eeprom, fd, write_file, &eeprom_opt);

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

static int full_rw_eeprom(struct eeprom *eeprom, const struct eeprom_opt *opt)
{
	const size_t dump_size = min(256, opt->data_size);
	char *data_w = malloc(opt->data_size);
	char *data_r = malloc(opt->data_size);
	char *it_r;
	char *it_w;
	unsigned i;
	int ret = 0;

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

	for (i = 0, it_w = data_w; i < opt->data_size; ++i)
		*it_w++ = (rand() & 0xFF);

	memset(data_r, 0, opt->data_size);

	LOG("beginning of the data to be written:");
	dump_hex_data(data_w, dump_size);

	LOG("writing to EEPROM ...");

	eeprom_seek(eeprom, 0);

	if (eeprom_write(eeprom, data_w, opt->data_size) < 0) {
		LOG("failed to write data");
		ret = -1;
	}

	LOG("reading the EEPROM ...");

	eeprom_seek(eeprom, 0);

	if (eeprom_read(eeprom, data_r, opt->data_size) < 0) {
		LOG("failed to read data");
		ret = -1;
	}

	LOG("beginning of the data read back:");
	dump_hex_data(data_r, dump_size);

	LOG("comparing results ...");

	for (i = 0, it_r = data_r, it_w = data_w; i < opt->data_size; ++i) {
		if (*it_r++ != *it_w++) {
			const int addr = (i > 128) ? (i - 128) : 0;

			LOG("error found at address 0x%04X", i);
			LOG("dump start at 0x%04X", addr);
			LOG("written:");
			dump_hex_data(&data_w[addr], dump_size);
			LOG("read:");
			dump_hex_data(&data_r[addr], dump_size);
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

static int pad_eeprom(struct eeprom *eeprom, size_t left,
		      const struct eeprom_opt *opt)
{
	static const size_t N_ZEROS = 64;
	char zeros[N_ZEROS];

	memset(zeros, 0, N_ZEROS);

	while (left) {
		size_t n = min(N_ZEROS, left);

		log_eeprom_progress(opt->data_size, (left - n), "Padding");

		if (eeprom_write(eeprom, zeros, n) < 0)
			return -1;

		left -= n;
	}

	return 0;
}

static int rw_file_eeprom(struct eeprom *eeprom, int fd, int write_file,
			  const struct eeprom_opt *opt)
{
	static const size_t buffer_size = 4096;
	const char *msg = write_file ? "Reading" : "Writing";
	char *buffer = malloc(buffer_size);
	size_t left;
	int ret;

	left = opt->data_size;

	if (buffer == NULL) {
		LOG("failed to allocate buffer");
		return -1;
	}

	eeprom_seek(eeprom, opt->skip);
	ret = 0;

	while (left && !ret && !g_abort) {
		const size_t rwsz =
			(left > buffer_size) ? buffer_size : left;

		if (write_file) {
			log_eeprom_progress(opt->data_size, left - rwsz, msg);

			if (eeprom_read(eeprom, buffer, rwsz) < 0)
				ret = -1;
			else if (write(fd, buffer, rwsz) < 0)
				ret = -1;

			left -= rwsz;
		} else {
			const ssize_t rdsz = read(fd, buffer, rwsz);

			if (rdsz >= 0)
				log_eeprom_progress(opt->data_size,
						    (left - rdsz), msg);

			if (rdsz < 0) {
				ret = -1;
			} else if (eeprom_write(eeprom, buffer, rdsz) < 0) {
				ret = -1;
			} else if ((size_t) rdsz == rwsz) {
				left -= rwsz;
			} else {
				if (opt->zero_padding) {
					LOG_PRINT("\n");
					left -= rdsz;
					ret = pad_eeprom(eeprom, left, opt);
				}

				left = 0;
			}
		}
	}

	LOG_PRINT("\n");
	free(buffer);

	return ret;
}

static int parse_eeprom_opt(struct ctx *ctx, struct eeprom_opt *eopt)
{
	const size_t opt_size = strlen(g_opt) + 1;
	static const char *sep = ", ";
	char *opt_str;
	char *opt;
	int ret = 0;

	opt_str = malloc(opt_size);
	assert(opt_str != NULL);
	memcpy(opt_str, g_opt, opt_size);

	while ((opt = strsep(&opt_str, sep)) != NULL) {
		static const char *opt_sep = "=";
		char *key;
		char *str_value;
		unsigned long ul_value;
		int is_int;

		key = strsep(&opt, opt_sep);

		if (key == NULL) {
			LOG("invalid EEPROM option");
			return -1;
		}

		str_value = strsep(&opt, opt_sep);

		if (str_value) {
			errno = 0;
			ul_value = strtoul(str_value, NULL, 10);
			is_int = errno ? 0 : 1;
		} else {
			is_int = 0;
		}

		if (!strcmp(key, "i2c_block_size")) {
			if (!is_int) {
				LOG("no or invalid I2C block size");
				ret = -1;
				goto exit_now;
			}

			LOG("I2C block size: %lu", ul_value);
			eopt->block_size = ul_value;
		} else if (!strcmp(key, "page_size")) {
			if (!is_int) {
				LOG("no or invalid EEPROM page size");
				ret = -1;
				goto exit_now;
			}

			LOG("EEPROM page size: %lu", ul_value);
			eopt->page_size = ul_value;
		} else if (!strcmp(key, "zero_padding")) {
			LOG("zero-padding enabled");
			eopt->zero_padding = 1;
		} else if (!strcmp(key, "data_size")) {
			if (!is_int) {
				LOG("no or invalid data size");
				ret = -1;
				goto exit_now;
			}

			LOG("data size: %lu", ul_value);
			eopt->data_size = ul_value;
		} else if (!strcmp(key, "skip")) {
			if (!is_int) {
				LOG("no or invalid skip size specified");
				ret = -1;
				goto exit_now;
			}

			LOG("skip: %lu", ul_value);
			eopt->skip = ul_value;
		} else if (!strcmp(key, "addr")) {
			if (str_value == NULL) {
				LOG("no I2C address configuration specified");
				ret = -1;
				goto exit_now;
			}

			eopt->i2c_addr = plconfig_get_i2c_addr(
				ctx->config, str_value, PLHW_NO_I2C_ADDR);

			if (eopt->i2c_addr != PLHW_NO_I2C_ADDR) {
				LOG("I2C address (%s): 0x%02X",
				    str_value, eopt->i2c_addr);
			} else {
				LOG("failed to find I2C address in config: %s",
				    str_value);
			}
		} else {
			LOG("invalid option name: %s", key);
			ret = -1;
			goto exit_now;
		}
	}

exit_now:
	free(opt_str);

	return ret;
}

static void log_eeprom_progress(size_t total, size_t rem, const char *msg)
{
	const size_t prog = total - rem;
	const int prog_percent = prog * 100 / total;

	LOG_PRINT("\r%s EEPROM... %i%% (%zu)", msg, prog_percent, prog);
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
	struct max17135 *max17135 = require_max17135(ctx);
	struct dac5820 *dac = require_dac(ctx);

	if ((cpld == NULL) || (max17135 == NULL))
		return -1;

	STEP(cpld_set_switch(cpld, CPLD_BPCOM_CLAMP, 1), "BPCOM clamp");
	STEP(cpld_set_switch(cpld, CPLD_HVEN, 1), "HV enable");
	STEP(max17135_wait_for_pok(max17135), "wait for POK");
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
		ctx->plep = plep_init(NULL, NULL, NULL);
	/* ToDo: parse options */

	return ctx->plep;
}

static int epdc_get_set_hw_opt(struct ctx *ctx, int argc, char **argv)
{
	static const char *opt_list[_PLEP_HW_OPT_N_] = {
		[PLEP_POWER_OFF_DELAY_MS] = "power_off_delay_ms",
		[PLEP_CLEAR_ON_EXIT] = "clear_on_exit",
		[PLEP_TEMPERATURE] = "temperature",
		[PLEP_TEMPERATURE_AUTO] = "temperature_auto",
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
		if (plep_get_hw_opt(ctx->plep, opt, &value)) {
			LOG("Error getting ePDC opt %s", opt_str);
			return -1;
		}

		LOG("ePDC opt %s: %d", opt_str, value);
	} else {
		value = atoi(argv[1]);

		if (plep_set_hw_opt(ctx->plep, opt, value)) {
			LOG("Error setting ePDC opt %s", opt_str);
			return -1;
		}

		LOG("ePDC opt %s set to: %d", opt_str, value);
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
			LOG_PRINT("\n");

		for (i = 0; i < length; ++i)
			LOG_PRINT("%02X ", *byte++);

		LOG_PRINT("\b\n");
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

static const char help_max17135[] =
"  With no arguments, all the status information is dumped.\n"
"  To set a timing value:\n"
"    timing TIMING_NUMBER TIMING_VALUE_MS\n"
"  To set or get the VCOM register value:\n"
"    vcom [VCOM_REGISTER_VALUE]\n"
"  To switch on/off the HV power supplies (en, cen, cen2):\n"
"    [en, cen, cen2] [on, off]\n";

static const char help_tps65185[] =
"  With no arguments, all the status information is dumped.\n"
"  To set or get the VCOM register value (0 to 511):\n"
"    vcom [VCOM_REGISTER_VALUE]\n"
"  To set or get the power up or down sequence timings:\n"
"    seq [up, down] [VDDH VPOS VEE VNEG STROBE1 STROBE2 STROBE3 STROBE4]\n"
"    Each voltage (VDDH..VNEG) takes a strobe value between 1 and 4, and\n"
"    each strobe (STROBE1..STROBE4) is a delay of either 3, 6, 9 or 12 ms.\n"
"  To set the power mode to \"active\" (wait until HV is turned on):\n"
"    active\n"
"  To set the power mode to \"standby\" (wait until HV is turned off):\n"
"    standby\n"
"  To set or get an individual power rail enable status:\n"
"    en [vgneg, vee, vpos, vddh, vcom, v3p3] [on, off]\n"
"    When no \"on\" or \"off\" value is given, the current state is logged.\n";

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
"  generated on stdout.\n"
"  First optional argument: reference voltage\n"
"    internal:  use the internal reference voltage (default)\n"
"    external:  use the external reference voltage\n"
"    vdd:       use VDD as reference voltage\n"
"  Second optional argument: channel\n"
"    channel number starting from 0: select that channel\n"
"    vcom: read the VCOM value on its dedicated channel\n";

static const char help_pbtn[] =
"  No arguments, just a small procedure to manually test the buttons.\n";

static const char help_eeprom[] =
"  The first argument is the EEPROM mode, which is typically 24c01 for\n"
"  128 bytes or 24c256 for 32 KBytes.  Then the second argument is one of\n"
"  the following commands:\n"
"    full_rw:        write random data, read it back and compare\n"
"    e2f FILE_NAME:  dump EEPROM contents to a file, or stdout by default\n"
"    f2e FILE_NAME:  dump file contents or stdin by default to EEPROM\n"
"  Options follow this format:\n"
"    -o option1=value1,option2=value2\n"
"  Supported options are:\n"
"    i2c_block_size=SIZE\n"
"      Maximum I2C block transfer size in bytes.  The default is 96, which\n"
"      should work with all I2C bus drivers, but it can be increased to 512\n"
"      for example in order to speed-up the data transfers when available.\n"
"    page_size=SIZE\n"
"      EEPROM page size.  A default page size is set based on the EEPROM\n"
"      mode, but each manufacturer may implement different page sizes.  This\n"
"      option overrides the default value.\n"
"    zero_padding\n"
"      Enable padding of the end of the EEPROM data with zeros, when writing\n"
"      the contents of a file smaller than the EEPROM capacity.  This is\n"
"      especially useful when storing plain text to ensure the data is well\n"
"      null-terminated.\n"
"    skip=SIZE\n"
"      Skip SIZE bytes from the EEPROM when either reading or writing.\n"
"    data_size=SIZE\n"
"      Size of the data to use.  Use this option when only a part of the\n"
"      EEPROM should be used, instead of its full capacity.\n"
"    addr=CONFIG\n"
"      Look for the CONFIG option in the plsdk.ini file and use this as the\n"
"      I2C address to communicate with the EEPROM.  The CONFIG key is used\n"
"      as-is and there is no naming convention; typical values are\n"
"      eeprom-i2c-addr-display and eeprom-i2c-addr-vcom.\n";

static const char help_power[] =
"  Supported arguments:\n"
"    on [seq] [vcom]\n"
"      turn the power on, with optional sequence name (seq0 by default) and\n"
"      optional VCOM register value (decimal, range varies with seq type)\n"
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
"      clear_on_exit:      clear the screen when the ePDC is shut down\n"
"      temperature_auto:   use internal temperature sensor to automatically\n"
"                          determine temperature for waveform selection\n"
"      temperature:        when automatic mode is disabled, set the\n"
"                          temperature in degrees Celsius used for waveform\n"
"                          selection\n";
