
#include "microjson.c"

#include "system.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <getopt.h>
#include <math.h>
#include <termio.h>

#include <poptIO.h>
#include <rpmdefs.h>

#define	_RPMMQTT_INTERNAL
#include "rpmmqtt.h"

#include "debug.h"

/*==============================================================*/

#define	TWIDDLE		'~'
#define	_CMD_NDEVS	32
#define	_URG_STRING_LEN	32

/*==============================================================*/
typedef	char		STRING_t[_URG_STRING_LEN];
#ifdef	REFERENCE
typedef struct timeval	TSTAMP_t;
typedef struct timeval	TDIFF_t;

typedef int32_t		FLOAT_t;
#define	_FSCALE	10000.
#define	_F2I(_x)	((FLOAT_t) ((_x) * _FSCALE))
#define	_I2F(_x)	((float)   ((_x) / _FSCALE))
#endif	/* REFERENECE */

typedef struct CTC_s CTC_t;
struct CTC_s {
    uint16_t	prescale;
    uint16_t	count;
};

typedef struct PWM_s PWM_t;
struct PWM_s {
    uint16_t	prescale;
    uint16_t	count;
    uint8_t	duty;
};

typedef	struct RANGE_s RANGE_t;
struct RANGE_s {
    FLOAT_t	min;
    FLOAT_t	val;
    FLOAT_t	max;
    uint8_t	units;
    uint8_t	npts;
    FLOAT_t	ref[5];
};

static const char *unitstr[] =
{
    "",
#define	UNITS_NONE	0
    "C",
#define	UNITS_CELSIUS	1
    "torr",
#define	UNITS_TORR	2
    "F",
#define	UNITS_FARENHEIT	3
    "inHg",
#define	UNITS_INHG	4
    "K",
#define	UNITS_KELVIN	5
    "atm",
#define	UNITS_ATM	6
    "psi",
#define	UNITS_PSI	7
    "MHz",
#define	UNITS_MHZ	8
    "cycles",
#define	UNITS_CYCLES	9
    "",
#define	UNITS_10	10
    "",
#define	UNITS_11	11
    "",
#define	UNITS_12	12
    "",
#define	UNITS_13	13
    "",
#define	UNITS_14	14
    "",
#define	UNITS_15	15
};

static struct RANGE_s _temperature_celsius =
	{ _F2I(-40.0), _F2I( 21.0), _F2I( 40.0), UNITS_CELSIUS, };

static struct RANGE_s _barometer_torr =
	{ _F2I( 225.), _F2I( 760.), _F2I( 825.), UNITS_TORR, };

typedef struct SENSOR_s * SENSOR_t;
struct SENSOR_s {
    STRING_t	name;
    uint16_t	pts;
    uint16_t	rval;
    uint16_t	rmax;
    FLOAT_t	gain;
    FLOAT_t	off;
    FLOAT_t	sys;
    FLOAT_t	ref;
    FLOAT_t	avg;
    FLOAT_t	max;
    FLOAT_t	min;
    FLOAT_t	temp;
    FLOAT_t	pres;
    TSTAMP_t	tstamp;
    FLOAT_t	sum;
    uint16_t	npts;
};

typedef struct FLAGS_s * FLAGS_t;
struct FLAGS_s {
    /* Assign FLAGS to DIO channels by ordering following items. */
    uint8_t	power_fail;		/* DIO[ 0] */
#define	CMD_power_fail			    CMD_0
    uint8_t	field_blank;		/* DIO[ 1] */
#define	CMD_field_blank			    CMD_1
    uint8_t	event_executing;	/* DIO[ 2] */
#define	CMD_event_executing		    CMD_2
    uint8_t	event_paused;		/* DIO[ 3] */
#define	CMD_event_paused		    CMD_3
    uint8_t	event_expired;		/* DIO[ 4] */
#define	CMD_event_expired		    CMD_4
    uint8_t	event_aborted;		/* DIO[ 5] */
#define	CMD_event_aborted		    CMD_5
    uint8_t	duration_error;		/* DIO[ 6] */
#define	CMD_duration_error		    CMD_6
    uint8_t	filter_temp_error;	/* DIO[ 7] */
#define	CMD_temp_error			    CMD_7
    uint8_t	inactive_temp_error;	/* DIO[ 8] */
#define	CMD_inactive_temp_error		    CMD_8
    uint8_t	flow_variation_error;	/* DIO[ 9] */
#define	CMD_flow_variation_error	    CMD_9
    uint8_t	out_of_range_error;	/* DIO[10] */
#define	CMD_out_of_range_error		    CMD_10
    uint8_t	filter_load_error;	/* DIO[11] */
#define	CMD_filoter_load_error		    CMD_11
    uint8_t	door_open;		/* DIO[12] */
#define	CMD_door_open			    CMD_12
#define	_NFLAGS	13
};

typedef	struct EINFO_s * EINFO_t;
struct EINFO_s {
    TSTAMP_t	start;			/* 0000 */
    TDIFF_t	duration;		/* 2400 */
    TSTAMP_t	default_start;
    TDIFF_t	default_duration;
    TSTAMP_t	interval;		/* 72:00 */
    TDIFF_t	min_duration;		/* 23:00 */
    TDIFF_t	max_duration;		/* 25:00 */
    enum EVENT_STATUS_e {
	EVENT_PAUSED	= 0,
	EVENT_WAITING	= 1,
	EVENT_EXECUTING	= 2,
	EVENT_COMPLETED	= 3,
	EVENT_ABORTED	= 4,
	EVENT_EXPIRED	= 6,
    }		status;
};

typedef	struct FINFO_s * FINFO_t;
struct FINFO_s {
    STRING_t	id;
    TSTAMP_t	insert;
    TSTAMP_t	remove;
    uint32_t	position;
    uint16_t	load_avg;	/* 6794 TDIFF_t? */
    uint16_t	load_avg_enable;
    uint16_t	rotational_encoder;
    uint16_t	vertical_encoder;
};

typedef struct VERSION_s * VERSION_t;
struct VERSION_s {
    uint16_t	major;		/* 00-99 */
    uint16_t	minor;		/* 00-99 */
    uint16_t	build;		/* 00-99 */
};

typedef struct INTERVAL_s * INTERVAL_t;
struct INTERVAL_s {
    TSTAMP_t	start;
    TSTAMP_t	end;
    TDIFF_t	duration;
};

typedef struct URG_s * URG_t;
struct URG_s {
    /* Site Log */
    uint16_t	model;		/* {0,1,2} */
    STRING_t	serial;

    struct VERSION_s	fw;
    struct VERSION_s	boot;

    STRING_t	boot_block_id;
    STRING_t	boot_block_unique_id;
    STRING_t	user1;
    STRING_t	user2;
    STRING_t	user3;
    STRING_t	user4;

    uint16_t	measurement_units; /* 0=metric low, 1=metric hi, 2=english */
    uint16_t	date_format;		/* 0=US, 1=ISO */
    uint16_t	cold_limit_fan;		/* -40 -> +30 C */
    uint16_t	cold_limit_lcd;		/* -40 -> +30 C */
    uint16_t	diff_limit_fan;		/* 2 -> 5 C */

    FLOAT_t	pres_leak_check;	/* 100 -> 600 mm Hg  */

    uint16_t	temp_min_cal_points;	/* 2 */
    uint16_t	pres_min_cal_points;	/* 2 */
    uint16_t	flow_min_cal_points;	/* 3 */

    uint32_t	baud_rate;		/* 19200 */
    uint16_t	log_period;		/* 1-60 min */
    uint16_t	log_version;		/* 6 */
    uint16_t	tx_period;		/* 120:00 */

    uint16_t	tx_current_event;
    uint16_t	debug_out;
    uint16_t	select_site_log;
    uint16_t	select_event_log;
    uint16_t	select_data_log;
    uint16_t	select_power_fail_log;
    uint16_t	select_QC_log;
    uint16_t	select_calibration_log;
    uint16_t	select_debug_log;

    struct EINFO_s einfo;
    struct FINFO_s finfo;

    uint16_t	min_avg_diff;		/* 150 */
    uint16_t	max_avg_diff;		/* 150 */
    uint16_t	min_load_diff;

    /* Assign SENSORS to A2D/D2A channels by ordering following structures. */
    struct SENSOR_s sensor;		/* A2D[0] */
#define	CMD_sensor			   CMD_0
    struct SENSOR_s ambient;		/* A2D[1] */
#define	CMD_ambient			   CMD_1
    struct SENSOR_s filter;		/* A2D[2] */
#define	CMD_filter			   CMD_2
    struct SENSOR_s meter;		/* A2D[3] */
#define	CMD_meter			   CMD_3
    struct SENSOR_s inactive;		/* A2D[4] */
#define	CMD_inactive			   CMD_4
    struct SENSOR_s barometer;		/* A2D[5] */
#define	CMD_barometer			   CMD_5
    struct SENSOR_s meter_drop;		/* A2D[6] */
#define	CMD_meter_drop			   CMD_6
    struct SENSOR_s flow_sensor;	/* A2D[7] */
#define	CMD_flow_sensor			   CMD_7
#define	_NSENSORS	8

    /* Event Log */
    struct INTERVAL_s set;
    struct INTERVAL_s actual;
    FLOAT_t	volume;
    FLOAT_t	set_flow_rate;		/* 0=16.7, 1=10.0 */
    FLOAT_t	avg_flow_rate;
    FLOAT_t	flow_CV;

    FLOAT_t	max_diff;
    TDIFF_t	max_diff_time;
    uint16_t	power_fail_count;

    struct FLAGS_s flags;

    /* Calibration Log */

    /* Data Log */
    TSTAMP_t	tstamp;
    FLOAT_t	flow_rate;
    TDIFF_t	elapsed_time;

    /* QC Log */
    STRING_t	qc_item;
    FLOAT_t	sys;
    FLOAT_t	ref;

    /* Powerfail Log */
    TSTAMP_t	start_tstamp;
    TSTAMP_t	end_tstamp;
    TDIFF_t	duration;

    /* Debug Log */
    uint16_t	code;
    uint32_t	data;

    /* Ambient temperature and pressure. */
    FLOAT_t	temp;
    FLOAT_t	pres;

};

static struct URG_s _urg = {
    /* Site Log */
    .model			= 2,			/* {0,1,2} */
    .serial			= "2.5-300-00498",
    .fw.major			= 06,			/* 00-99 */
    .fw.minor			= 04,			/* 00-99 */
    .fw.build			= 04,			/* 00-99 */
    .boot.major			= 2,			/* 00-99 */
    .boot.minor			= 5,			/* 00-99 */
    .boot.build			= 300,			/* 00-99 */
    .boot_block_id		= "",
    .boot_block_unique_id	= "",
    .user1			= "",
    .user2			= "",
    .user3			= "",
    .user4			= "",

    .temp_min_cal_points	= 2,			/* 2 */
    .pres_min_cal_points	= 2,			/* 2 */
    .flow_min_cal_points	= 3,			/* 3 */

    .baud_rate			= 19200,		/* 19200 */
    .log_period			= 5,			/* 1-60 min */
    .log_version		= 6,			/* 6 */
    .tx_period			= 2*60*60,		/* 120:00 */

    /* Calibration Log */
    .ambient			=
	{"Ambient",	3, 0x0000, 0x03ff,
			_F2I( 1.50), _F2I(-2019),
			_F2I( 21.0), _F2I( 21.0),
			_F2I( 21.0), _F2I( 21.3), _F2I( 20.8),
			_F2I( 25.0), _F2I(760.),
	},
    .filter			=
	{"Filter",	3, 0x0000, 0x03ff,
			_F2I( 1.49), _F2I(-2060),
			_F2I( 21.0), _F2I( 21.0),
			_F2I( 21.3), _F2I( 21.6), _F2I( 21.0),
			_F2I( 25.0), _F2I(760.),
	},
    .meter			=
	{"Meter",	3, 0x0000, 0x03ff,
			_F2I( 1.49), _F2I(-2060),
			_F2I( 22.0), _F2I( 22.0),
			_F2I( 22.0), _F2I( 22.1), _F2I( 21.8),
			_F2I( 25.0), _F2I(760.),
	},
    .inactive			=
	{"Inactive",	2, 0x0000, 0x03ff,
			_F2I( 1.49), _F2I(-2060),
			_F2I( 21.0), _F2I( 21.0),
			_F2I( 21.3), _F2I( 21.5), _F2I( 21.0),
			_F2I( 25.0), _F2I(760.),
	},
    .barometer			=
	{"Barometer",	4, 0x0000, 0x03ff,
			_F2I(34.42), _F2I(-1657),
			_F2I( 760.), _F2I( 760.),
			_F2I( 736.), _F2I( 737.), _F2I( 734.),
			_F2I( 25.0), _F2I(760.),
	},
    .meter_drop			=
	{"MeterDrop",	4, 0x0000, 0x03ff,
			_F2I(10.35), _F2I(-4238),
			_F2I( 10.0), _F2I( 10.0),
			_F2I( 19.4), _F2I( 19.9), _F2I( 18.8),
			_F2I( 25.0), _F2I(760.),
	},
    .flow_sensor		=
	{"FlowSensor",	4, 0x0000, 0x03ff,
			_F2I( 6.27), _F2I(    0),
			_F2I( 16.7), _F2I( 16.7),
			_F2I( 16.7), _F2I( 16.7), _F2I( 16.7),
			_F2I( 25.0), _F2I(760.),
	},

    /* Calibration/QC Log */
    .sensor			= {"Sensor", 0, 0, _F2I(    0), _F2I(    0), },

    /* Event Log */
    .set.start			= {0},
    .set.end			= {0},
    .set.duration		= {86400},
    .actual.start		= {0},
    .actual.end			= {0},
    .actual.duration		= {86400},

    .finfo.insert		= {0},
    .finfo.remove		= {0},
    .finfo.position		= 1,
    .finfo.id			= "111111",
    .finfo.rotational_encoder	= 6802,
    .finfo.vertical_encoder	= 1301,

    .volume			= _F2I(0.0833),
    .set_flow_rate		= _F2I(16.7),		/* 0=16.7, 1=10.0 */
    .avg_flow_rate		= _F2I(16.7),
    .flow_CV			= _F2I(0.16),

    .max_diff			= _F2I(0.4),
    .max_diff_time		= {86400},
    .power_fail_count		= 0,

    .flags.power_fail		= 0,
    .flags.field_blank		= 0,
    .flags.event_executing	= 0,
    .flags.event_paused		= 0,
    .flags.event_expired	= 0,
    .flags.event_aborted	= 1,
    .flags.duration_error	= 0,
    .flags.filter_temp_error	= 0,
    .flags.inactive_temp_error	= 0,
    .flags.flow_variation_error	= 0,
    .flags.out_of_range_error	= 0,
    .flags.filter_load_error	= 0,
    .flags.door_open		= 0,

    /* Debug Log */
    .code			= 0x0323,
    .data			= 0x1A2D1A2D,

    /* Ambient temperature and pressure. */
    .temp			= _F2I(25.),
    .pres			= _F2I(760.),
};

static URG_t urg = &_urg;

/*==============================================================*/
typedef enum CSVtype_e {
    CSVT_integer	= t_integer,
    CSVT_uinteger	= t_uinteger,
    CSVT_real		= t_real,
    CSVT_string		= t_string,
    CSVT_boolean	= t_boolean,
    CSVT_character	= t_character,
    CSVT_time		= t_time,
    CSVT_object		= t_object,
    CSVT_structobject	= t_structobject,
    CSVT_array		= t_array,
    CSVT_check		= t_check,
    CSVT_ignore		= t_ignore,

    CSVT_NULL,
#ifdef	DYING
    CSVT_FLOAT,
    CSVT_TSTAMP,
    CSVT_TDIFF,
#else
    CSVT_FLOAT		= t_FLOAT,
    CSVT_TSTAMP		= t_TSTAMP,
    CSVT_TDIFF		= t_TDIFF,
#endif

    CSVT_LAST
} CSVtype_t;

#define	_ENTRY(_sn)	[CSVT_##_sn] = #_sn
static char *CSVtypestr[] = {
    _ENTRY(integer),
    _ENTRY(uinteger),
    _ENTRY(real),
    _ENTRY(string),
    _ENTRY(boolean),
    _ENTRY(character),
    _ENTRY(time),
    _ENTRY(object),
    _ENTRY(structobject),
    _ENTRY(array),
    _ENTRY(check),
    _ENTRY(ignore),

    _ENTRY(NULL),
    _ENTRY(FLOAT),
    _ENTRY(TSTAMP),
    _ENTRY(TDIFF),
};
#undef	_ENTRY

typedef struct CSV_s *CSV_t;
struct CSV_s {
#ifdef	NOTYET
    char *attribute;
    json_type type;
    union {
	void *ptr;
        int *integer;
        unsigned int *uinteger;
        double *real;
        char *string;
        bool *boolean;
        char *character;
        struct json_array_t array;
        size_t offset;
    } attr;
    union {
        int integer;
        unsigned int uinteger;
        double real;
        bool boolean;
        char character;
        char *check;
    } dflt;
    size_t len;
    const struct json_enum_t *map;
    bool nodefault;
#else
    const char *sN;
    CSVtype_t type;
    union {
	void *_ptr;

        int *integer;
        unsigned int *uinteger;
        double *real;
        char *string;
        bool *boolean;
        char *character;

	int8_t *_i8p;
	int16_t *_i16p;
	int32_t *_i32p;
	int64_t *_i64p;
	uint8_t *_ui8p;
	uint16_t *_ui16p;
	uint32_t *_ui32p;
	uint64_t *_ui64p;
	float *_fp;
	double *_dp;
	TSTAMP_t *_tstamp;
	TDIFF_t *_tdiff;
	FLOAT_t *_float;
    } attr;
    union {
	const char *_dN;

        int integer;
        unsigned int uinteger;
        double real;
        bool boolean;
        char character;
        char *check;
    } dflt;
    size_t len;
    const struct json_enum_t *map;
    bool nodefault;
#endif
};

#define	_ENTRY(_t, _sN, _dN) \
	{#_sN, CSVT_##_t, {&_urg._sN}, {#_dN}, sizeof(_urg._sN)}

static struct CSV_s csvEVENT[] = {		/* #1 */
    _ENTRY(TSTAMP, set.start,		"Set Start"),
    _ENTRY(TSTAMP, set.end,		"Set Stop"),
    _ENTRY(TDIFF,  set.duration, 	"Set Duration"),
    _ENTRY(TSTAMP, actual.start,	"Actual Start"),
    _ENTRY(TSTAMP, actual.end,		"Actual Stop"),
    _ENTRY(TDIFF,  actual.duration,	"Actual Duration"),
    _ENTRY(TSTAMP, finfo.insert,	"Insert Time"),
    _ENTRY(TSTAMP, finfo.remove,	"Remove Time"),
    _ENTRY(uinteger, finfo.position,	"Filter Position"),
    _ENTRY(string, finfo.id,		"Filter ID"),
    _ENTRY(FLOAT,  volume,		"Volume"),
    _ENTRY(FLOAT,  set_flow_rate,	"Set Flow Rate"),
    _ENTRY(FLOAT,  avg_flow_rate,	"Average Flow Rate"),
    _ENTRY(FLOAT,  flow_CV,		"Flow CV"),
    _ENTRY(FLOAT,  ambient.avg,		"Average Ambient"),
    _ENTRY(FLOAT,  ambient.max,		"Maximum Ambient"),
    _ENTRY(FLOAT,  ambient.min,		"Minimum Ambient"),
    _ENTRY(FLOAT,  filter.avg,		"Average Filter"),
    _ENTRY(FLOAT,  filter.max,		"Maximum Filter"),
    _ENTRY(FLOAT,  filter.min,		"Minimum Filter"),
    _ENTRY(FLOAT,  meter.avg,		"Average Meter"),
    _ENTRY(FLOAT,  meter.max,		"Maximum Meter"),
    _ENTRY(FLOAT,  meter.min,		"Minimum Meter"),
    _ENTRY(FLOAT,  inactive.avg,	"Average Inactive"),
    _ENTRY(FLOAT,  inactive.max,	"Maximum Inactive"),
    _ENTRY(FLOAT,  inactive.min,	"Minimum Inactive"),
    _ENTRY(FLOAT,  barometer.avg,	"Average Barometer"),
    _ENTRY(FLOAT,  barometer.max,	"Maximum Barometer"),
    _ENTRY(FLOAT,  barometer.min,	"Minimum Barometer"),
    _ENTRY(FLOAT,  meter_drop.avg,	"Average Meter Drop"),
    _ENTRY(FLOAT,  meter_drop.max,	"Maximum Meter Drop"),
    _ENTRY(FLOAT,  meter_drop.min,	"Minimum Meter Drop"),
    _ENTRY(FLOAT,  max_diff,		"Max Difference"),
    _ENTRY(TDIFF,  max_diff_time,	"Max Difference Time"),
    _ENTRY(uinteger, power_fail_count,	"Power Fail Count"),
    _ENTRY(uinteger, finfo.vertical_encoder,	"Vertical Encoder"),
    _ENTRY(uinteger, finfo.rotational_encoder,	"Rotational Encoder"),
    _ENTRY(uinteger, flags.power_fail,	"Flag: Power Fail"),
    _ENTRY(uinteger, flags.field_blank,	"Flag: Field Blank"),
    _ENTRY(uinteger, flags.event_executing,"Flag: Event Executing"),
    _ENTRY(uinteger, flags.event_paused,	"Flag: Event Paused"),
    _ENTRY(uinteger, flags.event_expired,	"Flag: Event Expired"),
    _ENTRY(uinteger, flags.event_aborted,	"Flag: Event Aborted"),
    _ENTRY(uinteger, flags.duration_error,	"Flag: Duration Error"),
    _ENTRY(uinteger, flags.filter_temp_error,	"Flag: Filter Temp Error"),
    _ENTRY(uinteger, flags.inactive_temp_error,	"Flag: Inactive Temp Error"),
    _ENTRY(uinteger, flags.flow_variation_error,"Flag: Flow Variation Error"),
    _ENTRY(uinteger, flags.out_of_range_error,"Flag: Flow Out of Range Error"),
    _ENTRY(uinteger, flags.filter_load_error, "Flag: Filter Load Error"),
};
static int ncsvEVENT = (sizeof(csvEVENT)/sizeof(csvEVENT[0]));
 
static struct CSV_s csvDATA[] = {		/* #2 */
    _ENTRY(TSTAMP, tstamp,		"Date and Time"),
    _ENTRY(FLOAT,  ambient.temp,	"Ambient Temp"),
    _ENTRY(FLOAT,  filter.temp,		"Filter Temp"),
    _ENTRY(FLOAT,  inactive.temp,	"Inactive Temp"),
    _ENTRY(FLOAT,  meter.temp,		"Meter Temp"),
    _ENTRY(FLOAT,  ambient.pres,	"Ambient Pressure"),
    _ENTRY(FLOAT,  meter_drop.pres,	"Meter Drop Pressure"),
    _ENTRY(uinteger, finfo.position,	"Filter Position"),
    _ENTRY(string, finfo.id,		"Filter ID"),
    _ENTRY(FLOAT,  flow_rate,		"Flow Rate"),
    _ENTRY(FLOAT,  volume,		"Volume"),
    _ENTRY(TSTAMP, elapsed_time,	"Elapsed Time"),
    _ENTRY(uinteger, power_fail_count,	"Power Fail Count"),
    _ENTRY(uinteger, flags.power_fail,	"Flag: Power Fail"),
    _ENTRY(uinteger, flags.door_open,	"Flag: Door Open"),
    _ENTRY(uinteger, flags.event_executing,"Flag: Event Executing"),
    _ENTRY(uinteger, flags.event_expired,	"Flag: Event Paused"),
    _ENTRY(uinteger, flags.event_expired,	"Flag: Event Expired"),
    _ENTRY(uinteger, flags.duration_error,	"Flag: Event Duration Error"),
    _ENTRY(uinteger, flags.filter_temp_error,	"Flag: Filter Temp Error"),
    _ENTRY(uinteger, flags.inactive_temp_error,	"Flag: Inactive Temp Error"),
    _ENTRY(uinteger, flags.flow_variation_error,"Flag: Flow Variation Error"),
    _ENTRY(uinteger, flags.out_of_range_error,"Flag: Flow Out of Range Error"),
};
static int ncsvDATA = (sizeof(csvDATA)/sizeof(csvDATA[0]));

static struct CSV_s csvCALIBRATION[] = {	/* #3 */
    _ENTRY(TSTAMP, tstamp,		"Date and Time"),
    _ENTRY(string, sensor.name,		"Sensor"),
    _ENTRY(uinteger, sensor.pts,	"Points"),
    _ENTRY(FLOAT,  sensor.gain,		"Gain"),
    _ENTRY(FLOAT,  sensor.off,		"Offset"),
};
static int ncsvCALIBRATION = (sizeof(csvCALIBRATION)/sizeof(csvCALIBRATION[0]));

static struct CSV_s csvQC[] = {			/* #4 */
    _ENTRY(TSTAMP, tstamp,		"Date and Time"),
    _ENTRY(string, sensor.name,		"QC Item"),
    _ENTRY(FLOAT,  sensor.sys,		"System Value"),
    _ENTRY(FLOAT,  sensor.ref,		"Reference Value"),
};
static int ncsvQC = (sizeof(csvQC)/sizeof(csvQC[0]));

static struct CSV_s csvPOWERFAIL[] = {		/* #5 */
    _ENTRY(TSTAMP, start_tstamp,	"Start Date and Time"),
    _ENTRY(TSTAMP, end_tstamp,		"End Date and Time"),
    _ENTRY(TDIFF,  duration,		"Duration"),
};
static int ncsvPOWERFAIL = (sizeof(csvPOWERFAIL)/sizeof(csvPOWERFAIL[0]));

static struct CSV_s csvDEBUG[] = {		/* #6 */
    _ENTRY(TSTAMP, tstamp,		"Date and Time"),
    _ENTRY(uinteger,    code,		"Code"),
    _ENTRY(uinteger,    data,		"Data"),
};
static int ncsvDEBUG = (sizeof(csvDEBUG)/sizeof(csvDEBUG[0]));

static struct CSV_s csvSITE[] = {		/* #7 */
    _ENTRY(string, serial,		"Serial Number"),
    _ENTRY(string, user1,		"User Info 1"),
    _ENTRY(string, user2,		"User Info 2"),
    _ENTRY(string, user3,		"User Info 3"),
    _ENTRY(string, user4,		"User Info 4"),
};
static int ncsvSITE = (sizeof(csvSITE)/sizeof(csvSITE[0]));
#undef	_ENTRY

/*==============================================================*/
typedef enum TID_s {
    TID_0	=  0,
    TID_1	=  1,
    TID_2	=  2,
    TID_3	=  3,
    TID_4	=  4,
    TID_5	=  5,
    TID_6	=  6,
    TID_7	=  7,
    TID_8	=  8,
    TID_9	=  9,
    TID_10	= 10,
    TID_11	= 11,
    TID_12	= 12,
    TID_13	= 13,
    TID_14	= 14,
    TID_15	= 15,
    TID_A2D	= 'A',		/* A/D */
    TID_D2A	= 'D',		/* D/A */
    TID_DIO	= 'F',		/* Digital I/O */
    TID_GLOBAL	= 'G',		/* Global state */
    TID_PRES	= 'P',		/* Pressure */
    TID_STEP	= 'S',		/* Stepper */
    TID_TEMP	= 'T',		/* Temperature */
} TID_t;

typedef enum CMD_s {
    CMD_0	=  0,
    CMD_1	=  1,
    CMD_2	=  2,
    CMD_3	=  3,
    CMD_4	=  4,
    CMD_5	=  5,
    CMD_6	=  6,
    CMD_7	=  7,
    CMD_8	=  8,
    CMD_9	=  9,
    CMD_10	= 10,
    CMD_11	= 11,
    CMD_12	= 12,
    CMD_13	= 13,
    CMD_14	= 14,
    CMD_15	= 15,
    CMD_NDEVS	= _CMD_NDEVS,
    CMD_OFF	= '0',		/* Off. */
    CMD_ON	= '1',		/* On. */
    CMD_QUIT	= 'q',		/* Quit. */
    CMD_READ	= 'r',		/* Read. */
    CMD_WRITE	= 'w',		/* Write. */
    CMD_QUERY	= '?',		/* Query. */
    CMD_NAK	= 0x80,		/* NAK. */
} CMD_t;

typedef struct IO_s * IO_t;
struct IO_s {
    const char * role;
    const char *fmt;

    int fdno;

    int sv[2];
    
    int (*Open) (IO_t io);
    int (*Close) (IO_t io);

    ssize_t (*Chk) (IO_t io);
    ssize_t (*Get) (IO_t io);
    ssize_t (*Set) (IO_t io);

    int msgfmt;		/* 0=hex, 1=binary, 2=HDLC-like */
    int ntimeout;
    int maxtimeouts;
    int nretry;
    int maxretrys;

    struct timeval rtv;
    struct iovec riov;
    struct timeval wtv;
    struct iovec wiov;

    size_t nb;
    uint8_t *b;
    uint8_t * bs;
    uint8_t * be;
    uint16_t val;
    uint16_t retval;
    TID_t tid;
    CMD_t cmd;
    char valid;
    char retvalid;

    uint16_t ADvals[_CMD_NDEVS];
    uint16_t Fvals[_CMD_NDEVS];
    uint16_t Tvals[_CMD_NDEVS];
    uint16_t Pvals[_CMD_NDEVS];
    uint16_t Spos;

};

static volatile int exit_request;
sigset_t mask;
sigset_t omask;

static int _io_debug = 1;

#define	MSGBUFLEN	256

/*==============================================================*/
static int tstamp(struct timeval *tvp)
{
    int rc = gettimeofday(tvp, NULL);
    if (rc)
	perror("gettimeofday");
    return rc;
}

static const char hex[] = "0123456789ABCDEF";

static char * tohex(uint8_t *b, size_t nb)
{
    size_t nt = (b ? 3 * nb - 1 : 0);
    char * t = xmalloc(nt+1);
    char *te = t;

    if (b && nb)
    for (size_t i = 0; i < nb; i++) {
	*te++ = hex[ (*b >> 4) & 0x0F ];
	*te++ = hex[ (*b     ) & 0x0F ];
	*te++ = ' ';
	b++;
    }
    *te = '\0';
    return t;
}

static char * Xflbl(IO_t io, const char *_fn)
{
    static char b[MSGBUFLEN];
    size_t nb = sizeof(b);
    char *be = b;
    size_t nf;

    nf = snprintf(be, nb, "%s%s\t(%d):", io->role, _fn, io->fdno);
	be += nf;
	nb -= nf;
     *be++ = '\t';
     nb--;

     /* TIMESTAMP */
    {	struct timeval *tvp = strcmp(io->role, "avr")
		? &io->rtv : &io->wtv;
	struct tm tm;
	nf = strftime(be, nb, io->fmt, gmtime_r(&tvp->tv_sec, &tm));
	    be += nf;
	    nb -= nf;
	nf = snprintf(be, nb, ".%06u", (unsigned)tvp->tv_usec);
	    be += nf;
	    nb -= nf;
    }

     *be = '\0';
     
     return b;
}
#define	flbl(_io)	Xflbl((_io), __FUNCTION__)

static ssize_t Xcheck(IO_t io, const char * msg, struct timeval *tvp,
                ssize_t ret, int printit,
                const char * func, const char * fn, unsigned ln)
{
    int rc = (int) ret;
    if (tvp)
	(void) tstamp(tvp);
    if ((printit < 0) || (printit > 0 && tvp != NULL) || (rc < 0)) {
	char t[MSGBUFLEN];
	size_t nt = sizeof(t);
	char * te = t;
	size_t nf;
	nf = snprintf(te, nt, "%.4s%s\n\t%s:%u: %s(%d)",
                msg, Xflbl(io, func), fn, ln, msg+4, rc);
	    te += nf;
	    nt -= nf;
	if (tvp == &io->rtv) {
	    struct iovec *iov = &io->riov;
	    char *s = tohex((uint8_t *)iov->iov_base, iov->iov_len);
	    nf = snprintf(te, nt, "\n\t%s", s);
		te += nf;
		nt -= nf;
		free(s);
	}
	if (tvp == &io->wtv) {
	    struct iovec *iov = &io->wiov;
	    char *s = tohex((uint8_t *)iov->iov_base, iov->iov_len);
	    nf = snprintf(te, nt, "\n\t%s", s);
		te += nf;
		nt -= nf;
		free(s);
	}
	fprintf(stderr, "%s\n", t);
    }

    return ret;
}
#define check(_io, _msg, _tvp, _ret)  \
    Xcheck(_io, _msg, _tvp, _ret, _io_debug, __FUNCTION__, __FILE__, __LINE__)

/*==============================================================*/
/* http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html */

/* X25/FCS/PPP CRC-16 */
static const uint16_t fcstab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
static uint16_t pppfcs(uint16_t fcs, uint8_t *b, size_t nb)
{
    while (nb--)
	fcs = (fcs >> 8) ^ fcstab[(fcs ^ *b++) & 0xff];
    return fcs;
}

static int _Load(IO_t io, TID_t tid, CMD_t cmd,
		const uint8_t *s, size_t ns, struct iovec *iov)
{
    uint8_t b[MSGBUFLEN];
    uint8_t * bs = b;
    uint8_t * be = b;

    switch (io->msgfmt) {
    default:
    case 2:	/* === binary with start/stop flags. */
    case 1:	/* === binary without start/stop flags. */
	if (io->msgfmt == 2) {
	    *be++ = TWIDDLE;	/* start flag */
	    bs++;
	}
	*be++ = '\0';	/* count */
	*be++ = tid;	/* target id */
	*be++ = cmd;	/* target cmd */
	if (s && ns > 0) {	/* target cmd payload */
	    (void) memcpy(be, s, ns);
	    be += ns;
	}

	/* Fill in count. */
	*bs = (be - bs);

	/* Fill in X25/FCS/PPP checksum. */
	{   uint16_t crc = 0xffff;
	    crc = pppfcs(crc, bs, (be - bs));
	    *be++ = ((crc >> 8) & 0xFF);
	    *be++ = ((crc     ) & 0xFF);
	}
	if (io->msgfmt == 2)
	    *be++ = TWIDDLE;	/* stop flag */
	break;
    case 0:	/* === ascii/hex with CR/LF */
	*be++ = tid;	/* target id */
	*be++ = cmd;	/* target cmd */
	if (s && ns > 0) {	/* target cmd payload */
	    (void) memcpy(be, s, ns);
	    be += ns;
	}
	/* XXX CRC? */
	*be++ = '\r';
	*be++ = '\n';
	break;
    }

    if (iov) {
	iov->iov_base = memcpy(xmalloc(MSGBUFLEN), b, (be - b)); /* xmalloc(be-b) */
	iov->iov_len = (be - b);
     }

    return 0;
}

/*==============================================================*/
static int _Socketpair(IO_t io)
{
    int rc = check(io, "    socketpair", &io->rtv,
		socketpair(AF_UNIX, SOCK_STREAM, 0, io->sv));
    io->wtv = io->rtv;		/* structure assignment */
    return rc;
}

static int _Close(IO_t io)
{
    int rc = check(io, "    close", NULL,
		close(io->fdno));
    io->fdno = -1;
    return rc;
}

static ssize_t _Read(IO_t io)
{
    struct iovec *iov = &io->riov;
    int navail;
    ssize_t rc;
#if defined(linux)
    rc = check(io, "==> ioctl(FIONREAD)", NULL,
		ioctl(io->fdno, FIONREAD, &navail));
#else
    navail = MSGBUFLEN;
#endif

    iov->iov_len = navail;
    iov->iov_base = _free(iov->iov_base);
    iov->iov_base = xmalloc(MSGBUFLEN);		/* xmalloc(iov->iov_len) */

    rc = check(io, "<== read", &io->rtv,
		(iov->iov_len = read(io->fdno, iov->iov_base, iov->iov_len)));
    return rc;
}

static ssize_t _Write(IO_t io)
{
    struct iovec *iov = &io->wiov;
    ssize_t rc = check(io, "<== write", &io->wtv,
		write(io->fdno, iov->iov_base, iov->iov_len));
    return rc;
}

static ssize_t _Readv(IO_t io)
{
    struct iovec *iov = &io->riov;
    int navail;
    ssize_t rc;
#if defined(linux)
    rc = check(io, "==> ioctl(FIONREAD)", NULL,
		ioctl(io->fdno, FIONREAD, &navail));
#else
    navail = MSGBUFLEN;
#endif


    iov->iov_len = navail;
    iov->iov_base = _free(iov->iov_base);
    iov->iov_base = xmalloc(MSGBUFLEN);		/* xmalloc(iov->iov_len) */

    rc = check(io, "<== readv", &io->rtv,
		(iov->iov_len = readv(io->fdno, iov, 1)));
    return rc;
}

static ssize_t _Writev(IO_t io)
{
    struct iovec *iov = &io->wiov;
    ssize_t rc;

    rc = check(io, "<== writev", &io->wtv,
		writev(io->fdno, iov, 1));
    return rc;
}

static ssize_t _Recv(IO_t io)
{
    struct iovec *iov = &io->riov;
    static int _flags = 0;
    int navail;
    int rc;
#if defined(linux)
    rc = check(io, "==> ioctl(FIONREAD)", NULL,
		ioctl(io->fdno, FIONREAD, &navail));
#else
    navail = MSGBUFLEN;
#endif

    iov->iov_len = navail;
    iov->iov_base = _free(iov->iov_base);
    iov->iov_base = xmalloc(MSGBUFLEN);		/* xmalloc(iov->iov_len) */

    rc = check(io, "<== recv", &io->rtv,
		(iov->iov_len = recv(io->fdno, iov->iov_base, iov->iov_len, _flags)));

    return rc;
}

static ssize_t _Send(IO_t io)
{
    struct iovec *iov = &io->wiov;
    static int _flags = 0;
    ssize_t rc = check(io, "<== send", &io->wtv,
		send(io->fdno, iov->iov_base, iov->iov_len, _flags));
    return rc;
}

static ssize_t _Poll(IO_t io)
{
    struct timespec ts;
    fd_set rfds;
    int ntimeouts = 0;
    ssize_t rc = -1;

    while (!exit_request) {
	ts.tv_sec = 1;
	ts.tv_nsec = 0;
	FD_ZERO(&rfds);	FD_SET(io->fdno, &rfds);
	rc = check(io, "==> pselect", NULL,
		pselect(io->fdno+1, &rfds, NULL, NULL, &ts, &omask));
	if (rc < 0 && errno != EINTR) {
	    perror("pselect");
	    exit_request = 1;
	} else if (exit_request) {
fprintf(stderr, "    %s:\texit\n", flbl(io));
	    continue;
	} else if (rc == 0) {
fprintf(stderr, "    %s:\ttimeout\n", flbl(io));
	    ntimeouts++;
	    if (io->maxtimeouts > 0 && ntimeouts >= io->maxtimeouts)
		break;
	    continue;
	} else if (FD_ISSET(io->fdno, &rfds)) {
	    rc = io->Get(io);
	    break;
	}
    }

    return rc;
}

/*==============================================================*/
typedef struct MSG_s * MSG_t;
struct MSG_s {
    TID_t tid;
    CMD_t cmd;
    const char *pay;
    size_t npay;
};
struct MSG_s msgs[] = {
#ifdef	NOTNOW
  { TID_D2A,	CMD_ambient,	"\x05\x50", 2 },		/* d2a write */
  { TID_A2D,	CMD_ambient,	NULL, 0 },		/* a2d read */
  { TID_D2A,	CMD_ambient,	"\x05\x50", 2 },		/* d2a write */
  { TID_A2D,	CMD_ambient,	NULL, 0 },		/* a2d read */
  { TID_D2A,	CMD_ambient,	"\x05\x50", 2 },		/* d2a write */
  { TID_A2D,	CMD_ambient,	NULL, 0 },		/* a2d read */
  { TID_D2A,	CMD_ambient,	"\x05\x50", 2 },		/* d2a write */
  { TID_A2D,	CMD_ambient,	NULL, 0 },		/* a2d read */

  { TID_DIO,	CMD_0,	"\xa5\xa5", 2 },			/* dio write */
  { TID_DIO,	CMD_0,	NULL, 0 },			/* dio read */

  { TID_PRES,	CMD_8,		NULL, 0 },		/* pressure read */
  { TID_PRES,	CMD_8,		"\x9A\xFF", 2 },		/* dio read */
  { TID_PRES,	CMD_8,		NULL, 0 },		/* pressure read */

  { TID_STEP,	CMD_10,		NULL, 0 },		/* stepper read */
  { TID_STEP,	CMD_10,		"\xBC\xFF", 2 },		/* stepper move */
  { TID_STEP,	CMD_10,		NULL, 0 },		/* stepper read */

  { TID_TEMP,	CMD_12,		NULL, 0 },		/* temp read */
  { TID_TEMP,	CMD_12,		"\xDE\xFF", 2 },		/* temp write */
  { TID_TEMP,	CMD_12,		NULL, 0 },		/* temp write */
#endif

  { TID_GLOBAL,	CMD_QUIT,	NULL, 0 },		/* quit */
};
static size_t nmsgs = (sizeof(msgs)/sizeof(msgs[0]));

static int _Parse(IO_t io, struct iovec *iov)
{
    int rc = -1;	/* assume failure */

    io->b = (uint8_t *) iov->iov_base;
    io->nb = iov->iov_len;
    io->bs = io->b;

    io->valid = 0;
    io->val = 0xffff;
    io->retvalid = 0;
    io->retval = 0xffff;

    switch (io->msgfmt) {
    default:
    case 2:	/* === binary with start/stop flags. */
	if (!(io->b[0] == TWIDDLE && io->b[io->nb-1] == TWIDDLE))
	    goto exit;
	io->bs++;
	io->nb -= 2;
	/*@fallthrough@*/
    case 1:	/* === binary without start/stop flags. */
	if (io->bs[0] != (io->nb - 2))
	    goto exit;
	{
	    uint16_t crc = 0xffff;
	    crc = pppfcs(crc, io->bs, (io->nb - 2));
	    if (io->bs[io->nb-2] != ((crc >> 8) & 0xFF)
	     || io->bs[io->nb-1] != ((crc     ) & 0xFF))
		goto exit;
	}
	io->tid = io->bs[1];
	io->cmd = io->bs[2];
	if ((io->nb - 2) >= 5) {
	    io->valid = 1;
	    io->val = io->bs[3+0] << 8 | io->bs[3+1];
	}
	break;
    case 0:	/* === ascii/hex with CR/LF */
	if (!(io->b[io->nb-2] == '\r' && io->b[io->nb-1] == '\n'))
	    goto exit;
	/* XXX CRC? */
	io->tid = io->bs[0];
	io->cmd = io->bs[1];
	if ((io->nb - 2) >= 6) {
	    io->valid = 1;
	    io->val = 0;
	    for (int i = 0; i < 4; i++) {
		int c = io->bs[2+i];
		if (!isxdigit(c))
		    goto exit;
		io->val <<= 4;
		if (c >= '0' && c <= '9')
		    io->val += c - '0';
		else if (c >= 'A' && c <= 'F')
		    io->val += c - 'A';
		else if (c >= 'a' && c <= 'f')
		    io->val += c - 'a';
	    }
	}
	break;
    }
    io->be = io->bs + io->nb - 2;
    rc = 0;

exit:
fprintf(stderr, "<== %s: rc %d\n", flbl(io), rc);
    return rc;
}

static int _GetSet(IO_t io, const char *valname, uint16_t *vals)
{
    int ix = io->cmd;	/* io->cmd is the sensor index */
    int rc = -1;	/* assume failure */

    if (ix >= CMD_NDEVS)
	goto exit;

    if (io->valid) {
	/* Set/Save input value, return value in ACK msg. */
	vals[io->cmd] = io->val;
fprintf(stderr, "\t\t\t--> %s[%u] 0x%04X\n", valname, ix, vals[io->cmd]);
	io->retvalid = 1;
	io->retval = vals[ix];
    } else {
	/* Return current value. */
	io->retvalid = 1;
	io->retval = vals[ix];
	/* XXX Add some noise to AVR A2D values. */
	if (!strcmp(io->role, "avr") && io->tid == TID_A2D) {
#define	_JITTER	0x2
	    int16_t retval = io->retval;
	    int16_t jval = (random() % ((2 * _JITTER) + 1)) - _JITTER;
	    retval += jval;
	    if (retval < 0)
		retval = 0;
	    else if (retval > 0x0fff)
		retval = 0x0fff;
	    io->retval = retval;
	}
fprintf(stderr, "\t\t\t<-- %s[%u] 0x%04X\n", valname, ix, vals[io->cmd]);
    }
    rc = 0;

exit:
fprintf(stderr, "<== %s: rc %d\n", flbl(io), rc);
    return rc;
}

static int _Process(IO_t io)
{
    int ix;
    int rc = -1;	/* assume failure */

    switch (io->tid) {
    default:
	goto exit;
    case TID_A2D:	/* RDONLY */
	if (_GetSet(io, " A2D", io->ADvals))
	    goto exit;
	/* XXX set time stamp? */

	/* Scale and save a measurement. */
	ix = io->cmd;
	if (ix < _NSENSORS && !strcmp(io->role, "nuc")) {
	    struct SENSOR_s *sensors = &urg->sensor;
	    struct SENSOR_s *sensor = sensors + ix;
	    TSTAMP_t *tvp = strcmp(io->role, "avr")
			? &io->wtv : &io->rtv;
	    sensor->tstamp = *tvp;	/* structure assignment */
	    if (io->retvalid) {
		uint16_t rval = io->retval;
		FLOAT_t sval;
		sensor->rval = rval;
		/* XXX Convert units here? */
		sval = sensor->gain * rval + sensor->off;
		sensor->npts++;
		sensor->sum = sensor->sum + sval;
		sensor->avg = sensor->sum/sensor->npts;
		if (sval > sensor->max)
		    sensor->max = sval;
		if (sval < sensor->min)
		    sensor->min = sval;

		/* XXX Fill in fields that are not sensor calculated. */
		if (sensor == &urg->ambient)
		    urg->temp = sensor->avg;
		if (sensor == &urg->barometer)
		    urg->pres = sensor->avg;

#ifdef	NOTNOW
fprintf(stderr, "*** %s: sval %6.1f npts %u sum %10.3f min %6.1f avg %6.1f max %6.1f\n", __FUNCTION__, _I2F(sval), sensor->npts, _I2F(sensor->sum), _I2F(sensor->min), _I2F(sensor->avg), _I2F(sensor->max));
#endif
	    }
	}
	break;
    case TID_D2A:	/* WRONLY */
	if (_GetSet(io, " D2A", io->ADvals))
	    goto exit;
	/* XXX set time stamp? */
	break;
    case TID_DIO:
	if (_GetSet(io, " DIO", io->Fvals))
	    goto exit;
	io->retvalid = 1;
	/* XXX set time stamp? */
	ix = io->cmd;
	if (ix < _NFLAGS) {
	    struct FLAGS_s *flags = &urg->flags;
	    uint8_t *flag = &flags->power_fail;
	    /* XXX set time stamp? */
	    if (io->retvalid) {
		uint16_t mask = 1;
		for (int i = 0; i < _NFLAGS; i++) {
		    flag[i] = ((io->retval & mask) ? 1 : 0);
		    mask <<= 1;
		}
	    }
	}
	break;
    case TID_PRES:	/* RDONLY */
	if (_GetSet(io, "PRES", io->Pvals))
	    goto exit;
	/* XXX set time stamp? */
	break;
    case TID_STEP:
	if (io->valid) {
	    int16_t sval = io->val;
	    io->Spos += sval;
	} else {
	    io->retvalid = 1;
	    io->retval = io->Spos;
	}
	break;
    case TID_TEMP:	/* RDONLY */
	if (_GetSet(io, "TEMP", io->Tvals))
	    goto exit;
	/* XXX set time stamp? */
	break;
    case TID_GLOBAL:
	/* XXX set time stamp? */
	switch (io->cmd) {
	default:
	    goto exit;
	case CMD_QUIT:
	    exit_request = 1;
	    break;
	}
	break;
    }
    rc = 0;

exit:
fprintf(stderr, "<== %s: rc %d\n", flbl(io), rc);
    return rc;
}

/*==============================================================*/
static int _Child(IO_t io)
{
    static struct iovec ziov;	/* empty iovec */
    struct iovec *iov;
    int rc = 0;

fprintf(stderr, "==> %s\n", flbl(io));

    do {
	/* Read message. */
	rc = io->Chk(io);
	if (rc < 0)
	    break;

	/* Prepare to receive another input message. */
	io->wiov = io->riov;	/* structure assignment */
	io->riov = ziov;	/* structure assignment */

	/* Process input msg. */
	iov = &io->wiov;
	rc = _Parse(io, iov);
	if (rc) {
	    fprintf(stderr, "*** IOERR ***\n");
	} else
	    rc = _Process(io);

	/* Prepare the ACK/NAK response. */
	if (rc) {
	    io->cmd |= CMD_NAK;
	    io->retvalid = 0;
	}

	if (iov->iov_base)
	    free(iov->iov_base);
	*iov = ziov;	/* structure assignment */

	if (io->retvalid) {
	    uint8_t s[2];
	    size_t ns = sizeof(s);
	    s[0] = ((io->retval >> 8) & 0xFF);
	    s[1] = ((io->retval     ) & 0xFF);
	    rc = _Load(io, io->tid, io->cmd, s, ns, iov);
	} else {
	    rc = _Load(io, io->tid, io->cmd, NULL, 0, iov);
	}

	/* Send response. */
	rc = io->Set(io);

	/* Cleanup. */
	if (iov->iov_base)
	    free(iov->iov_base);
	io->wiov = ziov;	/* structure assignment */
	rc = 0;		/* XXX just in case. */

    } while (!exit_request);

    iov = &io->riov;
    if (iov->iov_base)
	free(iov->iov_base);
    *iov = ziov;	/* structure assignment */
    iov = &io->wiov;
    if (iov->iov_base)
	free(iov->iov_base);
    *iov = ziov;	/* structure assignment */
fprintf(stderr, "<== %s: rc %d\n", flbl(io), rc);
    return rc;
}

/*==============================================================*/
static int _Command(IO_t io, TID_t tid, CMD_t cmd,
	const uint8_t *s, size_t ns, uint16_t *retvalp)
{
    static struct iovec ziov;	/* empty iovec */
    struct iovec *iov;
    uint16_t retval = 0;
    int rc = -1;	/* assume failure */

    io->nretry = 0;

resend:
    /* Load message. */
    iov = &io->wiov;
    if (iov->iov_base)
	free(iov->iov_base);
    *iov = ziov;	/* structure assignment */
    rc = _Load(io, tid, cmd, s, ns, iov);

    /* Send message. */
    rc = io->Set(io);

    /* Sleep 1msec (and context switch) to permit client to reply. */
    {   const struct timespec ts = { 0, 1000000 };
	(void) nanosleep(&ts, NULL);
    }

    /* Read response. */
    iov = &io->riov;
    if (iov->iov_base)
	free(iov->iov_base);
    *iov = ziov;	/* structure assignment */
    rc = io->Chk(io);

    /* Process the response. */
    iov = &io->riov;
    rc = _Parse(io, iov);
    if (rc) {
	fprintf(stderr, "*** IOERR ***\n");
    } else
	rc = _Process(io);

    /* Retry on NAK (or IO error). */
    if (rc) {
	io->nretry++;
	if (io->maxretrys <= 0 || io->nretry < io->maxretrys) {
	    fprintf(stderr, "*** RETRY(%d:%d) ***\n",
			io->nretry, io->maxretrys);
	    goto resend;
	}
	fprintf(stderr, "*** MAXRETRY(%d:%d) ***\n",
			io->nretry, io->maxretrys);
	goto exit;
    }
    io->nretry = 0;

    /* Return a valid measurement. */
    retval = io->retval;
    if (retvalp)
	*retvalp = retval;

    rc = 0;

exit:
fprintf(stderr, "<== %s: rc %d retval %u\n", flbl(io), rc, retval);
    return rc;
}

static FLOAT_t _SAvg(IO_t io, CMD_t cmd, uint16_t val)
{
    int ix = cmd;
    struct SENSOR_s *sensors = &urg->sensor;
    struct SENSOR_s *sensor = sensors + ix;
    FLOAT_t sval;
    uint16_t retval;
    uint8_t s[2];
    size_t ns = sizeof(s);
    int navg = 5;
    int rc = -1;	/* assume failure */

    /* XXX Set the measurement point. */
    s[0] = ((val >> 8) & 0xFF);
    s[1] = ((val     ) & 0xFF);
    rc = _Command(io, TID_D2A, cmd, s, ns, &retval);

    /* Read the measurement. */
    for (int i = 0; i < navg; i++)
	rc = _Command(io, TID_A2D, cmd, NULL, 0, &retval);
    sval = sensor->avg;
    rc = 0;
    return sval;
}

static int _SCal(IO_t io, CMD_t cmd, RANGE_t *range)
{
    int ix = cmd;
    struct SENSOR_s *sensors = &urg->sensor;
    struct SENSOR_s *sensor = sensors + ix;
    FLOAT_t loval;
    FLOAT_t sval;
    FLOAT_t hival;
    uint16_t val;
    int rc = -1;	/* assume failure */

    sensor->gain = (range->max - range->min) / sensor->rmax;
    sensor->off = range->min;
    sensor->min = range->max;
    sensor->max = range->min;;

    /* Calibrate offset. */
    sensor->npts = 0;
    sensor->sum = 0;
    val = 0x0000;
    loval = _SAvg(io, cmd, val);
sensor->off = loval;

    /* Calibrate gain. */
    sensor->npts = 0;
    sensor->sum = 0;
    val = sensor->rmax;
    hival = _SAvg(io, cmd, val);
sensor->gain = (hival - loval)/sensor->rmax;

    /* Calibrate reference point. */
    sensor->npts = 0;
    sensor->sum = 0;
    val = (range->val - range->min) / sensor->gain;
    sval = _SAvg(io, cmd, val);

    /* Save reference point. */
    sensor->sys = sval;
    sensor->ref = range->val;

    /* Reset the measurement. */
    sensor->npts = 0;
    sensor->sum = 0;
    rc = 0;
fprintf(stderr, "\t  gain %9.4f\n", _I2F(sensor->gain));
fprintf(stderr, "\t   off %7.2f\n", _I2F(sensor->off));
fprintf(stderr, "\t loval %7.2f\n", _I2F(loval));
fprintf(stderr, "\t  sval %7.2f\n", _I2F(sval));
fprintf(stderr, "\t hival %7.2f\n", _I2F(hival));
fprintf(stderr, "\t   sys %7.2f\n", _I2F(sensor->sys));
fprintf(stderr, "\t   ref %7.2f\n", _I2F(sensor->ref));
fprintf(stderr, "<== %s: rc %d\n", flbl(io), rc);
    return rc;
}

/*==============================================================*/
static int _Parent(IO_t io)
{
    static struct iovec ziov;	/* empty iovec */
    struct iovec *iov;
    int rc = 0;

fprintf(stderr, "==> %s\n", flbl(io));

    /* Calibrate the temperature sensors. */
    rc = _SCal(io, CMD_ambient,	&_temperature_celsius);
fprintf(stderr, "====================\n");
#ifdef	NOTYET
    rc = _SCal(io, CMD_filter,	&_temperature_celsius);
fprintf(stderr, "====================\n");
    rc = _SCal(io, CMD_meter,	&_temperature_celsius);
fprintf(stderr, "====================\n");
    rc = _SCal(io, CMD_inactive,	&_temperature_celsius);
fprintf(stderr, "====================\n");
#endif

    /* Calibrate the pressure sensors. */
    rc = _SCal(io, CMD_barometer, &_barometer_torr);
fprintf(stderr, "====================\n");

#ifdef	NOTYET
    rc = _SCal(io, CMD_meter_drop, W2DO);
fprintf(stderr, "====================\n");
    rc = _SCal(io, CMD_flow_sensor, W2DO);
fprintf(stderr, "====================\n");
#endif

    /* Send all the canned messages. */
    for (size_t i = 0; i < nmsgs; i++) {
	MSG_t m = msgs + i;
	uint8_t * s = (uint8_t *) m->pay;
	size_t ns = (m->pay ? m->npay : 0);
	uint16_t retval;
	rc = _Command(io, m->tid, m->cmd, s, ns, &retval);

fprintf(stderr, "====================\n");
	if (exit_request || m->cmd == 'q' || m->cmd == 'Q') {
	    wait(NULL);
	    rc = 0;
	    break;
	}
    }

    iov = &io->riov;
    if (iov->iov_base)
	free(iov->iov_base);
    *iov = ziov;	/* structure assignment */
    iov = &io->wiov;
    if (iov->iov_base)
	free(iov->iov_base);
    *iov = ziov;	/* structure assignment */
fprintf(stderr, "<== %s: rc %d\n", flbl(io), rc);
    return rc;
}

static int _Fork(IO_t pio)
{
    pid_t pid;
    int rc = -1;

    switch ((pid = fork())) {
    case -1:
	perror("fork");
	rc = -1;
	goto exit;
    case 0:
    {	IO_t io = memcpy(xmalloc(sizeof(*pio)), pio, sizeof(*pio));
	io->role = "avr";	/* XXX */
	io->fdno = pio->sv[1];	/* XXX */
	rc = _Child(io);
fprintf(stderr, "    %s: exit(%d)\n", flbl(io), rc);
	free(io);
	exit(rc);
    }	break;
    default:
    {	IO_t io = memcpy(xmalloc(sizeof(*pio)), pio, sizeof(*pio));
	io->role = "nuc";	/* XXX */
	io->fdno = pio->sv[0];	/* XXX */
	rc = _Parent(io);
fprintf(stderr, "    %s: exit(%d)\n", flbl(io), rc);
	free(io);
    }	break;
    }

exit:
    return rc;
}

/*==============================================================*/
static IO_t newIO(int (*Open) (IO_t io))
{
    static const char _fmt[] = " %Y-%m-%d %H:%M:%S";
    IO_t io = calloc(1, sizeof(*io));

    if (Open) {
	int rc;

	/* XXX Bootstrap debugging. */
	io->role = "new";
	io->fmt = _fmt;

	/* XXX Ensure EBADF */
	io->fdno = -1;
	io->sv[0] = -1;
	io->sv[1] = -1;

	/* Create the channel */
	rc = (*Open) (io);
	if (rc != -1) {
	    io->msgfmt = 1;		/* 0=hex, 1=binary, 2=HDLC-like */
	    io->maxtimeouts = 4;
	    io->maxretrys = 4;
	    memset(io->ADvals, 0xff, sizeof(io->ADvals));
	    io->Close = _Close;
	    io->Chk = _Poll;
	    io->Get = _Readv;
	    io->Set = _Writev;
	}
    }

    return io;
}

static int _Doit(rpmmqtt mqtt)
{
    IO_t pio = newIO(_Socketpair);
    int rc = _Fork(pio);
    free(pio);
    return rc;
}

/*==============================================================*/

static void _PrintCSVT(CSV_t csv, FILE *fp)
{
    fprintf(stderr, "%12s %32s:", CSVtypestr[csv->type%CSVT_LAST], csv->sN);

    switch(csv->type) {
    case CSVT_boolean:
    case CSVT_character:
    case CSVT_time:
    case CSVT_object:
    case CSVT_structobject:
    case CSVT_array:
    case CSVT_check:
    case CSVT_ignore:
    default:
	fprintf(stderr, " %p", csv->attr._ptr);
	break;
    case CSVT_string:
      { const char * val = (const char *)csv->attr.string;
	fprintf(stderr, " %s", val);
      } break;
    case CSVT_real:
      {	double d;
	switch (csv->len) {
	default:
	case 0:
fprintf(stderr, "XXX FIXME: double alignment\n");
	    d = *(double *)csv->attr._ptr;
	    break;
	case sizeof(float):
	    d = *csv->attr._fp;
	    break;
	case sizeof(double):
	    d = *csv->attr._dp;
	    break;
	}
	fprintf(stderr, " %g", d);
      } break;
    case CSVT_integer:
      {	long long i64;
	switch (csv->len) {
	default:
	case 0:
fprintf(stderr, "XXX FIXME: int alignment\n");
	    i64 = *(int *)csv->attr._ptr;
	    break;
	case sizeof(int8_t):
	    i64 = *csv->attr._i8p;
	    break;
	case sizeof(int16_t):
	    i64 = *csv->attr._i16p;
	    break;
	case sizeof(int32_t):
	    i64 = *csv->attr._i32p;
	    break;
	case sizeof(int64_t):
	    i64 = *csv->attr._i64p;
	    break;
	}
	fprintf(stderr, " %lld", i64);
      } break;
    case CSVT_uinteger:
      {	unsigned long long ui64;
	switch (csv->len) {
	default:
	case 0:
fprintf(stderr, "XXX FIXME: unsigned alignment\n");
	    ui64 = *(unsigned *)csv->attr._ptr;
	    break;
	case sizeof(uint8_t):
	    ui64 = *csv->attr._ui8p;
	    break;
	case sizeof(uint16_t):
	    ui64 = *csv->attr._ui16p;
	    break;
	case sizeof(uint32_t):
	    ui64 = *csv->attr._ui32p;
	    break;
	case sizeof(uint64_t):
	    ui64 = *csv->attr._ui64p;
	    break;
	}
	fprintf(stderr, " %llu", ui64);
      } break;
    case CSVT_FLOAT:
      { FLOAT_t val = *(FLOAT_t *)csv->attr._ptr;
	fprintf(stderr, " %10.3f", _I2F(val));
      } break;
    case CSVT_TSTAMP:
      {	TSTAMP_t *tvp = (TSTAMP_t *)csv->attr._ptr;
	static const char _fmt[] = " %Y-%m-%d %H:%M:%S";
	static char b[MSGBUFLEN];
	size_t nb = sizeof(b);
	char *be = b;
	size_t nf;
	struct tm tm;
	nf = strftime(be, nb, _fmt, gmtime_r(&tvp->tv_sec, &tm));
	fprintf(stderr, " %s", b);
      } break;
    case CSVT_TDIFF:
      {	TDIFF_t *tvp = (TDIFF_t *)csv->attr._ptr;
	static const char _fmt[] = "            %H:%M:%S";
	static char b[MSGBUFLEN];
	size_t nb = sizeof(b);
	char *be = b;
	size_t nf;
	struct tm tm;
	nf = strftime(be, nb, _fmt, gmtime_r(&tvp->tv_sec, &tm));
	fprintf(stderr, " %s", b);
      } break;
    }
    fprintf(stderr, "\n");
}

static void _PrintSENSOR(const char *msg, struct SENSOR_s * sensor, FILE *fp)
{
#define	_ENTRY(_t, _sN, _dN) \
	{ #_sN, CSVT_##_t, {&sensor->_sN}, {#_dN}, sizeof(sensor->_sN)}
    struct CSV_s csvSENSOR[] = {	/* #3 */
	_ENTRY(TSTAMP,   tstamp,	"Date and Time"),
	_ENTRY(string,   name,		"Sensor"),
	_ENTRY(uinteger, pts,		"Points"),
	_ENTRY(uinteger, rval,		"A2D Value"),
	_ENTRY(uinteger, rmax,		"A2D Maximum"),
	_ENTRY(FLOAT,    gain,		"Gain"),
	_ENTRY(FLOAT,    off,		"Offset"),
	_ENTRY(FLOAT,    sys,		"System Value"),
	_ENTRY(FLOAT,    ref,		"Reference Value"),
	_ENTRY(FLOAT,    avg,		"Average"),
	_ENTRY(FLOAT,    max,		"Maximum"),
	_ENTRY(FLOAT,    min,		"Minimum"),
	_ENTRY(FLOAT,    temp,		"Temperature"),
	_ENTRY(FLOAT,    pres,		"Pressure"),
	_ENTRY(uinteger, npts,		"No. of Measurements"),
	_ENTRY(FLOAT,    sum,		"Sum of Measurements"),
    };
#undef	_ENTRY
    size_t ncsvSENSOR = (sizeof(csvSENSOR)/sizeof(csvSENSOR[0]));

    fprintf(fp, "============ %s\n", msg);

    /* Fill in fields that are not sensor calculated. */
    tstamp(&sensor->tstamp);
    sensor->temp = urg->temp;
    sensor->pres = urg->pres;

    sensor->rval = (sensor->avg - sensor->off)/sensor->gain;
    for (size_t i = 0; i < ncsvSENSOR; i++) {
	_PrintCSVT(csvSENSOR+i, fp);
    }
}

static void _PrintSTATS(const char *msg, struct SENSOR_s * sensor, FILE *fp)
{
#define _ENTRY(_t, _sN, _dN) \
	{ #_sN, CSVT_##_t, {&sensor->_sN}, {#_dN}, sizeof(sensor->_sN)}
    struct CSV_s csvSTATS[] = {	/* #3 */
	_ENTRY(FLOAT,  avg,		"Average Ambient"),
	_ENTRY(FLOAT,  max,		"Maximum Ambient"),
	_ENTRY(FLOAT,  min,		"Minimum Ambient"),
    };
#undef	_ENTRY
    size_t ncsvSTATS = (sizeof(csvSTATS)/sizeof(csvSTATS[0]));
    fprintf(fp, "============ %s\n", msg);
    for (size_t i = 0; i < ncsvSTATS; i++) {
	_PrintCSVT(csvSTATS+i, fp);
    }
}

static void _PrintQC(const char *msg, struct SENSOR_s * sensor, FILE *fp)
{
#define _ENTRY(_t, _sN, _dN) \
	{ #_sN, CSVT_##_t, {&sensor->_sN}, {#_dN}, sizeof(sensor->_sN)}
    struct CSV_s csvQC[] = {	/* #3 */
	_ENTRY(TSTAMP, tstamp,		"Date and Time"),
	_ENTRY(string, name,		"QC Item"),
	_ENTRY(FLOAT,  sys,		"System Value"),
	_ENTRY(FLOAT,  ref,		"Reference Value"),
    };
#undef	_ENTRY
    size_t ncsvQC = (sizeof(csvQC)/sizeof(csvQC[0]));
    fprintf(fp, "============ %s\n", msg);
    for (size_t i = 0; i < ncsvQC; i++) {
	_PrintCSVT(csvQC+i, fp);
    }
}

static void _PrintTABLE(const char * msg, CSV_t csvTABLE, size_t ncsvTABLE, FILE *fp)
{
    fprintf(fp, "============ %s\n", msg);
    for (size_t i = 0; i < ncsvTABLE; i++)
	_PrintCSVT(csvTABLE+i, fp);
}

static void _PrintALL(FILE *fp)
{
#ifdef	NOTYET
    _PrintTABLE("EVENT",	csvEVENT, ncsvEVENT, fp);
    _PrintTABLE("DATA",		csvDATA, ncsvDATA, fp);

    _PrintTABLE("CALIBRATION",	csvCALIBRATION, ncsvCALIBRATION, fp);
#endif

    _PrintSENSOR("SENSOR",	&_urg.ambient, fp);
#ifdef	NOTYET
    _PrintSENSOR("SENSOR",	&_urg.filter, fp);
    _PrintSENSOR("SENSOR",	&_urg.meter, fp);
    _PrintSENSOR("SENSOR",	&_urg.inactive, fp);
#endif
    _PrintSENSOR("SENSOR",	&_urg.barometer, fp);
#ifdef	NOTYET
    _PrintSENSOR("SENSOR",	&_urg.meter_drop, fp);
    _PrintSENSOR("SENSOR",	&_urg.flow_sensor, fp);
#endif

    _PrintSTATS("Ambient",	&_urg.ambient, fp);
#ifdef	NOTYET
    _PrintSTATS("Filter",	&_urg.filter, fp);
    _PrintSTATS("Meter",	&_urg.meter, fp);
    _PrintSTATS("Inactive",	&_urg.inactive, fp);
#endif
    _PrintSTATS("Barometer",	&_urg.barometer, fp);
#ifdef	NOTYET
    _PrintSTATS("MeterDrop",	&_urg.meter_drop, fp);
    _PrintSTATS("FlowSensor",	&_urg.flow_sensor, fp);
#endif

#ifdef	NOTYET
    _PrintTABLE("QC",		csvQC, ncsvQC, fp);
#endif

    _PrintQC("Ambient",		&_urg.ambient, fp);
#ifdef	NOTYET
    _PrintQC("Filter",		&_urg.filter, fp);
    _PrintQC("Meter",		&_urg.meter, fp);
    _PrintQC("Inactive",	&_urg.inactive, fp);
#endif
    _PrintQC("Barometer",	&_urg.barometer, fp);
#ifdef	NOTYET
    _PrintQC("MeterDrop",	&_urg.meter_drop, fp);
    _PrintQC("FlowSensor",	&_urg.flow_sensor, fp);

    _PrintTABLE("POWERFAIL",	csvPOWERFAIL, ncsvPOWERFAIL, fp);
    _PrintTABLE("DEBUG",	csvDEBUG, ncsvDEBUG, fp);
#endif
    _PrintTABLE("SITE",		csvSITE, ncsvSITE, fp);
}

/*==============================================================*/
#ifdef	DYING
/* forward ref */
static int json_internal_spew_object(char *b, size_t nb,
                                      const struct json_attr_t *attrs,
                                      /*@null@ */
                                      const struct json_array_t *parent,
                                      int offset,
                                      /*@null@ */ const char **end);
static
int json_spew_object(char *b, size_t nb, const struct json_attr_t *attrs,
                    /*@null@*/const char **end);

static
int json_spew_array(char *b, size_t nb, const struct json_array_t *arr,
		 const char **end)
{
    char *be = b;
    /*@-nullstate -onlytrans@ */
    int arrcount = 0;
    int rc = JSON_ERR_MISC;	/* assume failure */

fprintf(stderr, "==> %s()\n", __FUNCTION__);

    if (end != NULL)
	*end = NULL;		/* well-defined value on parse failure */

    if (nb > 1) {
	*be++ = '[';
	nb--;
    }

    for (int offset = 0; offset < arr->maxlen; offset++) {
	char *ep = NULL;
	size_t nf = 0;
	int substatus;
fprintf(stderr, "%16s: [%d:%d]\n", CSVtypestr[arr->element_type%CSVT_LAST], offset, arr->maxlen);
	if (offset && nb > 1) {
	    *be++ = ',';
	    if (--nb == 0)
		break;
	}
	switch (arr->element_type) {
	case t_string:
fprintf(stderr, "\t\t%p[%u] %s\n", arr->arr.strings.ptrs, offset, arr->arr.strings.ptrs[offset]);
	    nf = snprintf(be, nb, "\"%s\"", arr->arr.strings.ptrs[offset]);
	    break;
	case t_object:
	case t_structobject:
fprintf(stderr, "\t\tt_object(%p) %p[%u]\n", arr->arr.objects.subtype, arr->arr.objects.base, (unsigned)arr->arr.objects.stride);
	    substatus =
		json_internal_spew_object(be, nb, arr->arr.objects.subtype,
					arr, offset, &ep);
	    nf = strlen(be);	/* XXX (e - be) */
	    if (substatus) {
		rc = substatus;
		goto exit;
	    }
	    break;
	case t_integer:
fprintf(stderr, "\t\t%p[%u] %d\n", arr->arr.integers.store, offset, arr->arr.integers.store[offset]);
	    nf = snprintf(be, nb, "%d", arr->arr.integers.store[offset]);
	    break;
	case t_uinteger:
fprintf(stderr, "\t\t%p[%u] %u\n", arr->arr.uintegers.store, offset, arr->arr.uintegers.store[offset]);
	    nf = snprintf(be, nb, "%d", arr->arr.uintegers.store[offset]);
	    break;
	case t_time:
#ifdef MICROJSON_TIME_ENABLE
fprintf(stderr, "\t\t%p[%u] %g\n", arr->arr.reals.store, offset, arr->arr.reals.store[offset]);
	{   static const char _fmt[] = "\"%Y-%m-%dT%H:%M:%S\"";
	    struct timeval tv;
	    struct tm tm;
	    tv.tv_sec = arr->arr.reals.store[offset];
	    nf = strftime(be, nb, _fmt, gmtime_r(&tv.tv_sec, &tm));
	}   break;
#else
	    /*@fallthrough@*/
#endif				/* MICROJSON_TIME_ENABLE */
	case t_real:
fprintf(stderr, "\t\t%p[%u] %g\n", arr->arr.reals.store, offset, arr->arr.reals.store[offset]);
	    nf = snprintf(be, nb, "%g", arr->arr.reals.store[offset]);
	    break;
	case t_boolean:
fprintf(stderr, "\t\t%p[%u] %s\n", arr->arr.booleans.store, offset, (arr->arr.booleans.store[offset] ? "true" : "false"));
	    nf = snprintf(be, nb, "%s",
			(arr->arr.booleans.store[offset] ? "true" : "false"));
	    break;
	case t_character:
	case t_array:
	case t_check:
	case t_ignore:
	    json_debug_trace((1, "Invalid array subtype.\n"));
	    rc = JSON_ERR_SUBTYPE;
	    goto exit;
	}
	be += nf;
	nb -= nf;
	arrcount++;
    }

    if (nb > 1) {
	*be++ = ']';
	nb--;
    }
    rc = (nb ? 0 : JSON_ERR_SUBTOOLONG);

exit:
    *be = '\0';
    if (end)
	*end = be;
    return rc;
    /*@+nullstate +onlytrans@ */
}

static int json_internal_spew_object(char *b, size_t nb,
                                      const struct json_attr_t *attrs,
                                      /*@null@ */
                                      const struct json_array_t *parent,
                                      int offset,
                                      /*@null@ */ const char **end)
{
    char *be = b;
    const struct json_attr_t *cursor;
    int ix = 0;
    int rc = JSON_ERR_MISC;	/* assume failure */

fprintf(stderr, "==> %s() parent %p[%u]\n", __FUNCTION__, parent, offset);

    if (nb > 1) {
	*be++ = '{';
	nb--;
    }

    for (cursor = attrs; cursor->attribute != NULL; cursor++) {
	char *ep = NULL;
	size_t nf = 0;
	int substatus;
	const struct json_enum_t *mp;
	char * lptr;

	lptr = json_target_address(cursor, parent, offset);
fprintf(stderr, "%16s: %p\n", CSVtypestr[cursor->type%CSVT_LAST], lptr);
	if (lptr == NULL && cursor->type != t_array)
	    continue;
	if (cursor->dflt.check != NULL)
	    continue;

	if (ix++ && nb > 1) {
	    *be++ = ',';
	    nb--;
	}
	switch (cursor->type) {
	case t_integer:
	{   int val = *(int *)lptr;
	    for (mp = cursor->map; mp != NULL && mp->name != NULL; mp++) {
		if (mp->value != val)
		    continue;
		ep = mp->name;
		break;
	    }
	    if (ep) {
fprintf(stderr, "\t\t%s: %s\n", cursor->attribute, ep);
		nf = snprintf(be, nb, "\"%s\":\"%s\"", cursor->attribute, ep);
	    } else {
fprintf(stderr, "\t\t%s: %d\n", cursor->attribute, val);
		nf = snprintf(be, nb, "\"%s\":%d", cursor->attribute, val);
	    }
	}   break;
	case t_uinteger:
	{   unsigned val = *(unsigned *)lptr;
	    for (mp = cursor->map; mp != NULL && mp->name != NULL; mp++) {
		if (mp->value != (int)val)
		    continue;
		ep = mp->name;
		break;
	    }
	    if (ep) {
fprintf(stderr, "\t\t%s: %s\n", cursor->attribute, ep);
		nf = snprintf(be, nb, "\"%s\":\"%s\"", cursor->attribute, ep);
	    } else {
fprintf(stderr, "\t\t%s: %u\n", cursor->attribute, val);
		nf = snprintf(be, nb, "\"%s\":%u", cursor->attribute, val);
	    }
	}   break;
	case t_time:
#ifdef MICROJSON_TIME_ENABLE
fprintf(stderr, "\t\t%s: t_time\n", cursor->attribute);
	{   double val = *(double *)lptr;
	    static const char _fmt[] = "\"%Y-%m-%dT%H:%M:%S\"";
	    struct timeval tv;
	    struct tm tm;
	    tv.tv_sec = val;
	    nf = strftime(be, nb, _fmt, gmtime_r(&tv.tv_sec, &tm));
	}   break;
#else
	    /*@fallthrough@*/
#endif				/* MICROJSON_TIME_ENABLE */
	case t_real:
	{   double val = *(double *)lptr;
fprintf(stderr, "\t\t%s: %g\n", cursor->attribute, val);
	    nf = snprintf(be, nb, "\"%s\":%g", cursor->attribute, val);
	}   break;
	case t_string:
#ifdef	NOTYET
	    if (parent != NULL
	     && parent->element_type != t_structobject
	     && offset > 0)
		return JSON_ERR_NOPARSTR;
	    lptr[0] = '\0';
#endif
	{   const char *val = (const char *)lptr;
fprintf(stderr, "\t\t%s: %s\n", cursor->attribute, val);
	    nf = snprintf(be, nb, "\"%s\":\"%s\"", cursor->attribute, val);
	}   break;
	case t_boolean:
	{   bool val = *(bool *)lptr;
fprintf(stderr, "\t\t%s: %s\n", cursor->attribute, (val ? "true" : "false"));
	    nf = snprintf(be, nb, "\"%s\":%s", cursor->attribute, (val ? "true" : "false"));
	}   break;
	case t_character:
	{   const char val = *(const char *)lptr;
fprintf(stderr, "\t\t%s: %c\n", cursor->attribute, val);
	    nf = snprintf(be, nb, "\"%s\":\"%c\"", cursor->attribute, val);
	}   break;
	case t_object:
fprintf(stderr, "\t\t%s: t_object\n", cursor->attribute);
	    break;
	case t_structobject:
fprintf(stderr, "\t\t%s: t_structobject\n", cursor->attribute);
	    break;
	case t_array:
fprintf(stderr, "\t\t%s: t_array\n", cursor->attribute);
	    nf = snprintf(be, nb, "\"%s\":", cursor->attribute);
		be += nf;
		nb -= nf;
	    substatus = json_spew_array(be, nb, &cursor->addr.array, NULL);
	    nf = strlen(be);
	    if (substatus) {
		rc = substatus;
		goto exit;
	    }
	    break;
	case t_check:
fprintf(stderr, "\t\t%s: t_check\n", cursor->attribute);
	    break;
	case t_ignore:
fprintf(stderr, "\t\t%s: t_ignore\n", cursor->attribute);
	    break;
	}

	be += nf;
	nb -= nf;
	if (ix > 1 && nf == 0) {
	    /* back up over comma */
	    ix--;
	    be--;
	    nb++;
	}
    }

    if (nb > 1) {
	*be++ = '}';
	nb--;
    }
    rc = (nb ? 0 : JSON_ERR_SUBTOOLONG);

exit:
    *be = '\0';
    if (end)
	*end = be;
    return rc;
}

static
int json_spew_object(char *b, size_t nb, const struct json_attr_t *attrs,
                    /*@null@*/const char **end)
{
    int st;
fprintf(stderr, "==> %s()\n", __FUNCTION__);
    st = json_internal_spew_object(b, nb, attrs, NULL, 0, end);
    return st;
}
#endif

/*==============================================================*/

/* test_mjson.c - unit test for JSON parsing into fixed-extent structures
 *
 * This file is Copyright (c) 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 */

/*
 * Many of these structures and examples were dissected out of the GPSD code.
 */

#define MAXCHANNELS	20
#define MAXUSERDEVS	4
#define JSON_DATE_MAX	24	/* ISO8601 timestamp with 2 decimal places */

/* these values don't matter in themselves, they just have to be out-of-band */
#define DEVDEFAULT_BPS  	0
#define DEVDEFAULT_PARITY	'X'
#define DEVDEFAULT_STOPBITS	3
#define DEVDEFAULT_NATIVE	-1

typedef double timestamp_t;	/* Unix time in seconds with fractional part */

struct dop_t {
    /* Dilution of precision factors */
    double xdop, ydop, pdop, hdop, vdop, tdop, gdop;
};

struct version_t {
    char release[64];			/* external version */
    char rev[64];			/* internal revision ID */
    int proto_major, proto_minor;	/* API major and minor versions */
    char remote[PATH_MAX];		/* could be from a remote device */
};

struct devconfig_t {
    char path[PATH_MAX];
    int flags;
#define SEEN_GPS 	0x01
#define SEEN_RTCM2	0x02
#define SEEN_RTCM3	0x04
#define SEEN_AIS 	0x08
    char driver[64];
    char subtype[64];
    double activated;
    unsigned int baudrate, stopbits;	/* RS232 link parameters */
    char parity;			/* 'N', 'O', or 'E' */
    double cycle, mincycle;     	/* refresh cycle time in seconds */
    int driver_mode;    		/* is driver in native mode or not? */
};
struct gps_fix_t {
    timestamp_t time;	/* Time of update */
    int    mode;	/* Mode of fix */
#define MODE_NOT_SEEN	0	/* mode update not seen yet */
#define MODE_NO_FIX	1	/* none */
#define MODE_2D  	2	/* good for latitude/longitude */
#define MODE_3D  	3	/* good for altitude/climb too */
    double ept;		/* Expected time uncertainty */
    double latitude;	/* Latitude in degrees (valid if mode >= 2) */
    double epy;  	/* Latitude position uncertainty, meters */
    double longitude;	/* Longitude in degrees (valid if mode >= 2) */
    double epx;  	/* Longitude position uncertainty, meters */
    double altitude;	/* Altitude in meters (valid if mode == 3) */
    double epv;  	/* Vertical position uncertainty, meters */
    double track;	/* Course made good (relative to true north) */
    double epd;		/* Track uncertainty, degrees */
    double speed;	/* Speed over ground, meters/sec */
    double eps;		/* Speed uncertainty, meters/sec */
    double climb;       /* Vertical speed, meters/sec */
    double epc;		/* Vertical speed uncertainty */
};

struct gps_data_t {
    struct gps_fix_t	fix;	/* accumulated PVT data */

    /* this should move to the per-driver structure */
    double separation;		/* Geoidal separation, MSL - WGS84 (Meters) */

    /* GPS status -- always valid */
    int    status;		/* Do we have a fix? */
#define STATUS_NO_FIX	0	/* no */
#define STATUS_FIX	1	/* yes, without DGPS */
#define STATUS_DGPS_FIX	2	/* yes, with DGPS */

    /* precision of fix -- valid if satellites_used > 0 */
    int satellites_used;	/* Number of satellites used in solution */
    int used[MAXCHANNELS];	/* PRNs of satellites used in solution */
    struct dop_t dop;

    /* redundant with the estimate elements in the fix structure */
    double epe;  /* spherical position error, 95% confidence (meters)  */

    /* satellite status -- valid when satellites_visible > 0 */
    timestamp_t skyview_time;	/* skyview timestamp */
    int satellites_visible;	/* # of satellites in view */
    int PRN[MAXCHANNELS];	/* PRNs of satellite */
    int elevation[MAXCHANNELS];	/* elevation of satellite */
    int azimuth[MAXCHANNELS];	/* azimuth */
    double ss[MAXCHANNELS];	/* signal-to-noise ratio (dB) */

    struct devconfig_t dev;	/* device that shipped last update */

    struct {
	timestamp_t time;
	int ndevices;
	struct devconfig_t list[MAXUSERDEVS];
    } devices;

    struct version_t version;
};

static struct gps_data_t gpsdata;

/*
 * There's a splint limitation that parameters can be declared
 * @out@ or @null@ but not, apparently, both.  This collides with
 * the (admittedly tricky) way we use endptr. The workaround is to
 * declare it @null@ and use -compdef around the JSON reader calls.
 */
/*@-compdef@*/

static int json_tpv_read(const char *buf, struct gps_data_t *gpsdata,
			 /*@null@*/ const char **endptr)
{
    /*@ -fullinitblock @*/
    const struct json_attr_t json_attrs_1[] = {
	/* *INDENT-OFF* */
	{"class",  t_check,   .dflt.check = "TPV"},
	{"device", t_string,  .addr.string = gpsdata->dev.path,
			         .len = sizeof(gpsdata->dev.path)},
#ifdef MICROJSON_TIME_ENABLE
	{"time",   t_time,    .addr.real = &gpsdata->fix.time,
			         .dflt.real = NAN},
#else
	{"time",   t_ignore},
#endif /* MICROJSON_TIME_ENABLE */
	{"ept",    t_real,    .addr.real = &gpsdata->fix.ept,
			         .dflt.real = NAN},
	{"lon",    t_real,    .addr.real = &gpsdata->fix.longitude,
			         .dflt.real = NAN},
	{"lat",    t_real,    .addr.real = &gpsdata->fix.latitude,
			         .dflt.real = NAN},
	{"alt",    t_real,    .addr.real = &gpsdata->fix.altitude,
			         .dflt.real = NAN},
	{"epx",    t_real,    .addr.real = &gpsdata->fix.epx,
			         .dflt.real = NAN},
	{"epy",    t_real,    .addr.real = &gpsdata->fix.epy,
			         .dflt.real = NAN},
	{"epv",    t_real,    .addr.real = &gpsdata->fix.epv,
			         .dflt.real = NAN},
	{"track",   t_real,   .addr.real = &gpsdata->fix.track,
			         .dflt.real = NAN},
	{"speed",   t_real,   .addr.real = &gpsdata->fix.speed,
			         .dflt.real = NAN},
	{"climb",   t_real,   .addr.real = &gpsdata->fix.climb,
			         .dflt.real = NAN},
	{"epd",    t_real,    .addr.real = &gpsdata->fix.epd,
			         .dflt.real = NAN},
	{"eps",    t_real,    .addr.real = &gpsdata->fix.eps,
			         .dflt.real = NAN},
	{"epc",    t_real,    .addr.real = &gpsdata->fix.epc,
			         .dflt.real = NAN},
	{"mode",   t_integer, .addr.integer = &gpsdata->fix.mode,
			         .dflt.integer = MODE_NOT_SEEN},
	{NULL},
	/* *INDENT-ON* */
    };
    /*@ +fullinitblock @*/

    return json_read_object(buf, json_attrs_1, endptr);
}

static int json_sky_read(const char *buf, struct gps_data_t *gpsdata,
			 /*@null@*/ const char **endptr)
{
    bool usedflags[MAXCHANNELS];
    /*@ -fullinitblock @*/
    const struct json_attr_t json_attrs_2_1[] = {
	/* *INDENT-OFF* */
	{"PRN",	   t_integer, .addr.integer = gpsdata->PRN},
	{"el",	   t_integer, .addr.integer = gpsdata->elevation},
	{"az",	   t_integer, .addr.integer = gpsdata->azimuth},
	{"ss",	   t_real,    .addr.real = gpsdata->ss},
	{"used",   t_boolean, .addr.boolean = usedflags},
	/* *INDENT-ON* */
	{NULL},
    };
    const struct json_attr_t json_attrs_2[] = {
	/* *INDENT-OFF* */
	{"class",      t_check,   .dflt.check = "SKY"},
	{"device",     t_string,  .addr.string  = gpsdata->dev.path,
	                             .len = sizeof(gpsdata->dev.path)},
	{"hdop",       t_real,    .addr.real    = &gpsdata->dop.hdop,
	                             .dflt.real = NAN},
	{"xdop",       t_real,    .addr.real    = &gpsdata->dop.xdop,
	                             .dflt.real = NAN},
	{"ydop",       t_real,    .addr.real    = &gpsdata->dop.ydop,
	                             .dflt.real = NAN},
	{"vdop",       t_real,    .addr.real    = &gpsdata->dop.vdop,
	                             .dflt.real = NAN},
	{"tdop",       t_real,    .addr.real    = &gpsdata->dop.tdop,
	                             .dflt.real = NAN},
	{"pdop",       t_real,    .addr.real    = &gpsdata->dop.pdop,
	                             .dflt.real = NAN},
	{"gdop",       t_real,    .addr.real    = &gpsdata->dop.gdop,
	                             .dflt.real = NAN},
	{"satellites", t_array,   .addr.array.element_type = t_object,
				     .addr.array.arr.objects.subtype=json_attrs_2_1,
	                             .addr.array.maxlen = MAXCHANNELS,
	                             .addr.array.count = &gpsdata->satellites_visible},
	{NULL},
	/* *INDENT-ON* */
    };
    /*@ +fullinitblock @*/
    int status, i, j;

    for (i = 0; i < MAXCHANNELS; i++) {
	gpsdata->PRN[i] = 0;
	usedflags[i] = false;
    }

    status = json_read_object(buf, json_attrs_2, endptr);
    if (status != 0)
	return status;

    gpsdata->satellites_used = 0;
    gpsdata->satellites_visible = 0;
    (void)memset(gpsdata->used, '\0', sizeof(gpsdata->used));
    for (i = j = 0; i < MAXCHANNELS; i++) {
	if(gpsdata->PRN[i] > 0)
	    gpsdata->satellites_visible++;
	if (usedflags[i]) {
	    gpsdata->used[j++] = gpsdata->PRN[i];
	    gpsdata->satellites_used++;
	}
    }

    return 0;
}

static int json_devicelist_read(const char *buf, struct gps_data_t *gpsdata,
				/*@null@*/ const char **endptr)
{
    /*@ -fullinitblock @*/
    const struct json_attr_t json_attrs_subdevices[] = {
	/* *INDENT-OFF* */
	{"class",      t_check,      .dflt.check = "DEVICE"},
	{"path",       t_string,     STRUCTOBJECT(struct devconfig_t, path),
	                                .len = sizeof(gpsdata->devices.list[0].path)},
	{"activated",  t_real,       STRUCTOBJECT(struct devconfig_t, activated)},
	{"flags",      t_integer,    STRUCTOBJECT(struct devconfig_t, flags)},
	{"driver",     t_string,     STRUCTOBJECT(struct devconfig_t, driver),
	                                .len = sizeof(gpsdata->devices.list[0].driver)},
	{"subtype",    t_string,     STRUCTOBJECT(struct devconfig_t, subtype),
	                                .len = sizeof(gpsdata->devices.list[0].subtype)},
	{"native",     t_integer,    STRUCTOBJECT(struct devconfig_t, driver_mode),
				        .dflt.integer = -1},
	{"bps",	       t_uinteger,   STRUCTOBJECT(struct devconfig_t, baudrate),
				        .dflt.uinteger = DEVDEFAULT_BPS},
	{"parity",     t_character,  STRUCTOBJECT(struct devconfig_t, parity),
	                                .dflt.character = DEVDEFAULT_PARITY},
	{"stopbits",   t_uinteger,   STRUCTOBJECT(struct devconfig_t, stopbits),
				        .dflt.integer = DEVDEFAULT_STOPBITS},
	{"cycle",      t_real,       STRUCTOBJECT(struct devconfig_t, cycle),
				        .dflt.real = NAN},
	{"mincycle",   t_real,       STRUCTOBJECT(struct devconfig_t, mincycle),
				        .dflt.real = NAN},
	{NULL},
	/* *INDENT-ON* */
    };
    /*@-type@*//* STRUCTARRAY confuses splint */
    const struct json_attr_t json_attrs_devices[] = {
	{"class", t_check,.dflt.check = "DEVICES"},
	{"devices", t_array, STRUCTARRAY(gpsdata->devices.list,
					 json_attrs_subdevices,
					 &gpsdata->devices.ndevices)},
	{NULL},
    };
    /*@+type@*/
    /*@ +fullinitblock @*/
    int status;

    memset(&gpsdata->devices, '\0', sizeof(gpsdata->devices));
    status = json_read_object(buf, json_attrs_devices, endptr);
    if (status != 0) {
	return status;
    }
    return 0;
}

static int json_device_read(const char *buf,
		     /*@out@*/ struct devconfig_t *dev,
		     /*@null@*/ const char **endptr)
{
    char tbuf[JSON_DATE_MAX+1];
    /*@ -fullinitblock @*/
    /* *INDENT-OFF* */
    const struct json_attr_t json_attrs_device[] = {
	{"class",      t_check,      .dflt.check = "DEVICE"},

        {"path",       t_string,     .addr.string  = dev->path,
	                                .len = sizeof(dev->path)},
	{"activated",  t_string,     .addr.string = tbuf,
			                .len = sizeof(tbuf)},
	{"activated",  t_real,       .addr.real = &dev->activated},
	{"flags",      t_integer,    .addr.integer = &dev->flags},
	{"driver",     t_string,     .addr.string  = dev->driver,
	                                .len = sizeof(dev->driver)},
	{"subtype",    t_string,     .addr.string  = dev->subtype,
	                                .len = sizeof(dev->subtype)},
	{"native",     t_integer,    .addr.integer = &dev->driver_mode,
				        .dflt.integer = DEVDEFAULT_NATIVE},
	{"bps",	       t_uinteger,   .addr.uinteger = &dev->baudrate,
				        .dflt.uinteger = DEVDEFAULT_BPS},
	{"parity",     t_character,  .addr.character = &dev->parity,
                                        .dflt.character = DEVDEFAULT_PARITY},
	{"stopbits",   t_uinteger,   .addr.uinteger = &dev->stopbits,
				        .dflt.uinteger = DEVDEFAULT_STOPBITS},
	{"cycle",      t_real,       .addr.real = &dev->cycle,
				        .dflt.real = NAN},
	{"mincycle",   t_real,       .addr.real = &dev->mincycle,
				        .dflt.real = NAN},
	{NULL},
    };
    /* *INDENT-ON* */
    /*@ +fullinitblock @*/
    int status;

    tbuf[0] = '\0';
    status = json_read_object(buf, json_attrs_device, endptr);
    if (status != 0)
	return status;

    return 0;
}

static int json_version_read(const char *buf, struct gps_data_t *gpsdata,
			     /*@null@*/ const char **endptr)
{
    /*@ -fullinitblock @*/
    const struct json_attr_t json_attrs_version[] = {
	/* *INDENT-OFF* */
        {"class",     t_check,   .dflt.check = "VERSION"},
	{"release",   t_string,  .addr.string  = gpsdata->version.release,
	                            .len = sizeof(gpsdata->version.release)},
	{"rev",       t_string,  .addr.string  = gpsdata->version.rev,
	                            .len = sizeof(gpsdata->version.rev)},
	{"proto_major", t_integer, .addr.integer = &gpsdata->version.proto_major},
	{"proto_minor", t_integer, .addr.integer = &gpsdata->version.proto_minor},
	{"remote",    t_string,  .addr.string  = gpsdata->version.remote,
	                            .len = sizeof(gpsdata->version.remote)},
	{NULL},
	/* *INDENT-ON* */
    };
    /*@ +fullinitblock @*/
    int status;

    memset(&gpsdata->version, '\0', sizeof(gpsdata->version));
    status = json_read_object(buf, json_attrs_version, endptr);

    return status;
}

static int libgps_json_unpack(const char *buf,
		       struct gps_data_t *gpsdata, const char **end)
/* the only entry point - unpack a JSON object into gpsdata_t substructures */
{
    int status;
    char *classtag = strstr(buf, "\"class\":");

    if (classtag == NULL)
	return -1;
#define STARTSWITH(str, prefix)	strncmp(str, prefix, sizeof(prefix)-1)==0
    if (STARTSWITH(classtag, "\"class\":\"TPV\"")) {
	status = json_tpv_read(buf, gpsdata, end);
	gpsdata->status = STATUS_FIX;
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"SKY\"")) {
	status = json_sky_read(buf, gpsdata, end);
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"DEVICE\"")) {
	status = json_device_read(buf, &gpsdata->dev, end);
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"DEVICES\"")) {
	status = json_devicelist_read(buf, gpsdata, end);
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"VERSION\"")) {
	status = json_version_read(buf, gpsdata, end);
	return status;
    }
#undef STARTSWITH
    return 0;
}

static int libgps_json_repack(const char **cpp,
		       struct gps_data_t *gpsdata, const char **end)
/* the only entry point - unpack a JSON object into gpsdata_t substructures */
{
    int status = 0;

fprintf(stderr, "==> %s(%p)\n", __FUNCTION__, cpp);

#ifdef	NOTYET
    char *classtag = strstr(buf, "\"class\":");

    if (classtag == NULL)
	return -1;
#define STARTSWITH(str, prefix)	strncmp(str, prefix, sizeof(prefix)-1)==0
    if (STARTSWITH(classtag, "\"class\":\"TPV\"")) {
	status = json_tpv_read(buf, gpsdata, end);
	gpsdata->status = STATUS_FIX;
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"SKY\"")) {
	status = json_sky_read(buf, gpsdata, end);
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"DEVICE\"")) {
	status = json_device_read(buf, &gpsdata->dev, end);
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"DEVICES\"")) {
	status = json_devicelist_read(buf, gpsdata, end);
	return status;
    } else if (STARTSWITH(classtag, "\"class\":\"VERSION\"")) {
	status = json_version_read(buf, gpsdata, end);
	return status;
    }
#undef STARTSWITH
#endif
    return status;
}
/*@+compdef@*/

static void assert_case(int num, int status)
{
    if (status != 0) {
	(void)fprintf(stderr, "case %d FAILED, status %d (%s).\n", num,
		      status, json_error_string(status));
	exit(EXIT_FAILURE);
    }
}

static void assert_string(char *attr, char *fld, char *check)
{
    if (strcmp(fld, check)) {
	(void)fprintf(stderr,
		      "'%s' expecting string '%s', got '%s'.\n",
		      attr, check, fld);
	exit(EXIT_FAILURE);
    }
}

static void assert_integer(char *attr, int fld, int check)
{
    if (fld != check) {
	(void)fprintf(stderr,
		      "'%s' expecting integer %d, got %d.\n",
		      attr, check, fld);
	exit(EXIT_FAILURE);
    }
}

static void assert_uinteger(char *attr, uint fld, uint check)
{
    if (fld != check) {
	(void)fprintf(stderr,
		      "'%s' expecting uinteger %u, got %u.\n",
		      attr, check, fld);
	exit(EXIT_FAILURE);
    }
}

static void assert_boolean(char *attr, bool fld, bool check)
{
    /*@-boolcompare@*/
    if (fld != check) {
	(void)fprintf(stderr,
		      "'%s' expecting boolean %s, got %s.\n",
		      attr, 
		      check ? "true" : "false",
		      fld ? "true" : "false");
	exit(EXIT_FAILURE);
    }
    /*@+boolcompare@*/
}

/*
 * Floating point comparisons are iffy, but at least if any of these fail
 * the output will make it clear whether it was a precision issue
 */
static void assert_real(char *attr, double fld, double check)
{
    if (fld != check) {
	(void)fprintf(stderr,
		      "'%s' expecting real %f got %f.\n", 
		      attr, check, fld);
	exit(EXIT_FAILURE);
    }
}

/*@ -fullinitblock @*/

/* Case 1: TPV report */

/* *INDENT-OFF* */
static const char json_str1[] = "{\"class\":\"TPV\",\
    \"device\":\"GPS#1\",				\
    \"time\":\"2005-06-19T12:12:42.03Z\",		\
    \"lon\":46.498203637,\"lat\":7.568074350,           \
    \"alt\":1327.780,\"epx\":21.000,\"epy\":23.000,\"epv\":124.484,\"mode\":3}";

/* Case 2: SKY report */

static const char *json_str2 = "{\"class\":\"SKY\",\
         \"satellites\":[\
         {\"PRN\":10,\"el\":45,\"az\":196,\"ss\":34,\"used\":true},\
         {\"PRN\":29,\"el\":67,\"az\":310,\"ss\":40,\"used\":true},\
         {\"PRN\":28,\"el\":59,\"az\":108,\"ss\":42,\"used\":true},\
         {\"PRN\":26,\"el\":51,\"az\":304,\"ss\":43,\"used\":true},\
         {\"PRN\":8,\"el\":44,\"az\":58,\"ss\":41,\"used\":true},\
         {\"PRN\":27,\"el\":16,\"az\":66,\"ss\":39,\"used\":true},\
         {\"PRN\":21,\"el\":10,\"az\":301,\"ss\":0,\"used\":false}]}";

/* Case 3: String list syntax */

static const char *json_str3 = "[\"foo\",\"bar\",\"baz\"]";

static char *stringptrs[3];
static char stringstore[256];
static int stringcount;

/*@-type@*/
static const struct json_array_t json_array_3 = {
    .element_type = t_string,
    .arr.strings.ptrs = stringptrs,
    .arr.strings.store = stringstore,
    .arr.strings.storelen = sizeof(stringstore),
    .count = &stringcount,
    .maxlen = sizeof(stringptrs)/sizeof(stringptrs[0]),
};
/*@+type@*/

/* Case 4: test defaulting of unspecified attributes */

static const char *json_str4 = "{\"flag1\":true,\"flag2\":false}";

static bool flag1, flag2;
static double dftreal;
static int dftinteger;
static unsigned int dftuinteger;

static const struct json_attr_t json_attrs_4[] = {
    {"dftint",  t_integer, .addr.integer = &dftinteger, .dflt.integer = -5},
    {"dftuint", t_integer, .addr.uinteger = &dftuinteger, .dflt.uinteger = 10},
    {"dftreal", t_real,    .addr.real = &dftreal,       .dflt.real = 23.17},
    {"flag1",   t_boolean, .addr.boolean = &flag1,},
    {"flag2",   t_boolean, .addr.boolean = &flag2,},
    {NULL},
};

/* Case 5: test DEVICE parsing */

static const char *json_str5 = "{\"class\":\"DEVICE\",\
           \"path\":\"/dev/ttyUSB0\",\
           \"flags\":5,\
           \"driver\":\"Foonly\",\"subtype\":\"Foonly Frob\"\
           }";

/* Case 6: test parsing of subobject list into array of structures */

static const char *json_str6 = "{\"parts\":[\
{\"name\":\"Urgle\",\"flag\":true,\"count\":3},\
{\"name\":\"Burgle\",\"flag\":false,\"count\":1},\
{\"name\":\"Witter\",\"flag\":true,\"count\":4},\
{\"name\":\"Thud\",\"flag\":false,\"count\":1}]}";

struct dumbstruct_t {
    char name[64];
    bool flag;
    int count;
};
static struct dumbstruct_t dumbstruck[5];
static int dumbcount;

/*@-type@*/
static const struct json_attr_t json_attrs_6_subtype[] = {
    {"name",  t_string,  .addr.offset = offsetof(struct dumbstruct_t, name),
                         .len = 64},
    {"flag",  t_boolean, .addr.offset = offsetof(struct dumbstruct_t, flag),},
    {"count", t_integer, .addr.offset = offsetof(struct dumbstruct_t, count),},
    {NULL},
};

static const struct json_attr_t json_attrs_6[] = {
    {"parts", t_array, .addr.array.element_type = t_structobject,
                       .addr.array.arr.objects.base = (char*)&dumbstruck,
                       .addr.array.arr.objects.stride = sizeof(struct dumbstruct_t),
                       .addr.array.arr.objects.subtype = json_attrs_6_subtype,
                       .addr.array.count = &dumbcount,
                       .addr.array.maxlen = sizeof(dumbstruck)/sizeof(dumbstruck[0])-1},
    {NULL},
};
/*@+type@*/

/* Case 7: test parsing of version response */

static const char *json_str7 = "{\"class\":\"VERSION\",\
           \"release\":\"2.40dev\",\"rev\":\"dummy-revision\",\
           \"proto_major\":3,\"proto_minor\":1}";

/* Case 8: test parsing arrays of enumerated types */

static const char *json_str8 = "{\"fee\":\"FOO\",\"fie\":\"BAR\",\"foe\":\"BAZ\"}";
static const struct json_enum_t enum_table[] = {
    {"BAR", 6}, {"FOO", 3}, {"BAZ", 14}, {NULL}
};

static int fee, fie, foe;
static const struct json_attr_t json_attrs_8[] = {
    {"fee",  t_integer, .addr.integer = &fee, .map=enum_table},
    {"fie",  t_integer, .addr.integer = &fie, .map=enum_table},
    {"foe",  t_integer, .addr.integer = &foe, .map=enum_table},
    {NULL},
};

/* Case 9: Like case 6 but w/ an empty array */

static const char *json_str9 = "{\"parts\":[]}";

/* Case 10: Read array of integers */

static const char *json_str10 = "[23,-17,5]";
static int intstore[4], intcount;

/*@-type@*/
static const struct json_array_t json_array_10 = {
    .element_type = t_integer,
    .arr.integers.store = intstore,
    .count = &intcount,
    .maxlen = sizeof(intstore)/sizeof(intstore[0])-1,
};
/*@+type@*/

/* Case 11: Read array of booleans */

static const char *json_str11 = "[true,false,true]";
static bool boolstore[4];
static int boolcount;

/*@-type@*/
static const struct json_array_t json_array_11 = {
    .element_type = t_boolean,
    .arr.booleans.store = boolstore,
    .count = &boolcount,
    .maxlen = sizeof(boolstore)/sizeof(boolstore[0])-1,
};
/*@+type@*/

/* Case 12: Read array of reals */

static const char *json_str12 = "[23.1,-17.2,5.3]";
static double realstore[4]; 
static int realcount;

/*@-type@*/
static const struct json_array_t json_array_12 = {
    .element_type = t_real,
    .arr.reals.store = realstore,
    .count = &realcount,
    .maxlen = sizeof(realstore)/sizeof(realstore[0])-1,
};
/*@+type@*/

/*@ +fullinitblock @*/
/* *INDENT-ON* */

static void jsontest(int i)
{
    char b[BUFSIZ];
    size_t nb = sizeof(b);
    int status = 0;

    switch (i) 
    {
    case 1:
	status = libgps_json_unpack(json_str1, &gpsdata, NULL);
	assert_case(1, status);
	assert_string("device", gpsdata.dev.path, "GPS#1");
#ifdef MICROJSON_TIME_ENABLE
	assert_real("time", gpsdata.fix.time, 1119183162.030000);
#endif /* MICROJSON_TIME_ENABLE */
	assert_integer("mode", gpsdata.fix.mode, 3);
	assert_real("lon", gpsdata.fix.longitude, 46.498203637);
	assert_real("lat", gpsdata.fix.latitude, 7.568074350);
status = libgps_json_repack(NULL, &gpsdata, NULL);
	break;

    case 2:
	status = libgps_json_unpack(json_str2, &gpsdata, NULL);
	assert_case(2, status);
	assert_integer("used", gpsdata.satellites_used, 6);
	assert_integer("PRN[0]", gpsdata.PRN[0], 10);
	assert_integer("el[0]", gpsdata.elevation[0], 45);
	assert_integer("az[0]", gpsdata.azimuth[0], 196);
	assert_real("ss[0]", gpsdata.ss[0], 34);
	assert_integer("used[0]", gpsdata.used[0], 10);
	assert_integer("used[5]", gpsdata.used[5], 27);
	assert_integer("PRN[6]", gpsdata.PRN[6], 21);
	assert_integer("el[6]", gpsdata.elevation[6], 10);
	assert_integer("az[6]", gpsdata.azimuth[6], 301);
	assert_real("ss[6]", gpsdata.ss[6], 0);
status = libgps_json_repack(NULL, &gpsdata, NULL);
	break;

    case 3:
	status = json_read_array(json_str3, &json_array_3, NULL);
	assert_case(3, status);
	assert(stringcount == 3);
	assert(strcmp(stringptrs[0], "foo") == 0);
	assert(strcmp(stringptrs[1], "bar") == 0);
	assert(strcmp(stringptrs[2], "baz") == 0);
b[0] = '\0';
status = json_spew_array(b, nb, &json_array_3, NULL);
fprintf(stderr, "\t|%s|\n", json_str3);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 4:
	status = json_read_object(json_str4, json_attrs_4, NULL);
	assert_case(4, status);
	assert_integer("dftint", dftinteger, -5);	/* did the default work? */
	assert_uinteger("dftuint", dftuinteger, 10);	/* did the default work? */
	assert_real("dftreal", dftreal, 23.17);	/* did the default work? */
	assert_boolean("flag1", flag1, true);
	assert_boolean("flag2", flag2, false);
b[0] = '\0';
status = json_spew_object(b, nb, json_attrs_4, NULL);
fprintf(stderr, "\t|%s|\n", json_str4);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 5:
	status = libgps_json_unpack(json_str5, &gpsdata, NULL);
	assert_case(5, status);
	assert_string("path", gpsdata.dev.path, "/dev/ttyUSB0");
	assert_integer("flags", gpsdata.dev.flags, 5);
	assert_string("driver", gpsdata.dev.driver, "Foonly");
b[0] = '\0';
status = libgps_json_repack(NULL, &gpsdata, NULL);
	break;

    case 6:
	status = json_read_object(json_str6, json_attrs_6, NULL);
	assert_case(6, status);
	assert_integer("dumbcount", dumbcount, 4);
	assert_string("dumbstruck[0].name", dumbstruck[0].name, "Urgle");
	assert_string("dumbstruck[1].name", dumbstruck[1].name, "Burgle");
	assert_string("dumbstruck[2].name", dumbstruck[2].name, "Witter");
	assert_string("dumbstruck[3].name", dumbstruck[3].name, "Thud");
	assert_boolean("dumbstruck[0].flag", dumbstruck[0].flag, true);
	assert_boolean("dumbstruck[1].flag", dumbstruck[1].flag, false);
	assert_boolean("dumbstruck[2].flag", dumbstruck[2].flag, true);
	assert_boolean("dumbstruck[3].flag", dumbstruck[3].flag, false);
	assert_integer("dumbstruck[0].count", dumbstruck[0].count, 3);
	assert_integer("dumbstruck[1].count", dumbstruck[1].count, 1);
	assert_integer("dumbstruck[2].count", dumbstruck[2].count, 4);
	assert_integer("dumbstruck[3].count", dumbstruck[3].count, 1);
b[0] = '\0';
status = json_spew_object(b, nb, json_attrs_6, NULL);
fprintf(stderr, "\t|%s|\n", json_str6);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 7:
	status = libgps_json_unpack(json_str7, &gpsdata, NULL);
	assert_case(7, status);
	assert_string("release", gpsdata.version.release, "2.40dev");
	assert_string("rev", gpsdata.version.rev, "dummy-revision");
	assert_integer("proto_major", gpsdata.version.proto_major, 3);
	assert_integer("proto_minor", gpsdata.version.proto_minor, 1);
b[0] = '\0';
status = libgps_json_repack(NULL, &gpsdata, NULL);
	break;

    case 8:
	status = json_read_object(json_str8, json_attrs_8, NULL);
	assert_case(8, status);
	assert_integer("fee", fee, 3);
	assert_integer("fie", fie, 6);
	assert_integer("foe", foe, 14);
b[0] = '\0';
status = json_spew_object(b, nb, json_attrs_8, NULL);
fprintf(stderr, "\t|%s|\n", json_str8);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 9:
	/* yes, the '6' in the next line is correct */ 
	status = json_read_object(json_str9, json_attrs_6, NULL);
	assert_case(9, status);
	assert_integer("dumbcount", dumbcount, 0);
b[0] = '\0';
status = json_spew_object(b, nb, json_attrs_6, NULL);
fprintf(stderr, "\t|%s|\n", json_str9);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 10:
	status = json_read_array(json_str10, &json_array_10, NULL);
	assert_integer("count", intcount, 3);
	assert_integer("intstore[0]", intstore[0], 23);
	assert_integer("intstore[1]", intstore[1], -17);
	assert_integer("intstore[2]", intstore[2], 5);
	assert_integer("intstore[3]", intstore[3], 0);
b[0] = '\0';
status = json_spew_array(b, nb, &json_array_10, NULL);
fprintf(stderr, "\t|%s|\n", json_str10);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 11:
	status = json_read_array(json_str11, &json_array_11, NULL);
	assert_integer("count", boolcount, 3);
	assert_boolean("boolstore[0]", boolstore[0], true);
	assert_boolean("boolstore[1]", boolstore[1], false);
	assert_boolean("boolstore[2]", boolstore[2], true);
	assert_boolean("boolstore[3]", boolstore[3], false);
b[0] = '\0';
status = json_spew_array(b, nb, &json_array_11, NULL);
fprintf(stderr, "\t|%s|\n", json_str11);
fprintf(stderr, "\t|%s|\n", b);
	break;

    case 12:
	status = json_read_array(json_str12, &json_array_12, NULL);
	assert_integer("count", realcount, 3);
	assert_real("realstore[0]", realstore[0], 23.1);
	assert_real("realstore[1]", realstore[1], -17.2);
	assert_real("realstore[2]", realstore[2], 5.3);
	assert_real("realstore[3]", realstore[3], 0);
b[0] = '\0';
status = json_spew_array(b, nb, &json_array_12, NULL);
fprintf(stderr, "\t|%s|\n", json_str12);
fprintf(stderr, "\t|%s|\n", b);
	break;

#define MAXTEST 12

    default:
	(int)fputs("Unknown test number\n", stderr);
	break;
    }

    if (status > 0)
	printf("Parse failure!\n");
}

static int _DoJSON(rpmmqtt mqtt)
{
    int status = 0;
    _PrintALL(stderr);
    for (int i = 1; i <= MAXTEST; i++) {
fprintf(stderr, "======== test %d\n", i);
	jsontest(i);
    }
    return status;
}

/*==============================================================*/
static void hdl (int sig)
{
    exit_request = 1;
}

static struct poptOption optionsTable[] = {

 { NULL, '\0', POPT_ARG_INCLUDE_TABLE, rpmioAllPoptTable, 0,
	N_("Common options for all rpmio executables:"),
	NULL },

  POPT_AUTOHELP
  POPT_TABLEEND
};

int
main(int argc, char *argv[])
{
    poptContext optCon = rpmioInit(argc, argv, optionsTable);
    ARGV_t av = poptGetArgs(optCon);
#ifdef	UNUSED
    int ac = argvCount(av);
#endif
    rpmmqtt mqtt = rpmmqttNew((char **)av, 0);
    int rc = -1;

    struct sigaction act;
    memset (&act, 0, sizeof(act));
    act.sa_handler = hdl;
    if (sigaction(SIGTERM, &act, 0)) {
	perror ("sigaction");
	goto exit;
    }
 
    sigemptyset(&mask);
    sigaddset(&mask, SIGTERM);
    if (sigprocmask(SIG_BLOCK, &mask, &omask) < 0) {
	perror ("sigprocmask");
	goto exit;
    }

    rc = _Doit(mqtt);

    rc = _DoJSON(mqtt);

exit:
    mqtt = rpmmqttFree(mqtt);
    optCon = rpmioFini(optCon);
    return rc;
}
