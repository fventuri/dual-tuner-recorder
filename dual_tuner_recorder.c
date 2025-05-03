/* simple C program to record to file the I/Q streams
 * from a RSPduo in dual tuner mode
 * Franco Venturi - Sun Mar 16 10:27:24 AM EDT 2025
 */

/*
 * Copyright 2025 Franco Venturi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <sdrplay_api.h>

#define NUM_BLOCKS 100
#define NUM_SAMPLES 65536

#define NUM_GAIN_CHANGES 100

static char default_output_filename_raw[] = "RSPduo_dual_tuner_{TIMESTAMP}_{FREQKHZ}.iq";
static char default_output_filename_linrad[] = "RSPduo_dual_tuner_{TIMESTAMP}_{FREQKHZ}.raw";
static char default_output_filename_wav[] = "RSPduo_dual_tuner_{TIMESTAMP}_{FREQHZ}.wav";

#define UNUSED(x) (void)(x)

typedef enum {
    STREAMING_STATUS_UNKNOWN,
    STREAMING_STATUS_STARTING,
    STREAMING_STATUS_RUNNING,
    STREAMING_STATUS_TERMINATE,
    STREAMING_STATUS_DONE,
    STREAMING_STATUS_FAILED,
    STREAMING_STATUS_BLOCKS_BUFFER_FULL,
    STREAMING_STATUS_SAMPLES_BUFFER_FULL,
    STREAMING_STATUS_GAIN_CHANGES_BUFFER_FULL,
} StreamingStatus;

typedef enum {
    OUTPUT_TYPE_UNKNOWN,
    OUTPUT_TYPE_RAW,
    OUTPUT_TYPE_LINRAD,
    OUTPUT_TYPE_WAV,
} OutputType;

typedef struct {
    unsigned int first_sample_num;
    unsigned int num_samples;
    unsigned int samples_index;
    char rx_id;
} BlockDescriptor;

typedef struct {
    pthread_mutex_t *lock;
    void *resource;
    unsigned int read_index;
    unsigned int write_index;
    unsigned int size;
    unsigned int nused;
    unsigned int nready;
    pthread_cond_t *is_ready;
} ResourceDescriptor;

typedef struct {
   struct timespec ts;
   unsigned long long sample_num;
} TimeMarker;

typedef struct {
    struct timespec start_ts;
    struct timespec stop_ts;
    TimeMarker *markers;
    time_t timetick_curr;
    int marker_interval;
    int markers_curr_idx;
    int markers_max_idx;
} TimeInfo;

typedef struct {
    uint64_t sample_num;
    float currGain;
    uint8_t tuner;
    uint8_t gRdB;
    uint8_t lnaGRdB;
    uint8_t unused;
} GainChange;

typedef struct {
    struct timespec earliest_callback;
    struct timespec latest_callback;
    unsigned long long total_samples;
    unsigned int next_sample_num;
    unsigned int num_samples_min, num_samples_max;
    short imin, imax;
    short qmin, qmax;
    ResourceDescriptor *blocks_resource;
    ResourceDescriptor *samples_resource;
    TimeInfo *timeinfo;
} RXContext;

typedef struct {
    ResourceDescriptor *gain_changes_resource;
    unsigned long long *total_samples[2];
} EventContext;

typedef struct {
    RXContext *rx_contexts[2];
    EventContext *event_context;
} CallbackContext;

static void usage(const char* progname);
static int get_internal_decimation(const sdrplay_api_DeviceParamsT *device_params, double rspDuo_sample_freq);
static int generate_output_filename(const char *outfile_template, char *output_filename, int output_filename_max_size, double frequency_A, double frequency_B);
static int generate_gains_filename(const char *output_filename, char *gains_filename, int gains_filename_max_size);
static void rxA_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, void *cbContext);
static void rxB_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, void *cbContext);
static void event_callback(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tuner, sdrplay_api_EventParamsT *params, void *cbContext);
static void rx_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, RXContext *rxContext, char rx_id, StreamingStatus streaming_status_rx_callback);
static void update_timeinfo(TimeInfo *timeinfo, unsigned long long sample_num, StreamingStatus streaming_status_rx_callback);
static int write_linrad_header(int fd, double frequency_A, double frequency_B, double sample_rate);
static int write_wav_header(int fd, double frequency_A, double frequency_B, double sample_rate, float curr_gRdB_A, float curr_gRdB_B, int max_num_markers);
static int finalize_wav_file(int fd, unsigned long long data_size, unsigned long long sample_count, struct timespec *start_ts, struct timespec *stop_ts, int max_num_markers, TimeMarker *markers, int num_markers);

static unsigned int firstSampleNum = 0;
// perhaps we need a mutex around streaming_status
static StreamingStatus streaming_status = STREAMING_STATUS_STARTING;

// fv
static unsigned long long num_gain_changes[2] = {0L, 0L};

static void signal_handler(int signum) {
    UNUSED(signum);
    if (streaming_status == STREAMING_STATUS_RUNNING) {
        streaming_status = STREAMING_STATUS_TERMINATE;
    }
}

int main(int argc, char *argv[])
{
    const char *serial_number = NULL;
    double rspduo_sample_rate = 0.0;
    int decimation = 1;
    sdrplay_api_If_kHzT if_frequency = sdrplay_api_IF_Zero;
    sdrplay_api_Bw_MHzT if_bandwidth = sdrplay_api_BW_0_200;
    sdrplay_api_AgcControlT agc_A = sdrplay_api_AGC_DISABLE;
    sdrplay_api_AgcControlT agc_B = sdrplay_api_AGC_DISABLE;
    int gRdB_A = 40;
    int gRdB_B = 40;
    int LNAstate_A = 0;
    int LNAstate_B = 0;
    int DCenable = 1;
    int IQenable = 1;
    int dcCal = 3;
    int speedUp = 0;
    int trackTime = 1;
    int refreshRateTime = 2048;
    double frequency_A = 100e6;
    double frequency_B = 100e6;
    int streaming_time = 10;  /* streaming time in seconds */
    int marker_interval = 0;  /* store a marker tick every N seconds */
    char *outfile_template = NULL;
    OutputType output_type = OUTPUT_TYPE_RAW;
    int gains_file_enable = 0;
    int debug_enable = 0;
    int verbose = 0;

    int c;
    while ((c = getopt(argc, argv, "s:r:d:i:b:g:l:DIy:f:x:m:o:RLWGXvh")) != -1) {
        int n;
        switch (c) {
            case 's':
                serial_number = optarg;
                break;
            case 'r':
                if (sscanf(optarg, "%lg", &rspduo_sample_rate) != 1) {
                    fprintf(stderr, "invalid RSPduo sample rate: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'd':
                if (sscanf(optarg, "%d", &decimation) != 1) {
                    fprintf(stderr, "invalid decimation: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'i':
                if (sscanf(optarg, "%d", (int *)(&if_frequency)) != 1) {
                    fprintf(stderr, "invalid IF frequency: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'b':
                if (sscanf(optarg, "%d", (int *)(&if_bandwidth)) != 1) {
                    fprintf(stderr, "invalid IF bandwidth: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'g':
                if (strcmp(optarg, "AGC") == 0 || strcmp(optarg, "AGC,AGC") == 0) {
                    agc_A = sdrplay_api_AGC_50HZ;
                    agc_B = sdrplay_api_AGC_50HZ;
                } else if (sscanf(optarg, "AGC,%d", &gRdB_B) == 1) {
                    agc_A = sdrplay_api_AGC_50HZ;
                } else if (sscanf(optarg, "%d,AGC", &gRdB_A) == 1) {
                    agc_B = sdrplay_api_AGC_50HZ;
                } else {
                    n = sscanf(optarg, "%d,%d", &gRdB_A, &gRdB_B);
                    if (n < 1) {
                        fprintf(stderr, "invalid IF gain reduction: %s\n", optarg);
                        exit(1);
                    }
                    if (n == 1)
                        gRdB_B = gRdB_A;
                }
                break;
            case 'l':
                n = sscanf(optarg, "%d,%d", &LNAstate_A, &LNAstate_B);
                if (n < 1) {
                    fprintf(stderr, "invalid LNA state: %s\n", optarg);
                    exit(1);
                }
                if (n == 1)
                    LNAstate_B = LNAstate_A;
                break;
            case 'D':
                DCenable = 0;
                break;
            case 'I':
                IQenable = 0;
                break;
            case 'y':
                if (sscanf(optarg, "%d,%d,%d,%d", &dcCal, &speedUp, &trackTime, &refreshRateTime) != 4) {
                    fprintf(stderr, "invalid tuner DC offset compensation parameters: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'f':
                n = sscanf(optarg, "%lg,%lg", &frequency_A, &frequency_B);
                if (n < 1) {
                    fprintf(stderr, "invalid frequency: %s\n", optarg);
                    exit(1);
                }
                if (n == 1)
                    frequency_B = frequency_A;
                break;
            case 'x':
                if (sscanf(optarg, "%d", &streaming_time) != 1) {
                    fprintf(stderr, "invalid streaming time: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'm':
                if (sscanf(optarg, "%d", &marker_interval) != 1) {
                    fprintf(stderr, "invalid marker interval: %s\n", optarg);
                    exit(1);
                }
                break;
            case 'o':
                outfile_template = optarg;
                break;
            case 'R':
                output_type = OUTPUT_TYPE_RAW;
                break;
            case 'L':
                output_type = OUTPUT_TYPE_LINRAD;
                break;
            case 'W':
                output_type = OUTPUT_TYPE_WAV;
                break;
            case 'G':
                gains_file_enable = 1;
                break;
            case 'X':
                debug_enable = 1;
                break;
            case 'v':
                verbose = 1;
                break;

            // help
            case 'h':
                usage(argv[0]);
                exit(0);
            case '?':
            default:
                usage(argv[0]);
                exit(1);
        }
    }

    if (marker_interval > 0) {
        if (output_type != OUTPUT_TYPE_WAV) {
            fprintf(stderr, "time markers require WAV output types");
            exit(1);
        }
    }

    /* open SDRplay API and check version */
    sdrplay_api_ErrT err;
    err = sdrplay_api_Open();
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_Open() failed: %s\n", sdrplay_api_GetErrorString(err));
        exit(1);
    }
    float ver;
    err = sdrplay_api_ApiVersion(&ver);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_ApiVersion() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_Close();
        exit(1);
    }
    if (ver != SDRPLAY_API_VERSION) {
        fprintf(stderr, "SDRplay API version mismatch - expected=%.2f found=%.2f\n", SDRPLAY_API_VERSION, ver);
        sdrplay_api_Close();
        exit(1);
    }

    /* select device */
    err = sdrplay_api_LockDeviceApi();
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_LockDeviceApi() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_Close();
        exit(1);
    }
#ifdef SDRPLAY_MAX_DEVICES
#undef SDRPLAY_MAX_DEVICES
#endif
#define SDRPLAY_MAX_DEVICES 4
    unsigned int ndevices = SDRPLAY_MAX_DEVICES;
    sdrplay_api_DeviceT devices[SDRPLAY_MAX_DEVICES];
    err = sdrplay_api_GetDevices(devices, &ndevices, ndevices);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_GetDevices() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }
    int device_index = -1;
    for (unsigned int i = 0; i < ndevices; i++) {
        /* we are only interested in RSPduo's */
        if (devices[i].valid && devices[i].hwVer == SDRPLAY_RSPduo_ID) {
            if (serial_number == NULL || strcmp(devices[i].SerNo, serial_number) == 0) {
                device_index = i;
                break;
            }
        }
    }
    if (device_index == -1) {
        fprintf(stderr, "SDRplay RSPduo not found or not available\n");
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }
    sdrplay_api_DeviceT device = devices[device_index];

    /* select RSPduo dual tuner mode */
    if ((device.rspDuoMode & sdrplay_api_RspDuoMode_Dual_Tuner) != sdrplay_api_RspDuoMode_Dual_Tuner ||
        (device.tuner & sdrplay_api_Tuner_Both) != sdrplay_api_Tuner_Both) {
        fprintf(stderr, "SDRplay RSPduo dual tuner mode not available\n");
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }
    device.tuner = sdrplay_api_Tuner_Both;
    device.rspDuoMode = sdrplay_api_RspDuoMode_Dual_Tuner;
    device.rspDuoSampleFreq = rspduo_sample_rate;

    err = sdrplay_api_SelectDevice(&device);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_SelectDevice() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }

    err = sdrplay_api_UnlockDeviceApi();
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_UnlockDeviceApi() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }

    if (debug_enable) {
        err = sdrplay_api_DebugEnable(device.dev, sdrplay_api_DbgLvl_Verbose);
        if (err != sdrplay_api_Success) {
            fprintf(stderr, "sdrplay_api_DebugEnable() failed: %s\n", sdrplay_api_GetErrorString(err));
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
    }

    // select device settings
    sdrplay_api_DeviceParamsT *device_params;
    err = sdrplay_api_GetDeviceParams(device.dev, &device_params);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_GetDeviceParams() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }
    sdrplay_api_RxChannelParamsT *rx_channelA_params = device_params->rxChannelA ;
    sdrplay_api_RxChannelParamsT *rx_channelB_params = device_params->rxChannelB ;
    device_params->devParams->fsFreq.fsHz = rspduo_sample_rate;
    rx_channelA_params->ctrlParams.decimation.enable = decimation > 1;
    rx_channelA_params->ctrlParams.decimation.decimationFactor = decimation;
    rx_channelA_params->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
    rx_channelA_params->tunerParams.ifType = if_frequency;
    rx_channelA_params->tunerParams.bwType = if_bandwidth;
    rx_channelA_params->ctrlParams.agc.enable = agc_A;
    if (agc_A == sdrplay_api_AGC_DISABLE) {
        rx_channelA_params->tunerParams.gain.gRdB = gRdB_A;
    }
    rx_channelA_params->tunerParams.gain.LNAstate = LNAstate_A;
    rx_channelA_params->ctrlParams.dcOffset.DCenable = DCenable;
    rx_channelA_params->ctrlParams.dcOffset.IQenable = IQenable;
    rx_channelA_params->tunerParams.dcOffsetTuner.dcCal = dcCal;
    rx_channelA_params->tunerParams.dcOffsetTuner.speedUp = speedUp;
    rx_channelA_params->tunerParams.dcOffsetTuner.trackTime = trackTime;
    rx_channelA_params->tunerParams.dcOffsetTuner.refreshRateTime = refreshRateTime;
    rx_channelB_params->tunerParams.dcOffsetTuner.dcCal = dcCal;
    rx_channelB_params->tunerParams.dcOffsetTuner.speedUp = speedUp;
    rx_channelB_params->tunerParams.dcOffsetTuner.trackTime = trackTime;
    rx_channelB_params->tunerParams.dcOffsetTuner.refreshRateTime = refreshRateTime;
    rx_channelA_params->tunerParams.rfFreq.rfHz = frequency_A;

    int internal_decimation = get_internal_decimation(device_params, device.rspDuoSampleFreq);
    if (internal_decimation == -1) {
        fprintf(stderr, "invalid dual tuner settings - check sample rate, IF bandwidth, and IF frequency\n");
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }

    /* print settings */
    if (verbose) {
        fprintf(stderr, "SerNo=%s hwVer=%d tuner=0x%02x rspDuoMode=0x%02x rspDuoSampleFreq=%.0lf internalDecimation=%d\n", device.SerNo, device.hwVer, device.tuner, device.rspDuoMode, device.rspDuoSampleFreq, internal_decimation);
        fprintf(stderr, "RX A - LO=%.0lf BW=%d If=%d Dec=%d IFagc=%d IFgain=%d LNAgain=%d\n", rx_channelA_params->tunerParams.rfFreq.rfHz, rx_channelA_params->tunerParams.bwType, rx_channelA_params->tunerParams.ifType, rx_channelA_params->ctrlParams.decimation.decimationFactor, rx_channelA_params->ctrlParams.agc.enable, rx_channelA_params->tunerParams.gain.gRdB, rx_channelA_params->tunerParams.gain.LNAstate);
        fprintf(stderr, "RX A - DCenable=%d IQenable=%d dcCal=%d speedUp=%d trackTime=%d refreshRateTime=%d\n", (int)(rx_channelA_params->ctrlParams.dcOffset.DCenable), (int)(rx_channelA_params->ctrlParams.dcOffset.IQenable), (int)(rx_channelA_params->tunerParams.dcOffsetTuner.dcCal), (int)(rx_channelA_params->tunerParams.dcOffsetTuner.speedUp), rx_channelA_params->tunerParams.dcOffsetTuner.trackTime, rx_channelA_params->tunerParams.dcOffsetTuner.refreshRateTime);
        fprintf(stderr, "RX B - LO=%.0lf BW=%d If=%d Dec=%d IFagc=%d IFgain=%d LNAgain=%d\n", rx_channelB_params->tunerParams.rfFreq.rfHz, rx_channelB_params->tunerParams.bwType, rx_channelB_params->tunerParams.ifType, rx_channelB_params->ctrlParams.decimation.decimationFactor, rx_channelB_params->ctrlParams.agc.enable, rx_channelB_params->tunerParams.gain.gRdB, rx_channelB_params->tunerParams.gain.LNAstate);
        fprintf(stderr, "RX B - DCenable=%d IQenable=%d dcCal=%d speedUp=%d trackTime=%d refreshRateTime=%d\n", (int)(rx_channelB_params->ctrlParams.dcOffset.DCenable), (int)(rx_channelB_params->ctrlParams.dcOffset.IQenable), (int)(rx_channelB_params->tunerParams.dcOffsetTuner.dcCal), (int)(rx_channelB_params->tunerParams.dcOffsetTuner.speedUp), rx_channelB_params->tunerParams.dcOffsetTuner.trackTime, rx_channelB_params->tunerParams.dcOffsetTuner.refreshRateTime);
    }

    int errcode;
    pthread_mutex_t blocks_lock;
    errcode = pthread_mutex_init(&blocks_lock, NULL);
    if (errcode != 0) {
        fprintf(stderr, "pthread_mutex_init(blocks_lock) failed - errcode=%d\n" - errcode);
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }
    pthread_cond_t is_ready;
    errcode = pthread_cond_init(&is_ready, NULL);
    if (errcode != 0) {
        fprintf(stderr, "pthread_cond_init(is_ready) failed - errcode=%d\n" - errcode);
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }
    BlockDescriptor *blocks = (BlockDescriptor *)malloc(NUM_BLOCKS * sizeof(BlockDescriptor));
    ResourceDescriptor blocks_resource = {
        .lock = &blocks_lock,
        .resource = blocks,
        .read_index = 0,
        .write_index = 0,
        .size = NUM_BLOCKS,
        .nused = 0,
        .nready = 0,
        .is_ready = &is_ready,
    };
    pthread_mutex_t samples_lock;
    errcode = pthread_mutex_init(&samples_lock, NULL);
    if (errcode != 0) {
        fprintf(stderr, "pthread_mutex_init(bsamples_lock) failed - errcode=%d\n" - errcode);
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }
    short *insamples = (short *)malloc(NUM_SAMPLES * sizeof(short));
    ResourceDescriptor samples_resource = {
        .lock = &samples_lock,
        .resource = insamples,
        .read_index = 0,
        .write_index = 0,
        .size = NUM_SAMPLES,
        .nused = 0,
        .nready = 0,
        .is_ready = NULL,
    };

    int markers_max_idx = 0;
    TimeMarker *markers = NULL;
    if (marker_interval > 0) {
        /* add two extra slots to also store the start time marker and
         * because integer division truncates
        */
        markers_max_idx = streaming_time / marker_interval + 2;
        markers = (TimeMarker *)malloc(markers_max_idx * sizeof(TimeMarker));
        if (markers == NULL) {
            markers_max_idx = 0;
        }
    }
    TimeInfo timeinfo = {
        .start_ts = {0L, 0L},
        .stop_ts = {0L, 0L},
        .markers = markers,
        .timetick_curr = 0L,
        .marker_interval = marker_interval,
        .markers_curr_idx = 0,
        .markers_max_idx = markers_max_idx,
    };
    pthread_mutex_t gain_changes_lock;
    unsigned int gain_changes_size = 0;
    GainChange *gain_changes = NULL;
    if (gains_file_enable) {
        errcode = pthread_mutex_init(&gain_changes_lock, NULL);
        if (errcode != 0) {
            fprintf(stderr, "pthread_mutex_init(gain_changes_lock) failed - errcode=%d\n" - errcode);
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
        gain_changes_size = NUM_GAIN_CHANGES;
        gain_changes = (GainChange *)malloc(gain_changes_size * sizeof(GainChange));
    }
    ResourceDescriptor gain_changes_resource = {
        .lock = &gain_changes_lock,
        .resource = gain_changes,
        .read_index = 0,
        .write_index = 0,
        .size = gain_changes_size,
        .nused = 0,
        .nready = 0,
        .is_ready = NULL,
    };

    RXContext rx_contexts[2] = {
        { .earliest_callback = {0, 0},
          .latest_callback = {0, 0},
          .total_samples = 0,
          .next_sample_num = 0xffffffff,
          .num_samples_min = UINT_MAX,
          .num_samples_max = 0,
          .imin = SHRT_MAX,
          .imax = SHRT_MIN,
          .qmin = SHRT_MAX,
          .qmax = SHRT_MIN,
          .blocks_resource = &blocks_resource,
          .samples_resource = &samples_resource,
          .timeinfo = &timeinfo,
        },
        { .earliest_callback = {0, 0},
          .latest_callback = {0, 0},
          .total_samples = 0,
          .next_sample_num = 0xffffffff,
          .num_samples_min = UINT_MAX,
          .num_samples_max = 0,
          .imin = SHRT_MAX,
          .imax = SHRT_MIN,
          .qmin = SHRT_MAX,
          .qmax = SHRT_MIN,
          .blocks_resource = &blocks_resource,
          .samples_resource = &samples_resource,
          .timeinfo = NULL,
        },
    };

    EventContext event_context = {
        .gain_changes_resource = gains_file_enable ? &gain_changes_resource : NULL,
        .total_samples = {&rx_contexts[0].total_samples, &rx_contexts[1].total_samples},
    };

    CallbackContext callback_context = {
        .rx_contexts = {&rx_contexts[0], &rx_contexts[1]},
        .event_context = &event_context,
    };

    short *outsamples = (short *)malloc(NUM_SAMPLES * sizeof(short));

    sdrplay_api_CallbackFnsT callbackFns = {
        rxA_callback,
        rxB_callback,
        event_callback
    };

    err = sdrplay_api_Init(device.dev, &callbackFns, &callback_context);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_Init() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }
    // since sdrplay_api_Init() resets channelB settings to channelA values,
    // we need to update all the settings for channelB that are different
    sdrplay_api_ReasonForUpdateT reason_for_update = sdrplay_api_Update_None;
    if (agc_B != agc_A) {
        rx_channelB_params->ctrlParams.agc.enable = agc_B;
        reason_for_update |= sdrplay_api_Update_Ctrl_Agc;
    }
    if (agc_B == sdrplay_api_AGC_DISABLE) {
        if (gRdB_B != gRdB_A) {
            rx_channelB_params->tunerParams.gain.gRdB = gRdB_B;
            reason_for_update |= sdrplay_api_Update_Tuner_Gr;
        }
    }
    if (LNAstate_B != LNAstate_A) {
        rx_channelB_params->tunerParams.gain.LNAstate = LNAstate_B;
        reason_for_update |= sdrplay_api_Update_Tuner_Gr;
    }
    if (frequency_B != frequency_A) {
        rx_channelB_params->tunerParams.rfFreq.rfHz = frequency_B;
        reason_for_update |= sdrplay_api_Update_Tuner_Frf;
    }
    if (reason_for_update != sdrplay_api_Update_None) {
        err = sdrplay_api_Update(device.dev, sdrplay_api_Tuner_B, reason_for_update, sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            fprintf(stderr, "sdrplay_api_Update(0x%08x) failed: %s\n", reason_for_update, sdrplay_api_GetErrorString(err));
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
    }

    if (outfile_template == NULL) {
        switch (output_type) {
            case OUTPUT_TYPE_RAW:
                outfile_template = default_output_filename_raw;
                break;
            case OUTPUT_TYPE_LINRAD:
                outfile_template = default_output_filename_linrad;
                break;
            case OUTPUT_TYPE_WAV:
                outfile_template = default_output_filename_wav;
                break;
            default:
                fprintf(stderr, "error - unknown output type %d\n", output_type);
                sdrplay_api_ReleaseDevice(&device);
                sdrplay_api_Close();
                exit(1);
        }
    }
    char output_filename[PATH_MAX];
    errcode = generate_output_filename(outfile_template, output_filename, PATH_MAX, frequency_A, frequency_B);
    if (errcode != 0) {
        fprintf(stderr, "generate_output_filename(%s) failed\n", outfile_template);
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }

    int outfd = open(output_filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (outfd == -1) {
        fprintf(stderr, "open(%s) for writing failed: %s\n", output_filename, strerror(errno));
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        exit(1);
    }

    double output_sample_rate = rspduo_sample_rate / internal_decimation / decimation;

    if (output_type == OUTPUT_TYPE_LINRAD) {
        if (write_linrad_header(outfd, frequency_A, frequency_B, output_sample_rate) == -1) {
            fprintf(stderr, "write() Linrad header failed: %s\n", strerror(errno));
            close(outfd);
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
    } else if (output_type == OUTPUT_TYPE_WAV) {
        float curr_gRdB_A = rx_channelA_params->tunerParams.gain.gainVals.curr;
        float curr_gRdB_B = rx_channelB_params->tunerParams.gain.gainVals.curr;
        if (write_wav_header(outfd, frequency_A, frequency_B, output_sample_rate, curr_gRdB_A, curr_gRdB_B, markers_max_idx) == -1) {
            fprintf(stderr, "write() WAV header failed: %s\n", strerror(errno));
            close(outfd);
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
    }

    int gainsfd = -1;
    if (gains_file_enable) {
        char gains_filename[PATH_MAX] = "";
        errcode = generate_gains_filename(output_filename, gains_filename, PATH_MAX);
        if (errcode != 0) {
            fprintf(stderr, "generate_gains_filename(%s) failed\n", outfile_template);
            close(outfd);
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
        gainsfd = open(gains_filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (gainsfd == -1) {
            fprintf(stderr, "open(%s) for writing failed: %s\n", gains_filename, strerror(errno));
            close(outfd);
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGALRM, signal_handler);

    alarm(streaming_time);

    if (verbose) {
        fprintf(stderr, "streaming for %d seconds\n", streaming_time);
    }

    unsigned long long data_size = 0;
    unsigned long long output_samples = 0;
    streaming_status = STREAMING_STATUS_RUNNING;

    unsigned int next_sample_num = 0xffffffff;
    while (streaming_status == STREAMING_STATUS_RUNNING || streaming_status == STREAMING_STATUS_TERMINATE) {
        pthread_mutex_lock(blocks_resource.lock);
        while (blocks_resource.nready < 2) {
            pthread_cond_wait(blocks_resource.is_ready, blocks_resource.lock);
        }
        pthread_mutex_unlock(blocks_resource.lock);
        while (blocks_resource.nready >= 2) {
            pthread_mutex_lock(blocks_resource.lock);
            blocks_resource.nready -= 2;
            pthread_mutex_unlock(blocks_resource.lock);

            unsigned int read_index = blocks_resource.read_index;
            BlockDescriptor *blockA = (BlockDescriptor *) blocks_resource.resource + read_index;
            read_index = (read_index + 1) % blocks_resource.size;
            BlockDescriptor *blockB = (BlockDescriptor *) blocks_resource.resource + read_index;
            read_index = (read_index + 1) % blocks_resource.size;
            blocks_resource.read_index = read_index;

            if (!(blockA->rx_id == 'A' && blockB->rx_id == 'B')) {
                fprintf(stderr, "mismatch rx_id - %c %c\n", blockA->rx_id, blockB->rx_id);
                streaming_status = STREAMING_STATUS_FAILED;
                goto free_sample_resources;
            }
            if (!(blockA->first_sample_num == blockB->first_sample_num)) {
                fprintf(stderr, "mismatch first_sample_num - %u %u\n", blockA->first_sample_num, blockB->first_sample_num);
                streaming_status = STREAMING_STATUS_FAILED;
                goto free_sample_resources;
            }
            unsigned int first_sample_num = blockA->first_sample_num;
            if (!(blockA->num_samples == blockB->num_samples)) {
                fprintf(stderr, "mismatch num_samples - %u %u\n", blockA->num_samples, blockB->num_samples);
                streaming_status = STREAMING_STATUS_FAILED;
                goto free_sample_resources;
            }
            unsigned int num_samples = blockA->num_samples;
            if (num_samples == 0) {
                streaming_status = STREAMING_STATUS_DONE;
                goto free_sample_resources;
            }
            unsigned int dropped_samples;
            if (!(next_sample_num == 0xffffffff || blockA->first_sample_num == next_sample_num)) {
                if (next_sample_num < first_sample_num) {
                    dropped_samples = first_sample_num - next_sample_num;
                } else {
                    dropped_samples = UINT_MAX - (first_sample_num - next_sample_num) + 1;
                }
                fprintf(stderr, "dropped %u samples - next_sample_num=%d first_sample_num=%u\n", dropped_samples, next_sample_num, first_sample_num);
            }
            next_sample_num = first_sample_num + num_samples;

            /* rearrange samples in 'quadruples' (I_A, Q_A, I_B, Q_B) */
            /* I tuner A */
            int inoffset = blockA->samples_index;
            int outoffset = 0;
            for (unsigned int i = 0; i < num_samples; i++, inoffset++, outoffset += 4) {
                outsamples[outoffset] = insamples[inoffset];
            }
            /* Q tuner A */
            inoffset = blockA->samples_index + num_samples;
            outoffset = 1;
            for (unsigned int i = 0; i < num_samples; i++, inoffset++, outoffset += 4) {
                outsamples[outoffset] = insamples[inoffset];
            }
            /* I tuner B */
            inoffset = blockB->samples_index;
            outoffset = 2;
            for (unsigned int i = 0; i < num_samples; i++, inoffset++, outoffset += 4) {
                outsamples[outoffset] = insamples[inoffset];
            }
            /* Q tuner B */
            inoffset = blockB->samples_index + num_samples;
            outoffset = 3;
            for (unsigned int i = 0; i < num_samples; i++, inoffset++, outoffset += 4) {
                outsamples[outoffset] = insamples[inoffset];
            }

            uint8_t *outdata = (uint8_t *)outsamples;
            size_t bytes_left = num_samples * 4 * sizeof(short);
            while (bytes_left > 0) {
                ssize_t nwritten = write(outfd, outdata, bytes_left);
                if (nwritten == -1) {
                    fprintf(stderr, "write samples failed: %s\n", strerror(errno));
                    streaming_status = STREAMING_STATUS_FAILED;
                    goto free_sample_resources;
                }
                outdata += nwritten;
                bytes_left -= nwritten;
                data_size += nwritten;
            }
            output_samples += num_samples;

free_sample_resources:
            pthread_mutex_lock(blocks_resource.lock);
            blocks_resource.nused -= 2;
            pthread_mutex_unlock(blocks_resource.lock);
            unsigned int num_samples_A_and_B = blockA->num_samples + blockB->num_samples;
            if (num_samples_A_and_B > 0) {
                pthread_mutex_lock(samples_resource.lock);
                /* multiply by 2 to take into account that both I and Q */
                samples_resource.nused -= 2 * num_samples_A_and_B;
                pthread_mutex_unlock(samples_resource.lock);
            }

            if (!(streaming_status == STREAMING_STATUS_RUNNING || streaming_status == STREAMING_STATUS_TERMINATE)) {
                break;
            }
        }

        if (gainsfd != -1) {
            pthread_mutex_lock(gain_changes_resource.lock);
            unsigned int nready = gain_changes_resource.nready;
            gain_changes_resource.nready = 0;
            pthread_mutex_unlock(gain_changes_resource.lock);
            unsigned int read_index = gain_changes_resource.read_index;
            unsigned int size = gain_changes_resource.size;
            GainChange *gain_changes = (GainChange *) gain_changes_resource.resource;
            unsigned int items_left = nready;
            while (items_left > 0) {
                unsigned int nitems = items_left <= size - read_index ? items_left : size - read_index;
                uint8_t *gaindata = (uint8_t *)(gain_changes + read_index);
                size_t bytes_left = nitems * sizeof(GainChange);
                while (bytes_left > 0) {
                    ssize_t nwritten = write(gainsfd, gaindata, bytes_left);
                    if (nwritten == -1) {
                        fprintf(stderr, "write gains failed: %s\n", strerror(errno));
                        streaming_status = STREAMING_STATUS_FAILED;
                        goto free_gain_changes_resources;
                    }
                    gaindata += nwritten;
                    bytes_left -= nwritten;
                }
                read_index = (read_index + nitems) % size;
                items_left -= nitems;
            }
            gain_changes_resource.read_index = read_index;

free_gain_changes_resources:
            pthread_mutex_lock(gain_changes_resource.lock);
            gain_changes_resource.nused -= nready;
            pthread_mutex_unlock(gain_changes_resource.lock);
        }
    }

    err = sdrplay_api_Uninit(device.dev);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_Uninit() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_ReleaseDevice(&device);
        sdrplay_api_Close();
        close(outfd);
        if (gainsfd != -1) {
            close(gainsfd);
        }
        exit(1);
    }

    if (output_type == OUTPUT_TYPE_WAV) {
        if (finalize_wav_file(outfd, data_size, output_samples, &timeinfo.start_ts, &timeinfo.stop_ts, markers_max_idx, timeinfo.markers, timeinfo.markers_curr_idx) == -1) {
            fprintf(stderr, "finalize() WAV file failed: %s\n", strerror(errno));
            sdrplay_api_ReleaseDevice(&device);
            sdrplay_api_Close();
            exit(1);
        }
    }

#if 0
    for (int i = 0; i < 2; i++) {
        char rx_id = 'A' + i;
        RXContext *rx_context = callback_context->rx_contexts[i];
        /* estimate actual sample rate */
        double elapsed_sec = (rx_context->latest_callback.tv_sec - rx_context->earliest_callback.tv_sec) + 1e-9 * (rx_context->latest_callback.tv_nsec - rx_context->earliest_callback.tv_nsec);
        double actual_sample_rate = (double)(rx_context->total_samples) / elapsed_sec;
        int rounded_sample_rate_kHz = (int)(actual_sample_rate / 1000.0 + 0.5);
        if (verbose) {
            fprintf(stderr, "RX %c - total_samples=%llu actual_sample_rate=%.0lf rounded_sample_rate_kHz=%d\n", rx_id, rx_context->total_samples, actual_sample_rate, rounded_sample_rate_kHz);
        }
        sdrplay_api_RxChannelParamsT *rx_channel_params = NULL;
        switch (i) {
            case 0:
                rx_channel_params = device_params->rxChannelA;
                break;
            case 1:
                rx_channel_params = device_params->rxChannelB;
                break;
        }
        //fprintf(stdout, "%c\t%.0lf\t%d\t%d\t%d\t%lf\t%llu\t%.0lf\t%.3lf\t%d\t[%hd,%hd]\t[%hd,%hd]\t[%u,%u]\n", rx_id, device.rspDuoSampleFreq, rx_channel_params->tunerParams.bwType, rx_channel_params->tunerParams.ifType, rx_channel_params->ctrlParams.decimation.decimationFactor, elapsed_sec, rx_context->total_samples, actual_sample_rate, device.rspDuoSampleFreq / actual_sample_rate / rx_channel_params->ctrlParams.decimation.decimationFactor, internal_decimation, rx_context->imin, rx_context->imax, rx_context->qmin, rx_context->qmax, rx_context->num_samples_min, rx_context->num_samples_max);
#endif
    double elapsed_sec[2];
    double actual_sample_rate[2];
    for (int i = 0; i < 2; i++) {
        RXContext *rx_context = &rx_contexts[i];
        /* estimate actual sample rate */
        elapsed_sec[i] = (rx_context->latest_callback.tv_sec - rx_context->earliest_callback.tv_sec) + 1e-9 * (rx_context->latest_callback.tv_nsec - rx_context->earliest_callback.tv_nsec);
        actual_sample_rate[i] = (double)(rx_context->total_samples) / elapsed_sec[i];
    }
    fprintf(stderr, "total samples = %llu / %llu\n", rx_contexts[0].total_samples, rx_contexts[1].total_samples);
    fprintf(stderr, "elapsed time = %lf / %lf\n", elapsed_sec[0], elapsed_sec[1]);
    fprintf(stderr, "actual sample rate = %.0lf / %.0lf\n", actual_sample_rate[0], actual_sample_rate[1]);
    fprintf(stderr, "I samples range = [%hd,%hd] / [%hd,%hd]\n", rx_contexts[0].imin, rx_contexts[0].imax, rx_contexts[1].imin, rx_contexts[1].imax);
    fprintf(stderr, "Q samples range = [%hd,%hd] / [%hd,%hd]\n", rx_contexts[0].qmin, rx_contexts[0].qmax, rx_contexts[1].qmin, rx_contexts[1].qmax);
    fprintf(stderr, "samples per rx_callback range = [%u,%u] / [%u,%u]\n", rx_contexts[0].num_samples_min, rx_contexts[0].num_samples_max, rx_contexts[1].num_samples_min, rx_contexts[1].num_samples_max);
    fprintf(stderr, "output samples = %llu (x2)\n", output_samples);
    fprintf(stderr, "data size = %llu\n", data_size);
    // fv
    fprintf(stderr, "gain changes = %llu / %llu\n", num_gain_changes[0], num_gain_changes[1]);

    err = sdrplay_api_ReleaseDevice(&device);
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_ReleaseDevice() failed: %s\n", sdrplay_api_GetErrorString(err));
        sdrplay_api_Close();
        close(outfd);
        if (gainsfd != -1) {
            close(gainsfd);
        }
        exit(1);
    }

    /* all done: close SDRplay API */
    err = sdrplay_api_Close();
    if (err != sdrplay_api_Success) {
        fprintf(stderr, "sdrplay_api_Close() failed: %s\n", sdrplay_api_GetErrorString(err));
        close(outfd);
        if (gainsfd != -1) {
            close(gainsfd);
        }
        exit(1);
    }

    free(blocks);
    free(insamples);
    free(outsamples);
    free(markers);

    close(outfd);
    if (gainsfd != -1) {
        close(gainsfd);
    }

    return 0;
}

static void usage(const char* progname)
{
    fprintf(stderr, "usage: %s [options...]\n", progname);
    fprintf(stderr, "options:\n");
    fprintf(stderr, "    -s <RSPduo serial number>\n");
    fprintf(stderr, "    -r <RSPduo sample rate>\n");
    fprintf(stderr, "    -d <decimation>\n");
    fprintf(stderr, "    -i <IF frequency>\n");
    fprintf(stderr, "    -b <IF bandwidth>\n");
    fprintf(stderr, "    -g <IF gain reduction> (\"AGC\" to enable AGC)\n");
    fprintf(stderr, "    -l <LNA state>\n");
    fprintf(stderr, "    -D disable post tuner DC offset compensation (default: enabled)\n");
    fprintf(stderr, "    -I disable post tuner I/Q balance compensation (default: enabled)\n");
    fprintf(stderr, "    -y tuner DC offset compensation parameters <dcCal,speedUp,trackTime,refeshRateTime> (default: 3,0,1,2048)\n");
    fprintf(stderr, "    -f <center frequency>\n");
    fprintf(stderr, "    -x <streaming time (s)> (default: 10s)\n");
    fprintf(stderr, "    -m <time marker interval (s)> (default: 0 -> no time markers)\n");
    fprintf(stderr, "    -L output file in Linrad format\n");
    fprintf(stderr, "    -R output file in raw format (i.e. just the samples)\n");
    fprintf(stderr, "    -W output file in RIFF/RF64 format\n");
    fprintf(stderr, "    -G write gains file (default: disabled)\n");
    fprintf(stderr, "    -X enable SDRplay API debug log level (default: disabled)\n");
    fprintf(stderr, "    -v enable verbose mode (default: disabled)\n");
    fprintf(stderr, "    -h show usage\n");
}

static int get_internal_decimation(const sdrplay_api_DeviceParamsT *device_params, double rspDuo_sample_freq)
{
    if (device_params->devParams->fsFreq.fsHz != rspDuo_sample_freq)
        return -1;
    sdrplay_api_RxChannelParamsT *rx_channelA_params = device_params->rxChannelA ;
    double fs = rspDuo_sample_freq;
    sdrplay_api_Bw_MHzT bw = rx_channelA_params->tunerParams.bwType;
    sdrplay_api_If_kHzT ifreq = rx_channelA_params->tunerParams.ifType;

    if (fs == 8.192e6 && bw == sdrplay_api_BW_1_536 && ifreq == sdrplay_api_IF_2_048) {
        return 4;
    } else if (fs == 8e6 && bw == sdrplay_api_BW_1_536 && ifreq == sdrplay_api_IF_2_048) {
        return 4;
    } else if (fs == 8e6 && bw == sdrplay_api_BW_5_000 && ifreq == sdrplay_api_IF_2_048) {
        return 4;
    } else if (fs == 2e6 && bw == sdrplay_api_BW_0_200 && ifreq == sdrplay_api_IF_0_450) {
        return 4;
    } else if (fs == 2e6 && bw == sdrplay_api_BW_0_300 && ifreq == sdrplay_api_IF_0_450) {
        return 4;
    } else if (fs == 2e6 && bw == sdrplay_api_BW_0_600 && ifreq == sdrplay_api_IF_0_450) {
        return 2;
    } else if (fs == 6e6 && bw == sdrplay_api_BW_0_200 && ifreq == sdrplay_api_IF_1_620) {
        return 3;
    } else if (fs == 6e6 && bw == sdrplay_api_BW_0_300 && ifreq == sdrplay_api_IF_1_620) {
        return 3;
    } else if (fs == 6e6 && bw == sdrplay_api_BW_0_600 && ifreq == sdrplay_api_IF_1_620) {
        return 3;
    } else if (fs == 6e6 && bw == sdrplay_api_BW_1_536 && ifreq == sdrplay_api_IF_1_620) {
        return 3;
    }
    return -1;
}

static int generate_output_filename(const char *outfile_template, char *output_filename, int output_filename_max_size, double frequency_A, double frequency_B)
{
    const char freq_place_holder[] = "{FREQ}";
    int freq_place_holder_len = sizeof(freq_place_holder) - 1;
    const char freqhz_place_holder[] = "{FREQHZ}";
    int freqhz_place_holder_len = sizeof(freqhz_place_holder) - 1;
    const char freqkhz_place_holder[] = "{FREQKHZ}";
    int freqkhz_place_holder_len = sizeof(freqkhz_place_holder) - 1;
    const char timestamp_place_holder[] = "{TIMESTAMP}";
    int timestamp_place_holder_len = sizeof(timestamp_place_holder) - 1;
    const char tsiso8601_place_holder[] = "{TSISO8601}";
    int tsiso8601_place_holder_len = sizeof(tsiso8601_place_holder) - 1;

    time_t t = time(NULL);
    struct tm *tm = gmtime(&t);

    const char *src = outfile_template;
    char *dst = output_filename;
    char *dstlast = dst + output_filename_max_size - 1;
    char *p;
    while ((p = strchr(src, '{')) != NULL) {
        size_t sz = p - src;
        if (dst + sz > dstlast)
            return -1;
        strncpy(dst, src, sz);
        src = p;
        dst += sz;
        if (strncmp(src, freq_place_holder, freq_place_holder_len) == 0) {
            sz = dstlast - dst;
            size_t nf;
            if (frequency_A == frequency_B) {
                nf = snprintf(dst, sz, "%.0lf", frequency_A);
            } else {
                nf = snprintf(dst, sz, "%.0lf-%.0lf", frequency_A, frequency_B);
            }
            if (nf >= sz)
                return -1;
            src += freq_place_holder_len;
            dst += nf;
        } else if (strncmp(src, freqhz_place_holder, freqhz_place_holder_len) == 0) {
            sz = dstlast - dst;
            size_t nf;
            if (frequency_A == frequency_B) {
                nf = snprintf(dst, sz, "%.0lfHz", frequency_A);
            } else {
                nf = snprintf(dst, sz, "%.0lfHz-%.0lfHz", frequency_A, frequency_B);
            }
            if (nf >= sz)
                return -1;
            src += freqhz_place_holder_len;
            dst += nf;
        } else if (strncmp(src, freqkhz_place_holder, freqkhz_place_holder_len) == 0) {
            sz = dstlast - dst;
            size_t nf;
            if (frequency_A == frequency_B) {
                nf = snprintf(dst, sz, "%.0lfkHz", frequency_A / 1e3);
            } else {
                nf = snprintf(dst, sz, "%.0lfkHz-%.0lfkHz", frequency_A / 1e3, frequency_B / 1e3);
            }
            if (nf >= sz)
                return -1;
            src += freqkhz_place_holder_len;
            dst += nf;
        } else if (strncmp(src, timestamp_place_holder, timestamp_place_holder_len) == 0) {
            sz = dstlast - dst;
            size_t nts = strftime(dst, sz, "%Y%m%d_%H%M%SZ", tm);
            if (nts == 0)
                return -1;
            src += timestamp_place_holder_len;
            dst += nts;
        } else if (strncmp(src, tsiso8601_place_holder, tsiso8601_place_holder_len) == 0) {
            sz = dstlast - dst;
            size_t nts = strftime(dst, sz, "%Y%m%dT%H%M%SZ", tm);
            if (nts == 0)
                return -1;
            src += tsiso8601_place_holder_len;
            dst += nts;
        } else {
            if (dst == dstlast)
                return -1;
            *dst = *src;
            src++;
            dst++;
        }
    }
    size_t sz = strlen(src);
    if (dst + sz > dstlast)
        return -1;
    memcpy(dst, src, sz + 1);
    /* terminate string with a null */
    *(dst + sz) = '\0';
    return 0;
}

static int generate_gains_filename(const char *output_filename, char *gains_filename, int gains_filename_max_size) {
    const char gains_extension[] = ".gains";
    char *p = strrchr(output_filename, '.');
    size_t sz = p == NULL ? strlen(output_filename) : (size_t)(p - output_filename);
    if (sz + sizeof(gains_extension) > (size_t)gains_filename_max_size)
        return -1;
    memcpy(gains_filename, output_filename, sz);
    memcpy(gains_filename + sz, gains_extension, sizeof(gains_extension));
    return 0;
}

static void rxA_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, void *cbContext)
{
    StreamingStatus streaming_status_rx_callback = streaming_status;
    firstSampleNum = params->firstSampleNum;
    RXContext *rx_context = ((CallbackContext *)cbContext)->rx_contexts[0];
    update_timeinfo(rx_context->timeinfo, rx_context->total_samples, streaming_status_rx_callback);
    rx_callback(xi, xq, params, numSamples, reset, rx_context, 'A', streaming_status_rx_callback);
}

static void rxB_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, void *cbContext)
{
    StreamingStatus streaming_status_rx_callback = streaming_status;
    if (params->firstSampleNum != firstSampleNum) {
        fprintf(stderr, "firstSampleNum mismatch - RXA=%d RXB=%d\n", firstSampleNum, params->firstSampleNum);
    }
    RXContext *rx_context = ((CallbackContext *)cbContext)->rx_contexts[1];
    rx_callback(xi, xq, params, numSamples, reset, rx_context, 'B', streaming_status_rx_callback);
}

static void event_callback(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tuner, sdrplay_api_EventParamsT *params, void *cbContext)
{
    EventContext *eventContext = ((CallbackContext *)cbContext)->event_context;
    if (!(streaming_status == STREAMING_STATUS_RUNNING || streaming_status == STREAMING_STATUS_TERMINATE)) {
        return;
    }
    if (eventId == sdrplay_api_GainChange) {
        // fv
        num_gain_changes[tuner - 1]++;
        ResourceDescriptor *gain_changes_resource = eventContext->gain_changes_resource;
        if (gain_changes_resource != NULL) {
            pthread_mutex_lock(gain_changes_resource->lock);
            unsigned int gain_changes_write_index = gain_changes_resource->write_index;
            bool gain_changes_has_enough_space = gain_changes_resource->nused < gain_changes_resource->size;
            if (gain_changes_has_enough_space) {
                gain_changes_resource->write_index = (gain_changes_write_index + 1) % gain_changes_resource->size;
                gain_changes_resource->nused++;
            }
            pthread_mutex_unlock(gain_changes_resource->lock);
            if (!gain_changes_has_enough_space) {
                fprintf(stderr, "gain changes buffer full\n");
                streaming_status = STREAMING_STATUS_GAIN_CHANGES_BUFFER_FULL;
                return;
            }
            GainChange *gain_change = (GainChange *)gain_changes_resource->resource + gain_changes_write_index;
            sdrplay_api_GainCbParamT *gain_params = (sdrplay_api_GainCbParamT *)params;
            gain_change->sample_num = *eventContext->total_samples[tuner - 1];
            gain_change->currGain = gain_params->currGain;
            gain_change->tuner = tuner - 1;
            gain_change->gRdB = gain_params->gRdB;
            gain_change->lnaGRdB = gain_params->lnaGRdB;

            pthread_mutex_lock(gain_changes_resource->lock);
            gain_changes_resource->nready++;
            pthread_mutex_unlock(gain_changes_resource->lock);
        }
    }
    return;
}

static void rx_callback(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, RXContext *rxContext, char rx_id, StreamingStatus streaming_status_rx_callback)
{
    UNUSED(reset);

    if (streaming_status_rx_callback == STREAMING_STATUS_TERMINATE) {
        /* just return a block with num_samples set to 0
         * to signal the end of streaming
         */
        ResourceDescriptor *blocks_resource = rxContext->blocks_resource;
        pthread_mutex_lock(blocks_resource->lock);
        unsigned int blocks_write_index = blocks_resource->write_index;
        bool blocks_has_enough_space = blocks_resource->nused < blocks_resource->size;
        if (blocks_has_enough_space) {
            blocks_resource->write_index = (blocks_write_index + 1) % blocks_resource->size;
            blocks_resource->nused++;
        }
        pthread_mutex_unlock(blocks_resource->lock);
        if (!blocks_has_enough_space) {
            fprintf(stderr, "blocks buffer full\n");
            streaming_status = STREAMING_STATUS_BLOCKS_BUFFER_FULL;
            streaming_status_rx_callback = streaming_status;
            return;
        }
        BlockDescriptor *block = (BlockDescriptor *)blocks_resource->resource + blocks_write_index;
        block->first_sample_num = params->firstSampleNum;
        block->num_samples = 0;
        block->samples_index = 0;
        block->rx_id = rx_id;

        /* all done; let the writer thread know there's data ready */
        pthread_mutex_lock(blocks_resource->lock);
        blocks_resource->nready++;
        pthread_cond_signal(blocks_resource->is_ready);
        pthread_mutex_unlock(blocks_resource->lock);

        return;
    }

    /* only process samples when streaming status is set to RUNNING */
    if (streaming_status_rx_callback != STREAMING_STATUS_RUNNING) {
        return;
    }

    /* track callback timestamp */
    clock_gettime(CLOCK_REALTIME, &rxContext->latest_callback);
    if (rxContext->earliest_callback.tv_sec == 0) {
        rxContext->earliest_callback.tv_sec = rxContext->latest_callback.tv_sec;
        rxContext->earliest_callback.tv_nsec = rxContext->latest_callback.tv_nsec;
    }
    rxContext->total_samples += numSamples;

    /* check for dropped samples */
    unsigned int dropped_samples;
    if (rxContext->next_sample_num != 0xffffffff && params->firstSampleNum != rxContext->next_sample_num) {
        if (rxContext->next_sample_num < params->firstSampleNum) {
            dropped_samples = params->firstSampleNum - rxContext->next_sample_num;
        } else {
            dropped_samples = UINT_MAX - (params->firstSampleNum - rxContext->next_sample_num) + 1;
        }
        fprintf(stderr, "RX %c - dropped %d samples\n", rx_id, dropped_samples);
    }
    rxContext->next_sample_num = params->firstSampleNum + numSamples;

    rxContext->num_samples_min = rxContext->num_samples_min < numSamples ? rxContext->num_samples_min : numSamples;
    rxContext->num_samples_max = rxContext->num_samples_max > numSamples ? rxContext->num_samples_max : numSamples;

    short imin = SHRT_MAX;
    short imax = SHRT_MIN;
    short qmin = SHRT_MAX;
    short qmax = SHRT_MIN;
    for (unsigned int i = 0; i < numSamples; i++) {
        imin = imin < xi[i] ? imin : xi[i];
        imax = imax > xi[i] ? imax : xi[i];
    }
    for (unsigned int i = 0; i < numSamples; i++) {
        qmin = qmin < xq[i] ? qmin : xq[i];
        qmax = qmax > xq[i] ? qmax : xq[i];
    }
    rxContext->imin = rxContext->imin < imin ? rxContext->imin : imin;
    rxContext->imax = rxContext->imax > imax ? rxContext->imax : imax;
    rxContext->qmin = rxContext->qmin < qmin ? rxContext->qmin : qmin;
    rxContext->qmax = rxContext->qmax > qmax ? rxContext->qmax : qmax;

    ResourceDescriptor *samples_resource = rxContext->samples_resource;
    unsigned int samples_write_index = 0;

    unsigned int samples_space_required = 2 * numSamples;
    unsigned int samples_nused_max = samples_resource->size - samples_space_required;
    pthread_mutex_lock(samples_resource->lock);
    samples_write_index = samples_resource->write_index;
    if (samples_write_index > samples_nused_max) {
        samples_nused_max -= samples_resource->size - samples_write_index;
        samples_write_index = 0;
    }
    bool samples_has_enough_space = samples_resource->nused < samples_nused_max;
    if (samples_has_enough_space) {
        samples_resource->write_index = samples_write_index + samples_space_required;
        samples_resource->nused += samples_space_required;
    }
    pthread_mutex_unlock(samples_resource->lock);
    if (!samples_has_enough_space) {
        fprintf(stderr, "samples buffer full\n");
        streaming_status = STREAMING_STATUS_SAMPLES_BUFFER_FULL;
        streaming_status_rx_callback = streaming_status;
        return;
    }

    ResourceDescriptor *blocks_resource = rxContext->blocks_resource;
    pthread_mutex_lock(blocks_resource->lock);
    unsigned int blocks_write_index = blocks_resource->write_index;
    bool blocks_has_enough_space = blocks_resource->nused < blocks_resource->size;
    if (blocks_has_enough_space) {
        blocks_resource->write_index = (blocks_resource->write_index + 1) % blocks_resource->size;
        blocks_resource->nused++;
    }
    pthread_mutex_unlock(blocks_resource->lock);
    if (!blocks_has_enough_space) {
        fprintf(stderr, "blocks buffer full\n");
        streaming_status = STREAMING_STATUS_BLOCKS_BUFFER_FULL;
        streaming_status_rx_callback = streaming_status;
        return;
    }

    /* fill the samples buffer */
    short *samples = (short *)samples_resource->resource + samples_write_index;
    memcpy(samples, xi, numSamples * sizeof(short));
    samples += numSamples;
    memcpy(samples, xq, numSamples * sizeof(short));

    /* fill the block */
    BlockDescriptor *block = (BlockDescriptor *)blocks_resource->resource + blocks_write_index;
    block->first_sample_num = params->firstSampleNum;
    block->num_samples = numSamples;
    block->samples_index = samples_write_index;
    block->rx_id = rx_id;

    /* all done; let the writer thread know there's data ready */
    pthread_mutex_lock(blocks_resource->lock);
    blocks_resource->nready++;
    pthread_cond_signal(blocks_resource->is_ready);
    pthread_mutex_unlock(blocks_resource->lock);
}

static void update_timeinfo(TimeInfo *timeinfo, unsigned long long sample_num, StreamingStatus streaming_status_rx_callback) {
    if (streaming_status_rx_callback == STREAMING_STATUS_RUNNING) {
        if (timeinfo->start_ts.tv_sec == 0) {
            clock_gettime(CLOCK_REALTIME, &timeinfo->start_ts);
        }
        if (timeinfo->markers != NULL) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            time_t timetick_curr = ts.tv_sec / timeinfo->marker_interval;
            if (timetick_curr > timeinfo->timetick_curr) {
                if (timeinfo->markers_curr_idx < timeinfo->markers_max_idx) {
                    TimeMarker *tm = &timeinfo->markers[timeinfo->markers_curr_idx];
                    tm->ts.tv_sec = ts.tv_sec;
                    tm->ts.tv_nsec = ts.tv_nsec;
                    tm->sample_num = sample_num;
                    timeinfo->markers_curr_idx++;
                }
                timeinfo->timetick_curr = timetick_curr;
            }
        }
    } else if (streaming_status_rx_callback == STREAMING_STATUS_TERMINATE || streaming_status_rx_callback == STREAMING_STATUS_DONE) {
        if (timeinfo->stop_ts.tv_sec == 0) {
            clock_gettime(CLOCK_REALTIME, &timeinfo->stop_ts);
        }
    }
}

/* Linrad format */
#define LINRAD_REMEMBER_UNKNOWN -1
#define LINRAD_TWO_CHANNELS 2
#define LINRAD_IQ_DATA 4
#define LINRAD_DIGITAL_IQ 32

typedef struct __attribute__((packed)) {
    int remember_proprietary_chunk;
    double timestamp;
    double passband_center;
    int passband_direction;
    int rx_input_mode;
    int rx_rf_channels;
    int rx_ad_channels;
    int rx_ad_speed;
    unsigned char save_init_flag;
} LinradHeader;

static int write_linrad_header(int fd, double frequency_A, double frequency_B, double sample_rate) {
    if (frequency_A != frequency_B) {
        fprintf(stderr, "warning: Linrad head does not support different passband center frequencies for the two tuners\n");
    }
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    double timestamp = (double) ts.tv_sec + 1e-9 * ts.tv_nsec;

    LinradHeader linrad_header = {
        .remember_proprietary_chunk = LINRAD_REMEMBER_UNKNOWN,
        .timestamp = timestamp,
        .passband_center = frequency_A / 1e6,
        .passband_direction = 1,
        .rx_input_mode = LINRAD_TWO_CHANNELS | LINRAD_IQ_DATA | LINRAD_DIGITAL_IQ,
        .rx_rf_channels = 2,
        .rx_ad_channels = 4,
        .rx_ad_speed = sample_rate,
        .save_init_flag = 0
    };
    if (write(fd, &linrad_header, sizeof(linrad_header)) == -1) {
        return -1;
    }

    return 0;
}

/* RIFF/RF64/WAVE format
 *
 * References:
 *  - EBU - TECH 3306: https://tech.ebu.ch/docs/tech/tech3306v1_1.pdf
 *  - auxi chunk: SpectraVue User Guide: ver. 3.18 - https://www.moetronix.com/files/spectravue.pdf
 */
#define WAVE_FORMAT_PCM 1

struct RIFFChunk {
    char chunkId[4];
    uint32_t chunkSize;
    char riffType[4];
};

struct RF64Chunk {
    char chunkId[4];
    uint32_t chunkSize;
    char rf64Type[4];
};

struct JunkChunk {
    char chunkId[4];
    uint32_t chunkSize;
    char chunkData[28];
};

struct DataSize64Chunk {
    char chunkId[4];
    uint32_t chunkSize;
    uint32_t riffSizeLow;
    uint32_t riffSizeHigh;
    uint32_t dataSizeLow;
    uint32_t dataSizeHigh;
    uint32_t sampleCountLow;
    uint32_t sampleCountHigh;
    uint32_t tableLength;
};

struct FormatChunk {
    char chunkId[4];
    uint32_t chunkSize;
    uint16_t formatType;
    uint16_t channelCount;
    uint32_t sampleRate;
    uint32_t bytesPerSecond;
    uint16_t blockAlignment;
    uint16_t bitsPerSample;
};

struct SystemTime {
    uint16_t year;
    uint16_t month;
    uint16_t dayOfWeek;
    uint16_t day;
    uint16_t hour;
    uint16_t minute;
    uint16_t second;
    uint16_t milliseconds;
};

struct AuxiChunk {
    char chunkId[4];
    uint32_t chunkSize;
    struct SystemTime startTime;
    struct SystemTime stopTime;
    uint32_t centerFreq;
    uint32_t adFrequency;
    uint32_t ifFrequency;
    uint32_t bandwidth;
    uint32_t iqOffset;
    uint32_t dbOffset;
    uint32_t maxVal;
    uint32_t unused4;
    uint32_t unused5;
};

struct Guid {
    uint32_t data1;
    uint16_t data2;
    uint16_t data3;
    uint32_t data4;
    uint32_t data5;
};

struct MarkerEntry {
    uint32_t flags;
    uint32_t sampleOffsetLow;
    uint32_t sampleOffsetHigh;
    uint32_t byteOffsetLow;
    uint32_t byteOffsetHigh;
    uint32_t intraSmplOffsetHigh;
    uint32_t intraSmplOffsetLow;
    char labelText[256];
    uint32_t lablChunkIdentifier;
    struct Guid vendorAndProduct;
    uint32_t userData1;
    uint32_t userData2;
    uint32_t userData3;
    uint32_t userData4;
};

struct MarkerChunk {
    char chunkId[4];
    uint32_t chunkSize;
};

struct DataChunk {
    char chunkId[4];
    uint32_t chunkSize;
};

static int write_wav_header(int fd, double frequency_A, double frequency_B, double sample_rate, float curr_gRdB_A, float curr_gRdB_B, int max_num_markers) {
    if (frequency_A != frequency_B) {
        fprintf(stderr, "warning: WAV auxi chunk can store only one center frequency\n");
    }

    struct RIFFChunk riff_chunk = {
        .chunkId = {'R', 'I', 'F', 'F'},
        .chunkSize = 0,
        .riffType = {'W', 'A', 'V', 'E'}
    };

    struct JunkChunk junk_chunk = {
        .chunkId = {'J', 'U', 'N', 'K'},
        .chunkSize = sizeof(struct JunkChunk) - sizeof(char[4]) - sizeof(uint32_t)
    };

    struct FormatChunk fmt_chunk = {
        .chunkId = {'f', 'm', 't', ' '},
        .chunkSize = sizeof(struct FormatChunk) - sizeof(char[4]) - sizeof(uint32_t),
        .formatType = WAVE_FORMAT_PCM,
        .channelCount = 4,
        .sampleRate = sample_rate,
        .bytesPerSecond = sample_rate * 2 * 2 * sizeof(short),
        .blockAlignment = 2 * 2 * sizeof(short),
        .bitsPerSample = 16
    };

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    struct AuxiChunk auxi_chunk = {
        .chunkId = {'a', 'u', 'x',  'i'},
        .chunkSize = sizeof(struct AuxiChunk) - sizeof(char[4]) - sizeof(uint32_t),
        .startTime = {0, 0, 0, 0, 0, 0, 0, 0},   /* to be filled at the end */
        .stopTime = {0, 0, 0, 0, 0, 0, 0, 0},    /* to be filled at the end */
        .centerFreq = (uint32_t) frequency_A,
        .adFrequency = 0,
        .ifFrequency = 0,
        .bandwidth = 0,
        .iqOffset = 0,
        .dbOffset = 0xe49b72a9,    /* same value as in SDRuno */
        .maxVal = 0,
        .unused4 = (uint32_t) (curr_gRdB_A * 1000 + 0.5),
        .unused5 = (uint32_t) (curr_gRdB_B * 1000 + 0.5)
    };

    struct DataChunk data_chunk = {
        .chunkId = {'d', 'a', 't', 'a'},
        .chunkSize = 0
    };

    if (write(fd, &riff_chunk, sizeof(riff_chunk)) == -1) {
        return -1;
    }
    if (write(fd, &junk_chunk, sizeof(junk_chunk)) == -1) {
        return -1;
    }
    if (write(fd, &fmt_chunk, sizeof(fmt_chunk)) == -1) {
        return -1;
    }
    if (write(fd, &auxi_chunk, sizeof(auxi_chunk)) == -1) {
        return -1;
    }

    if (max_num_markers > 0) {
        int chunk_size = max_num_markers * sizeof(struct MarkerEntry);
        struct MarkerChunk marker_chunk = {
            .chunkId = {'r', '6', '4', 'm'},
            .chunkSize = chunk_size
        };
        if (write(fd, &marker_chunk, sizeof(marker_chunk)) == -1) {
            return -1;
        }
        struct MarkerEntry empty_marker;
        memset(&empty_marker, 0, sizeof(empty_marker));
        for (int i = 0; i < max_num_markers; i++) {
            if (write(fd, &empty_marker, sizeof(empty_marker)) == -1) {
                return -1;
            }
        }
    }

    if (write(fd, &data_chunk, sizeof(data_chunk)) == -1) {
        return -1;
    }

    return 0;
}

static int finalize_riff_file(int fd, unsigned long long data_size, struct timespec *start_ts, struct timespec *stop_ts);
static int finalize_rf64_file(int fd, unsigned long long data_size, unsigned long long sample_count, struct timespec *start_ts, struct timespec *stop_ts, int max_num_markers, TimeMarker *markers, int num_markers);

static int finalize_wav_file(int fd, unsigned long long data_size, unsigned long long sample_count, struct timespec *start_ts, struct timespec *stop_ts, int max_num_markers, TimeMarker *markers, int num_markers) {
    unsigned long long riff_size = sizeof(char[4]) +
                                   sizeof(struct JunkChunk) +
                                   sizeof(struct FormatChunk) +
                                   sizeof(struct AuxiChunk) +
                                   sizeof(struct DataChunk) +
                                   data_size;
    if (riff_size < UINT_MAX && max_num_markers == 0) {
        return finalize_riff_file(fd, data_size, start_ts, stop_ts);
    } else {
        return finalize_rf64_file(fd, data_size, sample_count, start_ts, stop_ts, max_num_markers, markers, num_markers);
    }
}

static int finalize_riff_file(int fd, unsigned long long data_size, struct timespec *start_ts, struct timespec *stop_ts) {
    // 1. fix data chunk size
    off_t offset = sizeof(struct RIFFChunk) +
                   sizeof(struct JunkChunk) +
                   sizeof(struct FormatChunk) +
                   sizeof(struct AuxiChunk) +
                   sizeof(char[4]);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(data chunk size) failed: %s\n", strerror(errno));
        return -1;
    }
    uint32_t data_size_int = (uint32_t)data_size;
    if (write(fd, &data_size_int, sizeof(data_size_int)) == -1) {
        return -1;
    }

    // 2. fix RIFF chunk size
    offset = sizeof(char[4]);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(RIFF chunk size) failed: %s\n", strerror(errno));
        return -1;
    }
    uint32_t riff_size = (uint32_t)(sizeof(char[4]) +
                                    sizeof(struct JunkChunk) +
                                    sizeof(struct FormatChunk) +
                                    sizeof(struct AuxiChunk) +
                                    sizeof(struct DataChunk) +
                                    data_size);
    if (write(fd, &riff_size, sizeof(riff_size)) == -1) {
        return -1;
    }

    // 3. set startTime and stopTime in auxi chunk
    offset = sizeof(struct RIFFChunk) +
             sizeof(struct JunkChunk) +
             sizeof(struct FormatChunk) +
             sizeof(char[4]) + sizeof(uint32_t);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(auxi chunk startTime) failed: %s\n", strerror(errno));
        return -1;
    }
    struct tm *startTime_tm = gmtime(&start_ts->tv_sec);
    struct SystemTime startTime = {
        .year = 1900 + startTime_tm->tm_year,
        .month = startTime_tm->tm_mon + 1,
        .dayOfWeek = startTime_tm->tm_wday,
        .day = startTime_tm->tm_mday,
        .hour = startTime_tm->tm_hour,
        .minute = startTime_tm->tm_min,
        .second = startTime_tm->tm_sec,
        .milliseconds = start_ts->tv_nsec * 1e-6 + 0.5
    };
    if (write(fd, &startTime, sizeof(startTime)) == -1) {
        return -1;
    }
    struct tm *stopTime_tm = gmtime(&stop_ts->tv_sec);
    struct SystemTime stopTime = {
        .year = 1900 + stopTime_tm->tm_year,
        .month = stopTime_tm->tm_mon + 1,
        .dayOfWeek = stopTime_tm->tm_wday,
        .day = stopTime_tm->tm_mday,
        .hour = stopTime_tm->tm_hour,
        .minute = stopTime_tm->tm_min,
        .second = stopTime_tm->tm_sec,
        .milliseconds = stop_ts->tv_nsec * 1e-6 + 0.5
    };
    if (write(fd, &stopTime, sizeof(stopTime)) == -1) {
        return -1;
    }

    return 0;
}

static int finalize_rf64_file(int fd, unsigned long long data_size, unsigned long long sample_count, struct timespec *start_ts, struct timespec *stop_ts, int max_num_markers, TimeMarker *markers, int num_markers) {
    unsigned long long markers_size = 0L;
    if (max_num_markers > 0) {
        markers_size = sizeof(struct MarkerChunk) +
                       max_num_markers * sizeof(struct MarkerEntry);
    }

    /* 1. Replace the chunkID ‘JUNK’ with ‘ds64’ chunk. */
    /* 2. Insert the RIFF size, 'data' chunk size and sample count in the 'ds64' chunk */
    unsigned long long riff_size = sizeof(char[4]) +
                                   sizeof(struct JunkChunk) +
                                   sizeof(struct FormatChunk) +
                                   sizeof(struct AuxiChunk) +
                                   markers_size +
                                   sizeof(struct DataChunk) +
                                   data_size;
    struct DataSize64Chunk ds64_chunk = {
        .chunkId = {'d', 's', '6', '4'},
        .chunkSize = sizeof(struct DataSize64Chunk) - sizeof(char[4]) - sizeof(uint32_t),
        .riffSizeLow = riff_size & 0xffffffff,
        .riffSizeHigh = riff_size >> 32,
        .dataSizeLow = data_size & 0xffffffff,
        .dataSizeHigh = data_size >> 32,
        .sampleCountLow = sample_count & 0xffffffff,
        .sampleCountHigh = sample_count >> 32,
        .tableLength = 0
    };
    off_t offset = sizeof(struct RIFFChunk);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(ds64 chunk) failed: %s\n", strerror(errno));
        return -1;
    }
    if (write(fd, &ds64_chunk, sizeof(ds64_chunk)) == -1) {
        return -1;
    }

    /* 3. Set RIFF size, 'data' chunk size and sample count in the 32 bit fields to -1 = FFFFFFFF hex */
    uint32_t negative1 = 0xffffffff;
    offset = sizeof(char[4]);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(RIFF chunk size) failed: %s\n", strerror(errno));
        return -1;
    }
    if (write(fd, &negative1, sizeof(negative1)) == -1) {
        return -1;
    }
    offset = sizeof(struct RIFFChunk) +
             sizeof(struct JunkChunk) +
             sizeof(struct FormatChunk) +
             sizeof(struct AuxiChunk) +
             markers_size +
             sizeof(char[4]);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(data chunk size) failed: %s\n", strerror(errno));
        return -1;
    }
    if (write(fd, &negative1, sizeof(negative1)) == -1) {
        return -1;
    }

    /* 4. Replace the ID ‘RIFF’ with ‘RF64’ in the first four bytes of the file */
    char rf64[4] = {'R', 'F', '6', '4'};
    offset = 0L;
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(RF64 ID) failed: %s\n", strerror(errno));
        return -1;
    }
    if (write(fd, &rf64, sizeof(rf64)) == -1) {
        return -1;
    }

    // 5. set startTime and stopTime in auxi chunk
    offset = sizeof(struct RIFFChunk) +
             sizeof(struct JunkChunk) +
             sizeof(struct FormatChunk) +
             sizeof(char[4]) + sizeof(uint32_t);
    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "lseek(auxi chunk startTime) failed: %s\n", strerror(errno));
        return -1;
    }
    struct tm *startTime_tm = gmtime(&start_ts->tv_sec);
    struct SystemTime startTime = {
        .year = 1900 + startTime_tm->tm_year,
        .month = startTime_tm->tm_mon + 1,
        .dayOfWeek = startTime_tm->tm_wday,
        .day = startTime_tm->tm_mday,
        .hour = startTime_tm->tm_hour,
        .minute = startTime_tm->tm_min,
        .second = startTime_tm->tm_sec,
        .milliseconds = start_ts->tv_nsec * 1e-6 + 0.5
    };
    if (write(fd, &startTime, sizeof(startTime)) == -1) {
        return -1;
    }
    struct tm *stopTime_tm = gmtime(&stop_ts->tv_sec);
    struct SystemTime stopTime = {
        .year = 1900 + stopTime_tm->tm_year,
        .month = stopTime_tm->tm_mon + 1,
        .dayOfWeek = stopTime_tm->tm_wday,
        .day = stopTime_tm->tm_mday,
        .hour = stopTime_tm->tm_hour,
        .minute = stopTime_tm->tm_min,
        .second = stopTime_tm->tm_sec,
        .milliseconds = stop_ts->tv_nsec * 1e-6 + 0.5
    };
    if (write(fd, &stopTime, sizeof(stopTime)) == -1) {
        return -1;
    }

    /* write the markers */
    if (max_num_markers > 0) {
        offset = sizeof(struct RIFFChunk) +
                 sizeof(struct JunkChunk) +
                 sizeof(struct FormatChunk) +
                 sizeof(struct AuxiChunk) +
                 sizeof(struct MarkerChunk);
        if (lseek(fd, offset, SEEK_SET) == -1) {
            fprintf(stderr, "lseek(marker chunk entries) failed: %s\n", strerror(errno));
            return -1;
        }
        for (int i = 0; i < num_markers; i++) {
            TimeMarker *marker = &markers[i];
            struct MarkerEntry marker_entry = {
                .flags = 0x1,
                .sampleOffsetLow = marker->sample_num & 0xffffffff,
                .sampleOffsetHigh = marker->sample_num >> 32,
                .byteOffsetLow = 0,
                .byteOffsetHigh = 0,
                .intraSmplOffsetHigh = 0,
                .intraSmplOffsetLow = 0,
                .labelText = {0},
                .lablChunkIdentifier = 0,
                .vendorAndProduct = {0, 0, 0, 0, 0},
                .userData1 = 0,
                .userData2 = 0,
                .userData3 = 0,
                .userData4 = 0
            };
            struct tm *tm = gmtime(&marker->ts.tv_sec);
            char buffer[20];
            /* build ISO8601/RFC3339 timestamp */
            strftime(buffer, sizeof(buffer), "%FT%T", tm);
            snprintf(marker_entry.labelText, 256, "%s.%09luZ", buffer, marker->ts.tv_nsec);
            if (write(fd, &marker_entry, sizeof(marker_entry)) == -1) {
                return -1;
            }
        }
    }

    return 0;
}
