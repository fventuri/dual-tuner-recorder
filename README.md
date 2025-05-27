# SDRplay RSPduo dual tuner recorder utility

`dual-tuner-recorder` is a command line utility to record to disk the stream of I/Q samples from both tuners of the SDRplay RSPduo.

The output formats are:
  - raw (i.e. just the I/Q values as an interlaced stream of 16 bit shorts, i.e. I tuner A, Q tuner A, I tuner B, Q tuner B)
  - Linrad, compatible with the Linrad SDR program
  - WAV, which generates a WAV file in RIFF/RF64 format with four PCM channels (PCM is another way of calling a stream of 16 bit shorts, as described in the raw format)

Most of the RSPduo parameters available though the API can be set through command line arguments; these include center frequency, sample rate, decimation, IF frequency, IF bandwidth, gains, and several others (see below).

Settings that should be different between the two tuners can be assigned by separating the values with a comma. For instance:
  - `-l 3` sets the LNA state for both tuners at 3
  - `-l 3,5` sets the LNA state for tuner A at 3 and for tuner B at 5

Some settings, like decimation, accept only one value, since the two tuners should always use the same decimation factor (if not, the two output streams would have a different sample rate).

The files in WAV format contain a special chunk called 'auxi' that follows the specification from SpectraVue users guide.
The two values 'unused4' and 'unused5' in the 'auxi' chunk contain the initial gains in 1/1000 of a dB (a 'milli dB'); in other words a value of 57539 means a gain of 57.539 dB. These gains shouldn't change during a recording unless AGC is enabled (see 'gains file' below).

The WAV files in RF64 format can also optionally contain time markers stored in a 'r64m' chunk, following the format described EBU technical specification 3306 v1.1 (July 2009). The command line argument '-m' enables these time markers at specified intervals; for instance '-m 60' creates a marker at the beginning of each minute; '-m 900' creates markers at 0, 15, 30, and 45 minutes past the hour. The labels for these time markers are the timestamps in ISO8601/RFC3339 format (including nanoseconds; for instance '2025-04-27T21:56:45.123456789Z')

The utility can also write a secondary file with the gain changes; anytime one of the gain values changes (because of AGC), a new entry is added to this file with:
   - sample number (uint64_t)
   - current gain (float)
   - tuner (uint8_t) - 0 for tuner A, 1 for tuner B
   - gRdB (uint8_t)
   - LNA gRdB  (uint8_t)
   - padding (1 byte)

Each entry is 16 bytes long; it can be read using Python struct module with a format '@Qf3Bx'; the Python script `show_gains.py`; it can be read using Python struct module with a format '@Qf3Bx'; the Python script `show_gains.py` shows how it can be done.

## Important note about sample rates, IF frequency, and IF bandwidth when operating the RSPduo in dual tuner mode

To operate in dual tuner mode, the RSPduo hardware/software requires one of a specific set of combinations of RSPduo sample rate, IF frequency, and IF bandwidth. The full list is shown in the table below. When one of these modes is selected, the RSPduo hw/sw will apply an 'internal decimation' (by 3 or 4) that will divide the RSPduo sample rate. The output sample rate (i.e. the sample rate of the I/Q samples that this utility will write to file) is therefore:

    fs_out = RSPduo sample rate / internal decimation / user selected decimation

For instance with RSPduo sample rate=6MHz, IF frequency=1620kHz, IF bandwidth=1536kHz, and decimation=1, the output sample rate will be 2Msps (6M / 3 / 1).

The `-r` argument expects the value of the RSPduo sample rate, i.e. the first column in the table below, not the output sample rate.

| RSPduo sample rate | IF frequency | IF bandwidth | Int decimation |  fs_out |
| :----------------: | :----------: | :----------: | :------------: | :-----: |
|      8192000       |    2048      |    1536      |       4        | 2048000 |
|      8000000       |    2048      |    1536      |       4        | 2000000 |
|      8000000       |    2048      |    5000      |       4        | 2000000 |
|      2000000       |     450      |     200      |       4        |  500000 |
|      2000000       |     450      |     300      |       4        |  500000 |
|      2000000       |     450      |     600      |       4        |  500000 |
|      6000000       |    1620      |     200      |       3        | 2000000 |
|      6000000       |    1620      |     300      |       3        | 2000000 |
|      6000000       |    1620      |     600      |       3        | 2000000 |
|      6000000       |    1620      |    1536      |       3        | 2000000 |


## Output filename

The name of the output file containing the samples can be specified with the `-o` option. The output file name template supports the following 'variables', which will be replaced by their values when generating the actual file name:
  - {FREQ} will be replaced by the center frequency in Hz
  - {FREQHZ} will be replaced by the center frequency in Hz followed by 'Hz'
  - {FREQKHZ} will be replaced by the center frequency in kHz followed by 'kHz'
  - {TIMESTAMP} will be replaced by the timestamp at the beginning of the recording in 'YYYYMMDD_HHMMSS' format
  - {TSIS8601} will be replaced by the timestamp at the beginning of the recording in IS08601 format

For instance, an output filename specified as 'RSPduo_dual_tuner_{TIMESTAMP}_{FREQHZ}.wav' could generate an output file with this actual name: RSPduo_dual_tuner_20250429_013247_162550000Hz.wav


## Build instructions

```
mkdir build
cd build
cmake ..
make (or ninja)
```


## Notes for Windows users

- the utility can be built with MinGW/MSYS2 (https://www.msys2.org/); MinGW can also be used to cross compile it on Linux to create a Windows executable
- the commands to build it are the same commands in the section 'Build instructions' above (in a msys2 terminal)
- alternatively you can download the precompiled binary for Windows in the 'Releases' page (https://github.com/fventuri/dual-tuner-recorder/releases)
- in this case you'll probably need to 'unblock' it before you run it for the first time
- you will also need to have the SDRplay API DLL (`sdrplay_api.dll`) in a location where `dual_tuner_recorder.exe` can find it, either by copying it to the same folder where you run `dual_tuner_recorder.exe`, or by adding to the `PATH` environment variable the path of the folder where `sdrplay_api.dll` is located


## Usage

These are the command line options for `dual_tuner_recorder`:

    -s <RSPduo serial number>
    -r <RSPduo sample rate>
    -d <decimation>
    -i <IF frequency>
    -b <IF bandwidth>
    -g <IF gain reduction> ("AGC" to enable AGC)
    -l <LNA state>
    -D disable post tuner DC offset compensation (default: enabled)
    -I disable post tuner I/Q balance compensation (default: enabled)
    -y tuner DC offset compensation parameters <dcCal,speedUp,trackTime,refeshRateTime> (default: 3,0,1,2048)
    -f <center frequency>
    -x <streaming time (s)> (default: 10s)
    -m <time marker interval (s)> (default: 0 -> no time markers)
    -o <output filename template>
    -z <zero sample gaps if smaller than size> (default: 100000)
    -j <blocks buffer capacity> (in number of blocks)
    -k <samples buffer capacity> (in number of samples)
    -L output file in Linrad format
    -R output file in raw format (i.e. just the samples)
    -W output file in RIFF/RF64 format
    -G write gains file (default: disabled)
    -X enable SDRplay API debug log level (default: disabled)
    -v enable verbose mode (default: disabled)


## Examples

- record local NOAA weather radio on 162.55MHz for 10 seconds using an RSPduo sample rate of 6MHz and IF=1620kHz (and LNA state set to 3):
```
dual_tuner_recorder -r 6000000 -i 1620 -b 1536 -l 3 -f 162550000
```
The output file will have a sample rate of 2Msps.

- same as above, but with a sample rate of 8MHz (lower sample resolution; 12 bits instead of 14 bits)
```
dual_tuner_recorder -r 8000000 -i 2048 -b 1536 -l 3 -f 162550000
```
In this case too, the output file will have a sample rate of 2Msps.

 - same as the first example, but with IF AGC enabled for both tuners and streaming for 5 minutes:
```
dual_tuner_recorder -r 6000000 -i 1620 -b 1536 -g AGC -l 3 -f 162550000 -x 300
```

 - same as the first example, but streaming for one hour and writing to a WAV file with time markers at 0, 5, 10, 15, ..., 50, 55 minutes after the hour:
```
dual_tuner_recorder -r 6000000 -i 1620 -b 1536 -l 3 -f 162550000 -x 3600 -W -m 300
```

 - same as the the example above, but with AGC enabled and storing the gain values in a `.gains` file:
```
dual_tuner_recorder -r 6000000 -i 1620 -b 1536 -g AGC -l 3 -f 162550000 -x 3600 -W -m 300 -G
```


## References

  - SDRplay API Specification (version 3.15): https://www.sdrplay.com/docs/SDRplay_API_Specification_v3.15.pdf
  - SDRplay RSPduo Technical Information: https://www.sdrplay.com/wp-content/uploads/2018/06/RSPDuo-Technical-Information-R1P1.pdf
  - Linrad: https://www.sm5bsz.com/linuxdsp/linrad.htm
  - EBU Technical Specification 3306 - MBWF/RF64: An extended File Format for Audio - version 1.1, July 2009: https://tech.ebu.ch/docs/tech/tech3306v1_1.pdf
  - SpectraVue User Guide version 3.18, Jan 27, 2013: https://www.moetronix.com/files/spectravue.pdf


## Copyright

(C) 2025 Franco Venturi - Licensed under the GNU GPL V3 (see [LICENSE](LICENSE))
