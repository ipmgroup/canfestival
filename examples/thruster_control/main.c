#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <getopt.h>
#include <unistd.h>

#include "canfestival.h"
//#include "handlers.h"
#include "motor.h"

#define DIRECT_MAPPING 1 // While resolving the options, determines if the default number for the position in the array is given (1), or the option is searched for (0).

static const char *usage = "USAGE: %s [OPTIONS]\n"
"   The program uses SDO protocol only and doesn't rely on PDO mapping.\n"
"    [OPTIONS]\n"
"    Configuration:\n"
"    --help, -h          Display this help.\n"
"    --verbose, -v       Display additional information.\n"
"    --masterid=N        Make the program identify with master id=N.\n"
"    --node=N            Select node with id=N.\n"
"    --baud=B            Set baud rate to B (ex. 500K).\n"
"    --channel=C         Set channel to C (ex. can0).\n"
"    --config            Display current configuration.\n"
"    --default           Display default configuration.\n"
"    --ct                Delete temporary file.\n"
"    --cc                Delete configuration file.\n"
"    --clean             Delete all associated files (temp, conf).\n"
"    --polling=P         Sets polling period for monitoring to at least P ms.\n"
//"    --tag               Enables tags. Used to simplify communication with other programs.\n"
"    \n"
"    If the motor control options have arguments, those arguments will be written to the corresponding registers, otherwise they will be read.\n"
"    Motor controls:\n"
"    --quickstart, -q                   Turns on everything you need (controlword = 0x%x).\n"
"    --monitor=N, -mN                   Monitors a given parameter N (ex. speed).\n"
"    --statusword                       Displays the status word.\n"
"    --system=N                         Turns the system on/off. N=<0|1>\n"
"    --motor=N                          Turns the motor on/off. N=<0|1>\n"
"    --field_weakening=N                Turns field weakening on/off. N=<0|1>\n"
"    --force_angle=N                    Turns force angle on/off. N=<0|1>\n"
"    --resistance_recalculation=N       Turns resistance recalculation on/off. N=<0|1>\n"
"    --power_wrap=N                     Turns power wrap on/off. N=<0|1>\n"
"    --speed=N, -sN                     Reads current speed or sets target speed to N.\n"
"    --acceleration=N, -aN              Reads current acceleration or sets it to N.\n"
"    --voltage                          Reads the voltage.\n"
"    --current                          Reads the current.\n"
"    --temperature                      Reads the temperature.\n"
"    --humidity                         Reads the humidity.\n"
"    --fault=N, -fN                     Reads the fault bit or sets it to N. N=<0|1>\n"
"    --status1                          Reads the status register 1.\n"
"    --status2                          Reads the status register 2.\n"
"    --status3                          Reads the status register 3.\n"
"    --status4                          Reads the status register 4.\n"
"    \n"
"    Profile management:\n"
"    --save_profile=N                   Saves the device\'s currently active profile if N=1.\n"
"    --active_profile=N                 Sets the active profile to N.\n"
"    --device_id=N                      Sets the device\'s nodeid to N.\n"
"    --motor_type=N                     _\n"
"    --motor_numPolePairs=F             _\n"
"    --motor_Rr=F                       _\n"
"    --motor_Rs=F                       _\n"
"    --motor_Ls_d=F                     _\n"
"    --motor_Ls_q=F                     _\n"
"    --motor_ratedFlux=F                _\n"
"    --IdRated=F                        _\n"
"    --maxCurrent_resEst=F              _\n"
"    --maxCurrent_indEst=F              _\n"
"    --maxCurrent=F                     _\n"
"    --fluxEstFreq_Hz=F                 _\n"
"    --speed_limit=F                    Sets the rpm limit of the motor. 0 - 51000 in steps of 200.\n"
"    --acceleration_limit=F             Sets the rpmps limit of the motor. 0 - 12750 in steps of 50.\n"
"    --kp=F                             Reads the kp register or sets it to F. 0.0 - 25.5.\n"
"    --ki=F                             Reads the ki register or sets it to F. 0.0 -  0.255.\n"
"\n";

#define CONFIG_FILE          "config.ini"
#define TEMP_FILE            "temp.ini" //Contains information that has to persist between calls, like the controlwordMem value.
#define CONFLEN              32 //Length of configuration entries.
#define CONFC                8 //Maximum number of configuration entries.

#define DEFAULT_BAUDRATE       "500K"
#define DEFAULT_NODEID         0x06
#define DEFAULT_MASTER_NODEID  0x00
#define DEFAULT_CHANNEL        "can0"
//#define DEFAULT_LIBRARY_PATH   "../../libcanfestival_can_socket.a"
#define DEFAULT_POLLING_PERIOD 1000

#define DEFAULT_CONTROLWORD    0x00

#define ARGLEN  64
#define ARGC    2
#define MAX_WAIT_CYCLES_WRITE_CB 500 // Wait cycles before reporting an unsuccessful write attempt.
#define WAIT_PER_CYCLE_WRITE_CB 1000 // Waiting time per cycle in µs.
#define MAX_WAIT_CYCLES_GLOBAL 1000 //
#define WAIT_PER_CYCLE_GLOBAL 1000 // in µs.

//#define TAG_BEGIN "#START"
#define TAG_OK "#OK\n"
#define TAG_ERROR "#ERROR\n"
#define TAG_END "#END\n"

char tag = 0;
char verbose = 0;
char baudrate[8] = "\0";
int32_t nodeid = 0;
int32_t master_nodeid = 0;
char channel[64] = "\0";
//char libPath[256] = "\0";
int32_t polling_period = 0;
int32_t controlwordMem = 0;
CO_Data* ConsoleAppOD_Data;
s_BOARD Board = { channel, baudrate };

UNS32 data = 0;

char arguments[ARGC][ARGLEN] = {"\0", "\0"};

char configData[CONFC][2][CONFLEN] = {
    {"\0", "\0"},
    {"\0", "\0"},
    {"\0", "\0"},
    {"\0", "\0"},
    {"\0", "\0"},
    {"\0", "\0"},
    {"\0", "\0"},
    {"\0", "\0"}
};

char canfestivalThreadReady = 0;
//char monitoring = 0;
char error = 0;

char configUpdate = 0; //Is set to 1 if the configuration changed and the file has to be updated.
char tempUpdate = 0; //Is set to 1 if the temp data changed.

typedef struct commandEntry{
    char command[ARGLEN];      //String that triggerst this command.
    int32_t index;             //Index.
    int32_t subindex;          //Subindex.
    int32_t mask;              //Filter for the specified data.
    int32_t startBit;          //Position at which the data starts.
    char regSize;              //Size of the register.

    int32_t readIndex;
    int32_t readSubindex;
    int32_t readMask;
    int32_t readStartBit;
    char readRegSize;

    int32_t defaultValue;      //Recommended startup values.
}commandEntry;

commandEntry commandList[] = {
    //Controlword/Statusword
    {"quickstart",                      0x6040, 0x0000, 0x803F, 0x00, 0x02, 0x6041, 0x0000, 0xFFFF, 0x00, 0x02, 0x000F}, //Must be the first entry in the array.
    //{"clean_temp",                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //Must be 2nd entry in the array.
    //{"clean_conf",                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //Must be 3rd entry in the array.
    {"system",                          0x6040, 0x0000, 0x0001, 0x00, 0x02, 0x6041, 0x0000, 0x0001, 0x00, 0x02, 0x0001},
    {"motor",                           0x6040, 0x0000, 0x0002, 0x01, 0x02, 0x6041, 0x0000, 0x0002, 0x01, 0x02, 0x0001},
    {"field_weakening",                 0x6040, 0x0000, 0x0004, 0x02, 0x02, 0x6041, 0x0000, 0x0004, 0x02, 0x02, 0x0001},
    {"force_angle",                     0x6040, 0x0000, 0x0008, 0x03, 0x02, 0x6041, 0x0000, 0x0008, 0x03, 0x02, 0x0001},
    {"resistance_recalculation",        0x6040, 0x0000, 0x0010, 0x04, 0x02, 0x6041, 0x0000, 0x0010, 0x04, 0x02, 0x0000},
    {"power_wrap",                      0x6040, 0x0000, 0x0020, 0x05, 0x02, 0x6041, 0x0000, 0x0020, 0x05, 0x02, 0x0000},
    {"motor_identification",            0x6040, 0x0000, 0x8000, 0x0F, 0x02, 0x6041, 0x0000, 0x8000, 0x0F, 0x02, 0x0000},
    //Target speed
    {"speed",                           0x60FF, 0x0000, 0xFFFFFFFF, 0x00, 0x04, 0x606C, 0x0000, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    //Acceleration
    //{"acceleration",                    0x60C5, 0x0000, 0xFFFFFFFF, 0x00, 0x04, 0x60C5, 0x0000, 0xFFFFFFFF, 0x00, 0x04, 0x000003E8},
    {"acceleration",                    0x6083, 0x0000, 0xFFFFFFFF, 0x00, 0x04, 0x6083, 0x0000, 0xFFFFFFFF, 0x00, 0x04, 0x000003E8},
    //ADC
    {"voltage",                         0x2313, 0x0001, 0xFFFF, 0x00, 0x02, 0x2313, 0x0001, 0xFFFF, 0x00, 0x02, 0x0018},
    {"current",                         0x6078, 0x0000, 0xFFFF, 0x00, 0x02, 0x6078, 0x0000, 0xFFFF, 0x00, 0x02, 0x00C8},
    {"temperature",                     0x2323, 0x0002, 0xFFFF, 0x00, 0x02, 0x2323, 0x0002, 0xFFFF, 0x00, 0x02, 0x0032},
    {"humidity",                        0x2313, 0x0002, 0xFFFF, 0x00, 0x02, 0x2313, 0x0002, 0xFFFF, 0x00, 0x02, 0x000F},
    //Status registers
    {"fault",                           0x2320, 0x0001, 0x0400, 0x0A, 0x02, 0x2320, 0x0001, 0x0400, 0x0A, 0x02, 0x0000}, //Only fault bit.
    {"status1",                         0x2320, 0x0001, 0x0400, 0x0A, 0x02, 0x2320, 0x0001, 0xFFFF, 0x00, 0x02, 0x0000}, //Entire registers.
    {"status2",                         0x2320, 0x0002, 0x0400, 0x0A, 0x02, 0x2320, 0x0002, 0xFFFF, 0x00, 0x02, 0x0000},
    {"status3",                         0x2320, 0x0003, 0x0400, 0x0A, 0x02, 0x2320, 0x0003, 0xFFFF, 0x00, 0x02, 0x0000},
    {"status4",                         0x2320, 0x0004, 0x0400, 0x0A, 0x02, 0x2320, 0x0004, 0xFFFF, 0x00, 0x02, 0x0000},
    //kp/ki
    {"kp",                              0x2331, 0x0001, 0xFFFFFFFF, 0x00, 0x04, 0x2331, 0x0001, 0xFFFFFFFF, 0x00, 0x04, 0x40600000}, //3.5
    {"ki",                              0x2331, 0x0002, 0xFFFFFFFF, 0x00, 0x04, 0x2331, 0x0002, 0xFFFFFFFF, 0x00, 0x04, 0x3D71A9FC}, //0.059
    //Save profile
    {"save_profile",                    0x1010, 0x0001, 0x00000001, 0x00, 0x04, 0x1010, 0x0001, 0x00000001, 0x00, 0x04, 0x00000000},
    //Profile select.
    {"active_profile",                  0x2000, 0x0000, 0xFFFF, 0x00, 0x02, 0x2000, 0x0000, 0xFFFF, 0x00, 0x02, 0x0000},
    //Profile parameters.
    {"device_id",                       0x2001, 0x0001, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0001, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_type",                      0x2001, 0x0002, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0002, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_numPolePairs",              0x2001, 0x0003, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0003, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_Rr",                        0x2001, 0x0004, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0004, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_Rs",                        0x2001, 0x0005, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0005, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_Ls_d",                      0x2001, 0x0006, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0006, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_Ls_q",                      0x2001, 0x0007, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0007, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"motor_ratedFlux",                 0x2001, 0x0008, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0008, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"IdRated",                         0x2001, 0x0009, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0009, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"maxCurrent_resEst",               0x2001, 0x000A, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x000A, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"maxCurrent_indEst",               0x2001, 0x000B, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x000B, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"maxCurrent",                      0x2001, 0x000C, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x000C, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"fluxEstFreq_Hz",                  0x2001, 0x000D, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x000D, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"speed_limit",                     0x2001, 0x000E, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x000E, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"acceleration_limit",              0x2001, 0x000F, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x000F, 0xFFFFFFFF, 0x00, 0x04, 0x00000000},
    {"kpp",                              0x2001, 0x0010, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0010, 0xFFFFFFFF, 0x00, 0x04, 0x40600000}, //3.5
    {"kip",                              0x2001, 0x0011, 0xFFFFFFFF, 0x00, 0x04, 0x2001, 0x0011, 0xFFFFFFFF, 0x00, 0x04, 0x3D71A9FC}, //0.059
    //Restart controller.
    //{"device_restart",                  0x0000, 0x0000, 0x0000, 0x00, 0x00, 0x0000, 0x0000, 0x0000, 0x00, 0x00, 0x0000},
    //Done
    {"\0", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

void loadConfig();
void writeConfig();
void readConfig();
void writeTemp();
void readTemp();
void readIni(char *filename, char configData[CONFC][2][CONFLEN]);
void writeIni(char *filename, char configData[CONFC][2][CONFLEN]);
//int executeCommand(char arguments[ARGC][ARGLEN]);
int executeCommand(int pos, int32_t val, char isReadCmd, char monitoring);
int32_t getValue(char *arg);
int getCmdPos(char *str);
int sendCanData(int pos, int32_t val);
int32_t readCanData(int pos);
void monitor(int pos);

int NodeInit(int NodeID);
void Init(CO_Data* d, UNS32 id);

int main(int argc, char *argv[]) {
    int ret = 0;
    char monitoring = 0;
    char isReadCmd = 0;
    int pos = -1;
    int32_t val = 0;
    float fval = 0;
    int i = 0;
    int optind = 0;

    loadConfig();

    TimerInit();
    NodeInit(master_nodeid);
    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
        usleep(WAIT_PER_CYCLE_GLOBAL);
    }

    struct option long_options[] = {
        {"help", no_argument, NULL, 'h'}, // Show help.
        {"verbose", no_argument, NULL, 'v'}, // Show additional information.
        {"masterid", required_argument, NULL, 7}, // Set master nodeid.
        {"baud", required_argument, NULL, 8}, // Set baud rate.
        {"nodeid", required_argument, NULL, 9}, // Set node id.
        {"channel", required_argument, NULL, 10}, // Set channel.
        //{"library", required_argument, NULL, 'l'}, // Select library.
        {"config", no_argument, NULL, 1}, // Display current configuration.
        {"default", no_argument, NULL, 2}, // Display default configuration.
        {"ct", no_argument, NULL, 3}, // Delete temporary file.
        {"cc", no_argument, NULL, 4}, // Delete configuration file.
        {"clean", no_argument, NULL, 5}, // Delete all associated files (temp and config).
        {"monitor", required_argument, NULL, 'm'}, // Monitors a parameter.
        {"polling", required_argument, NULL, 6}, // Setting polling delay in ms.
        {"tag", no_argument, NULL, 25}, // Sets tag = 1. Shows aditional tags.

        {"quickstart", optional_argument, NULL, 'q'},
        {"statusword", no_argument, NULL, 29},
        {"system", optional_argument, NULL, 11},
        {"motor", optional_argument, NULL, 12},
        {"field_weakening", optional_argument, NULL, 13},
        {"force_angle", optional_argument, NULL, 14},
        {"resistance_recalculation", optional_argument, NULL, 15},
        {"power_wrap", optional_argument, NULL, 16},
        {"motor_identification", optional_argument, NULL, 30},
        //Target speed
        {"speed", optional_argument, NULL, 's'},
        //Acceleration
        {"acceleration", optional_argument, NULL, 'a'},
        //ADC
        {"voltage", no_argument, NULL, 17},
        {"current", no_argument, NULL, 18},
        {"temperature", no_argument, NULL, 19},
        {"humidity", no_argument, NULL, 20},
        //Status registers
        {"fault", optional_argument, NULL, 'f'}, //Only fault bit.
        {"status1", optional_argument, NULL, 21}, //Entire registers.
        {"status2", optional_argument, NULL, 22},
        {"status3", optional_argument, NULL, 23},
        {"status4", optional_argument, NULL, 24},
        //kp/ki
        {"kp", optional_argument, NULL, 26},
        {"ki", optional_argument, NULL, 27},
        //Save profile.
        {"save_profile", required_argument, NULL, 256},
        //Profile select.
        {"active_profile", optional_argument, NULL, 257},
        //Profile data.
        {"device_id", optional_argument, NULL, 258},
        {"motor_type", optional_argument, NULL, 259},
        {"motor_numPolePairs", optional_argument, NULL, 260},
        {"motor_Rr", optional_argument, NULL, 261},
        {"motor_Rs", optional_argument, NULL, 262},
        {"motor_Ls_d", optional_argument, NULL, 263},
        {"motor_Ls_q", optional_argument, NULL, 264},
        {"motor_ratedFlux", optional_argument, NULL, 265},
        {"IdRated", optional_argument, NULL, 266},
        {"maxCurrent_resEst", optional_argument, NULL, 267},
        {"maxCurrent_indEst", optional_argument, NULL, 268},
        {"maxCurrent", optional_argument, NULL, 269},
        {"fluxEstFreq_Hz", optional_argument, NULL, 270},
        {"speed_limit", optional_argument, NULL, 271},
        {"acceleration_limit", optional_argument, NULL, 272},
        //Restart device.
        //{"device_restart", no_argument, NULL, 1023},

        {NULL, 0, NULL, 0}
    };
    while(1){
        int opt = getopt_long(argc, argv, "hvm:q::s::a::f::", long_options, &optind);
        if(opt == -1){
            break;
        }
        switch(opt){
            case '?':
                exit(EXIT_FAILURE);
                break;
            // Configuration.
            case 'v':
                verbose = 1;
                break;
            case 'h':
                //printf("opt: %s\n", argv[optind+1]);
                printf(usage, argv[0], commandList[0].defaultValue);
                exit(EXIT_SUCCESS);
                break;
            case 8:
                configUpdate = 1;
                sscanf(optarg, "%s", baudrate);
                if(verbose)
                    printf("Baudrate: %s\n", baudrate);
                break;
            case 9:
                configUpdate = 1;
                sscanf(optarg, "%d", &nodeid);
                if(verbose)
                    printf("NodeID: %d\n", nodeid);
                break;
            case 7/*'m'*/:
                configUpdate = 1;
                sscanf(optarg, "%d", &master_nodeid);
                if(verbose)
                    printf("Master NodeID: %d\n", master_nodeid);
                break;
            case 10:
                configUpdate = 1;
                sscanf(optarg, "%s", channel);
                if(verbose)
                    printf("Channel: %s\n", channel);
                break;
            //case 'l':
            //    configUpdate = 1;
            //    sscanf(optarg, "%s", libPath);
            //    if(verbose)
            //        printf("Library path: %s", libpath);
            //    break;
            case 1:
                printf("Current master nodeid: %d\n", master_nodeid);
                printf("Current baudrate: %s\n", baudrate);
                printf("Current nodeid: %d\n", nodeid);
                printf("Current channel: %s\n", channel);
                printf("Current polling period: %d\n", polling_period);
                //printf("Current library path: %s\n", libPath);
                printf("Saved controlword: %x\n", controlwordMem);
                break;
            case 2:
                printf("Default master nodeid: %d\n", DEFAULT_MASTER_NODEID);
                printf("Default baudrate: %s\n", DEFAULT_BAUDRATE);
                printf("Default nodeid: %d\n", DEFAULT_NODEID);
                printf("Default channel: %s\n", DEFAULT_CHANNEL);
                printf("Default polling period: %d\n", DEFAULT_POLLING_PERIOD);
                //printf("Default library path: %s\n", DEFAULT_LIBRARY_PATH);
                printf("Default controlword: %x\n", commandList[0].defaultValue);
                break;
            case 3:
                if(verbose) printf("Deleting: %s\n", TEMP_FILE);
                remove(TEMP_FILE);
                break;
            case 4:
                if(verbose) printf("Deleting %s\n", CONFIG_FILE);
                remove(CONFIG_FILE);
                break;
            case 5:
                if(verbose) printf("Deleting: %s\n", TEMP_FILE);
                remove(TEMP_FILE);
                if(verbose) printf("Deleting: %s\n", CONFIG_FILE);
                remove(CONFIG_FILE);
                break;
            case 6:
                configUpdate = 1;
                sscanf(optarg, "%d", &polling_period);
                if(verbose)
                    printf("Polling period: %d\n", polling_period);
                break;
            // Motor controls.
            case 25:
                tag = 1;
                break;
            case 'm':
                monitoring = 1;
                //sscanf(optarg, "%s", &arguments[0]);
                if(strncmp(optarg, "statusword", 11) == 0){
                    pos = 0;
                }else{
                    pos = getCmdPos(optarg);
                }
                if(pos > -1){
                    if(verbose)
                        printf("Monitoring: %s\n", optarg);
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }else{
                    if(verbose) printf("Can't monitor \"%s\".\n", optarg);
                }
                break;
            case 29: // Status word request. Handled in the "quickstart" section.
                //if(verbose) printf("Status word...\n");
            case 'q':
                #if DIRECT_MAPPING
                pos = 0;
                #else
                pos = getCmdPos("quickstart");
                #endif
                if(opt == 29){ // If a status word is requested, this is a read request.
                    isReadCmd = 1;
                }else if(optarg == NULL){
                    isReadCmd = 0; //Without a parameter we're still writing in this case.
                    val = commandList[0].defaultValue;
                }else{
                    sscanf(optarg, "%x", &val);
                }
                //printf("val = %x\n", val);
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 11:
                #if DIRECT_MAPPING
                pos = 1;
                #else
                pos = getCmdPos("system");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 12:
                #if DIRECT_MAPPING
                pos = 2;
                #else
                pos = getCmdPos("motor");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 13:
                #if DIRECT_MAPPING
                pos = 3;
                #else
                pos = getCmdPos("field_weakening");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 14:
                #if DIRECT_MAPPING
                pos = 4;
                #else
                pos = getCmdPos("force_angle");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 15:
                #if DIRECT_MAPPING
                pos = 5;
                #else
                pos = getCmdPos("resistance_recalculation");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 16:
                #if DIRECT_MAPPING
                pos = 6;
                #else
                pos = getCmdPos("power_wrap");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 30:
                #if DIRECT_MAPPING
                pos = 7;
                #else
                pos = getCmdPos("motor_identification");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 's':
                #if DIRECT_MAPPING
                pos = 8;
                #else
                pos = getCmdPos("speed");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 'a':
                #if DIRECT_MAPPING
                pos = 9;
                #else
                pos = getCmdPos("acceleration");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 17:
                #if DIRECT_MAPPING
                pos = 10;
                #else
                pos = getCmdPos("voltage");
                #endif
                //if(optarg == NULL){
                    isReadCmd = 1;
                //}
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 18:
                #if DIRECT_MAPPING
                pos = 11;
                #else
                pos = getCmdPos("current");
                #endif
                //if(optarg == NULL){
                    isReadCmd = 1;
                //}
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 19:
                #if DIRECT_MAPPING
                pos = 12;
                #else
                pos = getCmdPos("temperature");
                #endif
                //if(optarg == NULL){
                    isReadCmd = 1;
                //}
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 20:
                #if DIRECT_MAPPING
                pos = 13;
                #else
                pos = getCmdPos("humidity");
                #endif
                //if(optarg == NULL){
                    isReadCmd = 1;
                //}
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 'f':
                #if DIRECT_MAPPING
                pos = 14;
                #else
                pos = getCmdPos("fault");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 21:
                #if DIRECT_MAPPING
                pos = 15;
                #else
                pos = getCmdPos("status1");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 22:
                #if DIRECT_MAPPING
                pos = 16;
                #else
                pos = getCmdPos("status2");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 23:
                #if DIRECT_MAPPING
                pos = 17;
                #else
                pos = getCmdPos("status3");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 24:
                #if DIRECT_MAPPING
                pos = 18;
                #else
                pos = getCmdPos("status4");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 26:
                #if DIRECT_MAPPING
                pos = 19;
                #else
                pos = getCmdPos("kp");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 27:
                #if DIRECT_MAPPING
                pos = 20;
                #else
                pos = getCmdPos("ki");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 256:
                #if DIRECT_MAPPING
                pos = 21;
                #else
                pos = getCmdPos("save_profile");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                    //isReadCmd = 0;
                    //val = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 257:
                #if DIRECT_MAPPING
                pos = 22;
                #else
                pos = getCmdPos("active_profile");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%d", &val);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 258:
                #if DIRECT_MAPPING
                pos = 23;
                #else
                pos = getCmdPos("device_id");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 259:
                #if DIRECT_MAPPING
                pos = 24;
                #else
                pos = getCmdPos("motor_type");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 260:
                #if DIRECT_MAPPING
                pos = 25;
                #else
                pos = getCmdPos("motor_numPolePairs");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 261:
                #if DIRECT_MAPPING
                pos = 26;
                #else
                pos = getCmdPos("motor_Rr");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 262:
                #if DIRECT_MAPPING
                pos = 27;
                #else
                pos = getCmdPos("motor_Rs");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 263:
                #if DIRECT_MAPPING
                pos = 28;
                #else
                pos = getCmdPos("motor_Ls_d");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 264:
                #if DIRECT_MAPPING
                pos = 29;
                #else
                pos = getCmdPos("motor_Ls_q");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 265:
                #if DIRECT_MAPPING
                pos = 30;
                #else
                pos = getCmdPos("motor_ratedFlux");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 266:
                #if DIRECT_MAPPING
                pos = 31;
                #else
                pos = getCmdPos("IdRated");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 267:
                #if DIRECT_MAPPING
                pos = 32;
                #else
                pos = getCmdPos("maxCurrent_resEst");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 268:
                #if DIRECT_MAPPING
                pos = 33;
                #else
                pos = getCmdPos("maxCurrent_indEst");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 269:
                #if DIRECT_MAPPING
                pos = 34;
                #else
                pos = getCmdPos("maxCurrent");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 270:
                #if DIRECT_MAPPING
                pos = 35;
                #else
                pos = getCmdPos("fluxEstFreq_Hz");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 271:
                #if DIRECT_MAPPING
                pos = 36;
                #else
                pos = getCmdPos("speed_limit");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            case 272:
                #if DIRECT_MAPPING
                pos = 37;
                #else
                pos = getCmdPos("acceleration_limit");
                #endif
                if(optarg == NULL){
                    isReadCmd = 1;
                }else{
                    sscanf(optarg, "%f", &fval);
                }
                if(pos > -1){
                    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
                        usleep(WAIT_PER_CYCLE_GLOBAL);
                    }
                    canfestivalThreadReady = 0;
                    memcpy(&val, &fval, sizeof(val));
                    ret = executeCommand(pos, val, isReadCmd, monitoring);
                }
                break;
            default:
                break;
        }
    }

    //Update configuration file if necessary.
    if(configUpdate){
        writeConfig();
    }

    //Getting the first two non-option arguments.
    //<command> <value>
    //for(i = 0; (i < ARGC) & ((optind + i) < argc); i++){
        //printf("%s\n", argv[optind + i]);
    //    strncpy(arguments[i], argv[optind + i], sizeof(arguments[i]));
    //}

    //If there is nothing to do, exit.
    //if(arguments[0][0] == 0 || strcmp(arguments[0], "") == 0){
    //    return 0;
    //}

    //if(pos > -1) ret = executeCommand(pos, val, isReadCmd, monitoring);

    if(tempUpdate){
        writeTemp();
    }

    for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++){
        usleep(WAIT_PER_CYCLE_GLOBAL);
    }

    if(verbose) printf("Done.\n");
    if(tag) printf("%s", TAG_END);

    exit(EXIT_SUCCESS);
    return ret;
}

/*
Loads configuration from the config file.
If no such file exists, loads default values.
*/
void loadConfig(){
    if(access(TEMP_FILE, R_OK) != -1){
        readTemp();
    }else{
        controlwordMem = DEFAULT_CONTROLWORD; //commandList[0].defaultValue;
    }

    if(access(CONFIG_FILE, R_OK) != -1){
        readConfig();
    }else{
        strncpy(baudrate, DEFAULT_BAUDRATE, sizeof(DEFAULT_BAUDRATE));
        strncpy(channel, DEFAULT_CHANNEL, sizeof(DEFAULT_CHANNEL));
        nodeid = DEFAULT_NODEID;
        master_nodeid = DEFAULT_MASTER_NODEID;
        polling_period = DEFAULT_POLLING_PERIOD;
    }
}

int searchConfigData(char configData[CONFC][2][CONFLEN], char *label){
    int i = 0;
    for(i = 0; i < CONFC; i++){
        if(strcmp(configData[i][0], label) == 0){
            return i;
        }
    }
    return -1;
}

/*
Saves current configuration to the config file.
*/
void writeConfig(){
    if(verbose) printf("Saving configuration.\n");
    //master_nodeid
    sprintf(configData[0][0], "master_nodeid");
    sprintf(configData[0][1], "%d", master_nodeid);
    //nodeid
    sprintf(configData[1][0], "nodeid");
    sprintf(configData[1][1], "%d", nodeid);
    //baudrate
    sprintf(configData[2][0], "baudrate");
    sprintf(configData[2][1], "%s", baudrate);
    //channel
    sprintf(configData[3][0], "channel");
    sprintf(configData[3][1], "%s", channel);
    //polling_period
    sprintf(configData[4][0], "polling_period");
    sprintf(configData[4][1], "%d", polling_period);
    //terminate
    configData[5][0][0] = 0;

    writeIni(CONFIG_FILE, configData);
}

void readConfig(){
    readIni(CONFIG_FILE, configData);

    int pos = searchConfigData(configData, "master_nodeid");
    if(pos != -1) sscanf(configData[pos][1], "%d", &master_nodeid);

    pos = searchConfigData(configData, "nodeid");
    if(pos != -1) sscanf(configData[pos][1], "%d", &nodeid);

    pos = searchConfigData(configData, "baudrate");
    if(pos != -1) sscanf(configData[pos][1], "%s", (char*)&baudrate);

    pos = searchConfigData(configData, "channel");
    if(pos != -1) sscanf(configData[pos][1], "%s", (char*)&channel);

    pos = searchConfigData(configData, "polling_period");
    if(pos != -1) sscanf(configData[pos][1], "%d", &polling_period);
}

void writeTemp(){
    if(verbose) printf("Saving temp data.\n");
    //controlword
    sprintf(configData[0][0], "controlwordMem");
    sprintf(configData[0][1], "%x", controlwordMem);
    //terminate
    configData[1][0][0] = 0;

    writeIni(TEMP_FILE, configData);
}

void readTemp(){
    readIni(TEMP_FILE, configData);

    int pos = searchConfigData(configData, "controlwordMem");
    if(pos != -1) sscanf(configData[pos][1], "%x", &controlwordMem);
}

void readIni(char *filename, char configData[CONFC][2][CONFLEN]){
    FILE *f = fopen(filename, "r");

    int i = 0;
    for(i = 0; i < CONFC; i++){
        fscanf(f, "%s = %s\n", configData[i][0], configData[i][1]);
        //fputs(line, f);
    }

    fclose(f);
}

void writeIni(char *filename, char configData[CONFC][2][CONFLEN]){
    FILE *f = fopen(filename, "w+");
    //char line[(CONFLEN<<1)+4] = "\0";

    int i = 0;
    for(i = 0; i < CONFC && configData[i][0][0] != 0; i++){
        fprintf(f, "%s = %s\n", configData[i][0], configData[i][1]);
        //fputs(line, f);
    }

    fclose(f);
}

/*
Sends/reads the CAN frames associated with the given command.
*/
/*int executeCommand(char arguments[ARGC][ARGLEN]){
    int32_t val = 0;
    int pos = getCmdPos(arguments[0]);
    //If command was not found, abort.
    if(pos == -1){
        if(verbose) printf("Wrong parameter: %s\n", arguments[0]);
        return pos;
    }

    //Determines if the command is meant for reading or writing a register.
    char isReadCmd = (arguments[1] != NULL && strcmp(arguments[1], "") != 0)?0:1;

    //If we're not reading, get the value from the 2nd argument.
    if(!isReadCmd && !monitoring){
        val = getValue(arguments[1]);
    }

    if(monitoring == 1){
        printf("Monitoring\n");
        monitor(pos);
    }else if(pos == 0){ //Quickstart
        sendCanData(pos, commandList[pos].defaultValue);
        if(controlwordMem != commandList[pos].defaultValue){ //If the old controlword is different from the new one, set a flag to update the temp file.
            controlwordMem = commandList[pos].defaultValue;
            tempUpdate = 1;
        }
    //}else if(pos == 1){ //clean_temp
    //    remove(TEMP_FILE);
    //}else if(pos == 2){ //clean_conf
    //    remove(CONFIG_FILE);
    }else if(isReadCmd){ //Reading
        //int32_t data = readCanData(pos);
        readCanData(pos);
        for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++)
            usleep(WAIT_PER_CYCLE_GLOBAL);
        printf("%d\n", (data&commandList[pos].readMask));
        data = 0;
    }else if(commandList[pos].index == 0x6040){ //Editing controlword
        int32_t data = (~((~controlwordMem) | commandList[pos].mask)) | ((val << commandList[pos].startBit) & commandList[pos].mask); //Resets the correct bit to 0 and sets it to the new value.
        sendCanData(pos, data);
        if(controlwordMem != data){ //If the old controlword is different from the new one, set a flag to update the temp file.
            controlwordMem = data;
            tempUpdate = 1;
        }
    }else{ //Direct write
        sendCanData(pos, val);
    }

    return 0;
}*/

/*
Sends/reads the CAN frames associated with the given command.
*/
int executeCommand(int pos, int32_t val, char isReadCmd, char monitoring){
    int i = 0;
    //int32_t val = 0;
    //int pos = getCmdPos(arguments[0]);
    //If command was not found, abort.
    if(pos == -1){
        //if(verbose) printf("The given option does not exist.\n");
        return pos;
    }

    //Determines if the command is meant for reading or writing a register.
    //char isReadCmd = (arguments[1] != NULL && strcmp(arguments[1], "") != 0)?0:1;

    //If we're not reading, get the value from the 2nd argument.
    //if(!isReadCmd && !monitoring){
    //    val = getValue(arguments[1]);
    //}

    if(monitoring == 1){
        //if(verbose) printf("Monitoring\n");
        monitor(pos);
    }else if(pos == 0 && !isReadCmd){ //Quickstart
        sendCanData(pos, val);
        if(controlwordMem != val){ //If the old controlword is different from the new one, set a flag to update the temp file.
            controlwordMem = val;
            tempUpdate = 1;
        }
    //}else if(pos == 1){ //clean_temp
    //    remove(TEMP_FILE);
    //}else if(pos == 2){ //clean_conf
    //    remove(CONFIG_FILE);
    }else if(isReadCmd){ //Reading
        //int32_t data = readCanData(pos);
        readCanData(pos);
        for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++)
            usleep(WAIT_PER_CYCLE_GLOBAL);
        if(commandList[pos].index == 0x2001 || commandList[pos].index == 0x2331){
            float fdata = 0;
            int32_t idata = (data & commandList[pos].readMask) >> commandList[pos].readStartBit;
            memcpy(&fdata, &idata, sizeof(idata));
            printf("%f\n", fdata);
        }else{
            printf("%d\n", (data & commandList[pos].readMask) >> commandList[pos].readStartBit);
        }
        data = 0;
    }else if(commandList[pos].index == 0x6040){ //Editing controlword
        int32_t data = (~((~controlwordMem) | commandList[pos].mask)) | ((val << commandList[pos].startBit) & commandList[pos].mask); //Resets the correct bit to 0 and sets it to the new value.
        sendCanData(pos, data);
        if(controlwordMem != data){ //If the old controlword is different from the new one, set a flag to update the temp file.
            controlwordMem = data;
            tempUpdate = 1;
        }
    }else{ //Direct write
        sendCanData(pos, val);
    }

    return 0;
}

/*
Returns the numerical representation of the string *arg.
*/
int32_t getValue(char *arg){
    int32_t val = 0;
    if(strcmp(arg, "on") == 0 || strcmp(arg, "true") == 0 || strcmp(arg, "up") == 0){
        val = 1;
    }else if(strcmp(arg, "off") == 0 || strcmp(arg, "false") == 0 || strcmp(arg, "down") == 0){
        val = 0;
    }else if(strncmp(arg, "0x", 2) == 0){
        sscanf(arg, "%x", &val);
    }else{
        sscanf(arg, "%d", &val);
    }

    return val;
}

/*
Searches for a string in commandList[i].command and returns it's position within the array.
If the string wasn't found, returns -1.
*/
int getCmdPos(char *str){
    char *ptr = NULL;
    int i = 0;
    do{
        ptr = commandList[i].command;
        if(strcmp(str, ptr) == 0){
            return i;                    //If the string is found, return the position.
        }
        //printf("%d %d\n", i, ptr);
        i++;
    }while(ptr[0] != 0);

    return -1; //Command not found.
}

//##############################################################################

/* Callback function that check the write SDO demand */
void CheckWriteSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;
    UNS8 status;

    //int i = 0;
    //do{
        if(verbose) printf("Getting write result...\n");
        status = getWriteResultNetworkDict(ConsoleAppOD_Data, nodeid, &abortCode);
        if(verbose){
            char str[64] = "\0";
            if(status == SDO_RESET) strcpy(str, "SDO_RESET");
            else if(status == SDO_FINISHED) strcpy(str, "SDO_FINISHED");
            else if(status == SDO_ABORTED_INTERNAL) strcpy(str, "SDO_ABORTED_INTERNAL");
            else if(status == SDO_ABORTED_RCV) strcpy(str, "SDO_ABORTED_RCV");
            else if(status == SDO_DOWNLOAD_IN_PROGRESS) strcpy(str, "SDO_DOWNLOAD_IN_PROGRESS");
            else if(status == SDO_UPLOAD_IN_PROGRESS) strcpy(str, "SDO_UPLOAD_IN_PROGRESS");
            else if(status == SDO_BLOCK_DOWNLOAD_IN_PROGRESS) strcpy(str, "SDO_BLOCK_DOWNLOAD_IN_PROGRESS");
            else if(status == SDO_BLOCK_UPLOAD_IN_PROGRESS) strcpy(str, "SDO_BLOCK_UPLOAD_IN_PROGRESS");
            else if(status == SDO_PROVIDED_BUFFER_TOO_SMALL) strcpy(str, "SDO_PROVIDED_BUFFER_TOO_SMALL");
            else strcpy(str, "Unknown error");

            printf("%s: %d\n", str, status);
        }
        //if(status == SDO_FINISHED){
        //    break;
        //}else{
        //    usleep(WAIT_PER_CYCLE_WRITE_CB);
        //}
        //i++;
    //}while(i < MAX_WAIT_CYCLES_WRITE_CB);

	if(status != SDO_FINISHED){
		if(verbose) printf("Result : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
        if(tag) printf("%s", TAG_ERROR);
        error = 1;
    }else{
		if(verbose) printf("Send data OK\n");
        if(tag) printf("%s", TAG_OK);
    }

    //canfestivalThreadReady = 1;
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(ConsoleAppOD_Data, nodeid, SDO_CLIENT);
    canfestivalThreadReady = 1;
}

/*
Sends data via the CAN interface.
*/
int sendCanData(int pos, int32_t val){
    canfestivalThreadReady = 0;
    if(verbose){
        printf("Writing to 0x%x.%x.%x: 0x%x\n", nodeid, commandList[pos].index, commandList[pos].subindex, val);
    }

    EnterMutex();
    writeNetworkDictCallBack(ConsoleAppOD_Data, nodeid, commandList[pos].index, commandList[pos].subindex, commandList[pos].regSize, 0, &val, CheckWriteSDO, 0);
    //writeNetworkDict(ConsoleAppOD_Data, nodeid, commandList[pos].index, commandList[pos].subindex, commandList[pos].regSize, 0, &val, 0);
    LeaveMutex();
    //canfestivalThreadReady = 1;
    return 0;
}

void ConsoleAppOD_initialisation(CO_Data* d){
	if(verbose) printf("Node_initialisation\n");
}

void ConsoleAppOD_preOperational(CO_Data* d){
	if(verbose) printf("Node_preOperational\n");
    canfestivalThreadReady = 1;
}

void ConsoleAppOD_operational(CO_Data* d){
	if(verbose) printf("Node_operational\n");
}

void ConsoleAppOD_stopped(CO_Data* d){
	if(verbose) printf("Node_stopped\n");
}

void ConsoleAppOD_post_sync(CO_Data* d){
	//if(verbose) printf("Master_post_sync\n");
}

void ConsoleAppOD_post_TPDO(CO_Data* d){
	//if(verbose) printf("Master_post_TPDO\n");
}

void ConsoleAppOD_post_SlaveBootup(CO_Data* d, UNS8 nodeid){
	if(verbose) printf("Slave %x boot up\n", nodeid);
}

int NodeInit(int NodeID){
    ConsoleAppOD_Data = &Motor_Data; //Dictionary data.

    /* Load can library */
	//LoadCanDriver(LibraryPath);

    /* Define callback functions */
    ConsoleAppOD_Data->initialisation = ConsoleAppOD_initialisation;
    ConsoleAppOD_Data->preOperational = ConsoleAppOD_preOperational;
    ConsoleAppOD_Data->operational = ConsoleAppOD_operational;
    ConsoleAppOD_Data->stopped = ConsoleAppOD_stopped;
    ConsoleAppOD_Data->post_sync = ConsoleAppOD_post_sync;
    ConsoleAppOD_Data->post_TPDO = ConsoleAppOD_post_TPDO;
    ConsoleAppOD_Data->post_SlaveBootup = ConsoleAppOD_post_SlaveBootup;

    /* Open CAN socket device */
	if(!canOpen(&Board, ConsoleAppOD_Data)) return -1;

    /* Defining the node Id */
    setNodeId(ConsoleAppOD_Data, NodeID); //master_nodeid
    /* Start Timer thread */
    StartTimerLoop(&Init);
    return 0;
}

void Init(CO_Data* d, UNS32 id){
	if(Board.baudrate)
	{
		/* Init node state*/
		setState(ConsoleAppOD_Data, Initialisation);
	}
}

/* Callback function that check the read SDO demand */
void CheckReadSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode = 0;
	//UNS32 data = 0;
	UNS32 size = 64;

	if(getReadResultNetworkDict(ConsoleAppOD_Data, nodeid, &data, &size, &abortCode) != SDO_FINISHED){
		if(verbose) printf("Result : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
        if(tag) printf("%s", TAG_ERROR);
        error = 1;
    }else{
		if(verbose) printf("Result: %x\n", data);
        if(tag) printf("%s", TAG_OK);
    }

    //canfestivalThreadReady = 1;
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(ConsoleAppOD_Data, nodeid, SDO_CLIENT);
    canfestivalThreadReady = 1;
    //printf("CheckReadSDO end\n");
}

/*
Reads data from the given index.
*/
int32_t readCanData(int pos){
    canfestivalThreadReady = 0;
    if(verbose){
        printf("Reading from 0x%x.%x.%x. \n", nodeid, commandList[pos].readIndex, commandList[pos].readSubindex);
    }

    EnterMutex();
    readNetworkDictCallback(ConsoleAppOD_Data, (UNS8)nodeid, (UNS16)commandList[pos].readIndex, (UNS8)commandList[pos].readSubindex, (UNS8)0, CheckReadSDO, 0);
    LeaveMutex();

    //printf("readCanData end\n");
    return 0;
}

/*
Reads a parameter repeatedly and prints it.
*/
void monitor(int pos){
    long pp = polling_period * 1000;
    //char input = '\0';
    int i = 0;

    while(1/*!error*/){
        readCanData(pos);
        for(i = 0; i < MAX_WAIT_CYCLES_GLOBAL && !canfestivalThreadReady; i++)
            usleep(WAIT_PER_CYCLE_GLOBAL);
        if(commandList[pos].index == 0x2001){
            float fdata = 0;
            int32_t idata = (data & commandList[pos].readMask) >> commandList[pos].readStartBit;
            memcpy(&fdata, &idata, sizeof(idata));
            printf("%f\n", fdata);
        }else{
            printf("%d\n", (data & commandList[pos].readMask) >> commandList[pos].readStartBit);
        }
        data = 0;

        fflush(stdout);
        usleep(pp);
    }
}
