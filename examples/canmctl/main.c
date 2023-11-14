#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <getopt.h>
#include <unistd.h>

#include "canfestival.h"
#include "handlers.h"
#include "motor.h"

#define MAX_NODES 127

static const char *usage = "USAGE: %s [OPTIONS]\n"
"   the program uses SDO protocol only and don't rely on PDO mapping\n"
"    --help, -h          display this help\n"
"    --node=N, -nN       select node with id=N\n"
"                        if omnitted or N == 0, can bus will be scanned\n"
"\n";

void shutdown(void);

static void Motor_GOBJ(uint8_t node_id, uint16_t index, uint8_t subindex);
static void Motor_repl_GOBJ(CO_Data* d, UNS8 nodeid);

int main(int argc, char *argv[]) {
    uint8_t node_id = 0, log_mask = 0x01, cmd = 0, mode = 0;
    struct option long_options[] = {
        { "help",       no_argument,        NULL,       'h' },
        { "log-level",  required_argument,  NULL,       'l' },

        { "node",       required_argument,  NULL,       'n' },

        { "command",    required_argument,  NULL,       'c' },
        { "mode",       required_argument,  NULL,       'm' },

        { NULL,         0,                  NULL,       0   }
    };

    while (1) {
        int c = getopt_long(argc, argv, "hl:n:c:m:", long_options, NULL);

        if (c == -1)
            break;

        switch (c) {
        case '?':
            exit(EXIT_FAILURE);

        case 'h':
            printf(usage, argv[0]);
            exit(EXIT_SUCCESS);

        case 'l':
            sscanf(optarg, "%" SCNi32, &log_mask);
            break;

        case 'n':
            sscanf(optarg, "%" SCNu8, &node_id);
            break;

        case 'c':
            cmd = optarg[0];
            break;

        case 'm':
            mode = optarg[0];
            break;

        default:
            break;
        }
    }

	TimerInit();

    s_BOARD board = { "vcan0", "500K" };
	if(!canOpen(&board, &Motor_Data))
        goto error;

    setNodeId(&Motor_Data, 0);

	StartTimerLoop(&Init);
    atexit(shutdown);

    while (!ready)
        usleep(1000);

    Motor_GOBJ(6, 0x1000, 0);

    int c;
    while ((c = getchar()) >= 0 && c != 'e') {
        Controlword++;
        sendSYNC(&Motor_Data);
        sendOnePDOevent(&Motor_Data, 0);
        printf("Status: %04" PRIx16 "\n", Statusword);
    }

    return EXIT_SUCCESS;

error:
    TimerCleanup();
    perror(argv[0]);
    return EXIT_FAILURE;
}

void shutdown(void) {
	StopTimerLoop(&Exit);
    canClose(&Motor_Data);
	TimerCleanup();
}

static void Motor_GOBJ(uint8_t node_id, uint16_t index, uint8_t subindex) {
	EnterMutex();
    printf("GOBJ > %" PRIu8 ",%" PRIx16 ",%" PRIu8 "\n", node_id, index, subindex);
    readNetworkDictCallback(&Motor_Data, node_id, index, subindex, 0, Motor_repl_GOBJ, 0);
	LeaveMutex();
}

static void Motor_repl_GOBJ(CO_Data* d, uint8_t nodeid) {
	uint32_t data = 0, size = 64, abortCode;

	if(getReadResultNetworkDict(&Motor_Data, nodeid, &data, &size, &abortCode) != SDO_FINISHED)
		printf("GOBJ < failed. AbortCode: 0x%08" PRIX32 "\n", abortCode);
	else
		printf("GOBJ < %08" PRIX32 "\n", data);

	closeSDOtransfer(&Motor_Data, nodeid, SDO_CLIENT);
}
