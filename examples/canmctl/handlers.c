#include <stdio.h>

#include "handlers.h"
#include "motor.h"

volatile int ready = 0;

static void Motor_initialisation(CO_Data* d);
static void Motor_preOperational(CO_Data* d);
static void Motor_operational(CO_Data* d);
static void Motor_stopped(CO_Data* d);
static void Motor_post_sync(CO_Data* d);
static void Motor_post_TPDO(CO_Data* d);
static void Motor_post_SlaveBootup(CO_Data* d, UNS8 nodeid);

void Init(CO_Data* d, UNS32 id) {
    (void)d;
    (void)id;

	Motor_Data.initialisation = Motor_initialisation;
	Motor_Data.preOperational = Motor_preOperational;
	Motor_Data.operational = Motor_operational;
	Motor_Data.stopped = Motor_stopped;
	Motor_Data.post_sync = Motor_post_sync;
	Motor_Data.post_TPDO = Motor_post_TPDO;
	Motor_Data.post_SlaveBootup=Motor_post_SlaveBootup;

    setState(&Motor_Data, Initialisation);
}

void Exit(CO_Data* d, UNS32 id) {
    (void)d;
    (void)id;

    setState(&Motor_Data, Stopped);
}

static void Motor_initialisation(CO_Data* d) {
	printf("Node_initialisation\n");
}

static void Motor_preOperational(CO_Data* d) {
	printf("Node_preOperational\n");
    ready = 1;
    setState(&Motor_Data, Operational);
}

static void Motor_operational(CO_Data* d) {
	printf("Node_operational\n");
    PDOInit(&Motor_Data);
}

static void Motor_stopped(CO_Data* d) {
	printf("Node_stopped\n");
    PDOStop(&Motor_Data);
}

static void Motor_post_sync(CO_Data* d) {
	printf("Master_post_sync\n");
}

static void Motor_post_TPDO(CO_Data* d) {
	printf("Master_post_TPDO\n");
}

static void Motor_post_SlaveBootup(CO_Data* d, UNS8 nodeid) {
	printf("Slave %x boot up\n", nodeid);
}
