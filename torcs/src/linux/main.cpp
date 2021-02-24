/***************************************************************************

    file                 : main.cpp
    created              : Sat Mar 18 23:54:30 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: main.cpp,v 1.14.2.3 2012/06/01 01:59:42 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdlib.h>

#include <GL/glut.h>

#include <tgfclient.h>
#include <client.h>

#include "linuxspec.h"
#include <raceinit.h>
#include <iostream>

// CHANGES
#include <GL/freeglut.h>
#include <plib/ssg.h>
#include <signal.h>
#include <pthread.h>

#include <sys/shm.h>
#include<semaphore.h>

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

bool is_re_gui_without_select = false;
bool auto_back = false;
static int key = 817;
// END

extern bool bKeepModules;

static void
init_args(int argc, char **argv, const char **raceconfig)
{
	int i;
	char *buf;

	i = 1;

	while(i < argc) {
		if(strncmp(argv[i], "-l", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLocalDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-L", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLibDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-D", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetDataDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-s", 2) == 0) {
			i++;
			SetSingleTextureMode();
		} else if(strncmp(argv[i], "-k", 2) == 0) {
			i++;
			// Keep modules in memory (for valgrind)
			printf("Unloading modules disabled, just intended for valgrind runs.\n");
			bKeepModules = true;
#ifndef FREEGLUT
		} else if(strncmp(argv[i], "-m", 2) == 0) {
			i++;
			GfuiMouseSetHWPresent(); /* allow the hardware cursor */
#endif
		} else if(strncmp(argv[i], "-r", 2) == 0) {
			i++;
			*raceconfig = "";

			if(i < argc) {
				*raceconfig = argv[i];
				i++;
			}

			if((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)) {
				printf("Please specify a race configuration xml when using -r\n");
				exit(1);
			}
// CHANGES
        } else if(strncmp(argv[i], "_mkey", 5) == 0){
            ++i;
            if(i < argc){
                key = atoi(argv[i]);
                fprintf(stderr,"KEY: %i\n",key);
                ++i;
            }else{
                printf("Please specify a memory sharing key when using -mkey\n");
                exit(1);
            }
        } else if(strncmp(argv[i], "_rgs", 4) == 0){
            ++i;
            *raceconfig = "";

            if(i < argc){
                *raceconfig = argv[i];
                ++i;
            }

            if ((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)){
                printf("Please specify a race configuration xml when using -rgs\n");
                exit(1);
            }
            is_re_gui_without_select = true;
        }else if (strncmp(argv[i], "_screen", 7) == 0){
            i++;
            char* screenSpec = nullptr;

            if(i < argc) {
                screenSpec = argv[i];
                i++;
            }

            if((strlen(screenSpec) == 0)) {
                printf("Please specify a screen id\n");
                exit(1);
            }

            char* port;
            port = getenv("DISPLAY");
            char newPort[100];
            sprintf(newPort, "%s.%s", port, screenSpec);
            setenv("DISPLAY", newPort , 1);
            // printf("PORT: %s", newPort);

        } else if (strncmp(argv[i], "_back", 5) == 0){
            i++;
            auto_back = true;

// END
		} else {
			i++;		/* ignore bad args */
		}
	}

#ifdef FREEGLUT
	GfuiMouseSetHWPresent(); /* allow the hardware cursor (freeglut pb ?) */
#endif
}

//CHANGES
struct shared_use_st
{
    sem_t written;
    sem_t read;

    uint8_t data[IMAGE_WIDTH*IMAGE_HEIGHT*3];
    uint8_t data_left[IMAGE_WIDTH*IMAGE_HEIGHT*3];
    uint8_t data_right[IMAGE_WIDTH*IMAGE_HEIGHT*3];
    uint8_t data_between[IMAGE_WIDTH*IMAGE_HEIGHT*3];
    int pid;
    int isEnd;
    double dist;

    double steerCmd;
    double accelCmd;
    double brakeCmd;

    // for reward building
    double speed;
    double angle_in_rad;
    int damage;
    double pos;
    int segtype;
    double radius;
    int frontCarNum;
    double frontDist;
};

volatile sem_t* sem_written = NULL;
volatile sem_t* sem_read = NULL;

volatile uint8_t* pdata = NULL;
volatile uint8_t* pdata_left = NULL;
volatile uint8_t* pdata_right = NULL;
volatile uint8_t* pdata_between = NULL;
// volatile int* pcontrol = NULL;
// volatile int* ppause = NULL;
volatile int* pisEnd = NULL;
volatile double* pdist = NULL;

volatile double* psteerCmd_ghost = NULL;
volatile double* paccelCmd_ghost = NULL;
volatile double* pbrakeCmd_ghost = NULL;

volatile double* pspeed_ghost = NULL;
volatile double* pangle_in_rad_ghost = NULL;
volatile int* pdamage_ghost = NULL;
volatile double* ppos_ghost = NULL;
volatile int* psegtype_ghost = NULL;
volatile double* pradius_ghost = NULL;
volatile int* pfrontCarNum_ghost = NULL;
volatile double* pfrontDist_ghost = NULL;

void *shm = NULL;

//END


/*
 * Function
 *	main
 *
 * Description
 *	LINUX entry point of TORCS
 *
 * Parameters
 *
 *
 * Return
 *
 *
 * Remarks
 *
 */
int
main(int argc, char *argv[])
{

    // Block certain signals?
    /*
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    pthread_sigmask(SIG_BLOCK, &mask,nullptr);
    */

	const char *raceconfig = "";

	init_args(argc, argv, &raceconfig);

    volatile struct shared_use_st *shared = nullptr;
    int shmid;
    fprintf(stderr, "shm key: %i\n",key);
    shmid = shmget((key_t)key, sizeof(shared_use_st), 0666|IPC_CREAT);
    if(shmid == -1){
        fprintf(stderr, "shmget failed\n");
        fprintf(stderr, "error: %s",strerror(errno));
        exit(EXIT_FAILURE);
    }

    shm = shmat(shmid, 0, 0);
    if(shm == (void*)-1)
    {
        fprintf(stderr, "shmat failed\n");
        exit(EXIT_FAILURE);
    }
    // printf("\n********** TORCS: Memory sharing started, attached at %X **********\n \n", shm);
    // set up shared memory
    shared = (struct shared_use_st*)shm;
    shared->pid = getpid();

    sem_written=&shared->written;
    sem_read=&shared->read;
    pdata=shared->data;
    pdata_left = shared->data_left;
    pdata_right= shared->data_right;
    pdata_between = shared->data_between;
    pisEnd=&shared->isEnd;
    pdist=&shared->dist;

    psteerCmd_ghost=&shared->steerCmd;
    paccelCmd_ghost=&shared->accelCmd;
    pbrakeCmd_ghost=&shared->brakeCmd;

    pspeed_ghost = &shared->speed;
    pangle_in_rad_ghost = &shared->angle_in_rad;
    pdamage_ghost = &shared->damage;
    ppos_ghost = &shared->pos;
    psegtype_ghost = &shared->segtype;
    pradius_ghost = &shared->radius;
    pfrontCarNum_ghost = &shared->frontCarNum;
    pfrontDist_ghost = &shared->frontDist;

	LinuxSpecInit();			/* init specific linux functions */

    if(is_re_gui_without_select){
        GfScrInit(argc,argv);
        ssgInit();
        GfInitClient();
        ReGuiWithoutSelect(raceconfig);
    }

	if(strlen(raceconfig) == 0) {
		GfScrInit(argc, argv);	/* init screen */
		TorcsEntry();			/* launch TORCS */
		glutMainLoop();			/* event loop of glut */
	} else {
		// Run race from console, no Window, no OpenGL/OpenAL etc.
		// Thought for blind scripted AI training
		ReRunRaceOnConsole(raceconfig);
	}

	return 0;					/* just for the compiler, never reached */
}
