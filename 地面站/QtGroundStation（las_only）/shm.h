#ifndef _SHM_H_
#define _SHM_H_

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>


union semun
{
	int val;
	struct semid_ds *buf;
	unsigned short *array;
};

void init_a_semaphore(int sid,int semnum,int initval);


int semaphore_P(int sem_id);


int semaphore_V(int sem_id);


#endif
