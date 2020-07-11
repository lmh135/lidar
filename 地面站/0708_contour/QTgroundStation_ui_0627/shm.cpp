#include "shm.h"



void init_a_semaphore(int sid,int semnum,int initval)
{
    union semun semopts;
    semopts.val = initval;
    semctl(sid,semnum,SETVAL,semopts);
}

int semaphore_P(int sem_id)
{
    struct sembuf sb;
    sb.sem_num = 0;
    sb.sem_op = -1;
    sb.sem_flg = SEM_UNDO;

    if(semop(sem_id,&sb,1) == -1)
    {
        printf("sem p fail!");
        return 0;
    }
    return 1;
}

int semaphore_V(int sem_id)
{
    struct sembuf sb;
    sb.sem_num = 0;
    sb.sem_op = 1;
    sb.sem_flg = SEM_UNDO;

    if(semop(sem_id,&sb,1) == -1)
    {
        printf("sem v fail!");
        return 0;
    }
    return 1;
}
