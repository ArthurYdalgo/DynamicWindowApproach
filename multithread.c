
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> //Header file for sleep(). man 3 sleep for details.
#include <pthread.h>

typedef struct threadedPlanningArgs
{
    int thread_id;
    int value;
} threadedPlanningArgs;

float *threadedCosts;

int use_threads = 0;

// A normal C function that is executed as a thread
// when its name is specified in pthread_create()
void *myThreadFun(void *vargp)
{
    sleep(1);
    threadedCosts[((struct threadedPlanningArgs *)vargp)->thread_id] = 10 + ((struct threadedPlanningArgs *)vargp)->value;
    printf("Calculated from Thread %d\n", ((struct threadedPlanningArgs *)vargp)->thread_id);
    return NULL;
}

int main()
{
    int thread_count = 5;
    if(use_threads){
        printf("Before Thread\n");
    }

    pthread_t *pthreads = (pthread_t*)malloc(thread_count * sizeof(pthread_t));
    threadedCosts = malloc(thread_count * sizeof(float));
    threadedPlanningArgs *threadedArgs = (threadedPlanningArgs *)malloc(thread_count * sizeof(threadedPlanningArgs));
    for (int i=0; i < thread_count; i++)
    {
        // pthreads[i] = (pthread_t*)malloc(sizeof(pthread_t));
        // threadedArgs[i] = (threadedPlanningArgs)malloc(sizeof(threadedPlanningArgs));
        threadedArgs[i].thread_id = i;
        threadedArgs[i].value = thread_count + i;
        if(use_threads){
            pthread_create(&pthreads[i], NULL, myThreadFun, &threadedArgs[i]); 
        }else{
            myThreadFun(&threadedArgs[i]);
        }
    }

    if(use_threads){
        printf("\n\nJoiningThreads\n\n");
        for (int i=0; i < thread_count; i++)
        {
            pthread_join(pthreads[i], NULL);
        }
    }
    for (int i=0; i < thread_count; i++)
    {
        printf("Cost: %f\n", threadedCosts[i]);
    }
    exit(0);
}
