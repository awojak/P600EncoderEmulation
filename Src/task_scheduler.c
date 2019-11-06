/*
 * task_scheduler.c
 *
 *  Created on: 22.10.2019
 *      Author: Neo
 */

#include "task_scheduler.h"

//Tick variable
volatile unsigned int tick = 0;

/**
 * Initialize scheduler before add tasks
 */
void SchedulerInit(SchedulerTasks* sts)
{
	sts->tasks_count = 0;
}

void TaskTick()
{
	//increment task tick
	tick++;
}

TaskCreateState TaskCreate(SchedulerTasks* sts, Task *t, void *fun_ptr, unsigned char priority)
{
	unsigned char index;
	if(sts->tasks_count < MAX_TASKS)
	{
		t->active = TaskInactive;
		t->do_task = 0;
		t->fun_ptr = fun_ptr;
		t->period = -1;
		t->priority = priority;
		t->next_exe = 0;
		//increment tasks counter
		sts->tasks_count++;
		index = sts->tasks_count;
		//store task reference
		sts->tasks[index] = t;

		return CreatedTaskSuccessfully;
	} else
	{
		//No more space for new tasks
		return CreatedTaskUnsuccessfully;
	}
}

/**
 * Start task
 */
void TaskStart(Task* t, int period)
{
	t->active = TaskActive;
	t->period = period;
	t->next_exe = 0;
}

/**
 * Start event task, not periodically
 */
void TaskEventStart(Task* t)
{
	t->active = TaskActive;
	t->period = -1;
}

/**
 * Do immediately task
 */
void TaskEventTrigger(Task* t)
{
	t->do_task = 1;
}
/**
 * Stop task
 */
void TaskStop(Task* t)
{
	t->active = TaskInactive;
}
/**
 * Scheduler for tasks
 */
void Scheduler(SchedulerTasks* sts)
{
	//Infinity loop
	while(1) {
	// TODO: Implement priority
	int i;
	for(i=0; i<sts->tasks_count; i++)
	{
		if(sts->tasks[i]->active == TaskActive)
		{
			if((tick >= sts->tasks[i]->next_exe) && sts->tasks[i]->period>=0)
			{
				//Do task
				sts->tasks[i]->fun_ptr();
				//Save next execution
				sts->tasks[i]->next_exe = tick + sts->tasks[i]->period;

			}
			if(sts->tasks[i]->do_task)
			{
				//Do task immediately
				sts->tasks[i]->fun_ptr();
				//Reset do task to wait for next event
				sts->tasks[i]->do_task = 0;
			}
		}
	}
	}
}
