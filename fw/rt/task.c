#include "rt/task.h"
#include "hal/hal.h"
#include "lib/list.h"
#include "lib/types.h"

typedef struct {

	/* Pending task queue.
	 * */
	tcb_t		*top[TASK_PRI_MAX];
	u32_t		top_mask;

	/* Tail of wakeup list.
	 * */
	tcb_t		*rt_wakeup_tail;

	/* A list of all tasks.
	 * */
	list_node_t	list;
}
task_global_t;

tcb_t			*rt_task;
static task_global_t	rt;

enum {
	TASK_REQ_NULL		= 0,
	TASK_REQ_SUSPEND	= 1,
	TASK_REQ_RESUME		= 2,
};

static void
rt_idle_task()
{
	do {
		hal_cpu_relax();
	}
	while (1);
}

void rt_task_init()
{
	tcb_t		*tcb;
	int		i;

	rt_task = NULL;
	rt.top_mask = 0;

	for (i = 0; i < TASK_PRI_MAX; ++i)
		rt.top[i] = NULL;

	kqu_init(&rt.kqu);
	list_init(&rt.list);

	tcb = rt_task_new(rt_idle_task, "idle",
			TASK_PRI_IDLE, Ki / 4);
	rt_task_kick(tcb);

	rt_task = tcb;
	hal_yield();
}


void task_tcb_insert(tcb_t *tcb)
{
}

void task_tcb_remove(tcb_t *tcb)
{
	t = rt_top + tcb->pri;

	if (!list_is_empty(tcb)) {

		if (tcb == *t)
			*t = list_prev(t);
	}
	else {
		rt_top_mask &= ~((1UL << 31) >> tcb->pri);
		*t = NULL;
	}

	list_remove(tcb);
}

static void
task_tcb_remove(tcb_t *tcb)
{
	
}

static void 
rt_task_service(tcb_t *tcb)
{
	tcb_t		**t;

	switch (tcb->req) {

		case TASK_REQ_NULL:
			break;

		case TASK_REQ_WAIT:

			

			break;

		case TASK_REQ_KICK:

			t = rt_top + tcb->pri;

			if (*t != NULL)

				list_insert_prev(*t, tcb);
			else {
				rt_top_mask |= ((1UL << 31) >> tcb->pri);
				*t = list_up(tcb);
			}

			break;
	}
}

void rt_task_shedule()
{
	tcb_t		*r, **t;
	int		i;

	/* Handle all requests.
	 * */
	while ((r = kqu_get(&rt.kqu)) != NULL) {

		rt_task_service(r);
	}

	/* .
	 * */
	for (i = 0; i < TASK_PRI_MAX; ++i) {

		t = rt_top + i;

		if (*t != NULL) {

			*t = list_next(*t);
			rt_task = *t;

			rt_task->trun++;

			break;
		}
	}
}

static void
rt_task_queue(tcb_t *tcb)
{
}

tcb_t *rt_task_new(void *entry,
		const char *id,
		int pri,
		int ss)
{
	tcb_t		*tcb;

	tcb = halloc(sizeof(tcb_t), HEAP_RAM0);

	if (tcb != NULL) {

		tcb->sblk = halloc(ss, HEAP_RAM0);

		if (tcb->sblk != NULL) ;
		else {
			hfree(tcb);

			return NULL;
		}
	}
	else
		return NULL;

	tcb->sp = ((u8_t *) tcb->sblk) + ss;

	tcb->pri = pri;
	tcb->id = id;

	list_insert_prev(&rt_task_list, &tcb->list);

	tcb->trun = 0;

	return tcb;
}

void rt_task_suspend(tcb_t *tcb)
{
	if (tcb->req != TASK_REQ_NULL)
		hal_yield();

	tcb->req = TASK_REQ_WAIT;
	kqu_put(&rt.kqu, tcb);
}

void rt_task_resume(tcb_t *tcb)
{
	tcb->req = TASK_REQ_WAIT;
	kqu_put(&rt.kqu, tcb);
}

void rt_task_yield()
{
	hal_yield();
}

