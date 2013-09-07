#ifndef _H_TASK_
#define _H_TASK_

#include "lib/list.h"
#include "lib/types.h"

#define TASK_PRI_MAX			32
#define TASK_PRI_IDLE			(TASK_PRI_MAX - 1)

typedef struct {

	/* Core part of TCB.
	 * */
	list_node_t	node;
	void		*sp;
	int		pri;
	int		ste;

	/* Some useful stuff.
	 * */
	const char	*id;
	list_node_t	list;
	void		*stack;
	int		runc;
}
tcb_t;

enum {
	TASK_STATE_NULL		= 0,
	TASK_STATE_SUSPEND,
	TASK_STATE_WAIT,
	TASK_STATE_PENDING,
	TASK_STATE_RUNING,
};

typedef struct {

	int		flag;
	list_node_t	list;
}
signal_t;

void task_init();

void task_tcb_init(tcb_t *tcb,
		void *entry,
		void *stack,
		const char *id,
		int pri);

void task_tcb_kill(tcb_t *tcb);

void task_tcb_insert(tcb_t *tcb);
void task_tcb_remove(tcb_t *tcb);

tcb_t *task_search(const char *id);
void task_suspend(tcb_t *tcb);
void task_resume(tcb_t *tcb);
void task_wait(signal_t *si);
void task_signal(signal_t *si);
void task_yield();

void task_request(tcb_t *tcb);

#endif /* _H_TASK_ */

