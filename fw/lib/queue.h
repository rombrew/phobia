#ifndef _H_QUEUE_
#define _H_QUEUE_

/* Wait-free MPSC queue.
 * */

#include "hal/hal.h"
#include "lib/types.h"

typedef struct queue_node queue_node_t;

struct queue_node {

	queue_node_t	*next;
};

typedef struct {

	queue_node_t	*head, *tail;
	queue_node_t	stub;
}
queue_t;

inline void 
queue_init(queue_t *q)
{
	q->head = &q->stub;
	q->tail = &q->stub;
	q->stub.next = 0;
}

inline void
queue_put(queue_t *q, void *vnode)
{
	queue_node_t	*tail, *node = vnode;

	node->next = NULL;

	do {
		tail = (void *) hal_exclusive_load(&q->tail);
	}
	while (hal_exclusive_store(&q->tail, node));

	hal_memory_barrier();

	tail->next = node;
}

inline void *
queue_get(queue_t *q)
{
	queue_node_t	*head = q->tail;
	queue_node_t	*next = head->next;

	if (next) {

		q->head = next;
		return head;
	}

	return head;
}

#endif /* _H_QUEUE_ */

