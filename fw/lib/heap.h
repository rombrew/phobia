#ifndef _H_HEAP_
#define _H_HEAP_

#include "lib/list.h"

#define HEAP_MAGIC		0x55555555

typedef struct {

	void		*beg;
	void		*end;

	list_node_t	free;
	list_node_t	busy;
}
heap_t;

typedef struct {

	list_node_t	node;
	int		size;
	int		magic;
}
heap_block_t;

typedef struct {

	int		magic;
	heap_block_t	*block;
}
heap_block_end_t;

heap_t *heap_init(void *beg, void *end);

void *halloc(heap_t *h, int sz);
void hfree(void *p);

#endif /* _H_HEAP_ */

