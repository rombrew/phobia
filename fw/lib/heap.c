#include "lib/heap.h"
#include "lib/list.h"
#include "lib/types.h"

static inline void *
ptr_round_up(void *p)
{
	return (void *) (((u32_t) p + 7UL) & ~7UL);
}

static inline void *
block_to_ptr(heap_block_t *blk)
{
	return blk + 1;
}

static inline heap_block_t *
ptr_to_block(void *p)
{
	return ((heap_block_t *) p) - 1;
}

heap_t *heap_init(void *beg, void *end)
{
}

void *halloc(heap_t *h, int sz)
{
	heap_block_t	*l, *blk;

	sz = ptr_round_up(sz);

	list_for_each(&h->free, l) {

		//if (block->size >)
	}
}

void hfree(void *p)
{
}

