#include "lib/heap.h"
#include "lib/list.h"
#include "lib/types.h"

static inline void *
ptr_round_up(void *p)
{
	return ((u32_t) p + 7UL) & ~7UL;
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

void *halloc(int sz, int n)
{
	heap_block_t	*l, *blk;

	sz = PTR_ROUND_UP(sz);

	list_for_each(&heap.free, l) {

		if (block->size >)
	}
}

void hfree(void *p)
{
}

