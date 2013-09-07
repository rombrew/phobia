#ifndef _H_MUTEX_
#define _H_MUTEX_

#include "lib/list.h"

typedef struct {

	int		lock;
	list_node_t	list;
}
mutex_t;

void mutex_lock(mutex_t *mu);
int mutex_try_lock(mutex_t *mu);
void mutex_unlock(mutex_t *mu);

#endif /* _H_MUTEX_ */

