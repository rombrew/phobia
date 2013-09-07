#ifndef _H_LIST_
#define _H_LIST_

typedef struct list_node list_node_t;

struct list_node {

	list_node_t	*next;
	list_node_t	*prev;
};

inline void *
list_init(void *vhead)
{
	list_node_t	*head = vhead;

	head->next = head;
	head->prev = head;

	return head;
}

inline void *
list_insert_prev(void *vhead, void *vnode)
{
	list_node_t	*head = vhead, *node = vnode;
	list_node_t	*prev;

	prev = head->prev;
	head->prev = node;
	prev->next = node;
	node->prev = prev;
	node->next = head;

	return node;
}

inline void *
list_insert_next(void *vhead, void *vnode)
{
	list_node_t	*head = vhead, *node = vnode;
	list_node_t	*next;

	next = head->next;
	head->next = node;
	next->prev = node;
	node->next = next;
	node->prev = head;

	return node;
}

inline void *
list_remove(void *vnode)
{
	list_node_t	*node = vnode;
	list_node_t	*next, *prev;

	next = node->next;
	prev = node->prev;
	next->prev = prev;
	prev->next = next;

	return node;
}

inline void *
list_next(void *vnode)
{
	list_node_t	*node = vnode;

	return node->next;
}

inline void *
list_prev(void *vnode)
{
	list_node_t	*node = vnode;

	return node->prev;
}

inline int
list_is_empty(void *vhead)
{
	list_node_t	*head = vhead;

	return head->next == head;
}

#define list_for_each(head, node)		\
	for (	node = list_next(head);		\
		node != (void *) (head);	\
		node = list_next(node))

#define list_for_each_safe(head, node, next)	\
	for (	node = list_next(head), next = list_next(node);	\
		node != (void *) (head);			\
		node = next, next = list_next(node))

#endif /* _H_LIST_ */

