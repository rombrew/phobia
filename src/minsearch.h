#ifndef _H_MINSEARCH_
#define _H_MINSEARCH_

/* Define the maximal function argument SIZE to be allocated.
 * */
#define MINSEARCH_MAX		8

typedef struct {

	/* The number of variables to search for.
	 * */
	int		n_size_of_x;

	/* User defined pointer to pass into the target function call (OPTIONAL).
	 * */
	void		*link;

	/* Data range to construct the initial simplex.
	 * */
	float		x0_range;

	/* Data constraint throughout the entire search.
	 * */
	float		x_maximal;

	/* Target function.
	 * */
	float		(* pfun) (void *link, const float *xarg);

	/* Step function called at each step (OPTIONAL).
	 * */
	void		(* pstep) (void *link);

	/* Data points of the simplex and function values.
	 * */
	float		xarg[MINSEARCH_MAX * (MINSEARCH_MAX + 1)];
	float		xnul[MINSEARCH_MAX];
	float		xref[MINSEARCH_MAX];
	float		xcon[MINSEARCH_MAX];
	float		fval[MINSEARCH_MAX + 1];

	/* Index of the BEST and WORST point of the simplex.
	 * */
	int		n_best;
	int		n_worst;
	int		n_last;

	/* Number of current step.
	 * */
	int		n_step;

	/* Tolerance of the simplex.
	 * */
	float		x_tol;

	/* Termination criteria constants.
	 * */
	int		n_final_step;
	float		x_final_tol;
}
minproblem_t;

/* The Nelder-Mead simplex algorithm (a derivative-free optimization method).
 * */
void minsearch(minproblem_t *m);
void minsolution(minproblem_t *m, float *xsol);

#endif /* _H_MINSEARCH_ */

