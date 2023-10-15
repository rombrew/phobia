#ifndef _H_LSE_
#define _H_LSE_

/* Define the maximal full size to be allocated. This is the sum of \x and \z
 * row-vector sizes.
 * */
#define LSE_FULL_MAX			10

/* Define the maximal number of cascades. A large value gives greater precision
 * on large datasets but consumes more memory. Reasonable values are from 2 to 4.
 * */
#define LSE_CASCADE_MAX			2

/* Define whether to use fast Givens transformation in QR update. Typical this
 * is useful for fairly large matrix sizes. Also consumes a few of memory.
 * */
#define LSE_FAST_TRANSFORM		0

/* Define native floating-point type to use inside of LSE.
 * */
typedef float		lse_float_t;

typedef struct {

	/* The size of the upper-triangular matrix.
	 * */
	int		len;

	/* The number of data rows that matrix keep.
	 * */
	int		keep;

	/* The marker of lazy merging.
	 * */
	int		lazy;

	/* Content of the upper-triangular matrix.
	 * */
	lse_float_t	*m;

#if LSE_FAST_TRANSFORM != 0
	/* Content of the scale diagonal matrix.
	 * */
	lse_float_t	*d;
#endif /* LSE_FAST_TRANSFORM */
}
lse_upper_t;

typedef struct {

	/* The length of the row-vector.
	 * */
	int		len;

	/* Content of the row-vector.
	 * */
	lse_float_t	*m;
}
lse_row_t;

typedef struct {

	/* Cascades in actual use.
	 * */
	int		n_cascades;

	/* Input DATA sizes.
	 * */
	int		n_len_of_x;
	int		n_len_of_z;

	/* Processed DATA sizes.
	 * */
	int		n_threshold;
	int		n_total;

	/* \R(i) is row-major upper-triangular matrix array with block
	 * structure as shown. We store only the upper triangular elements.
	 *
	 *                                 [0 1 2 3]
	 *                                 [  4 5 6]
	 *        [Rx  S ]                 [    7 8]
	 * R(i) = [0   Rz],        (ex.) = [      9].
	 *
	 * Rx - upper-triangular matrix size of \x,
	 * Rz - upper-triangular matrix size of \z,
	 * S  - rectangular matrix size of \x by \z.
	 *
	 * */
	lse_upper_t	rm[LSE_CASCADE_MAX];

	/* LS solution \b is a column-major matrix.
	 * */
	lse_row_t	sol;

	/* Standard deviation of \z row-vector.
	 * */
	lse_row_t	std;

	/* Approximate extremal singular values of \Rx.
	 * */
	struct {

		lse_float_t	min;
		lse_float_t	max;
	}
	esv;

	/* We allocate the maximal amount of memory.
	 * */
	lse_float_t	vm[LSE_CASCADE_MAX * LSE_FULL_MAX * (LSE_FULL_MAX + 1) / 2

#if LSE_FAST_TRANSFORM != 0
			 + LSE_CASCADE_MAX * LSE_FULL_MAX
#endif /* LSE_FAST_TRANSFORM */

			 + LSE_FULL_MAX * LSE_FULL_MAX / 4 + LSE_FULL_MAX / 2 + 1];
}
lse_t;

/* The function determines the size of LSE structure. So you can allocate LSE
 * structure dynamically with size returned.
 * */
int lse_getsize(int n_cascades, int n_full);

/* The function construct the instance of LSE.
 * */
void lse_construct(lse_t *ls, int n_cascades, int n_len_of_x, int n_len_of_z);

/* The function updates \R with a new data row-vector \xz which contains \x and
 * \z concatenated. We does QR update of \R by orthogonal transformation. Note
 * that the contents of \xz will be destroyed.
 * */
void lse_insert(lse_t *ls, lse_float_t *xz);

/* The function introduces ridge regularization with \la. Most reasonable \la
 * value is \n_len_of_x * \esv.max * \machine_epsilon.
 * */
void lse_ridge(lse_t *ls, lse_float_t la);

/* The function scales all cascades of \R with forgetting factor \la. It is
 * reasonable to use this function with only one cascade allocated.
 * */
void lse_forget(lse_t *ls, lse_float_t la);

/* The function calculates the final LS solution \b.
 * */
void lse_solve(lse_t *ls);

/* The function calculates standard deviation of \z.
 * */
void lse_std(lse_t *ls);

/* The function estimates the approximate largest and smallest singular values
 * of \Rx in \n_approx iterations. A rather computationally heavy function if
 * \n_approx is large (most reasonable is 4). You can calculate the conditional
 * number or detect a rank deficiency based on retrieved values.
 *
 * WARNING: You need to provide enough memory to use this function. We use
 * empty cascades of \R as temporal storage.
 *
 * */
void lse_esv(lse_t *ls, int n_approx);

#endif /* _H_LSE_ */

