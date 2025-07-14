#ifndef _H_LSE_
#define _H_LSE_

/* Define the maximal full size to be allocated. This is the sum of \x and \z
 * row-vector sizes.
 * */
#define LSE_FULL_MAX			8

/* Define the maximal number of cascades. A large value gives greater precision
 * on large datasets but consumes more memory. Reasonable values are from 2 to 4.
 * */
#define LSE_CASCADE_MAX			2

/* Define whether to use fast Givens transformation in QR update. Typical this
 * is useful for fairly large matrix sizes. Also consumes a few of memory.
 * */
#define LSE_FAST_GIVENS			0

/* Define native floating-point type to use inside of LSE.
 * */
typedef float		lse_float_t;

typedef struct {

	/* The size of the upper-triangular matrix.
	 * */
	int		len;

	/* The number of rows in actual use.
	 * */
	int		rows;

	/* The number of data rows that matrix keeps.
	 * */
	int		keep;

	/* The marker of lazy merging.
	 * */
	int		lazy;

	/* Content of the upper-triangular matrix.
	 * */
	lse_float_t	* restrict m;

#if LSE_FAST_GIVENS != 0
	/* Content of the scale diagonal matrix.
	 * */
	lse_float_t	* restrict d;
#endif /* LSE_FAST_GIVENS */
}
lse_upper_t;

typedef struct {

	/* The length of the row-vector.
	 * */
	int		len;

	/* Content of the row-vector.
	 * */
	lse_float_t	* restrict m;
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

	/* \rm(i) is row-major upper-triangular matrix array with block
	 * structure as shown. We store only the upper triangular elements.
	 *
	 *                                    [0 1 2 3]
	 *                                    [  4 5 6]
	 *          [ RX  S  ]                [    7 8]
	 * \rm(i) = [ 0   RZ ],       (ex.) = [      9].
	 *
	 * \RX - upper-triangular matrix size of \x,
	 * \RZ - upper-triangular matrix size of \z,
	 * \S  - rectangular matrix size of \x by \z.
	 *
	 * */
	lse_upper_t	rm[LSE_CASCADE_MAX];

	/* LS solution \b is a column-major matrix.
	 * */
	lse_row_t	sol;

	/* Standard deviation of \z row-vector.
	 * */
	lse_row_t	std;

	/* Extremal singular values of \RX.
	 * */
	struct {

		lse_float_t	min;
		lse_float_t	max;
	}
	esv;

	/* We allocate the maximal amount of memory.
	 * */
	lse_float_t	vm[LSE_CASCADE_MAX * LSE_FULL_MAX * (LSE_FULL_MAX + 1) / 2

#if LSE_FAST_GIVENS != 0
			 + LSE_CASCADE_MAX * LSE_FULL_MAX
#endif /* LSE_FAST_GIVENS */

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

/* The function indicates the LSE instance that you will not calculate the
 * standard deviation. This allows QR updates to be made faster.
 * */
void lse_nostd(lse_t *ls);

/* The function updates \rm with a new data row-vector \xz which contains \x
 * and \z concatenated. We are doing QR update of \rm by orthogonal
 * transformation. Note that the contents of \xz will be destroyed.
 * */
void lse_insert(lse_t *ls, lse_float_t *xz);

/* The function introduces ridge regularization with \la. Most reasonable \la
 * value is \n_len_of_x * \esv.max * \machine_epsilon.
 * */
void lse_ridge(lse_t *ls, lse_float_t la);

/* The function scales all cascades of \rm with forgetting factor \la. It is
 * reasonable to use this function with only one cascade allocated.
 * */
void lse_forget(lse_t *ls, lse_float_t la);

/* The function updates \rm of \ls instance with data rows from \rm of \lb
 * instance. This is a merge of two LSE instances.
 * */
void lse_merge(lse_t *ls, lse_t *lb);

/* The function calculates the final LS solution \b.
 * */
void lse_solve(lse_t *ls);

/* The function calculates standard deviation of \z.
 * */
void lse_std(lse_t *ls);

/* The function estimates the approximate largest and smallest singular values
 * of \RX in \n_approx iterations. A rather computationally heavy function if
 * \n_approx is large (most reasonable is 2). You can calculate the conditional
 * number or detect a rank deficiency based on retrieved values.
 *
 * WARNING: You need to provide enough memory to use this function. We use
 * empty cascades of \rm as temporal storage.
 *
 * */
void lse_esv(lse_t *ls, int n_approx);

#endif /* _H_LSE_ */

