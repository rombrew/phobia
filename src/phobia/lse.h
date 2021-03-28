#ifndef _H_LSE_
#define _H_LSE_

/* Define the maximal full size to be allocated. This is the sum of \x and \z
 * row-vector sizes.
 * */
#define LSE_FULL_MAX		10

/* Define the number of cascades. Large value gives a more precision on large
 * datasets but consumes more memory. Reasonable values are from 2 to 5.
 * */
#define LSE_CASCADE_MAX		2

/* Define native floating-point type to use inside of LSE.
 * */
typedef float		lse_float_t;

typedef struct {

	/* The number of data rows that matrix keep.
	 * */
	int		n_keep;

	/* Data of upper-triangular matrix.
	 * */
	lse_float_t	*m;
}
lse_triu_t;

typedef struct {

	/* Input data sizes.
	 * */
	int		n_size_of_x;
	int		n_size_of_z;
	int		n_full;

	int		n_threshold;
	int		n_total;

	/* \R(i) is row-major upper-triangular matrix with block structure as
	 * shown. We aggregate input data into \R(0). After we got enough data
	 * we merge it into \R(1) and so on. This is called the cascading
	 * update.
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
	lse_triu_t	triu[LSE_CASCADE_MAX];

	/* LS solution \b is a column-major matrix.
	 *
	 * b = Rx \ S.
	 *
	 * */
	lse_float_t	*b;

	/* LS standard deviation of \z row-vector.
	 *
	 * e(i) = norm(Rz(:,i)) / sqrt(n_total).
	 *
	 * */
	lse_float_t	*e;

	/* We allocate the maximal amount of memory.
	 * */
	lse_float_t	vm[LSE_CASCADE_MAX * LSE_FULL_MAX * (LSE_FULL_MAX + 1) / 2];
}
lse_t;

/* The function configures the instance of LSE.
 * */
void lse_initiate(lse_t *lse, int n_size_of_x, int n_size_of_z);

/* The function calculates the final LS solution \b.
 *
 * NOTE: The cascade structure is being collapsed so do it only once after all
 * data is accepted to get the best precision.
 *
 * NOTE: The memory for \b and \e is allocated in place of \R(0) in normal case
 * when total number of taken rows is greater than size of \x.
 *
 * */
void lse_finalise(lse_t *lse);

/* The function takes a new data row-vector \v which contains \x and \z
 * concatenated.
 *
 * R(0) = cholupdate(R(0), [x z]).
 *
 * */
void lse_insert(lse_t *lse, lse_float_t *v);

#endif /* _H_LSE_ */

