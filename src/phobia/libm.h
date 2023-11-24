#ifndef _H_LIB_M_
#define _H_LIB_M_

#define M_EPSILON		1.19209E-7f
#define M_PI_F			3.14159265f
#define M_2_PI_F 		6.28318531f
#define M_LOG_E 		0.69314718f
#define M_LOG_10		0.30103000f

static inline float m_fabsf(float x) { return __builtin_fabsf(x); }
static inline float m_sqrtf(float x) { return __builtin_sqrtf(x); }

int m_isfinitef(float x);

void m_rotatef(float x[2], float r);
void m_rsumf(float *sum, float *rem, float x);
float m_wrapf(float x);

float m_atan2f(float y, float x);
float m_sinf(float x);
float m_cosf(float x);
float m_log2f(float x);
float m_log10f(float x);
float m_logf(float x);
float m_exp2f(float x);
float m_exp10f(float x);
float m_expf(float x);

void m_la_eigf(const float a[3], float v[4], int m);

typedef struct {

	float		seed[4];
	int		nb;
}
m_seed_t;

void m_lf_randseed(m_seed_t *lf, int seed);
float m_lf_urandf(m_seed_t *lf);
float m_lf_gaussf(m_seed_t *lf);

#endif /* _H_LIB_M_ */

