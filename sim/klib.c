#include "klib.h"

void abc2dq(const float abc[3], const float e[2], float dq[2])
{
	float		xy[2];

	xy[0] = abc[0];
	xy[1] = 0.57735027f * abc[0] + 1.1547005f * abc[1];

	dq[0] = e[0] * xy[0] + e[1] * xy[1];
	dq[1] = -e[1] * xy[0] + e[0] * xy[1];
}

void dq2abc(const float dq[2], const float e[2], float abc[3])
{
	float		xy[2];

	xy[0] = e[0] * dq[0] - e[1] * dq[1];
	xy[1] = e[1] * dq[0] + e[0] * dq[1];

	abc[0] = xy[0];
	abc[1] = -0.5f * xy[0] + 0.86602540f * xy[1];
	abc[2] = -0.5f * xy[0] - 0.86602540f * xy[1];
}


