#ifndef _H_KLIB_
#define _H_KLIB_

float sqrtf(float x);

void abc2dq(const float abc[3], const float e[2], float dq[2]);
void dq2abc(const float dq[2], const float e[2], float abc[3]);

#endif /* _H_KLIB_ */

