#ifndef QUATERNION_H_
#define QUATERNION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

namespace rov_io {


int quaternion_multiply_array(double a[4], double b[4], double c[4])
{
	if(a==NULL||b==NULL||c==NULL){
		printf("ERROR: in rc_quaternion_multiply_array, received NULL pointer\n");
		return -1;
	}

	int i,j;
	double tmp[4][4];
	// construct tmp matrix
	tmp[0][0] =  a[0];
	tmp[0][1] = -a[1];
	tmp[0][2] = -a[2];
	tmp[0][3] = -a[3];
	tmp[1][0] =  a[1];
	tmp[1][1] =  a[0];
	tmp[1][2] = -a[3];
	tmp[1][3] =  a[2];
	tmp[2][0] =  a[2];
	tmp[2][1] =  a[3];
	tmp[2][2] =  a[0];
	tmp[2][3] = -a[1];
	tmp[3][0] =  a[3];
	tmp[3][1] = -a[2];
	tmp[3][2] =  a[1];
	tmp[3][3] =  a[0];
	// multiply
	for(i=0;i<4;i++){
		c[i]=0.0;
		for(j=0;j<4;j++) c[i]+=tmp[i][j]*b[j];
	}
	return 0;
}

int quaternion_rotate_array(double p[4], double q[4])
{
	double conj[4], tmp[4];
	if(p==NULL||q==NULL){
		printf("ERROR: in rc_quaternion_rotate_array, received NULL pointer\n");
		return -1;
	}
	// make a conjugate of q
	conj[0]= q[0];
	conj[1]=-q[1];
	conj[2]=-q[2];
	conj[3]=-q[3];
	// multiply tmp=pq*
	quaternion_multiply_array(p,conj,tmp); 
	// multiply p'=q*tmp
	quaternion_multiply_array(q,tmp,p);
	return 0;
}


int quaternion_rotate_vector_array(double v[3], double q[4])
{
	double vq[4];
	if(v==NULL||q==NULL){
		printf("ERROR: in rc_quaternion_rotate_vector_array, received NULL pointer\n");
		return -1;
	}
	// duplicate v into a quaternion with 0 real part
	vq[0]=0.0;
	vq[1]=v[0];
	vq[2]=v[1];
	vq[3]=v[2];
	// rotate quaternion vector
	quaternion_rotate_array(vq, q);
	// populate v with result
	v[0]=vq[1];
	v[1]=vq[2];
	v[2]=vq[3];
	return 0;
}


} //namespace

#ifdef __cplusplus
}
#endif

#endif //QUATERNION_H_