#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "svm.h"
#include <iostream>


struct svm_parameter param;
struct svm_problem prob;
struct svm_model* model;



svm_node createSvmNode(int idx, double val) {
        svm_node svx;    
        svx.index = idx;
        svx.value = val;
        return svx;
}

svm_node* createSNodeArray(double px, double py, double pz) {
    svm_node* arr = new svm_node[4];
    arr[0] = createSvmNode(1,px);
    arr[1] = createSvmNode(2,py);
    arr[2] = createSvmNode(3,pz);
    arr[3] = createSvmNode(-1,pz);
    return arr;
}

svm_problem createSvmProblem() {
    struct svm_problem prx;
    prx.l = 4;
    double* y = new double[4];
    y[0] = 1; y[1]=1; y[2]=2; y[3]=2;
    prx.y = y;
    struct svm_node** data = new struct svm_node*[4];

    data[0] = createSNodeArray(1,1,0);
    data[1] = createSNodeArray(2,2,0);
    data[2] = createSNodeArray(3,3,0);
    data[3] = createSNodeArray(4,4,0);
    prx.x = data;
    return prx;
}

void getHyperplanes(const svm_model* mx) {
	const double * const *sv_coef = mx->sv_coef;
	const svm_node * const *SV = mx->SV;

   for (int i=0;i<mx->l;++i) {
       const svm_node *p = SV[i];
       std::cout << "SV Coef : " << sv_coef[0][i] << std::endl;
        while(p->index != -1)
        {
            printf("%d:%.8g ",p->index,p->value);
            p++;
        }
        printf("\n");
   }

}



int main() {
	const char *error_msg;
    param.svm_type = C_SVC;
    param.kernel_type = LINEAR;
    param.degree = 3;
    param.cache_size = 64;
    param.eps = 0.01;
    param.C = 10000;
    param.nr_weight= 0;

    prob = createSvmProblem();
	error_msg = svm_check_parameter(&prob,&param);

	if(error_msg)
	{
		fprintf(stderr,"ERROR: %s\n",error_msg);
		exit(1);
	}
    model = svm_train(&prob,&param);
    getHyperplanes(model);
    char model_file_name[] = "tmod.txt";
    if(svm_save_model(model_file_name,model))
    {
        fprintf(stderr, "can't save model to file %s\n", model_file_name);
        exit(1);
    }    
}

