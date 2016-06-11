/***********************************************************************************
	PROGRAM FOR DEFUZZIFICATION USING HEIGHT DEFUZZIFICATION METHOD
	WRITTEN BY:GYANDA SARAN,NML,JAMSHEDPUR
	DATE:12|11|2003
************************************************************************************/

#include"stdio.h"
 
defuz(int var)
{
	int i,z;
	double peak[5],load[10];
	double z_star,u=0.0,v=0.0;
	FILE *fp1,*fp2;

	//printf("IN DEFUZZIFICATION MODULE\n");

	fp1 = fopen("module4.txt","r");
	
	fscanf(fp1,"%d",&z);

	for(i=0;i<z;i++)
	{
		fscanf(fp1,"%lf %lf",&load[i],&peak[i]);
		//printf("load=%lf\tpeak=%lf",load[i],peak[i]);
		u=peak[i]*load[i]+u;                        //Height defuzzification method
		v=load[i]+v;
	}
	
	z_star=u/v;
	
	//printf("z_star = %lf\n",z_star);

	
	//return(z_star);
	fp2 = fopen("module5.txt","w");
	
	fprintf(fp2,"%lf\t",z_star);
	
	fclose(fp2);

	//printf("OUT DEFUZZIFICATION MODULE\n");

	
}
