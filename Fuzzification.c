/*************************************************************************************
     PROGRAM FOR FUZZIFICATION OF RELATIVE DEVIATION AND TREND
	 WRITTEN BY: GYANDA SARAN,NML,JAMSHEDPUR
	 DATE: 12|11|2003
*************************************************************************************/

#include"stdio.h"
#include"math.h"

void Fuzzification1(double deviation,double trend)
{
	int i,mf_cut1,mf_cut2;
	double mu1,mu2,mu3,mu4;				   //Membership value of the MF's
	double x[10],a[5],b[5],y[10],c[5],d[5];//coordinates,peaks and support of the MF's          
	char *ptr1,*ptr2,*ptr3,*ptr4;
	
	FILE *fp1,*fp2,*fp3;
	
/*************************************************************************************
    Reading the values of peak and support of relative deviation from respective files
**************************************************************************************/
	
	fp1 = fopen("MFdeviation.txt","r");
	
	if (fp1==NULL)
	{
		puts("cannot open file MFdeviation.txt");
		exit(1);
	}
	
	for(i=0;i<=6;i++)
	{
		fscanf(fp1,"%lf",&x[i]);
	}
	
	for(i=0;i<=4;i++)
	{
		fscanf(fp1,"%lf",&a[i]);
	}
	
	for(i=0;i<=4;i++)
	{
		fscanf(fp1,"%lf",&b[i]);
	}
	
	fclose(fp1);
	
/*****************************************************************************
    Reading the values of peak and support of trend
*****************************************************************************/
	
	fp2 = fopen("MFtrend.txt","r");
	
	if (fp2==NULL)
	{
		puts("cannot open file MFtrend.txt");
		exit(1);
	}
	
	for(i=0;i<=6;i++)
	{
		fscanf(fp2,"%lf",&y[i]);
	}
	
	for(i=0;i<=4;i++)
	{
		fscanf(fp2,"%lf",&c[i]);
	}
	
	
	for(i=0;i<=4;i++)
	{
		fscanf(fp2,"%lf",&d[i]);
	}
	
	fclose(fp2);

/****************************************************************************
    Membership value calculation and linguistic states selection 
	for relative deviation 	
****************************************************************************/
		
	//printf("IN FUZZIFICATION MODULE\n");

	if(deviation<=x[1])
	{
		char LS1[] = "LN";
		char LS2[] = "LN";
		ptr1=LS1;
		ptr2=LS2;
		mu1=1.0;
		mu2=0.0;
		mf_cut1=1;
	}
	
	else if((deviation>x[1])&&(deviation<=x[2]))
	{
		char LS1[] = "LN";
		char LS2[] = "SN";
		ptr1=LS1;
		ptr2=LS2;
		mu1=1.0-fabs(2.0*(deviation-a[0]))/b[0];
		mu2=1.0-fabs(2.0*(deviation-a[1]))/b[1];
		mf_cut1=2;
	}
	
	else if((deviation>x[2])&&(deviation<=x[3]))
	{
		char LS1[] = "SN";
		char LS2[] = "Z";
		ptr1=LS1;
		ptr2=LS2;
		mu1=1.0-fabs(2.0*(deviation-a[1]))/b[1];			
		mu2=1.0-fabs(2.0*(deviation-a[2]))/b[2];
		mf_cut1=2;
	}
	
	else if((deviation>x[3])&&(deviation<=x[4]))
	{
		char LS1[] = "Z";
		char LS2[] = "SP";
		ptr1=LS1;
		ptr2=LS2;
		mu1=1.0-fabs(2.0*(deviation-a[2]))/b[2];
		mu2=1.0-fabs(2.0*(deviation-a[3]))/b[3];
		mf_cut1=2;
	}
	
	else if((deviation>x[4])&&(deviation<=x[5]))
	{
		char LS1[] = "SP";
		char LS2[] = "LP";
		ptr1=LS1;
		ptr2=LS2;
		mu1=1.0-fabs(2.0*(deviation-a[3]))/b[3];
		mu2=1.0-fabs(2.0*(deviation-a[4]))/b[4];
		mf_cut1=2;
	}
	
	else if(deviation>x[5])
	{
		char LS1[] = "LP";
		char LS2[] = "LP";
		ptr1=LS1;
		ptr2=LS2;
		mu1=1.0-(2.0*fabs(deviation-a[4]))/b[4];
		mu2=0.0;
		mf_cut1=1;
	}
	
/**************************************************************************
	Membership value calculation and linguistic states selection 
	for trend
**************************************************************************/

	if(trend<=y[1])
	{
		char LS3[] = "HD";
		char LS4[] = "HD";
		ptr3=LS3;
		ptr4=LS4;
		mu3=1.0;
		mu4=0.0;
		mf_cut2=1;
	}
	
	else if((trend>y[1])&&(trend<=y[2]))
	{
		char LS3[] = "HD";
		char LS4[] = "SD";
		ptr3=LS3;
		ptr4=LS4;
		mu3=1.0-fabs(2.0*(trend-c[0]))/d[0];
		mu4=1.0-fabs(2.0*(trend-c[1]))/d[1];
		mf_cut2=2;
	}
	
	else if((trend>y[2])&&(trend<=y[3]))
	{
		char LS3[] = "SD";
		char LS4[] = "OK";
		ptr3=LS3;
		ptr4=LS4;
		mu3=1.0-fabs(2.0*(trend-c[1]))/d[1];			
		mu4=1.0-fabs(2.0*(trend-c[2]))/d[2];
		mf_cut2=2;
	}
	
	else if((trend>y[3])&&(trend<=y[4]))
	{
		char LS3[] = "OK";
		char LS4[] = "SI";
		ptr3=LS3;
		ptr4=LS4;
		mu3=1.0-fabs(2.0*(trend-c[2]))/d[2];
		mu4=1.0-fabs(2.0*(trend-c[3]))/d[3];
		mf_cut2=2;
	}
	
	else if((trend>y[4])&&(trend<=y[5]))
	{
		char LS3[] = "SI";
		char LS4[] = "HI";
		ptr3=LS3;
		ptr4=LS4;
		mu3=1.0-fabs(2.0*(trend-c[3]))/d[3];
		mu4=1.0-fabs(2.0*(trend-c[4]))/d[4];
		mf_cut2=2;
	}
	
	else if(trend>y[5])
	{
		char LS3[] = "HI";
		char LS4[] = "HI";
		ptr3=LS3;
		ptr4=LS4;
		mu3=1.0-(2.0*fabs(trend-c[4]))/d[4];
		mu4=0.0;
		mf_cut2=1;
	}
		
	fp3 = fopen("module2.txt","w");
	
	fprintf(fp3,"%d  %d\n",mf_cut1,mf_cut2);	
	
	if((mf_cut1==1)&&(mf_cut2==1))
	
	{
	while(*ptr1!='\0')
		{
			fprintf(fp3, "%c",*ptr1);
			ptr1++;
		}	
	fprintf(fp3,"\t%lf",mu1);
	fprintf(fp3,"\n");
	while(*ptr3!='\0')
		{
			fprintf(fp3, "%c",*ptr3);
			ptr3++;
		}
	fprintf(fp3,"\t%lf",mu3);
	}
	
	else if((mf_cut1==1)&&(mf_cut2==2))
	
	{
	while(*ptr1!='\0')
		{
			fprintf(fp3, "%c",*ptr1);
			ptr1++;
		}	
	fprintf(fp3,"\t%lf",mu1);
	fprintf(fp3,"\n");
	while(*ptr3!='\0')
		{
			fprintf(fp3, "%c",*ptr3);
			ptr3++;
		}
	fprintf(fp3,"\t%lf",mu3);
	fprintf(fp3,"\n");
	while(*ptr4!='\0')
		{
			fprintf(fp3, "%c",*ptr4);
			ptr4++;
		}
	fprintf(fp3,"\t%lf",mu4);
	fprintf(fp3,"\n");
	}
	
	else if((mf_cut1==2)&&(mf_cut2==1))
	
	{
	while(*ptr1!='\0')
		{
			fprintf(fp3, "%c",*ptr1);
			ptr1++;
		}	
	fprintf(fp3,"\t%lf",mu1);
	fprintf(fp3,"\n");
	while(*ptr2!='\0')
		{
			fprintf(fp3, "%c",*ptr2);
			ptr2++;
		}
	fprintf(fp3,"\t%lf",mu2);
	fprintf(fp3,"\n");
	while(*ptr3!='\0')
		{
			fprintf(fp3, "%c",*ptr3);
			ptr3++;
		}
	fprintf(fp3,"\t%lf",mu3);
	fprintf(fp3,"\n");
	}	
	
	else if((mf_cut1==2)&&(mf_cut2==2))	
	{
	while(*ptr1!='\0')
		{
			fprintf(fp3, "%c",*ptr1);
			ptr1++;
		}	
	fprintf(fp3,"\t%lf",mu1);
	fprintf(fp3,"\n");
	while(*ptr2!='\0')
		{
			fprintf(fp3, "%c",*ptr2);
			ptr2++;
		}
	fprintf(fp3,"\t%lf",mu2);
	fprintf(fp3,"\n");
	while(*ptr3!='\0')
		{
			fprintf(fp3, "%c",*ptr3);
			ptr3++;
		}
	fprintf(fp3,"\t%lf",mu3);
	fprintf(fp3,"\n");
	while(*ptr4!='\0')
		{
			fprintf(fp3, "%c",*ptr4);
			ptr4++;
		}
	fprintf(fp3,"\t%lf",mu4);
	fprintf(fp3,"\n");
	}
	fclose(fp3);
	//printf("OUT OF FUZZIFICATION MODULE\n");

}