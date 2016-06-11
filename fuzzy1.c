# include <stdio.h>
# include <conio.h>
# include <string.h>
main()
{
      float arr1_min[5]={-6,-4,-2,0,2};
      float arr1_max[5]={-2,0,2,4,6};
      float arrout_min[9],arrout_max[9],y1[2],y2[2],mid1[5],mid2[5],x1,x2;
      float arr2_min[5]={-4.5,-3,-1.5,0,1.5};
      float arr2_max[5]={-1.5,0,1.5,3,4.5};
      float pslope1[5],nslope1[5],pslope2[5],nslope2[5],out_mem[5],x;
      int i,c,k,p1[3],p2[3],k1,k2,j,out[5];
      char e1[5],e2[5],co[5],*cont;
      char *in_var1[]={"nb","ns","z","ps","pb"};
      char *in_var2[]={"nb","ns","z","ps","pb"};
      char *out_strs[]= {"nvb","nb","ns","nm","z","ps","pm","pb","pvb"};
      printf(" \n enter the input variables ");
      scanf("%s%s",&e1,&e2);
      printf(" \n enter the output variable ");
      scanf("%s",&co);
      //printf("\n enter the minimum values for input variable 1 ");
      /*for(i=0;i<5;i++)      
                            scanf("%f",&arr1_min[i]);*/
                            //arr1_min[]={-6,-4,-2,0,2};
      //printf("\n enter the maximum values for input variable 1 ");
      /*for(i=0;i<5;i++)
                            scanf("%f",&arr1_max[i]);*/
                            //arr1_max={-2,0,2,4,6};
      //printf("\n enter the minimum values for input variable 2 ");
     /* for(i=0;i<5;i++)
                            scanf("%f",&arr2_min[i]);*/
                            //arr2_min={-4.5,-3,-1.5,0,1.5};
      /*printf("\n enter the maximum values for input variable 2 ");                    
      for(i=0;i<5;i++)
                            scanf("%f",&arr2_max[i]);*/
                            //arr2_max={-1.5, 0, 1.5, 3, 4.5};
     /* printf("\n enter the minimum values for output variable");
      for(i=0;i<9;i++)
                            scanf("%f",&arrout_min[i]);
      printf("\n enter the maximum values for output variable");
      for(i=0;i<9;i++)
                            scanf("%f",&arrout_max[i]);*/
      printf("\n enter the crisp value for variable 1 ");   
      scanf("%f",&x1);      
      printf("\n enter the crisp value for variable 2 ");
      scanf("%f",&x2);            
      for(i=0;i<5;i++)
      {
                      mid1[i]=(arr1_min[i]+arr1_max[i])/2;
                     // printf("\n %f ",mid1[i]);
                      pslope1[i]=(1/(mid1[i]-arr1_min[i]));
                      //printf("\n %f ",pslope1[i]);
                      nslope1[i]=(1/(arr1_max[i]-mid1[i]));
                      //printf("\n %f ",nslope1[i]);
                      mid2[i]=(arr2_min[i]+arr2_max[i])/2;
                      //printf("\n %f ",mid2[i]);
                      pslope2[i]=(1/(mid2[i]-arr2_min[i]));
                      //printf("\n %f ",pslope2[i]);
                      nslope2[i]=(1/(arr2_max[i]-mid2[i]));
                      //printf("\n %f ",nslope2[i]);
                      mid3[i]=(arrout_min[i]+arrout_max[i])/2;
                      pslope3[i]=(1/(mid3[i]-arrout_min[i]));
                      nslope3[i]=(1/(arrout_max[i]-mid3[i]));
                      }
      c=0; 
      for(i=0;i<5;i++)
      {
                      if((x1>arr1_min[i])&&(x1<arr1_max[i]))
                       p1[c++]=i;
                      }
      for(i=0;i<c;i++)
      {
                      k=p1[i];
                      //printf("\n %d ",k);
                      if(x1>mid1[k])
                           y1[i]=nslope1[k]*(arr1_max[k]-x1);
                      else
                           y1[i]=pslope1[k]*(x1-arr1_min[k]);
                     printf("\n fuzzy variable : %s ",in_var1[k]);
                     printf("\n membership value %f ",y1[i]);
                      }
      c=0;
      for(i=0;i<5;i++)
      {
                      if((x2>arr2_min[i])&&(x2<arr2_max[i]))
                      p2[c++]=i;
                      }
      
      for(i=0;i<c;i++)
      {
                      k=p2[i];
                      if(x2>mid2[k])
                      y2[i]=nslope2[k]*(arr2_max[k]-x2);
                      //printf("\n %f ",nslope2[k])                      
                      else
                      y2[i]=pslope2[k]*(x2-arr2_min[k]);
                      printf("\n fuzzy variable : %s ",in_var2[k]);
                      printf("\n membership value %f ",y2[i]);
                      }
      k=0;
      for(i=0;i<c;i++)
      {k1=p1[i];
         for(j=0;j<c;j++)
                      {k2=p2[j];
                         if(y1[i]<y2[j])
                         out_mem[k]=y1[i];
                         else
                         out_mem[k]=y2[j];
         printf("\n the minimum membership value between fuzzy variable %s and fuzzy variable %s is %f ",in_var1[k1],in_var2[k2],out_mem[k]);
         k=k+1;
                      }
      }
      // rule base
      k=0;
      for(i=0;i<2;i++)
      {
         k1=p1[i];
         //printf("\n %d ",k1);
         
         for(j=0;j<2;j++)
         {
         k2=p2[j];
         //printf("\n %d %d",k1,k2);
         printf("\n input fuzzy variables are %s %s ",in_var1[k1],in_var2[k2]);
         if((strcmp(in_var1[k1],"nb")==0)&&(strcmp(in_var2[k2],"nb")==0))
                         { c=0;
                           }
         if((strcmp(in_var1[k1],"nb")==0)&&(strcmp(in_var2[k2],"ns")==0))
                         //{ char* cont="nb";
                           //continue;}
                           c=1;     
         if((strcmp(in_var1[k1],"nb")==0)&&(strcmp(in_var2[k2],"z")==0))
                         /*{ char* cont="nm";
                           continue;}*/
                           c=2;
         if((strcmp(in_var1[k1],"nb")==0)&&(strcmp(in_var2[k2],"ps")==0))
                         /*{ char* cont="ns";
                           continue;}*/
                           c=3;
         if((strcmp(in_var1[k1],"nb")==0)&&(strcmp(in_var2[k2],"pb")==0))
                         /*{ char* cont="z";
                           continue;}*/
                           c=4;
         if((strcmp(in_var1[k1],"ns")==0)&&(strcmp(in_var2[k2],"nb")==0))
                         /*{char* cont="nb";
                          continue;}*/
                          c=1;
         if((strcmp(in_var1[k1],"ns")==0)&&(strcmp(in_var2[k2],"ns")==0))
                         /*{ char* cont="nm";
                           continue;}*/
                           c=2;
         if((strcmp(in_var1[k1],"ns")==0)&&(strcmp(in_var2[k2],"z")==0))
                         /*{char* cont="ns";
                         continue;}*/
                         c=3;   
         if((strcmp(in_var1[k1],"ns")==0)&&(strcmp(in_var2[k2],"ps")==0))
                         /*{char* cont="z";
                         continue;}*/
                         c=4;  
         if((strcmp(in_var1[k1],"ns")==0)&&(strcmp(in_var2[k2],"pb")==0))
                         /*{char* cont="ps";
                          continue;}*/
                          c=5;  
         if((strcmp(in_var1[k1],"z")==0)&&(strcmp(in_var2[k2],"nb")==0))
                         /*{char* cont="nm";
                          continue;}*/
                          c=2;  
         if((strcmp(in_var1[k1],"z")==0)&&(strcmp(in_var2[k2],"ns")==0))
                         /*{char* cont="ns";
                          continue;}*/
                          c=3;  
         if((strcmp(in_var1[k1],"z")==0)&&(strcmp(in_var2[k2],"z")==0))
                         /*{char* cont="z"; 
                          continue;}*/
                          c=4;
         if((strcmp(in_var1[k1],"z")==0)&&(strcmp(in_var2[k2],"ps")==0))
                         /*{char* cont="ps";
                          continue;}*/
                          c=5;  
         if((strcmp(in_var1[k1],"z")==0)&&(strcmp(in_var2[k2],"pb")==0))
                         /*{char* cont="pb";
                          continue;}*/
                          c=6;  
         if((strcmp(in_var1[k1],"ps")==0)&&(strcmp(in_var2[k2],"nb")==0))
                         /*{char* cont="ns";
                          continue;}*/
                          c=3;  
         if((strcmp(in_var1[k1],"ps")==0)&&(strcmp(in_var2[k2],"ns")==0))
                         /*{char* cont="z";
                         continue;}*/
                         c=4;  
         if((strcmp(in_var1[k1],"ps")==0)&&(strcmp(in_var2[k2],"z")==0))
                         /*{char* cont="ps";
                         continue;}*/
                         c=5;  
         if((strcmp(in_var1[k1],"ps")==0)&&(strcmp(in_var2[k2],"ps")==0))
                         /*{char* cont="pm";
                          continue;}*/
                          c=6;  
         if((strcmp(in_var1[k1],"ps")==0)&&(strcmp(in_var2[k2],"pb")==0))
                         /*{char* cont="pb";
                          continue;}*/
                          c=7;  
         if((strcmp(in_var1[k1],"pb")==0)&&(strcmp(in_var2[k2],"nb")==0))
                         /*{char* cont="z";
                          continue;}*/
                          c=4;  
         if ((strcmp(in_var1[k1],"pb")==0)&&(strcmp(in_var2[k2],"ns")==0))
                         /*{char* cont="ps";
                         continue;}*/
                         c=5;
         if((strcmp(in_var1[k1],"pb")==0)&&(strcmp(in_var2[k2],"z")==0))
                         /*{char* cont="pm";
                         continue;}*/
                         c=6;  
         if((strcmp(in_var1[k1],"pb")==0)&&(strcmp(in_var2[k2],"ps")==0))
                         /*{char* cont="pb";
                         continue;}*/
                         c=7;  
         if((strcmp(in_var1[k1],"pb")==0)&&(strcmp(in_var2[k2],"pb")==0))
                         /*{char* cont="pvb";
                          continue;}*/
                          c=8;
                          //printf("\n %d",c);
                          printf(" output fuzzy variable : %s ",out_strs[c]);
                          out[k++]=c;
                          }
                         //printf("\n %d",c);
                         printf("\n");
                          }  
             //for(i=0;i<k;i++)  
             //printf("\n %s ",cont[i]);    
         for(i=0;i<k-1;i++)
         {
                  if(out[i]==out[i+1])
                    { if(out_mem[i]>out_mem[i+1])
                         y=out_mem[i];
                      else
                         y=out_mem[i+1];
                         }
                  else
                       y=out_mem[i];
                         c=out[i];
                         x=(y/pslope3[c]);
                         a=arrout_max[c]-arrout_min[c];
                         b=a-(2*x)
                         area[i]=0.5*(a+b)*y;
                         area_sum=area_sum+area[i]*out_mem[i];
                         sum=sum+y;
                         }
         crisp=(area_sum/sum);
                                
      getch();
      return 0;
      }
      
      
