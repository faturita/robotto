#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#define pi 3.1415926535 //‰~ü—¦

//ŠÖß’·‚³‚Ìİ’è
#define LB 0.317
#define LS 0.450
#define LE 0.480
#define LW 0.070

//ŠÖ”‚ÌéŒ¾
void kinematics(double theta[],int n);
void I_kinematics(double p[],int n);

//TEST
void I_kinematics2(double p[],int n);

//•Ï”‚ÌéŒ¾
double T[4][4] = {0}; //‰^“®Šw‚Ì‰ğ
double end_p[3] = {0};

int main(){
    int i,j;
    
//•Ï”‚ÌéŒ¾
    double P[3];//end-point‚ÌÀ•W
    double angle[6] = {0,0,0,0,0,0};//ŠÖßŠp

    //Input angles
    for(i=0; i<=5; i++){
        printf("angle[%d] = ",i+1);
        scanf("%lf",&angle[i]);
    }
    
//‰^“®Šw
    kinematics(angle,6);

    printf("Kinematics\n");
    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            printf("%f ",T[i][j]);
        }
        printf("\n");
    }
    printf("\n");
    
    // printf("%f %f %f\n",end_p[0],end_p[1],end_p[2]);
    //‹t‰^“®Šw
    // I_kinematics(end_p,3);
    I_kinematics2(end_p,3);
//printf("%f\n",s[0]);
    return 0;
}


//‰^“®Šw
void kinematics(double theta[],int n){
    int i;
    double s[6] = {0};
    double c[6] = {0};
    double c23,s23;
    
    for(i=0; i<6; i++){
        s[i] = sin(theta[i]*pi/180.0);
        c[i] = cos(theta[i]*pi/180.0);
    }
    
    c23 = c[1]*c[2] - s[1]*s[2];
    s23 = s[1]*c[2] + s[2]*c[1];
    
//(1,1)¬•ª
    T[0][0] = c[5]*c[4]*c[0]*c[3]*c23 - c[5]*c[4]*s[0]*s[3]
        - c[5]*s[4]*c[0]*s23 - s[5]*c[0]*s[3]*c23 - s[5]*s[0]*c[3];
    
//(1,2)¬•ª
    T[0][1] = -s[5]*c[4]*c[0]*c23*c[3] + s[5]*c[4]*s[0]*s[3]
        + s[5]*s[4]*c[0]*s23 - c[5]*c[0]*c23*s[3] - c[5]*s[0]*c[3];
    
//(1,3)¬•ª
    T[0][2] = s[4]*c[0]*c23*c[3] - s[4]*s[0]*s[3] + c[0]*s23*c[4];
    
//(1,4)¬•ª
    T[0][3] = LS*c[0]*s[1] + LE*c[0]*s23 + LW*s[4]*c[0]*c23*c[3]
        - LW*s[4]*s[0]*s[3] + LW*c[0]*s23*c[4];
    
//(2,1)¬•ª
    T[1][0] = c[5]*c[4]*s[0]*c23*c[3] + c[5]*c[4]*c[0]*s[3]
        - c[5]*s[4]*s[0]*s23 - s[5]*s[0]*c23*s[3] + s[5]*c[0]*c[3];
    
//(2,2)¬•ª
    T[1][1] = -s[5]*c[4]*s[0]*c23*c[3] - s[5]*c[4]*c[0]*s[3]
        + s[5]*s[4]*s[0]*s23 - c[5]*s[0]*c23*s[3] + c[5]*c[0]*c[3];
//(2,3)¬•ª
    T[1][2] = s[4]*s[0]*c23*c[3] + s[4]*c[0]*s[3] + s[0]*s23*c[4];

//(2,4)¬•ª
    T[1][3] = LS*s[0]*s[1] + LE*s[0]*s23 + LW*s[4]*s[0]*c23*c[3]
        + LW*s[4]*c[0]*s[3] + LW*s[0]*s23*c[4];

//(3,1)¬•ª
    T[2][0] = -c[5]*s23*c[3]*c[4] - c[5]*c23*s[4] + s[5]*s23*s[3];

//(3,2)¬•ª
    T[2][1] = s[5]*s23*c[3]*c[4] + s[5]*c23*s[4] + c[5]*s23*s[3];

//(3,3)¬•ª
    T[2][2] = -s23*c[3]*s[4] + c23*c[4];
    
//(3,4)¬•ª
    T[2][3] = LB + LS*c[1] + LE*c23  - LW*s23*c[3]*s[4]
        + LW*c23*c[4];
//(4,1)¬•ª
    T[3][0] = 0;

//(4,2)¬•ª
    T[3][1] = 0;

//(4,3)¬•ª
    T[3][2] = 0;

//(4,4)¬•ª
    T[3][3] = 1.0;

//end-point‚ÌÀ•W
    end_p[0] = T[0][3];
    end_p[1] = T[1][3];
    end_p[2] = T[2][3];
    return;
}

//‹t‰^“®Šw
void I_kinematics(double end_p[],int n){

    //•Ï”‚ÌéŒ¾
    int i;
    int j;
    double p[3] = {0};
    double I_angle[6][3] = {0};
    double to_angle = 180.0/pi;
    double s[5] = {0};
    double c[5] = {0};
    double s23 = 0;
    double c23 = 0; 
    double A,B;
    double a,b,d;
    double I_T[4][3] = {0}; //T_03
    double x,y,z;
    double n2[2] = {0};
    double t[2] = {0};
    double b2[3] = {0};
    double pos[2] = {0};

    p[0] = end_p[0] - LW*T[0][2];
    p[1] = end_p[1] - LW*T[1][2];
    p[2] = end_p[2] - LW*T[2][2];
    
    //angle of link 1
    I_angle[0][0] = atan2(p[1],p[0]);

    //angle of link 2
    s[0] = sin(I_angle[0][0]);
    c[0] = cos(I_angle[0][0]);
    
    A = p[0]*c[0] + p[1]*s[0];
    B = -p[2] + LB;

    a = A*A + B*B + LS*LS - LE*LE - 2*LS*B;
    b = A*A + B*B + LS*LS - LE*LE + 2*LS*B;
    
    I_angle[1][0] = 2*atan2(2*LS*A - sqrt(4*LS*LS*A*A - a*b),a);
    I_angle[1][1] = 2*atan2(2*LS*A + sqrt(4*LS*LS*A*A - a*b),a);
    
    //angle of link 3
    I_angle[2][0] = atan2(sqrt(4*LS*LS*LE*LE - (A*A + B*B - (LE*LE + LS*LS))*(A*A + B*B - (LE*LE + LS*LS))), A*A + B*B - (LE*LE + LS*LS));
    I_angle[2][1] = atan2(sqrt(4*LS*LS*LE*LE + (A*A + B*B - (LE*LE + LS*LS))*(A*A + B*B - (LE*LE + LS*LS))), A*A + B*B - (LE*LE + LS*LS));

    for(i=1; i<=2; i++){
        s[i] = sin(I_angle[i][0]);
        c[i] = cos(I_angle[i][0]);
    }
        
    //angle of link 4,5,6

    s23 = s[1]*c[2] + s[2]*c[1];
    c23 = c[1]*c[2] - s[1]*s[2];

    x = LS*c[0]*s[1];
    y = LS*s[0]*s[1];
    z = LS*c[1]+ LB;

    I_T[0][0] = c[0]*c23;
    I_T[0][1] = s[0]*c23;
    I_T[0][2] = -s23;
    I_T[0][3] = -(I_T[0][0]*x + I_T[0][1]*y + I_T[0][2]*z);

    I_T[1][0] = -s[0];
    I_T[1][1] = c[0];
    I_T[1][2] = 0;
    I_T[1][3] = -(I_T[1][0]*x + I_T[1][1]*y + I_T[1][2]*z);

    I_T[2][0] = c[0]*s23;
    I_T[2][1] = s[0]*s23;
    I_T[2][2] = c23;
    I_T[2][3] = -(I_T[2][0]*x + I_T[2][1]*y + I_T[2][2]*z);

    n2[0] = I_T[0][0]*T[0][0] + I_T[0][1]*T[1][0] + I_T[0][2]*T[2][0];
    n2[1] = I_T[1][0]*T[0][0] + I_T[1][1]*T[1][0] + I_T[1][2]*T[2][0];

    t[0] = I_T[0][0]*T[0][1] + I_T[0][1]*T[1][1] + I_T[0][2]*T[2][1];
    t[1] = I_T[1][0]*T[0][1] + I_T[1][1]*T[1][1] + I_T[1][2]*T[2][1];
    
    b2[0] = I_T[0][0]*T[0][2] + I_T[0][1]*T[1][2] + I_T[0][2]*T[2][2];
    b2[1] = I_T[1][0]*T[0][2] + I_T[1][1]*T[1][2] + I_T[1][2]*T[2][2];
    b2[2] = I_T[2][0]*T[0][2] + I_T[2][1]*T[1][2] + I_T[2][2]*T[2][2];
    
    pos[0] = I_T[0][0]*T[0][3] + I_T[0][1]*T[1][3] + I_T[0][2]*T[2][3] + I_T[0][3];
    pos[1] = I_T[1][0]*T[0][3] + I_T[1][1]*T[1][3] + I_T[1][2]*T[2][3] + I_T[1][3];

    //angle of link 4
    I_angle[3][0] = atan2(b2[1],b2[0]);
    
    //angle of link 5
    c[3] = cos(I_angle[3][0]);
    s[3] = sin(I_angle[3][0]);
    
    I_angle[4][0] = atan2(c[3]*b2[0] + s[3]*b2[1],b2[2]);

    //angle of link6
    I_angle[5][0] = atan2(-s[3]*n2[0] + c[3]*n2[1],-s[3]*t[0] + c[3]*t[1]);
    
    //test‚Åo—Í
    // printf("%f\n",1/rad);
    // printf("%f %f %f\n",p[0],p[1],p[2]);
    printf("Inverse Kinematics\n");
    printf("link 1\n %f\n",I_angle[0][0]*to_angle);
    printf("link 2\n2- %f 2+ %f\n",I_angle[1][0]*to_angle,I_angle[1][1]*to_angle);
    printf("link 3\n3- %f 3+ %f\n",I_angle[2][0]*to_angle,I_angle[2][1]*to_angle);
    printf("link 4\n %f\n",I_angle[3][0]*to_angle);
    printf("link 5\n %f\n",I_angle[4][0]*to_angle);
    printf("link 6\n %f\n",I_angle[5][0]*to_angle);
    
    return;
}

//‹t‰^“®Šw(TEST)
void I_kinematics2(double end_p[],int n){

        //•Ï”‚ÌéŒ¾
    int i;
    int j;
    double p[3] = {0};
    double I_angle[6][4] = {0};
    double to_angle = 180.0/pi;
    double s[5] = {0};
    double c[5] = {0};
    double s23 = 0;
    double c23 = 0; 
    double A,B;
    double a,b,d;
    double I_T[4][3] = {0}; //T_03
    double x,y,z;
    double n2[2] = {0};
    double t[2] = {0};
    double b2[3] = {0};
    double pos[2] = {0};

    p[0] = end_p[0] - LW*T[0][2];
    p[1] = end_p[1] - LW*T[1][2];
    p[2] = end_p[2] - LW*T[2][2];
    
    //angle of link 1
    I_angle[0][0] = atan2(p[1],p[0]);

    //angle of link 2
    s[0] = sin(I_angle[0][0]);
    c[0] = cos(I_angle[0][0]);
    
    A = p[0]*c[0] + p[1]*s[0];
    B = -p[2] + LB;

    a = A*A + B*B + LS*LS - LE*LE - 2*LS*B;
    b = A*A + B*B + LS*LS - LE*LE + 2*LS*B;
    
    I_angle[1][0] = 2*atan2(2*LS*A - sqrt(4*LS*LS*A*A - a*b),a);
    I_angle[1][1] = 2*atan2(2*LS*A + sqrt(4*LS*LS*A*A - a*b),a);
    
    //angle of link 3
    I_angle[2][0] = atan2(sqrt(4*LS*LS*LE*LE - (A*A + B*B - (LE*LE + LS*LS))*(A*A + B*B - (LE*LE + LS*LS))), A*A + B*B - (LE*LE + LS*LS));
    I_angle[2][1] = atan2(sqrt(4*LS*LS*LE*LE + (A*A + B*B - (LE*LE + LS*LS))*(A*A + B*B - (LE*LE + LS*LS))), A*A + B*B - (LE*LE + LS*LS));


    //angle2 - , angle3 -
    for(i=1; i<=2; i++){
        s[i] = sin(I_angle[i][0]);
        c[i] = cos(I_angle[i][0]);
    }
        
    //angle of link 4,5,6

    s23 = s[1]*c[2] + s[2]*c[1];
    c23 = c[1]*c[2] - s[1]*s[2];

    x = LS*c[0]*s[1];
    y = LS*s[0]*s[1];
    z = LS*c[1]+ LB;

    I_T[0][0] = c[0]*c23;
    I_T[0][1] = s[0]*c23;
    I_T[0][2] = -s23;
    I_T[0][3] = -(I_T[0][0]*x + I_T[0][1]*y + I_T[0][2]*z);

    I_T[1][0] = -s[0];
    I_T[1][1] = c[0];
    I_T[1][2] = 0;
    I_T[1][3] = -(I_T[1][0]*x + I_T[1][1]*y + I_T[1][2]*z);

    I_T[2][0] = c[0]*s23;
    I_T[2][1] = s[0]*s23;
    I_T[2][2] = c23;
    I_T[2][3] = -(I_T[2][0]*x + I_T[2][1]*y + I_T[2][2]*z);

    n2[0] = I_T[0][0]*T[0][0] + I_T[0][1]*T[1][0] + I_T[0][2]*T[2][0];
    n2[1] = I_T[1][0]*T[0][0] + I_T[1][1]*T[1][0] + I_T[1][2]*T[2][0];

    t[0] = I_T[0][0]*T[0][1] + I_T[0][1]*T[1][1] + I_T[0][2]*T[2][1];
    t[1] = I_T[1][0]*T[0][1] + I_T[1][1]*T[1][1] + I_T[1][2]*T[2][1];
    
    b2[0] = I_T[0][0]*T[0][2] + I_T[0][1]*T[1][2] + I_T[0][2]*T[2][2];
    b2[1] = I_T[1][0]*T[0][2] + I_T[1][1]*T[1][2] + I_T[1][2]*T[2][2];
    b2[2] = I_T[2][0]*T[0][2] + I_T[2][1]*T[1][2] + I_T[2][2]*T[2][2];
    
    pos[0] = I_T[0][0]*T[0][3] + I_T[0][1]*T[1][3] + I_T[0][2]*T[2][3] + I_T[0][3];
    pos[1] = I_T[1][0]*T[0][3] + I_T[1][1]*T[1][3] + I_T[1][2]*T[2][3] + I_T[1][3];

    printf("%f %f %f %f\n",n2[0],t[0],b2[0],pos[0]);
    printf("%f %f %f %f \n",n2[1],t[1],b2[1],pos[1]);
    printf("0  0  %f 0\n",b2[2]);
    printf("\n");

    //angle of link 4
    I_angle[3][0] = atan2(b2[1],b2[0]);
    
    //angle of link 5
    c[3] = cos(I_angle[3][0]);
    s[3] = sin(I_angle[3][0]);
    
    I_angle[4][0] = atan2(c[3]*b2[0] + s[3]*b2[1],b2[2]);

    //angle of link6
    I_angle[5][0] = atan2(-s[3]*n2[0] + c[3]*n2[1],-s[3]*t[0] + c[3]*t[1]);


//angle2 + , angle3 -
    for(i=1; i<=2; i++){
        s[i] = sin(I_angle[i][1]);
        c[i] = cos(I_angle[i][0]);
    }
        
    //angle of link 4,5,6

    s23 = s[1]*c[2] + s[2]*c[1];
    c23 = c[1]*c[2] - s[1]*s[2];

    x = LS*c[0]*s[1];
    y = LS*s[0]*s[1];
    z = LS*c[1]+ LB;

    I_T[0][0] = c[0]*c23;
    I_T[0][1] = s[0]*c23;
    I_T[0][2] = -s23;
    I_T[0][3] = -(I_T[0][0]*x + I_T[0][1]*y + I_T[0][2]*z);

    I_T[1][0] = -s[0];
    I_T[1][1] = c[0];
    I_T[1][2] = 0;
    I_T[1][3] = -(I_T[1][0]*x + I_T[1][1]*y + I_T[1][2]*z);

    I_T[2][0] = c[0]*s23;
    I_T[2][1] = s[0]*s23;
    I_T[2][2] = c23;
    I_T[2][3] = -(I_T[2][0]*x + I_T[2][1]*y + I_T[2][2]*z);

    n2[0] = I_T[0][0]*T[0][0] + I_T[0][1]*T[1][0] + I_T[0][2]*T[2][0];
    n2[1] = I_T[1][0]*T[0][0] + I_T[1][1]*T[1][0] + I_T[1][2]*T[2][0];

    t[0] = I_T[0][0]*T[0][1] + I_T[0][1]*T[1][1] + I_T[0][2]*T[2][1];
    t[1] = I_T[1][0]*T[0][1] + I_T[1][1]*T[1][1] + I_T[1][2]*T[2][1];
    
    b2[0] = I_T[0][0]*T[0][2] + I_T[0][1]*T[1][2] + I_T[0][2]*T[2][2];
    b2[1] = I_T[1][0]*T[0][2] + I_T[1][1]*T[1][2] + I_T[1][2]*T[2][2];
    b2[2] = I_T[2][0]*T[0][2] + I_T[2][1]*T[1][2] + I_T[2][2]*T[2][2];
    
    pos[0] = I_T[0][0]*T[0][3] + I_T[0][1]*T[1][3] + I_T[0][2]*T[2][3] + I_T[0][3];
    pos[1] = I_T[1][0]*T[0][3] + I_T[1][1]*T[1][3] + I_T[1][2]*T[2][3] + I_T[1][3];

    //angle of link 4
    I_angle[3][1] = atan2(b2[1],b2[0]);
    
    //angle of link 5
    c[3] = cos(I_angle[3][0]);
    s[3] = sin(I_angle[3][0]);
    
    I_angle[4][1] = atan2(c[3]*b2[0] + s[3]*b2[1],b2[2]);

    //angle of link6
    I_angle[5][1] = atan2(-s[3]*n2[0] + c[3]*n2[1],-s[3]*t[0] + c[3]*t[1]);


//angle2 - , angle3 +
    for(i=1; i<=2; i++){
        s[i] = sin(I_angle[i][0]);
        c[i] = cos(I_angle[i][1]);
    }
        
    //angle of link 4,5,6

    s23 = s[1]*c[2] + s[2]*c[1];
    c23 = c[1]*c[2] - s[1]*s[2];

    x = LS*c[0]*s[1];
    y = LS*s[0]*s[1];
    z = LS*c[1]+ LB;

    I_T[0][0] = c[0]*c23;
    I_T[0][1] = s[0]*c23;
    I_T[0][2] = -s23;
    I_T[0][3] = -(I_T[0][0]*x + I_T[0][1]*y + I_T[0][2]*z);

    I_T[1][0] = -s[0];
    I_T[1][1] = c[0];
    I_T[1][2] = 0;
    I_T[1][3] = -(I_T[1][0]*x + I_T[1][1]*y + I_T[1][2]*z);

    I_T[2][0] = c[0]*s23;
    I_T[2][1] = s[0]*s23;
    I_T[2][2] = c23;
    I_T[2][3] = -(I_T[2][0]*x + I_T[2][1]*y + I_T[2][2]*z);

    n2[0] = I_T[0][0]*T[0][0] + I_T[0][1]*T[1][0] + I_T[0][2]*T[2][0];
    n2[1] = I_T[1][0]*T[0][0] + I_T[1][1]*T[1][0] + I_T[1][2]*T[2][0];

    t[0] = I_T[0][0]*T[0][1] + I_T[0][1]*T[1][1] + I_T[0][2]*T[2][1];
    t[1] = I_T[1][0]*T[0][1] + I_T[1][1]*T[1][1] + I_T[1][2]*T[2][1];
    
    b2[0] = I_T[0][0]*T[0][2] + I_T[0][1]*T[1][2] + I_T[0][2]*T[2][2];
    b2[1] = I_T[1][0]*T[0][2] + I_T[1][1]*T[1][2] + I_T[1][2]*T[2][2];
    b2[2] = I_T[2][0]*T[0][2] + I_T[2][1]*T[1][2] + I_T[2][2]*T[2][2];
    
    pos[0] = I_T[0][0]*T[0][3] + I_T[0][1]*T[1][3] + I_T[0][2]*T[2][3] + I_T[0][3];
    pos[1] = I_T[1][0]*T[0][3] + I_T[1][1]*T[1][3] + I_T[1][2]*T[2][3] + I_T[1][3];

    //angle of link 4
    I_angle[3][2] = atan2(b2[1],b2[0]);
    
    //angle of link 5
    c[3] = cos(I_angle[3][0]);
    s[3] = sin(I_angle[3][0]);
    
    I_angle[4][2] = atan2(c[3]*b2[0] + s[3]*b2[1],b2[2]);

    //angle of link6
    I_angle[5][2] = atan2(-s[3]*n2[0] + c[3]*n2[1],-s[3]*t[0] + c[3]*t[1]);


        //angle2 - , angle3 -
    for(i=1; i<=2; i++){
        s[i] = sin(I_angle[i][1]);
        c[i] = cos(I_angle[i][1]);
    }
        
    //angle of link 4,5,6

    s23 = s[1]*c[2] + s[2]*c[1];
    c23 = c[1]*c[2] - s[1]*s[2];

    x = LS*c[0]*s[1];
    y = LS*s[0]*s[1];
    z = LS*c[1]+ LB;

    I_T[0][0] = c[0]*c23;
    I_T[0][1] = s[0]*c23;
    I_T[0][2] = -s23;
    I_T[0][3] = -(I_T[0][0]*x + I_T[0][1]*y + I_T[0][2]*z);

    I_T[1][0] = -s[0];
    I_T[1][1] = c[0];
    I_T[1][2] = 0;
    I_T[1][3] = -(I_T[1][0]*x + I_T[1][1]*y + I_T[1][2]*z);

    I_T[2][0] = c[0]*s23;
    I_T[2][1] = s[0]*s23;
    I_T[2][2] = c23;
    I_T[2][3] = -(I_T[2][0]*x + I_T[2][1]*y + I_T[2][2]*z);

    n2[0] = I_T[0][0]*T[0][0] + I_T[0][1]*T[1][0] + I_T[0][2]*T[2][0];
    n2[1] = I_T[1][0]*T[0][0] + I_T[1][1]*T[1][0] + I_T[1][2]*T[2][0];

    t[0] = I_T[0][0]*T[0][1] + I_T[0][1]*T[1][1] + I_T[0][2]*T[2][1];
    t[1] = I_T[1][0]*T[0][1] + I_T[1][1]*T[1][1] + I_T[1][2]*T[2][1];
    
    b2[0] = I_T[0][0]*T[0][2] + I_T[0][1]*T[1][2] + I_T[0][2]*T[2][2];
    b2[1] = I_T[1][0]*T[0][2] + I_T[1][1]*T[1][2] + I_T[1][2]*T[2][2];
    b2[2] = I_T[2][0]*T[0][2] + I_T[2][1]*T[1][2] + I_T[2][2]*T[2][2];
    
    pos[0] = I_T[0][0]*T[0][3] + I_T[0][1]*T[1][3] + I_T[0][2]*T[2][3] + I_T[0][3];
    pos[1] = I_T[1][0]*T[0][3] + I_T[1][1]*T[1][3] + I_T[1][2]*T[2][3] + I_T[1][3];

    //angle of link 4
    I_angle[3][3] = atan2(b2[1],b2[0]);
    
    //angle of link 5
    c[3] = cos(I_angle[3][0]);
    s[3] = sin(I_angle[3][0]);
    
    I_angle[4][3] = atan2(c[3]*b2[0] + s[3]*b2[1],b2[2]);

    //angle of link6
    I_angle[5][3] = atan2(-s[3]*n2[0] + c[3]*n2[1],-s[3]*t[0] + c[3]*t[1]);


    //o—Í
    printf("angle 1 : %f %f %f %f\n",I_angle[0][0]*to_angle,I_angle[0][0]*to_angle,I_angle[0][0]*to_angle,I_angle[0][0]*to_angle);
    printf("angle 2 : %f %f %f %f\n",I_angle[1][0]*to_angle,I_angle[1][1]*to_angle,I_angle[1][0]*to_angle,I_angle[1][1]*to_angle);
    printf("angle 3 : %f %f %f %f\n",I_angle[2][0]*to_angle,I_angle[2][0]*to_angle,I_angle[2][1]*to_angle,I_angle[2][1]*to_angle);
    printf("angle 4 : %f %f %f %f\n",I_angle[3][0]*to_angle,I_angle[3][1]*to_angle,I_angle[3][2]*to_angle,I_angle[3][3]*to_angle);
    printf("angle 5 : %f %f %f %f\n",I_angle[4][0]*to_angle,I_angle[4][1]*to_angle,I_angle[4][2]*to_angle,I_angle[4][3]*to_angle);
    printf("angle 6 : %f %f %f %f\n",I_angle[5][0]*to_angle,I_angle[5][1]*to_angle,I_angle[5][2]*to_angle,I_angle[5][3]*to_angle);
    
    return;
}
