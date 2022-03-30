#define VISP_DEBUG 
//#define SERVO_DEBUG
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <heltec.h>
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <math.h> 
#include "BluetoothSerial.h"
#ifdef VISP_DEBUG
#include <geometry_msgs/Vector3.h>
#endif
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


#define MIN 850
#define MAX 2150


#define INV1 1
#define INV2 3
#define INV3 5

#define LED_W 13
#define LED_BUILTIN 25

#define deg30 PI/6
#define drone_height 0.32
#define pos_limit 25 
#define servo_limit 25

static const int servosPins[6] = {22, 27, 14, 12, 23, 19};

Servo servos[6];

//Zero positions of servos, in this positions their arms are perfectly horizontal, in us
static int zero[6]={1690,1300,1730,1300,1660,1300};
//In this array is stored requested position for platform - x,y,z,rot(x),rot(y),rot(z)
static float arr[6]={0.0,0.0,0.0, radians(0),radians(0),radians(0)},
             newarr[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
//complexity of calculating new degree of rotation
static float theta_a[6]={0.0,0.0,0.0, 0.0,0.0,0.0};
//Array of current servo positions in us
static int servo_pos[6],servo_val[6],prev_val[6];

//rotation of servo arms in respect to axis x

const float beta[] = {PI/2,-PI/2,-PI/6, 5*PI/6,-5*PI/6,PI/6},

//maximum servo positions, 0 is horizontal position
servo_min=radians(-30),servo_max=radians(140),
//servo_mult - multiplier used for conversion radians->servo pulse in us
//L1-effective length of servo arm, L2 - length of base and platform connecting arm
//z_home - height of platform above base, 0 is height of servo arms
servo_mult=(MAX-MIN)/radians(170),L1 = 24,L2 = 164.5, z_home = 158;
//RD distance from center of platform to attachment points (arm attachment point)
//pD distance from center of base to center of servo rotation points (servo axis)
//theta_p-angle between two servo axis points, theta_r - between platform attachment points
//theta_angle-helper variable
//p[][]=x y values for servo rotation points
//re[]{}=x y z values of platform attachment points positions
//equations used for p and re will affect postion of X axis, they can be changed to achieve
//specific X axis position
const float RD = 56.56,PD =119.21,theta_p = radians(46.81),
theta_angle=(PI/3-theta_p)/2, theta_r = radians(14.17),
     
      p[2][6]={
         {
            -PD*cos(deg30-theta_angle),-PD*cos(deg30-theta_angle),
            PD*sin(theta_angle),PD*cos(deg30+theta_angle),
            PD*cos(deg30+theta_angle),PD*sin(theta_angle)
         },
         {
            -PD*sin(deg30-theta_angle),PD*sin(deg30-theta_angle),
            PD*cos(theta_angle),PD*sin(deg30+theta_angle),
            -PD*sin(deg30+theta_angle),-PD*cos(theta_angle)
         }
      },
      re[3][6] = {
            {
               -RD*sin(deg30+theta_r/2),-RD*sin(deg30+theta_r/2),
               -RD*sin(deg30-theta_r/2),RD*cos(theta_r/2),
               RD*cos(theta_r/2),-RD*sin(deg30-theta_r/2),
            },{
               -RD*cos(deg30+theta_r/2),RD*cos(deg30+theta_r/2),
               RD*cos(deg30-theta_r/2),RD*sin(theta_r/2),
               -RD*sin(theta_r/2),-RD*cos(deg30-theta_r/2),
            },{
               0,0,0,0,0,0
            }
};
//arrays used for servo rotation calculation
//H[]-center position of platform can be moved with respect to base, this is
//translation vector representing this move
static float M[3][3], rxp[3][6], T[3], H[3] = {0,0,z_home};

int pos = 0;    // variable to store the servo position
String text;//display text

ros::NodeHandle nh;
static float quaternion[4],camera_z;
static float rotx, roty,rotz;

uint8_t statusMsg;
void positionCb( const geometry_msgs::PoseStamped& position_msg){
   camera_z=position_msg.pose.position.z;
   quaternion[0]= position_msg.pose.orientation.x;
   quaternion[1]= position_msg.pose.orientation.y;
   quaternion[2]= position_msg.pose.orientation.z;
   quaternion[3]= position_msg.pose.orientation.w;
}
void statusCb( const std_msgs::Int8& status_msg){
   statusMsg=status_msg.data;
}


ros::Subscriber<geometry_msgs::PoseStamped> sub("/visp_auto_tracker/object_position", &positionCb );
ros::Subscriber<std_msgs::Int8> sub1("/visp_auto_tracker/status", &statusCb );

#ifdef VISP_DEBUG
geometry_msgs::Vector3 euler_angles;
ros::Publisher pub("eulerxyz", &euler_angles);
#endif
BluetoothSerial SerialBT;

bool BTinterruptFlag=false, BTconnected;
uint8_t BTaddress[6]  = {0x6C, 0xDD, 0xBC, 0x5A, 0xDB, 0x21};
void setup() {

   Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/); 
   Heltec.display->clear();
   pinMode(LED_W,OUTPUT);
   pinMode(LED_BUILTIN,OUTPUT);
   
   SerialBT.begin("Stewart_platform");
   //BTconnected = SerialBT.connect(BTaddress);

   for(int i = 0; i < 6; ++i){
      if(!servos[i].attach(servosPins[i],MIN,MAX)) {
         Heltec.display->clear();
         Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
         Heltec.display->setFont(ArialMT_Plain_10);
         Heltec.display->drawString(0, 0, "Servo");
         Heltec.display->drawString(25, 0, String(i));
         Heltec.display->drawString(30, 0, "Error");
         Heltec.display->display();
      }else{ 
             
         setPos(arr);   
      }
   }
       
    

   nh.initNode();
   nh.subscribe(sub);
   nh.subscribe(sub1);

   #ifdef VISP_DEBUG
   nh.advertise(pub);
   #endif

   while (!nh.connected() ){

      text="Pending ROS communication Up!"; 
      nh.spinOnce();
      textDisplay(0,0,16,text);
   

      delay(50);
   }
   
   
   while(statusMsg==NULL){
      text="Camera is Down. Please Reset...";
      textDisplay(0,0,16,text);
      nh.spinOnce();
      delay(50);
      
   }
      
   text="Communication Done!";
   textDisplay(0,0,16,text);
   digitalWrite(LED_W, HIGH);

   
   
   delay(1000);
}

//============================================================================================

float getAlpha(int *i){
   static int n;
   static float th=0;
   static float q[3], dl[3], dl2;
   double min=servo_min;
   double max=servo_max;
   n=0;
   th=theta_a[*i];
   while(n<20){
    //calculation of position of base attachment point (point on servo arm where is leg connected)
      q[0] = L1*cos(th)*cos(beta[*i]) + p[0][*i];
      q[1] = L1*cos(th)*sin(beta[*i]) + p[1][*i];
      q[2] = L1*sin(th);
    //calculation of distance between according platform attachment point and base attachment point
      dl[0] = rxp[0][*i] - q[0];
      dl[1] = rxp[1][*i] - q[1];
      dl[2] = rxp[2][*i] - q[2];
      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);
    //if this distance is the same as leg length, value of theta_a is corrent, we return it
      if(abs(L2-dl2)<0.01){
         return th;
      }
    //if not, we split the searched space in half, then try next value
      if(dl2<L2){
         max=th;
      }else{
         min=th;
      }
      n+=1;
      if(max==servo_min || min==servo_max){
         return th;
      }
      th = min+(max-min)/2;
   }
   return th;
}

//function calculating rotation matrix
void getmatrix(float pe[])
{
   float psi=pe[5];
   float theta=pe[4];
   float phi=pe[3];
   M[0][0] = cos(psi)*cos(theta);
   M[1][0] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
   M[2][0] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);

   M[0][1] = sin(psi)*cos(theta);
   M[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
   M[2][1] = cos(theta)*sin(phi);

   M[0][2] = -sin(theta);
   M[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
   M[2][2] = cos(theta)*cos(phi);
}
//calculates wanted position of platform attachment poins using calculated rotation matrix
//and translation vector
void getrxp(float pe[])
{
   for(int i=0;i<6;i++){
      rxp[0][i] = T[0]+M[0][0]*(re[0][i])+M[0][1]*(re[1][i])+M[0][2]*(re[2][i]);
      rxp[1][i] = T[1]+M[1][0]*(re[0][i])+M[1][1]*(re[1][i])+M[1][2]*(re[2][i]);
      rxp[2][i] = T[2]+M[2][0]*(re[0][i])+M[2][1]*(re[1][i])+M[2][2]*(re[2][i]);
   }
}
//function calculating translation vector - desired move vector + home translation vector
void getT(float pe[])
{
   T[0] = pe[0]+H[0];
   T[1] = pe[1]+H[1];
   T[2] = pe[2]+H[2];
}

unsigned char setPos(float pe[]){
   unsigned char errorcount;
   errorcount=0;
   
   #ifdef SERVO_DEBUG
   String tempstring1="servo_val= ",tempstring2="prev_val= ";
   #endif

   for(int i = 0; i < 6; i++)
   {
      getT(pe);
      getmatrix(pe);
      getrxp(pe);
      theta_a[i]=getAlpha(&i);
      if(digitalRead(LED_W)==HIGH){//after setup

            
         if(i==INV1||i==INV2||i==INV3){
            servo_val[i]= zero[i]-(theta_a[i])*servo_mult;

               if(abs((servo_val[i]-prev_val[i]))>pos_limit){  
                  if(servo_val[i]>prev_val[i]){
                     servo_val[i]=prev_val[i]+pos_limit;
                  }else{
                     servo_val[i]=prev_val[i]-pos_limit;
                  }
               }
                   
         }else{
            servo_val[i]= zero[i]+(theta_a[i])*servo_mult;

               if(abs((servo_val[i]-prev_val[i]))>pos_limit){
                  if(servo_val[i]>prev_val[i]){
                     servo_val[i]=prev_val[i]+pos_limit;
                  }else{
                     servo_val[i]=prev_val[i]-pos_limit;
                  }
               }  

                    
                      

         }
                    
               
      }else{//during setup

            if(i==INV1||i==INV2||i==INV3){
               servo_val[i]= zero[i] - (theta_a[i])*servo_mult;
                 
                  
            }else{
               servo_val[i]= zero[i] + (theta_a[i])*servo_mult;
                  
                 
            }   
      }
      prev_val[i]=servo_val[i];
      servo_pos[i] = constrain(servo_val[i], MIN,MAX);
      
      #ifdef SERVO_DEBUG
      tempstring1+=String(servo_val[i]);
      tempstring1+=", ";
      tempstring2+=String(prev_val[i]);
      tempstring2+=", ";
      #endif


   }
    

    
   for(int i = 0; i < 6; i++)
   {
        if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MIN||servo_pos[i]==MAX){
            errorcount++;
        }
        servos[i].writeMicroseconds(servo_pos[i]);
        
   }

   #ifdef SERVO_DEBUG
   Heltec.display->clear();
   Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
   Heltec.display->setFont(ArialMT_Plain_10);
   Heltec.display->drawStringMaxWidth(0, 0,120, tempstring1);
   Heltec.display->drawStringMaxWidth(0, 22,120, tempstring2);
   Heltec.display->drawStringMaxWidth(0, 44,120, String(ESP.getFreeHeap()));
   Heltec.display->display();
   #endif
   #ifndef SERVO_DEBUG
   delay(15);
   #endif
   return errorcount;
}

void textDisplay(int x,int y,int font, String text ){
   Heltec.display->clear();
   Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
   
   if(font==16){
      Heltec.display->setFont(ArialMT_Plain_16);
   }else if(font==24){
      Heltec.display->setFont(ArialMT_Plain_24);
   }else{
      Heltec.display->setFont(ArialMT_Plain_10);
   }
   
   Heltec.display->drawStringMaxWidth(x, y,128, text);
   Heltec.display->display();

}
void quat2Euler(float q[]){
   rotx = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
   roty = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
   rotz = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}
void BTzeroing(void){
   bool zeroingFLag =false;
   
   while(!zeroingFLag){
      int count=0;
      for(int i=0;i<6;i++){
         
         if(abs(zero[i]-servo_val[i])>pos_limit){
            if(zero[i]>servo_val[i]){
               servo_val[i]+=pos_limit;
            }else{
               servo_val[i]-=pos_limit;
            }
         }else{
            servo_val[i]=zero[i];
            
         }
         servos[i].writeMicroseconds(servo_val[i]);
         
         prev_val[i]=servo_val[i];
         
         

      }
      delay(50);
      for(int j=0;j<6;j++){
         
         if(servo_val[j]==zero[j]){
            count++;
         }else{
            count=0;
         }
         //textDisplay(0,0,16,String(count));
         if(count==6){
            zeroingFLag=true;
         
         }
      }
   }
   
}
//============================================

void loop() {
   
   quat2Euler(quaternion);
 /*q[0]=x, q[1]=y, q[2]=z, q[3]=w
   rotx = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
   roty = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
   rotz = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
 /*rotx = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), 1-2.0f*(q[0]*q[0]+q[1]*q[1]));
 roty = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
 rotz = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1-2.0f*(q[1]*q[1]+q[2]*q[2]));
 rotx = PI-rotx;
 roty = -roty;*/
  

   if(SerialBT.available()>0){
      if(SerialBT.read()!=NULL){
      char c = SerialBT.read();
      if(c >='2'){
         BTinterruptFlag=true;
      }
      if(c =='1'){
         BTinterruptFlag=false;
      }
      SerialBT.write(c);
      }   
      

   }
   
   if(!nh.connected()&&!BTinterruptFlag){

      text="Communication Down, Pending Reconnect...";
      textDisplay(0,0,16,text);
      digitalWrite(LED_W,LOW);
  
   }else{
      digitalWrite(LED_W,HIGH);
      if(BTinterruptFlag==true){
      text="BT interrupts. Resting";
      textDisplay(30,10,16,text);
      digitalWrite(LED_BUILTIN, LOW);
      BTzeroing();      
            
      }else{     
         if(statusMsg==3&&camera_z>drone_height&&BTinterruptFlag==false){
            text="QRcode Detected. Moving!";

            
            if(abs(rotx)<=radians(servo_limit)&&abs(roty)<=radians(servo_limit)){
               //x and y axis realign, in platform rotx=visp -roty; in platform roty= rotz
            newarr[3]=-(rotx*sin(rotz)+roty*cos(rotz));
            newarr[4]=(rotx*cos(rotz)-roty*sin(rotz));
            setPos(newarr);
            }

            textDisplay(30,5,16,text);
            digitalWrite(LED_BUILTIN, HIGH);
         }else{
            text="Resting";
            textDisplay(40,25,16,text);
            digitalWrite(LED_BUILTIN, LOW);
         }
      }

      
   }
   #ifdef VISP_DEBUG
   //quat2Euler(quaternion);
   euler_angles.x=degrees(rotx);
   euler_angles.y=degrees(roty);
   euler_angles.z=degrees(rotz);
   pub.publish(&euler_angles);
   #endif

   nh.spinOnce();
   delay(10);
}


