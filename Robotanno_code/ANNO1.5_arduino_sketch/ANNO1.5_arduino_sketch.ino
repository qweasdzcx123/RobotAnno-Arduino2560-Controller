/*
  代码版本 CODE VERSION                         安诺机器机械臂V1.0 ANNO ROBOTIC ARM V1.0 
  公司名称 COMPANY NAME                         安诺机器人（深圳）有限公司 ANNO ROBOTIC(SHENZHEN)CO.,Ltd.
  联系地址 CONTACT ADRESS                       广东省深圳市宝安区西乡固戍南昌第一工业区1栋705安诺机器人 705 Anno Robot, Building 1, Nanchang First Industrial Zone, Gushu, Xixiang, Bao'an District, Shenzhen, Guangdong, China
  联系方式 CONTACT INFORMATION                  +86 0755-36950696
  代码贡献者 CODE CONTRIBUTOR                   滕瑞  Rui Teng      std.tr@qq.com    17732114926                        
*/
// SPEED // millisecond multiplier // raise value to slow robot speeds // DEFAULT = 200
const int SpeedMult = 200;

/* 
MOTOR DIRECTION - motor directions can be changed on the caibration page in the software but can also
be changed here: example using DM542T driver(CW) set to 1 - if using ST6600 or DM320T driver(CCW) set to 0 
DEFAULT = 111011   */

const int J1rotdir = 1;
const int J2rotdir = 1;
const int J3rotdir = 1;
const int J4rotdir = 0;
const int J5rotdir = 1;
const int J6rotdir = 1;
const int TRACKrotdir = 0;



#include <Servo.h>
#include <EEPROM.h>

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

int data_done=0;
int sizes=0;
int datas_flag=0;
int front=0;
int ends=1;
int memory=0;
int size_flag=0;
int analog_flag=1022;
union int_data
{
  int a;
  byte b[2];
  };
union int_datas
{
  int a;
  byte b[3];
  };  
int_data readdata[19]={0};
const int analogPin=A15;
String inData;
String upData;
String function;
int_datas write_count;
int_data addr;
int moveline=0;
int waitline=0;
/*
 * 
 *地址1-1800存储运动指令   1800-1900存储存等待行数与等待动作触发的位置 1900-1998存储等待的时间     
 *Address 1-1800 stores motion instructions 1800-1900 stores the number of waiting lines 
 *and the position of the waiting action trigger 1900-1998 stores the waiting time
 *1999存储运动指令的行数 2000-2001分别为数据写入成功标识符与下载指令的总行数 
 *1999 Stores the number of lines of motion instructions 2000-2001 is 
 *the total number of lines for data write success identifier and download instruction
 *2002-2004记录写入的次数 2005-2006存储运动指令开始读取的地址 
 *2002-2004 Record Write Times 2005-2006 Stores the address at which the motion instruction starts reading
*/
int waitaddr=1800;    
int wait_pos=0;
int wait_minute=0;
int waitpos=0;

char upChar[9]={'M','A','B','C','D','E','F','G','R'};
const int analogpin =A15;
const int moveaddr=1999;
const int J1stepPin = 10; // 2  3
const int J1dirPin = 11;

const int J2stepPin = 8;  //4 5
const int J2dirPin = 9;

const int J3stepPin = 6;//6  7
const int J3dirPin = 7;

const int J4stepPin = 4;//8 9
const int J4dirPin = 5;

const int J5stepPin = 2;//10 11
const int J5dirPin = 3;

const int J6stepPin = 14; //12 13
const int J6dirPin = 15;

const int TRstepPin = 20;
const int TRdirPin = 21;



const int J1calPin = 12; //14 15 16 17 18 19 
const int J2calPin = 13;
const int J3calPin = 16;
const int J4calPin = 17;
const int J5calPin = 18;
const int J6calPin = 19;

const int Input22 = 22;
const int Input23 = 23;
const int Input24 = 24;
const int Input25 = 25;
const int Input26 = 26;
const int Input27 = 27;
const int Input28 = 28;
const int Input29 = 29;
const int Input30 = 30;
const int Input31 = 31;
const int Input32 = 32;
const int Input33 = 33;
const int Input34 = 34;
const int Input35 = 35;
const int Input36 = 36;
const int Input37 = 37;

const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;
const int Output42 = 42;
const int Output43 = 43;
const int Output44 = 44;
const int Output45 = 45;
const int Output46 = 46;
const int Output47 = 47;
const int Output48 = 48;
const int Output49 = 49;
const int Output50 = 50;
const int Output51 = 51;
const int Output52 = 52;
const int Output53 = 53;



void setup() {
  // run once:
  Serial.begin(9600);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(TRstepPin, OUTPUT);
  pinMode(TRdirPin, OUTPUT);
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);

  pinMode(Input22, INPUT);
  pinMode(Input23, INPUT);
  pinMode(Input24, INPUT);
  pinMode(Input25, INPUT);
  pinMode(Input26, INPUT);
  pinMode(Input27, INPUT);
  pinMode(Input28, INPUT);
  pinMode(Input29, INPUT);
  pinMode(Input30, INPUT);
  pinMode(Input31, INPUT);
  pinMode(Input32, INPUT);
  pinMode(Input33, INPUT);
  pinMode(Input34, INPUT);
  pinMode(Input35, INPUT);
  pinMode(Input36, INPUT);
  pinMode(Input37, INPUT);

  pinMode(Output38, OUTPUT);
  pinMode(Output39, OUTPUT);
  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);
  pinMode(Output42, OUTPUT);
  pinMode(Output43, OUTPUT);
  pinMode(Output44, OUTPUT);
  pinMode(Output45, OUTPUT);
  pinMode(Output46, OUTPUT);
  pinMode(Output47, OUTPUT);
  pinMode(Output48, OUTPUT);
  pinMode(Output49, OUTPUT);
  pinMode(Output50, OUTPUT);
  pinMode(Output51, OUTPUT);
  pinMode(Output52, OUTPUT);
  pinMode(Output53, OUTPUT);
 pinMode(analogPin,INPUT_PULLUP); 

  servo0.attach(A0);
  servo1.attach(A1);
  servo2.attach(A2);
  servo3.attach(A3);
  servo4.attach(A4);
  servo5.attach(A5);
  servo6.attach(A6);
  servo7.attach(A7);

  digitalWrite(Output38, HIGH);
  digitalWrite(Output39, HIGH);
  digitalWrite(Output40, HIGH);
  digitalWrite(Output41, HIGH);
  digitalWrite(Output42, HIGH);
  digitalWrite(Output43, HIGH);
  digitalWrite(Output44, HIGH);
  digitalWrite(Output45, HIGH);

  digitalWrite(TRstepPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);
  //digitalWrite(ANNALOGpin,HIGH);
//  analogWrite(ANNALOGpin,HIGH);

  addr.b[0]=EEPROM.read(2005);
  addr.b[1]=EEPROM.read(2006);

}


void loop() {

  //test led
  if (digitalRead(J1calPin) == HIGH || digitalRead(J2calPin) == HIGH || digitalRead(J3calPin) == HIGH || digitalRead(J4calPin) == HIGH || digitalRead(J5calPin) == HIGH || digitalRead(J6calPin) == HIGH)
  {
    digitalWrite(J6dirPin, HIGH);
  }
  
  else
  {
    digitalWrite(J6dirPin, LOW);
  }
  analog_flag = analogRead(analogPin);
  if( analog_flag<500){    //按键按下 Button press
    delay(100);
    analog_flag = analogRead(analogPin);
  }
  
  while (Serial.available() > 0 ||analog_flag<500)    //等待数据读取和按键检测 Waiting for data reading and button detection
  {   
      char recieved = Serial.read();                   //将串口中的数据逐个字节读取并存储到inData中  Read and store the data in the serial port byte by byte into inData
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n' ||analog_flag<500)    // 检测到'\n'后，开始识别存储的那一行代码  
    {
      String function = inData.substring(0, 2);     //截取inData中首两个字节用于识别指令类型 After detecting '\n', start to identify the stored line of code
      /*
      向上位机发送机械臂运动过程，使上位机更新实时机械臂状态
      Send the robot arm movement process to the upper computer,
      so that the upper computer updates the real-time robot arm status.
      */
      
      if(function=="UP")
      {  
         int Line_flag=EEPROM.read(moveaddr);
         for(int i=0;i<Line_flag;i++){
        for(int j=0;j<19;j++)
        {
          readdata[j].b[0]=EEPROM.read(addr.a);
          readdata[j].b[1]=EEPROM.read(addr.a+1);
          addr.a=addr.a+2;
          if(j==0){
          upData+=upChar[0];
          upData+=upChar[1];
          upData+=readdata[j].a;
          
          }
          else if(j==1)
          {
            upData+=upChar[j+1];
            upData+=readdata[j].a;
            }
             else if(j==2)
          {
            upData+=upChar[j+1];
            upData+=readdata[j].a;
            }
             else if(j==3)
          {
            upData+=upChar[j+1];
            upData+=readdata[j].a;
            }
             else if(j==4)
          {
            upData+=upChar[j+1];
            upData+=readdata[j].a;
            }
             else if(j==5)
          {
            upData+=upChar[j+1];
            upData+=readdata[j].a;
            }
                   else if(j==6)
          {
            upData+=upChar[j+1];
            upData+=readdata[j].a;
            }
                      
                      else if(j==12)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;
            }         
            else if(j==13)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;
 
            }     
            else if(j==14)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;

            }         
            else if(j==15)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;
            }        
            else if(j==16)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;
         
            }          
            else if(j==17)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;

            }
             else if(j==18)
          {
            upData+=upChar[8];
            upData+=readdata[j].a;
            upData+=upChar[8];
            upData+= J1rotdir ;
            upData+=upChar[8];
            upData+= J2rotdir ;
            upData+=upChar[8];
            upData+= J3rotdir ;
            upData+=upChar[8];
            upData+= J4rotdir ;
            upData+=upChar[8];
            upData+= J5rotdir ;
            upData+=upChar[8];
            upData+= J6rotdir;
            upData+=upChar[8];
            upData+= TRACKrotdir ; 
            upData+='\n';
            }
                           
        }
          Serial.print(upData);
          upData="";
         }
                upData="";
               addr.b[0]=EEPROM.read(2005);
                addr.b[1]=EEPROM.read(2006);
        
        }
        
        /*
           查看地址中的数据   其中包含有等待的时间 产生等待的位置 命令行行数 命令存储标志 每一行命令的数据
           View the data in the address. It contains the waiting time. 
           The waiting position is generated. The number of command lines. The command storage flag. The data of each line command.  
         */
    if(function=="RD")                      
    { int waits=EEPROM.read(waitaddr+100);
      int waitsA=EEPROM.read(waitaddr+100);
      Serial.print("time1:");
      Serial.print(waits);
      Serial.print("time2:");
      Serial.print(waitsA);
      int waitss=EEPROM.read(waitaddr+2);
      Serial.print("timePOS1:");
      Serial.print(waitss);
      waitss=EEPROM.read(waitaddr+3);
       Serial.print("timePOS2:");
      Serial.print(waitss);
      int data=EEPROM.read(2000);
      Serial.print("Data:");
      Serial.print(data);
      data=EEPROM.read(moveaddr);
      Serial.print("LINE:");
      Serial.print(data);
         for(int i=0;i<data;i++){
        for(int j=0;j<19;j++)
        {
          readdata[j].b[0]=EEPROM.read(addr.a);
          readdata[j].b[1]=EEPROM.read(addr.a+1);
          addr.a=addr.a+2;
           Serial.print("datas:");
          Serial.print(readdata[j].a);
          }
           Serial.print(" \n");
      }
                addr.b[0]=EEPROM.read(2005);
                addr.b[1]=EEPROM.read(2006);
      inData= "" ;
      
    }
 /*
  *  选择模式  Selection mode
  *   在接受到下载指令后，擦写EERPOM并将接受到的每一行命令存储到相应地址
  *    After receiving the download command, erase EERPOM and 
  *    store each line of command received to the corresponding address
 */
     if(function == "MD")
     {   
         
         String function = inData.substring(2,4);
         if(function== "PR")
         {
          Serial.print("OK");
          inData= "";
         }
       //                      进入下载模式  
    if(function=="DO")
      {
         data_done=0;
         datas_flag=0;
         inData= "";                   //下载程序前清空  inData中数据
         Serial.print("OK2");
        while(!data_done){                  //等待命令行发送
        while(Serial.available()>0){
                   char recievedS = Serial.read();
         inData += recievedS ;
         if(recievedS == '\n')
         {
          String functions = inData.substring(0,2);   
          if(functions == "SI")    //读取下载的程序共有多少条命令行
          {    
                int Intindex=inData.indexOf('I');
                int Overindex=inData.indexOf('L');
                sizes=inData.substring(Intindex+1,Overindex).toInt();//得到代码行数
                EEPROM.update(2001,sizes);
                Serial.print(sizes);
                inData= "";
                           }
      if (functions == "DL")//               开始下载
             {
           inData="";
           while(!datas_flag){             //等待命令行数据
           while(Serial.available()>0){
         char recievedS = Serial.read();
         inData += recievedS ;
         if(recievedS == '\n')
           {

            String functiona = inData.substring(0,2);
            if(functiona=="MJ")        //存储运动指令   Store motion instructions 
            {  
                moveline++;
                Serial.print("OK5");
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int TRstart = inData.indexOf('T');
        int Adstart = inData.indexOf('G');
        int Asstart = inData.indexOf('H');
        int Ddstart = inData.indexOf('I');
        int Dsstart = inData.indexOf('K');
        int SPstart = inData.indexOf('S');
        int_data all_data[19]={0};
        int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
        all_data[12].a=J1dir;
        int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
        all_data[13].a=J2dir;
        int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
        all_data[14].a=J3dir;
        int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
        all_data[15].a=J4dir;
        int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
        all_data[16].a=J5dir;
        int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
        all_data[17].a=J6dir;
        int TRdir = inData.substring(TRstart + 1, TRstart + 2).toInt();
        all_data[18].a=TRdir;
        int J1step = inData.substring(J1start + 2, J2start).toInt();
        all_data[0].a=J1step;
        int J2step = inData.substring(J2start + 2, J3start).toInt();
        all_data[1].a=J2step;
        int J3step = inData.substring(J3start + 2, J4start).toInt();
        all_data[2].a=J3step;
        int J4step = inData.substring(J4start + 2, J5start).toInt();
        all_data[3].a=J4step;
        int J5step = inData.substring(J5start + 2, J6start).toInt();
        all_data[4].a=J5step;
        int J6step = inData.substring(J6start + 2, TRstart).toInt();
        all_data[5].a=J6step;
        int TRstep = inData.substring(TRstart + 2, SPstart).toInt();
        all_data[6].a=TRstep;
        int SpeedIn = inData.substring(SPstart + 1, Adstart).toInt();
        all_data[7].a=SpeedIn;
        int ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
        all_data[8].a=ACCdur;
        int ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
        all_data[9].a=ACCspd;
        int DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
        all_data[10].a=DCCdur;
        int DCCspd = inData.substring(Dsstart + 1).toInt();
        all_data[11].a=DCCspd;
                         for(int i=0;i<19;i++)  
              {
                EEPROM.update(addr.a,all_data[i].b[0]);
                EEPROM.update(addr.a+1,all_data[i].b[1]);
                addr.a=addr.a+2;
                  Serial.print("OK6");
               }
                size_flag++;
                inData="";
              }
               
          if (functiona == "WT")  //     存储等待指令  Store wait instruction
      {
        waitline++;
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        wait_minute= WaitTime;
        wait_pos=size_flag;
        EEPROM.update(waitaddr+waitline+1,wait_pos);
        EEPROM.update(waitaddr+100+waitline-1,wait_minute);
        Serial.print("Done");
        inData="";
        size_flag++;
      } 
               
              else if(functiona !="MJ"&&functiona !="WT"){  //退出下载模式   Exit download mode
                datas_flag = 1;
                data_done =1 ;
                size_flag=0;
                waitline=0;
                moveline=0;
                  Serial.print("OK8");
                inData="";
                addr.b[0]=EEPROM.read(2005);
                addr.b[1]=EEPROM.read(2006);
                
                }              
            }
           }
           if(size_flag==sizes)
           {  
              Serial.print("OK9");
            datas_flag =1;
            data_done =1;
            size_flag=0;
            inData="";
            write_count.b[0]=EEPROM.read(2002);   //写入次数 Number of writes
            write_count.b[1]=EEPROM.read(2003);
            write_count.b[2]=EEPROM.read(2004);
            EEPROM.update(moveaddr,moveline);//更新运行程序时间 和 等待时间代码行数 Update run time and wait time code lines
            EEPROM.update(waitaddr+1,waitline);
            addr.b[0]=EEPROM.read(2005);
            addr.b[1]=EEPROM.read(2006);
            waitline=0;
            moveline=0;
            EEPROM.update(2000,1);//写入是否完成  Whether the writing is completed
            /*
             * 写入次数大于99000次更换命令行存储的位置坐标  但并未更新写入次数 所以此处不会更新
             * The number of writes is greater than 99000 times. 
             * The position coordinates stored in the command line are replaced. 
             * The number of writes is not updated, so it will not be updated here.      
            */
            if(write_count.a>99000)           
            {
              int_data flag;
              flag.a=2006;
              EEPROM.write(2005,flag.b[0]);
              EEPROM.write(2006,flag.b[1]);
              int_datas flags;
              flags.a=0;
              EEPROM.write(2002,flags.b[0]);
              EEPROM.write(2003,flags.b[1]);
              EEPROM.write(2004,flags.b[2]);
              
              }        
            
            }
          }
              }
         if(functions !="SI"&&functions !="DL")//   退出下载模式  Exit download mode
             {
              Serial.print("OK10");
                inData="";
                data_done = 1;
                datas_flag = 1;
                size_flag=0;
                addr.b[0]=EEPROM.read(2005);
                addr.b[1]=EEPROM.read(2006);
             }
         
          }
       
                 }
        }
      
           }
           
        else{
            data_done = 1;
           datas_flag = 1;
            inData="";
            addr.b[0]=EEPROM.read(2005);
                addr.b[1]=EEPROM.read(2006);
            }
            inData="";
      }                                             //下载模式结束  End of download mode
    
       
      if (function == "SV")                         //驱动舵机  Drive servo
      {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        if (servoNum == 0)
        {
          servo0.write(servoPOS);
        }
        if (servoNum == 1)
        {
          servo1.write(servoPOS);
        }
        if (servoNum == 2)
        {
          servo2.write(servoPOS);
        }
        if (servoNum == 3)
        {
          servo3.write(servoPOS);
        }
        if (servoNum == 4)
        {
          servo4.write(servoPOS);
        }
        if (servoNum == 5)
        {
          servo5.write(servoPOS);
        }
        if (servoNum == 6)
        {
          servo6.write(servoPOS);
        }
        if (servoNum == 7)
        {
          servo7.write(servoPOS);
        }
        Serial.print("Servo Done");
      }
      //-----COMMAND TO WAIT TIME---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WT")                         
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.print("Done");
      }

      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "JF")
      {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH)
        {
          Serial.println("True\n");
        }
        if (digitalRead(IJInputNum) == LOW)
        {
          Serial.println("False\n");
        }
      }
      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ON")                 //高输出口
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        Serial.print("Done");
      }
      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "OF")                 
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WI")                   //读取输入口
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(100);
        }
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WO")                    
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();

        //String InputStr =  String("Input" + InputNum);
        //uint8_t Input = atoi(InputStr.c_str ());
        while (digitalRead(InputNum) == HIGH) {
          delay(100);
        }
        Serial.print("Done");
      }

      //-----COMMAND TO CALIBRATE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LL")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int SPstart = inData.indexOf('S');
        ///
        int J1caldir = inData.substring(J1start + 1, J1start + 2).toInt();
        int J2caldir = inData.substring(J2start + 1, J2start + 2).toInt();
        int J3caldir = inData.substring(J3start + 1, J3start + 2).toInt();
        int J4caldir = inData.substring(J4start + 1, J4start + 2).toInt();
        int J5caldir = inData.substring(J5start + 1, J5start + 2).toInt();
        int J6caldir = inData.substring(J6start + 1, J6start + 2).toInt();
        ///
        int J1step = (inData.substring(J1start + 2, J2start).toInt());
        int J2step = (inData.substring(J2start + 2, J3start).toInt());
        int J3step = (inData.substring(J3start + 2, J4start).toInt());
        int J4step = (inData.substring(J4start + 2, J5start).toInt());
        int J5step = (inData.substring(J5start + 2, J6start).toInt());
        int J6step = (inData.substring(J6start + 2).toInt());
        ///
        float SpeedIn = inData.substring(SPstart + 1).toFloat();


        //RESET COUNTERS
        int J1done = 0;
        int J2done = 0;
        int J3done = 0;
        int J4done = 0;
        int J5done = 0;
        int J6done = 0;

        String J1calStat = "0";

        //SET DIRECTIONS
        // J1 //
        if (J1rotdir == 1 && J1caldir == 1) {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1rotdir == 0 && J1caldir == 1) {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1rotdir == 1 && J1caldir == 0) {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1rotdir == 0 && J1caldir == 0) {
          digitalWrite(J1dirPin, LOW);
        }

        // J2 //
        if (J2rotdir == 1 && J2caldir == 1) {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2rotdir == 0 && J2caldir == 1) {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2rotdir == 1 && J2caldir == 0) {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2rotdir == 0 && J2caldir == 0) {
          digitalWrite(J2dirPin, LOW);
        }

        // J3 //
        if (J3rotdir == 1 && J3caldir == 1) {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3rotdir == 0 && J3caldir == 1) {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3rotdir == 1 && J3caldir == 0) {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3rotdir == 0 && J3caldir == 0) {
          digitalWrite(J3dirPin, LOW);
        }

        // J4 //
        if (J4rotdir == 1 && J4caldir == 1) {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4rotdir == 0 && J4caldir == 1) {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4rotdir == 1 && J4caldir == 0) {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4rotdir == 0 && J4caldir == 0) {
          digitalWrite(J4dirPin, LOW);
        }

        // J5 //
        if (J5rotdir == 1 && J5caldir == 1) {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5rotdir == 0 && J5caldir == 1) {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5rotdir == 1 && J5caldir == 0) {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5rotdir == 0 && J5caldir == 0) {
          digitalWrite(J5dirPin, LOW);
        }

        // J6 //
        if (J6rotdir == 1 && J6caldir == 1) {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6rotdir == 0 && J6caldir == 1) {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6rotdir == 1 && J6caldir == 0) {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6rotdir == 0 && J6caldir == 0) {
          digitalWrite(J6dirPin, LOW);
        }

        float AdjSpeed = (SpeedIn / 100);
        float CalcRegSpeed = ((SpeedMult * 2) / AdjSpeed);
        int Speed = int(CalcRegSpeed);

        //DRIVE MOTORS FOR CALIBRATION
        while (digitalRead(J1calPin) == LOW && J1done < J1step || digitalRead(J2calPin) == LOW && J2done < J2step || digitalRead(J3calPin) == LOW && J3done < J3step || digitalRead(J4calPin) == LOW && J4done < J4step || digitalRead(J5calPin) == LOW && J5done < J5step || digitalRead(J6calPin) == LOW && J6done < J6step)
        {
          if (J1done < J1step && (digitalRead(J1calPin) == LOW))
          {
            digitalWrite(J1stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J1done < J1step && (digitalRead(J1calPin) == LOW))
          {
            digitalWrite(J1stepPin, HIGH);
            J1done = ++J1done;
          }
          delayMicroseconds(5);
          if (J2done < J2step && (digitalRead(J2calPin) == LOW))
          {
            digitalWrite(J2stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J2done < J2step && (digitalRead(J2calPin) == LOW))
          {
            digitalWrite(J2stepPin, HIGH);
            J2done = ++J2done;
          }
          delayMicroseconds(5);
          if (J3done < J3step && (digitalRead(J3calPin) == LOW))
          {
            digitalWrite(J3stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J3done < J3step && (digitalRead(J3calPin) == LOW))
          {
            digitalWrite(J3stepPin, HIGH);
            J3done = ++J3done;
          }
          delayMicroseconds(5);
          if (J4done < J4step && (digitalRead(J4calPin) == LOW))
          {
            digitalWrite(J4stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J4done < J4step && (digitalRead(J4calPin) == LOW))
          {
            digitalWrite(J4stepPin, HIGH);
            J4done = ++J4done;
          }
          delayMicroseconds(5);
          if (J5done < J5step && (digitalRead(J5calPin) == LOW))
          {
            digitalWrite(J5stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J5done < J5step && (digitalRead(J5calPin) == LOW))
          {
            digitalWrite(J5stepPin, HIGH);
            J5done = ++J5done;;
          }
          delayMicroseconds(5);
          if (J6done < J6step && (digitalRead(J6calPin) == LOW))
          {
            digitalWrite(J6stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J6done < J6step && (digitalRead(J6calPin) == LOW))
          {
            digitalWrite(J6stepPin, HIGH);
            J6done = ++J6done;
          }
          ///////////////DELAY BEFORE RESTARTING LOOP
          delayMicroseconds(Speed);
        }
        //OVERDRIVE
        int OvrDrv = 0;
        while (OvrDrv <= 20)
        {
          if (J1step > 0)
          {
            digitalWrite(J1stepPin, LOW);
          }
          if (J2step > 0)
          {
            digitalWrite(J2stepPin, LOW);
          }
          if (J3step > 0)
          {
            digitalWrite(J3stepPin, LOW);
          }
          if (J4step > 0)
          {
            digitalWrite(J4stepPin, LOW);
          }
          if (J5step > 0)
          {
            digitalWrite(J5stepPin, LOW);
          }
          if (J6step > 0)
          {
            digitalWrite(J6stepPin, LOW);
          }
          ///////////////DELAY AND SET HIGH
          delayMicroseconds(Speed);
          if (J1step > 0)
          {
            digitalWrite(J1stepPin, HIGH);
          }
          if (J2step > 0)
          {
            digitalWrite(J2stepPin, HIGH);
          }
          if (J3step > 0)
          {
            digitalWrite(J3stepPin, HIGH);
          }
          if (J4step > 0)
          {
            digitalWrite(J4stepPin, HIGH);
          }
          if (J5step > 0)
          {
            digitalWrite(J5stepPin, HIGH);
          }
          if (J6step > 0)
          {
            digitalWrite(J6stepPin, HIGH);
          }
          OvrDrv = ++OvrDrv;
          ///////////////DELAY BEFORE RESTARTING LOOP AND SETTING LOW AGAIN
          delayMicroseconds(Speed);
        }
        //SEE IF ANY SWITCHES NOT MADE
        delay(500);
        ///
        int J1pass = 1;
        int J2pass = 1;
        int J3pass = 1;
        int J4pass = 1;
        int J5pass = 1;
        int J6pass = 1;
        ///
        if (J1step > 0) {
          if (digitalRead(J1calPin) == LOW) {
            J1pass = 0;
          }
        }
        if (J2step > 0) {
          if (digitalRead(J2calPin) == LOW) {
            J2pass = 0;
          }
        }
        if (J3step > 0) {
          if (digitalRead(J3calPin) == LOW) {
            J3pass = 0;
          }
        }
        if (J4step > 0) {
          if (digitalRead(J4calPin) == LOW) {
            J4pass = 0;
          }
        }
        if (J5step > 0)
        { if (digitalRead(J5calPin) == LOW) {
            J5pass = 0;
          }
        }
        if (J6step > 0)
        { if (digitalRead(J6calPin) == LOW) {
            J6pass = 0;
          }
        }
        if ((J1pass + J2pass + J3pass + J4pass + J5pass + J6pass) == 6)
        {
          Serial.println("pass\n");
        }
        else
        {
          Serial.println("fail\n");
        }
        inData = ""; // Clear recieved buffer
      }



      //-----COMMAND TO MOVE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MJ")                                 
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int TRstart = inData.indexOf('T');
        int Adstart = inData.indexOf('G');
        int Asstart = inData.indexOf('H');
        int Ddstart = inData.indexOf('I');
        int Dsstart = inData.indexOf('K');
        int SPstart = inData.indexOf('S');
        int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
        int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
        int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
        int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
        int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
        int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
        int TRdir = inData.substring(TRstart + 1, TRstart + 2).toInt();
        int J1step = inData.substring(J1start + 2, J2start).toInt();
        int J2step = inData.substring(J2start + 2, J3start).toInt();
        int J3step = inData.substring(J3start + 2, J4start).toInt();
        int J4step = inData.substring(J4start + 2, J5start).toInt();
        int J5step = inData.substring(J5start + 2, J6start).toInt();
        int J6step = inData.substring(J6start + 2, TRstart).toInt();
        int TRstep = inData.substring(TRstart + 2, SPstart).toInt();
        float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();
        float ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
        float ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
        float DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
        float DCCspd = inData.substring(Dsstart + 1).toInt();
        Serial.print("command recieved1");

    

        //FIND HIGHEST STEP
        int HighStep = J1step;
        if (J2step > HighStep)
        {
          HighStep = J2step;
        }
        if (J3step > HighStep)
        {
          HighStep = J3step;
        }
        if (J4step > HighStep)
        {
          HighStep = J4step;
        }
        if (J5step > HighStep)
        {
          HighStep = J5step;
        }
        if (J6step > HighStep)
        {
          HighStep = J6step;
        }
        if (TRstep > HighStep)
        {
          HighStep = TRstep;
        }

        //FIND ACTIVE JOINTS
        int J1active = 0;
        int J2active = 0;
        int J3active = 0;
        int J4active = 0;
        int J5active = 0;
        int J6active = 0;
        int TRactive = 0;
        int Jactive = 0;

        if (J1step >= 1)
        {
          J1active = 1;
        }
        if (J2step >= 1)
        {
          J2active = 1;
        }
        if (J3step >= 1)
        {
          J3active = 1;
        }
        if (J4step >= 1)
        {
          J4active = 1;
        }
        if (J5step >= 1)
        {
          J5active = 1;
        }
        if (J6step >= 1)
        {
          J6active = 1;
        }
        if (TRstep >= 1)
        {
          TRactive = 1;
        }
        Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

        int J1_PE = 0;
        int J2_PE = 0;
        int J3_PE = 0;
        int J4_PE = 0;
        int J5_PE = 0;
        int J6_PE = 0;
        int TR_PE = 0;

        int J1_SE_1 = 0;
        int J2_SE_1 = 0;
        int J3_SE_1 = 0;
        int J4_SE_1 = 0;
        int J5_SE_1 = 0;
        int J6_SE_1 = 0;
        int TR_SE_1 = 0;

        int J1_SE_2 = 0;
        int J2_SE_2 = 0;
        int J3_SE_2 = 0;
        int J4_SE_2 = 0;
        int J5_SE_2 = 0;
        int J6_SE_2 = 0;
        int TR_SE_2 = 0;

        int J1_LO_1 = 0;
        int J2_LO_1 = 0;
        int J3_LO_1 = 0;
        int J4_LO_1 = 0;
        int J5_LO_1 = 0;
        int J6_LO_1 = 0;
        int TR_LO_1 = 0;

        int J1_LO_2 = 0;
        int J2_LO_2 = 0;
        int J3_LO_2 = 0;
        int J4_LO_2 = 0;
        int J5_LO_2 = 0;
        int J6_LO_2 = 0;
        int TR_LO_2 = 0;

        //reset
        int J1cur = 0;
        int J2cur = 0;
        int J3cur = 0;
        int J4cur = 0;
        int J5cur = 0;
        int J6cur = 0;
        int TRcur = 0;

        int J1_PEcur = 0;
        int J2_PEcur = 0;
        int J3_PEcur = 0;
        int J4_PEcur = 0;
        int J5_PEcur = 0;
        int J6_PEcur = 0;
        int TR_PEcur = 0;

        int J1_SE_1cur = 0;
        int J2_SE_1cur = 0;
        int J3_SE_1cur = 0;
        int J4_SE_1cur = 0;
        int J5_SE_1cur = 0;
        int J6_SE_1cur = 0;
        int TR_SE_1cur = 0;

        int J1_SE_2cur = 0;
        int J2_SE_2cur = 0;
        int J3_SE_2cur = 0;
        int J4_SE_2cur = 0;
        int J5_SE_2cur = 0;
        int J6_SE_2cur = 0;
        int TR_SE_2cur = 0;

        int highStepCur = 0;
        float curDelay = 0;


        //SET DIRECTIONS

        /////// J1 /////////
        if (J1dir == 1 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1dir == 1 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, LOW);
        }

        /////// J2 /////////
        if (J2dir == 1 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2dir == 1 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, LOW);
        }

        /////// J3 /////////
        if (J3dir == 1 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3dir == 1 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, LOW);
        }

        /////// J4 /////////
        if (J4dir == 1 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4dir == 1 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, LOW);
        }

        /////// J5 /////////
        if (J5dir == 1 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5dir == 1 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, LOW);
        }

        /////// J6 /////////
        if (J6dir == 1 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6dir == 1 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, LOW);
        }

        /////// TRACK /////////
        if (TRdir == 1 && TRACKrotdir == 1)
        {
          digitalWrite(TRdirPin, LOW);
        }
        else if (TRdir == 1 && TRACKrotdir == 0)
        {
          digitalWrite(TRdirPin, HIGH);
        }
        else if (TRdir == 0 && TRACKrotdir == 1)
        {
          digitalWrite(TRdirPin, HIGH);
        }
        else if (TRdir == 0 && TRACKrotdir == 0)
        {
          digitalWrite(TRdirPin, LOW);
        }



        /////CALC SPEEDS//////
        float ACCStep = (HighStep * (ACCdur / 100));
        float DCCStep = HighStep - (HighStep * (DCCdur / 100));
        float AdjSpeed = (SpeedIn / 100);
        //REG SPEED
        float CalcRegSpeed = (SpeedMult / AdjSpeed);
        int REGSpeed = int(CalcRegSpeed);

        //ACC SPEED
        float ACCspdT = (ACCspd / 100);
        float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
        float ACCSpeed = (CalcACCSpeed);
        float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

        //DCC SPEED
        float DCCspdT = (DCCspd / 100);
        float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
        float DCCSpeed = (CalcDCCSpeed);
        float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
        DCCSpeed = REGSpeed;




        ///// DRIVE MOTORS /////
        while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
        {

          ////DELAY CALC/////
          if (highStepCur <= ACCStep)
          {
            curDelay = (ACCSpeed / Jactive);
            ACCSpeed = ACCSpeed + ACCinc;
          }
          else if (highStepCur >= DCCStep)
          {
            curDelay = (DCCSpeed / Jactive);
            DCCSpeed = DCCSpeed + DCCinc;
          }
          else
          {
            curDelay = (REGSpeed / Jactive);
          }

          /////// J1 ////////////////////////////////
          ///find pulse every
          if (J1cur < J1step)
          {
            J1_PE = (HighStep / J1step);
            ///find left over 1
            J1_LO_1 = (HighStep - (J1step * J1_PE));
            ///find skip 1
            if (J1_LO_1 > 0)
            {
              J1_SE_1 = (HighStep / J1_LO_1);
            }
            else
            {
              J1_SE_1 = 0;
            }
            ///find left over 2
            if (J1_SE_1 > 0)
            {
              J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
            }
            else
            {
              J1_LO_2 = 0;
            }
            ///find skip 2
            if (J1_LO_2 > 0)
            {
              J1_SE_2 = (HighStep / J1_LO_2);
            }
            else
            {
              J1_SE_2 = 0;
            }
            /////////  J1  ///////////////
            if (J1_SE_2 == 0)
            {
              J1_SE_2cur = (J1_SE_2 + 1);
            }
            if (J1_SE_2cur != J1_SE_2)
            {
              J1_SE_2cur = ++J1_SE_2cur;
              if (J1_SE_1 == 0)
              {
                J1_SE_1cur = (J1_SE_1 + 1);
              }
              if (J1_SE_1cur != J1_SE_1)
              {
                J1_SE_1cur = ++J1_SE_1cur;
                J1_PEcur = ++J1_PEcur;
                if (J1_PEcur == J1_PE)
                {
                  J1cur = ++J1cur;
                  J1_PEcur = 0;
                  digitalWrite(J1stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J1stepPin, HIGH);
                }
              }
              else
              {
                J1_SE_1cur = 0;
              }
            }
            else
            {
              J1_SE_2cur = 0;
            }
          }

          /////// J2 ////////////////////////////////
          ///find pulse every
          if (J2cur < J2step)
          {
            J2_PE = (HighStep / J2step);
            ///find left over 1
            J2_LO_1 = (HighStep - (J2step * J2_PE));
            ///find skip 1
            if (J2_LO_1 > 0)
            {
              J2_SE_1 = (HighStep / J2_LO_1);
            }
            else
            {
              J2_SE_1 = 0;
            }
            ///find left over 2
            if (J2_SE_1 > 0)
            {
              J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
            }
            else
            {
              J2_LO_2 = 0;
            }
            ///find skip 2
            if (J2_LO_2 > 0)
            {
              J2_SE_2 = (HighStep / J2_LO_2);
            }
            else
            {
              J2_SE_2 = 0;
            }
            /////////  J2  ///////////////
            if (J2_SE_2 == 0)
            {
              J2_SE_2cur = (J2_SE_2 + 1);
            }
            if (J2_SE_2cur != J2_SE_2)
            {
              J2_SE_2cur = ++J2_SE_2cur;
              if (J2_SE_1 == 0)
              {
                J2_SE_1cur = (J2_SE_1 + 1);
              }
              if (J2_SE_1cur != J2_SE_1)
              {
                J2_SE_1cur = ++J2_SE_1cur;
                J2_PEcur = ++J2_PEcur;
                if (J2_PEcur == J2_PE)
                {
                  J2cur = ++J2cur;
                  J2_PEcur = 0;
                  digitalWrite(J2stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J2stepPin, HIGH);
                }
              }
              else
              {
                J2_SE_1cur = 0;
              }
            }
            else
            {
              J2_SE_2cur = 0;
            }
          }

          /////// J3 ////////////////////////////////
          ///find pulse every
          if (J3cur < J3step)
          {
            J3_PE = (HighStep / J3step);
            ///find left over 1
            J3_LO_1 = (HighStep - (J3step * J3_PE));
            ///find skip 1
            if (J3_LO_1 > 0)
            {
              J3_SE_1 = (HighStep / J3_LO_1);
            }
            else
            {
              J3_SE_1 = 0;
            }
            ///find left over 2
            if (J3_SE_1 > 0)
            {
              J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
            }
            else
            {
              J3_LO_2 = 0;
            }
            ///find skip 2
            if (J3_LO_2 > 0)
            {
              J3_SE_2 = (HighStep / J3_LO_2);
            }
            else
            {
              J3_SE_2 = 0;
            }
            /////////  J3  ///////////////
            if (J3_SE_2 == 0)
            {
              J3_SE_2cur = (J3_SE_2 + 1);
            }
            if (J3_SE_2cur != J3_SE_2)
            {
              J3_SE_2cur = ++J3_SE_2cur;
              if (J3_SE_1 == 0)
              {
                J3_SE_1cur = (J3_SE_1 + 1);
              }
              if (J3_SE_1cur != J3_SE_1)
              {
                J3_SE_1cur = ++J3_SE_1cur;
                J3_PEcur = ++J3_PEcur;
                if (J3_PEcur == J3_PE)
                {
                  J3cur = ++J3cur;
                  J3_PEcur = 0;
                  digitalWrite(J3stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J3stepPin, HIGH);
                }
              }
              else
              {
                J3_SE_1cur = 0;
              }
            }
            else
            {
              J3_SE_2cur = 0;
            }
          }

          /////// J4 ////////////////////////////////
          ///find pulse every
          if (J4cur < J4step)
          {
            J4_PE = (HighStep / J4step);
            ///find left over 1
            J4_LO_1 = (HighStep - (J4step * J4_PE));
            ///find skip 1
            if (J4_LO_1 > 0)
            {
              J4_SE_1 = (HighStep / J4_LO_1);
            }
            else
            {
              J4_SE_1 = 0;
            }
            ///find left over 2
            if (J4_SE_1 > 0)
            {
              J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
            }
            else
            {
              J4_LO_2 = 0;
            }
            ///find skip 2
            if (J4_LO_2 > 0)
            {
              J4_SE_2 = (HighStep / J4_LO_2);
            }
            else
            {
              J4_SE_2 = 0;
            }
            /////////  J4  ///////////////
            if (J4_SE_2 == 0)
            {
              J4_SE_2cur = (J4_SE_2 + 1);
            }
            if (J4_SE_2cur != J4_SE_2)
            {
              J4_SE_2cur = ++J4_SE_2cur;
              if (J4_SE_1 == 0)
              {
                J4_SE_1cur = (J4_SE_1 + 1);
              }
              if (J4_SE_1cur != J4_SE_1)
              {
                J4_SE_1cur = ++J4_SE_1cur;
                J4_PEcur = ++J4_PEcur;
                if (J4_PEcur == J4_PE)
                {
                  J4cur = ++J4cur;
                  J4_PEcur = 0;
                  digitalWrite(J4stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J4stepPin, HIGH);
                }
              }
              else
              {
                J4_SE_1cur = 0;
              }
            }
            else
            {
              J4_SE_2cur = 0;
            }
          }

          /////// J5 ////////////////////////////////
          ///find pulse every
          if (J5cur < J5step)
          {
            J5_PE = (HighStep / J5step);
            ///find left over 1
            J5_LO_1 = (HighStep - (J5step * J5_PE));
            ///find skip 1
            if (J5_LO_1 > 0)
            {
              J5_SE_1 = (HighStep / J5_LO_1);
            }
            else
            {
              J5_SE_1 = 0;
            }
            ///find left over 2
            if (J5_SE_1 > 0)
            {
              J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
            }
            else
            {
              J5_LO_2 = 0;
            }
            ///find skip 2
            if (J5_LO_2 > 0)
            {
              J5_SE_2 = (HighStep / J5_LO_2);
            }
            else
            {
              J5_SE_2 = 0;
            }
            /////////  J5  ///////////////
            if (J5_SE_2 == 0)
            {
              J5_SE_2cur = (J5_SE_2 + 1);
            }
            if (J5_SE_2cur != J5_SE_2)
            {
              J5_SE_2cur = ++J5_SE_2cur;
              if (J5_SE_1 == 0)
              {
                J5_SE_1cur = (J5_SE_1 + 1);
              }
              if (J5_SE_1cur != J5_SE_1)
              {
                J5_SE_1cur = ++J5_SE_1cur;
                J5_PEcur = ++J5_PEcur;
                if (J5_PEcur == J5_PE)
                {
                  J5cur = ++J5cur;
                  J5_PEcur = 0;
                  digitalWrite(J5stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J5stepPin, HIGH);
                }
              }
              else
              {
                J5_SE_1cur = 0;
              }
            }
            else
            {
              J5_SE_2cur = 0;
            }
          }

          /////// J6 ////////////////////////////////
          ///find pulse every
          if (J6cur < J6step)
          {
            J6_PE = (HighStep / J6step);
            ///find left over 1
            J6_LO_1 = (HighStep - (J6step * J6_PE));
            ///find skip 1
            if (J6_LO_1 > 0)
            {
              J6_SE_1 = (HighStep / J6_LO_1);
            }
            else
            {
              J6_SE_1 = 0;
            }
            ///find left over 2
            if (J6_SE_1 > 0)
            {
              J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
            }
            else
            {
              J6_LO_2 = 0;
            }
            ///find skip 2
            if (J6_LO_2 > 0)
            {
              J6_SE_2 = (HighStep / J6_LO_2);
            }
            else
            {
              J6_SE_2 = 0;
            }
            /////////  J6  ///////////////
            if (J6_SE_2 == 0)
            {
              J6_SE_2cur = (J6_SE_2 + 1);
            }
            if (J6_SE_2cur != J6_SE_2)
            {
              J6_SE_2cur = ++J6_SE_2cur;
              if (J6_SE_1 == 0)
              {
                J6_SE_1cur = (J6_SE_1 + 1);
              }
              if (J6_SE_1cur != J6_SE_1)
              {
                J6_SE_1cur = ++J6_SE_1cur;
                J6_PEcur = ++J6_PEcur;
                if (J6_PEcur == J6_PE)
                {
                  J6cur = ++J6cur;
                  J6_PEcur = 0;
                  digitalWrite(J6stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J6stepPin, HIGH);
                }
              }
              else
              {
                J6_SE_1cur = 0;
              }
            }
            else
            {
              J6_SE_2cur = 0;
            }
          }

          /////// TR ////////////////////////////////
          ///find pulse every
          if (TRcur < TRstep)
          {
            TR_PE = (HighStep / TRstep);
            ///find left over 1
            TR_LO_1 = (HighStep - (TRstep * TR_PE));
            ///find skip 1
            if (TR_LO_1 > 0)
            {
              TR_SE_1 = (HighStep / TR_LO_1);
            }
            else
            {
              TR_SE_1 = 0;
            }
            ///find left over 2
            if (TR_SE_1 > 0)
            {
              TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
            }
            else
            {
              TR_LO_2 = 0;
            }
            ///find skip 2
            if (TR_LO_2 > 0)
            {
              TR_SE_2 = (HighStep / TR_LO_2);
            }
            else
            {
              TR_SE_2 = 0;
            }
            /////////  TR  ///////////////
            if (TR_SE_2 == 0)
            {
              TR_SE_2cur = (TR_SE_2 + 1);
            }
            if (TR_SE_2cur != TR_SE_2)
            {
              TR_SE_2cur = ++TR_SE_2cur;
              if (TR_SE_1 == 0)
              {
                TR_SE_1cur = (TR_SE_1 + 1);
              }
              if (TR_SE_1cur != TR_SE_1)
              {
                TR_SE_1cur = ++TR_SE_1cur;
                TR_PEcur = ++TR_PEcur;
                if (TR_PEcur == TR_PE)
                {
                  TRcur = ++TRcur;
                  TR_PEcur = 0;
                  digitalWrite(TRstepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(TRstepPin, HIGH);
                }
              }
              else
              {
                TR_SE_1cur = 0;
              }
            }
            else
            {
              TR_SE_2cur = 0;
            }
          }


          // inc cur step
          highStepCur = ++highStepCur;


        }
        ////////MOVE COMPLETE///////////
        inData = ""; // Clear recieved buffer
        //Serial.print("Move Done");
      }

   if(function == "DE"||analog_flag<500)   //离线运动  Offline movement
        {
      int Down_flag=EEPROM.read(2000);
       if(Down_flag>0){
        int Line_flag=EEPROM.read(2001);
         for(int i=0;i<Line_flag;i++){
        waitpos=EEPROM.read(waitaddr+2);
        if(i==waitpos)
        {
          wait_minute=EEPROM.read(waitaddr+100);
          delay(wait_minute*1000);
          waitaddr++;
          Serial.print(waitaddr);
          }
        else{  
        for(int j=0;j<19;j++)
        {
          readdata[j].b[0]=EEPROM.read(addr.a);
          readdata[j].b[1]=EEPROM.read(addr.a+1);
          addr.a=addr.a+2;
          }
          
         
         int J1dir = readdata[12].a;
        int J2dir = readdata[13].a;
        int J3dir = readdata[14].a;
        int J4dir = readdata[15].a;
        int J5dir = readdata[16].a;
        int J6dir = readdata[17].a;
        int TRdir = readdata[18].a;
        int J1step = readdata[0].a;
        int J2step = readdata[1].a;
        int J3step = readdata[2].a;
        int J4step = readdata[3].a;
        int J5step = readdata[4].a;
        int J6step = readdata[5].a;
        int TRstep = readdata[6].a;
        float SpeedIn = readdata[7].a;
        float ACCdur = readdata[8].a;
        float ACCspd = readdata[9].a;
        float DCCdur = readdata[10].a;
        float DCCspd = readdata[11].a;
        Serial.print("command recieved2");


           int HighStep = J1step;
        if (J2step > HighStep)
        {
          HighStep = J2step;
        }
        if (J3step > HighStep)
        {
          HighStep = J3step;
        }
        if (J4step > HighStep)
        {
          HighStep = J4step;
        }
        if (J5step > HighStep)
        {
          HighStep = J5step;
        }
        if (J6step > HighStep)
        {
          HighStep = J6step;
        }
        if (TRstep > HighStep)
        {
          HighStep = TRstep;
        }

        //FIND ACTIVE JOINTS
        int J1active = 0;
        int J2active = 0;
        int J3active = 0;
        int J4active = 0;
        int J5active = 0;
        int J6active = 0;
        int TRactive = 0;
        int Jactive = 0;

        if (J1step >= 1)
        {
          J1active = 1;
        }
        if (J2step >= 1)
        {
          J2active = 1;
        }
        if (J3step >= 1)
        {
          J3active = 1;
        }
        if (J4step >= 1)
        {
          J4active = 1;
        }
        if (J5step >= 1)
        {
          J5active = 1;
        }
        if (J6step >= 1)
        {
          J6active = 1;
        }
        if (TRstep >= 1)
        {
          TRactive = 1;
        }
        Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

        int J1_PE = 0;
        int J2_PE = 0;
        int J3_PE = 0;
        int J4_PE = 0;
        int J5_PE = 0;
        int J6_PE = 0;
        int TR_PE = 0;

        int J1_SE_1 = 0;
        int J2_SE_1 = 0;
        int J3_SE_1 = 0;
        int J4_SE_1 = 0;
        int J5_SE_1 = 0;
        int J6_SE_1 = 0;
        int TR_SE_1 = 0;

        int J1_SE_2 = 0;
        int J2_SE_2 = 0;
        int J3_SE_2 = 0;
        int J4_SE_2 = 0;
        int J5_SE_2 = 0;
        int J6_SE_2 = 0;
        int TR_SE_2 = 0;

        int J1_LO_1 = 0;
        int J2_LO_1 = 0;
        int J3_LO_1 = 0;
        int J4_LO_1 = 0;
        int J5_LO_1 = 0;
        int J6_LO_1 = 0;
        int TR_LO_1 = 0;

        int J1_LO_2 = 0;
        int J2_LO_2 = 0;
        int J3_LO_2 = 0;
        int J4_LO_2 = 0;
        int J5_LO_2 = 0;
        int J6_LO_2 = 0;
        int TR_LO_2 = 0;

        //reset
        int J1cur = 0;
        int J2cur = 0;
        int J3cur = 0;
        int J4cur = 0;
        int J5cur = 0;
        int J6cur = 0;
        int TRcur = 0;

        int J1_PEcur = 0;
        int J2_PEcur = 0;
        int J3_PEcur = 0;
        int J4_PEcur = 0;
        int J5_PEcur = 0;
        int J6_PEcur = 0;
        int TR_PEcur = 0;

        int J1_SE_1cur = 0;
        int J2_SE_1cur = 0;
        int J3_SE_1cur = 0;
        int J4_SE_1cur = 0;
        int J5_SE_1cur = 0;
        int J6_SE_1cur = 0;
        int TR_SE_1cur = 0;

        int J1_SE_2cur = 0;
        int J2_SE_2cur = 0;
        int J3_SE_2cur = 0;
        int J4_SE_2cur = 0;
        int J5_SE_2cur = 0;
        int J6_SE_2cur = 0;
        int TR_SE_2cur = 0;

        int highStepCur = 0;
        float curDelay = 0;


        //SET DIRECTIONS

        /////// J1 /////////
        if (J1dir == 1 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1dir == 1 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, LOW);
        }

        /////// J2 /////////
        if (J2dir == 1 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2dir == 1 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, LOW);
        }

        /////// J3 /////////
        if (J3dir == 1 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3dir == 1 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, LOW);
        }

        /////// J4 /////////
        if (J4dir == 1 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4dir == 1 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, LOW);
        }

        /////// J5 /////////
        if (J5dir == 1 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5dir == 1 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, LOW);
        }

        /////// J6 /////////
        if (J6dir == 1 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6dir == 1 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, LOW);
        }

        /////// TRACK /////////
        if (TRdir == 1 && TRACKrotdir == 1)
        {
          digitalWrite(TRdirPin, LOW);
        }
        else if (TRdir == 1 && TRACKrotdir == 0)
        {
          digitalWrite(TRdirPin, HIGH);
        }
        else if (TRdir == 0 && TRACKrotdir == 1)
        {
          digitalWrite(TRdirPin, HIGH);
        }
        else if (TRdir == 0 && TRACKrotdir == 0)
        {
          digitalWrite(TRdirPin, LOW);
        }



        /////CALC SPEEDS//////
        float ACCStep = (HighStep * (ACCdur / 100));
        float DCCStep = HighStep - (HighStep * (DCCdur / 100));
        float AdjSpeed = (SpeedIn / 100);
        //REG SPEED
        float CalcRegSpeed = (SpeedMult / AdjSpeed);
        int REGSpeed = int(CalcRegSpeed);

        //ACC SPEED
        float ACCspdT = (ACCspd / 100);
        float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
        float ACCSpeed = (CalcACCSpeed);
        float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

        //DCC SPEED
        float DCCspdT = (DCCspd / 100);
        float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
        float DCCSpeed = (CalcDCCSpeed);
        float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
        DCCSpeed = REGSpeed;




        ///// DRIVE MOTORS /////
        while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
        {

          ////DELAY CALC/////
          if (highStepCur <= ACCStep)
          {
            curDelay = (ACCSpeed / Jactive);
            ACCSpeed = ACCSpeed + ACCinc;
          }
          else if (highStepCur >= DCCStep)
          {
            curDelay = (DCCSpeed / Jactive);
            DCCSpeed = DCCSpeed + DCCinc;
          }
          else
          {
            curDelay = (REGSpeed / Jactive);
          }

          /////// J1 ////////////////////////////////
          ///find pulse every
          if (J1cur < J1step)
          {
            J1_PE = (HighStep / J1step);
            ///find left over 1
            J1_LO_1 = (HighStep - (J1step * J1_PE));
            ///find skip 1
            if (J1_LO_1 > 0)
            {
              J1_SE_1 = (HighStep / J1_LO_1);
            }
            else
            {
              J1_SE_1 = 0;
            }
            ///find left over 2
            if (J1_SE_1 > 0)
            {
              J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
            }
            else
            {
              J1_LO_2 = 0;
            }
            ///find skip 2
            if (J1_LO_2 > 0)
            {
              J1_SE_2 = (HighStep / J1_LO_2);
            }
            else
            {
              J1_SE_2 = 0;
            }
            /////////  J1  ///////////////
            if (J1_SE_2 == 0)
            {
              J1_SE_2cur = (J1_SE_2 + 1);
            }
            if (J1_SE_2cur != J1_SE_2)
            {
              J1_SE_2cur = ++J1_SE_2cur;
              if (J1_SE_1 == 0)
              {
                J1_SE_1cur = (J1_SE_1 + 1);
              }
              if (J1_SE_1cur != J1_SE_1)
              {
                J1_SE_1cur = ++J1_SE_1cur;
                J1_PEcur = ++J1_PEcur;
                if (J1_PEcur == J1_PE)
                {
                  J1cur = ++J1cur;
                  J1_PEcur = 0;
                  digitalWrite(J1stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J1stepPin, HIGH);
                }
              }
              else
              {
                J1_SE_1cur = 0;
              }
            }
            else
            {
              J1_SE_2cur = 0;
            }
          }

          /////// J2 ////////////////////////////////
          ///find pulse every
          if (J2cur < J2step)
          {
            J2_PE = (HighStep / J2step);
            ///find left over 1
            J2_LO_1 = (HighStep - (J2step * J2_PE));
            ///find skip 1
            if (J2_LO_1 > 0)
            {
              J2_SE_1 = (HighStep / J2_LO_1);
            }
            else
            {
              J2_SE_1 = 0;
            }
            ///find left over 2
            if (J2_SE_1 > 0)
            {
              J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
            }
            else
            {
              J2_LO_2 = 0;
            }
            ///find skip 2
            if (J2_LO_2 > 0)
            {
              J2_SE_2 = (HighStep / J2_LO_2);
            }
            else
            {
              J2_SE_2 = 0;
            }
            /////////  J2  ///////////////
            if (J2_SE_2 == 0)
            {
              J2_SE_2cur = (J2_SE_2 + 1);
            }
            if (J2_SE_2cur != J2_SE_2)
            {
              J2_SE_2cur = ++J2_SE_2cur;
              if (J2_SE_1 == 0)
              {
                J2_SE_1cur = (J2_SE_1 + 1);
              }
              if (J2_SE_1cur != J2_SE_1)
              {
                J2_SE_1cur = ++J2_SE_1cur;
                J2_PEcur = ++J2_PEcur;
                if (J2_PEcur == J2_PE)
                {
                  J2cur = ++J2cur;
                  J2_PEcur = 0;
                  digitalWrite(J2stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J2stepPin, HIGH);
                }
              }
              else
              {
                J2_SE_1cur = 0;
              }
            }
            else
            {
              J2_SE_2cur = 0;
            }
          }

          /////// J3 ////////////////////////////////
          ///find pulse every
          if (J3cur < J3step)
          {
            J3_PE = (HighStep / J3step);
            ///find left over 1
            J3_LO_1 = (HighStep - (J3step * J3_PE));
            ///find skip 1
            if (J3_LO_1 > 0)
            {
              J3_SE_1 = (HighStep / J3_LO_1);
            }
            else
            {
              J3_SE_1 = 0;
            }
            ///find left over 2
            if (J3_SE_1 > 0)
            {
              J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
            }
            else
            {
              J3_LO_2 = 0;
            }
            ///find skip 2
            if (J3_LO_2 > 0)
            {
              J3_SE_2 = (HighStep / J3_LO_2);
            }
            else
            {
              J3_SE_2 = 0;
            }
            /////////  J3  ///////////////
            if (J3_SE_2 == 0)
            {
              J3_SE_2cur = (J3_SE_2 + 1);
            }
            if (J3_SE_2cur != J3_SE_2)
            {
              J3_SE_2cur = ++J3_SE_2cur;
              if (J3_SE_1 == 0)
              {
                J3_SE_1cur = (J3_SE_1 + 1);
              }
              if (J3_SE_1cur != J3_SE_1)
              {
                J3_SE_1cur = ++J3_SE_1cur;
                J3_PEcur = ++J3_PEcur;
                if (J3_PEcur == J3_PE)
                {
                  J3cur = ++J3cur;
                  J3_PEcur = 0;
                  digitalWrite(J3stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J3stepPin, HIGH);
                }
              }
              else
              {
                J3_SE_1cur = 0;
              }
            }
            else
            {
              J3_SE_2cur = 0;
            }
          }

          /////// J4 ////////////////////////////////
          ///find pulse every
          if (J4cur < J4step)
          {
            J4_PE = (HighStep / J4step);
            ///find left over 1
            J4_LO_1 = (HighStep - (J4step * J4_PE));
            ///find skip 1
            if (J4_LO_1 > 0)
            {
              J4_SE_1 = (HighStep / J4_LO_1);
            }
            else
            {
              J4_SE_1 = 0;
            }
            ///find left over 2
            if (J4_SE_1 > 0)
            {
              J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
            }
            else
            {
              J4_LO_2 = 0;
            }
            ///find skip 2
            if (J4_LO_2 > 0)
            {
              J4_SE_2 = (HighStep / J4_LO_2);
            }
            else
            {
              J4_SE_2 = 0;
            }
            /////////  J4  ///////////////
            if (J4_SE_2 == 0)
            {
              J4_SE_2cur = (J4_SE_2 + 1);
            }
            if (J4_SE_2cur != J4_SE_2)
            {
              J4_SE_2cur = ++J4_SE_2cur;
              if (J4_SE_1 == 0)
              {
                J4_SE_1cur = (J4_SE_1 + 1);
              }
              if (J4_SE_1cur != J4_SE_1)
              {
                J4_SE_1cur = ++J4_SE_1cur;
                J4_PEcur = ++J4_PEcur;
                if (J4_PEcur == J4_PE)
                {
                  J4cur = ++J4cur;
                  J4_PEcur = 0;
                  digitalWrite(J4stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J4stepPin, HIGH);
                }
              }
              else
              {
                J4_SE_1cur = 0;
              }
            }
            else
            {
              J4_SE_2cur = 0;
            }
          }

          /////// J5 ////////////////////////////////
          ///find pulse every
          if (J5cur < J5step)
          {
            J5_PE = (HighStep / J5step);
            ///find left over 1
            J5_LO_1 = (HighStep - (J5step * J5_PE));
            ///find skip 1
            if (J5_LO_1 > 0)
            {
              J5_SE_1 = (HighStep / J5_LO_1);
            }
            else
            {
              J5_SE_1 = 0;
            }
            ///find left over 2
            if (J5_SE_1 > 0)
            {
              J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
            }
            else
            {
              J5_LO_2 = 0;
            }
            ///find skip 2
            if (J5_LO_2 > 0)
            {
              J5_SE_2 = (HighStep / J5_LO_2);
            }
            else
            {
              J5_SE_2 = 0;
            }
            /////////  J5  ///////////////
            if (J5_SE_2 == 0)
            {
              J5_SE_2cur = (J5_SE_2 + 1);
            }
            if (J5_SE_2cur != J5_SE_2)
            {
              J5_SE_2cur = ++J5_SE_2cur;
              if (J5_SE_1 == 0)
              {
                J5_SE_1cur = (J5_SE_1 + 1);
              }
              if (J5_SE_1cur != J5_SE_1)
              {
                J5_SE_1cur = ++J5_SE_1cur;
                J5_PEcur = ++J5_PEcur;
                if (J5_PEcur == J5_PE)
                {
                  J5cur = ++J5cur;
                  J5_PEcur = 0;
                  digitalWrite(J5stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J5stepPin, HIGH);
                }
              }
              else
              {
                J5_SE_1cur = 0;
              }
            }
            else
            {
              J5_SE_2cur = 0;
            }
          }

          /////// J6 ////////////////////////////////
          ///find pulse every
          if (J6cur < J6step)
          {
            J6_PE = (HighStep / J6step);
            ///find left over 1
            J6_LO_1 = (HighStep - (J6step * J6_PE));
            ///find skip 1
            if (J6_LO_1 > 0)
            {
              J6_SE_1 = (HighStep / J6_LO_1);
            }
            else
            {
              J6_SE_1 = 0;
            }
            ///find left over 2
            if (J6_SE_1 > 0)
            {
              J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
            }
            else
            {
              J6_LO_2 = 0;
            }
            ///find skip 2
            if (J6_LO_2 > 0)
            {
              J6_SE_2 = (HighStep / J6_LO_2);
            }
            else
            {
              J6_SE_2 = 0;
            }
            /////////  J6  ///////////////
            if (J6_SE_2 == 0)
            {
              J6_SE_2cur = (J6_SE_2 + 1);
            }
            if (J6_SE_2cur != J6_SE_2)
            {
              J6_SE_2cur = ++J6_SE_2cur;
              if (J6_SE_1 == 0)
              {
                J6_SE_1cur = (J6_SE_1 + 1);
              }
              if (J6_SE_1cur != J6_SE_1)
              {
                J6_SE_1cur = ++J6_SE_1cur;
                J6_PEcur = ++J6_PEcur;
                if (J6_PEcur == J6_PE)
                {
                  J6cur = ++J6cur;
                  J6_PEcur = 0;
                  digitalWrite(J6stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J6stepPin, HIGH);
                }
              }
              else
              {
                J6_SE_1cur = 0;
              }
            }
            else
            {
              J6_SE_2cur = 0;
            }
          }

          /////// TR ////////////////////////////////
          ///find pulse every
          if (TRcur < TRstep)
          {
            TR_PE = (HighStep / TRstep);
            ///find left over 1
            TR_LO_1 = (HighStep - (TRstep * TR_PE));
            ///find skip 1
            if (TR_LO_1 > 0)
            {
              TR_SE_1 = (HighStep / TR_LO_1);
            }
            else
            {
              TR_SE_1 = 0;
            }
            ///find left over 2
            if (TR_SE_1 > 0)
            {
              TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
            }
            else
            {
              TR_LO_2 = 0;
            }
            ///find skip 2
            if (TR_LO_2 > 0)
            {
              TR_SE_2 = (HighStep / TR_LO_2);
            }
            else
            {
              TR_SE_2 = 0;
            }
            /////////  TR  ///////////////
            if (TR_SE_2 == 0)
            {
              TR_SE_2cur = (TR_SE_2 + 1);
            }
            if (TR_SE_2cur != TR_SE_2)
            {
              TR_SE_2cur = ++TR_SE_2cur;
              if (TR_SE_1 == 0)
              {
                TR_SE_1cur = (TR_SE_1 + 1);
              }
              if (TR_SE_1cur != TR_SE_1)
              {
                TR_SE_1cur = ++TR_SE_1cur;
                TR_PEcur = ++TR_PEcur;
                if (TR_PEcur == TR_PE)
                {
                  TRcur = ++TRcur;
                  TR_PEcur = 0;
                  digitalWrite(TRstepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(TRstepPin, HIGH);
                }
              }
              else
              {
                TR_SE_1cur = 0;
              }
            }
            else
            {
              TR_SE_2cur = 0;
            }
          }


          // inc cur step
          highStepCur = ++highStepCur;


        }
        ////////MOVE COMPLETE///////////
        inData = ""; // Clear recieved buffer
        //Serial.print("Move Done");   
        }
      }
                waitaddr=1800;
                addr.b[0]=EEPROM.read(2005);
                addr.b[1]=EEPROM.read(2006);
            }
             inData= "" ;
             analog_flag=1022;
          }
  
      
      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}
