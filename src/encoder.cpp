/***************************************************************************************
 *           Rotaryencoder Speed Checker
 *            Author : Tetsuro Hirayama
 *
 * 一定の周期(64msec, 128msec, 512msec, 1024msec, 4096ms)で特定の処理を
 * 行うためのフレームワーク
 *
 * 適当なサンプリング周期を作成して、REの角変位をそれに同調させて周速度を計算する。
 *
 * 以下の各関数が各周期毎に実行される。
 *  execute64msTask()   -> 64msec毎に実行
 *
 *　128msec, 512msec, 1024msec, 4096msは使えるけど使う必要がないので省略
 *****************************************************************************************/
#include <ros/ros.h>		//ROSに必要な関数を取り込むため必須！
#include <pigpiod_if2.h>	//ラズパイのピンにアクセスするためのライブラリ「PIGPIO」関数を取り込む

#define pinA 6
#define pinB 7
/* Encoder用変数定義 */
float rot_count = 0;  // 一周を300countとして状態が4ステップなので1200ステップに拡張。
long sum = 0;
int direction = 0;
int parse;

int main(int argc, char **argv)	//int main(void)だとROSの関数が使えない
{
	ros::init(argc, argv, "led_test_node");	//ノード名の初期化「ros::init(argc, argv, "起動時に指定するノード名");」(必須！)
	ros::NodeHandle nh;	//ROSシステムとの通信のためのノードハンドルを宣言（必須！）
	int pi = pigpio_start(0, 0);	//PIGPIOを使うときは必ず宣言
	int current_a;
	int current_b;
	int now_a = 0, now_b = 0;
	set_pull_up_down(pi, pinA, PI_PUD_DOWN);	//GPIOピンの設定「set_mode(pi, "ピン番号", "PI_OUTPUTかPI_INPUT");」
	set_pull_up_down(pi, pinB, PI_PUD_DOWN);	//GPIOピンの設定「set_mode(pi, "ピン番号", "PI_OUTPUTかPI_INPUT");」
	current_a = gpio_read(pi, pinA);
	current_b = gpio_read(pi, pinB);
	/* rotary振り分け関数 */
    if(( current_a == 0 ) && ( current_b == 0 ))    // A,Bとも０
    {
      parse = 0;
    }
    if(( current_a == 1 ) && ( current_b == 0 ))    // A=1 B = 0 CCW
    {
      parse = 1;
    }
    if(( current_a == 1 ) && ( current_b == 1 ))    // A,Bとも０
    {
      parse = 2;
    }
    if(( current_a == 0 ) && ( current_b == 1 ))    // A,Bとも０
    {
      parse = 3;
    }
    int callback(pi, pinA, 2, rotary_changedPin );
    int callback(pi, pinB, 2, rotary_changedPin );
    while(rod::ok()){				//ctrl + Cが押されるまで繰り返す
    	now_a = digitalRead(pinA);
 		now_b = digitalRead(pinB);
  		//シリアルコンソールに現在の値を出力。
  		printf(rot_count); //printfで良い。
 		printf("      ");
  		sum=rot_count/2500;
  		printf(sum);
  		printf(" m \t ");//1mごとに表示
  		printf("      ");
  		printf("%d\n", parse);
    }
    pigpio_stop(pi);	//PIGPIOデーモンとの接続を終了しライブラリが使用するリソースを解放(PIGPIOを使うときは必ず宣言)
    return 0;
}

//// main文END //// 以下サブルーチン ////////////////////////////////////////////////////////////////////////
//pinの割り込み処理
void rotary_changedPin(void)
{
  int now_a;
  int now_b;
  now_a = gpio_read(pi, pinA);
  now_b = gpio_read(pi, pinB);
  if( parse == 0 )
  {
    if(( now_a == 1 ) && ( now_b == 0 )) // reverse=CCW
    {
      rot_count+=0.25;
      direction = 0; //CCW
      parse = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
        rot_count-=0.25;
        direction = 1;    //CW
        parse = 1;//1
        return;
    } else {
         // fatal error
    }
  }
  if( parse == 1 )
  {
    if(( now_a == 0 ) && (now_b == 0 )) // reverse = CCW
    {
      rot_count+=0.25;
      direction= 0;    //CCW
      parse = 0;
      return;
    } else if (( now_a == 1 ) && ( now_b == 1)) //foward = CW
    {
        rot_count-=0.25;
        direction = 1;    // CW
        parse = 2;
        return;
    } else {
       // fatal error
    }
  }
 /////////////////////////////////////////////////////////////////
  if( parse == 2 )
  {
    if (( now_a == 1 ) && ( now_b == 0)) //foward = CW
    {
        rot_count-=0.25;
        direction = 0;    // CW
        parse = 3;
        return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
        rot_count+=0.25;
        direction = 1;    //CW
        parse = 1;//1
        return;
    } else {
         // fatal error
    }
  }
  ////////////////////////////////////////////////////////////////////
  if( parse == 3 )
  {
    if(( now_a == 1 ) && (now_b == 1 )) // reverse = CCW
    {
      rot_count+=0.25;
      direction= 0;    //CCW
      parse = 2;
      return;
    } else if (( now_a == 0 ) && ( now_b == 0)) //foward = CW
    {
        rot_count-=0.25;
        direction = 1;    // CW
        parse = 0;
        return;
    } else {
       // fatal error
    }
  }
}
