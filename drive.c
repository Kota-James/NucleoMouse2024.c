/*
 * drive.c
 *
 *  Created on: Aug 31, 2024
 *      Author: nkota
 */


#include "global.h"


/*==========================================================
    走行系 上位関数
==========================================================*/
/*
    マウスフラグ(MF)
      6Bit:デフォルトインターバルフラグ
      5Bit:減速フラグ
      4Bit:加速フラグ
      3Bit:制御フラグ
      1Bit:二次走行フラグ
*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(void){

  MF.FLAG.CTRL = 1;                   //制御を有効にする
  driveA(PULSE_SEC_HALF);             //半区画のパルス分加速しながら走行。走行後は停止しない
  get_wall_info();                    //壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(void){

  MF.FLAG.CTRL = 1;                   //制御を有効にする
  driveD(PULSE_SEC_HALF);             //半区画のパルス分減速しながら走行。走行後は停止する
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_section
// 1区画分進んで停止する。1区画走行用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void){

  half_sectionA();                    //半区画分加速しながら走行する
  half_sectionD();                    //半区画分減速しながら走行し停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(void){

  MF.FLAG.CTRL = 1;                   //制御を有効にする
  driveU(PULSE_SEC_HALF);             //半区画のパルス分等速走行。走行後は停止しない
  driveU(PULSE_SEC_HALF);             //半区画のパルス分等速走行。走行後は停止しない
  get_wall_info();                    //壁情報を取得
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_R90
// 右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void){

  MF.FLAG.CTRL = 0;                   //制御無効
  drive_set_dir(ROTATE_R);            //右に旋回するようモータの回転方向を設定
  drive_wait();                       //機体が安定するまで待機
  driveC(PULSE_ROT_R90);              //デフォルトインターバルで指定パルス分回転。回転後に停止する
  drive_wait();                       //機体が安定するまで待機
  drive_set_dir(FORWARD);             //前進するようにモータの回転方向を設定
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_L90
// 左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void){

  MF.FLAG.CTRL = 0;                   //制御を無効にする
  drive_set_dir(ROTATE_L);            //左に旋回するようモータの回転方向を設定
  drive_wait();                       //機体が安定するまで待機
  driveC(PULSE_ROT_L90);              //デフォルトインターバルで指定パルス分回転。回転後に停止する
  drive_wait();                       //機体が安定するまで待機
  drive_set_dir(FORWARD);             //前進するようにモータの回転方向を設定
}



//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_L90_S
//本当は右に曲がるはずだが、左に曲がるため関数名だけ変更
//先輩たちも同様の事象が起こっているため、放置
// スラローム走行で右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90_S(void){

  MF.FLAG.CTRL = 1;                   //制御無効
  drive_set_dir(FORWARD);            //右に旋回するようモータの回転方向を設定


  if(ad_fl >= WALL_BASE_FL * 2 && ad_fr >= WALL_BASE_FR * 2){       //前壁が確実にある時b
      while(ad_fl < ROT_START_FL && ad_fr < ROT_START_FR){          //センサ値が基準値に達するまで
          driveU2(1, ARR_OFFSET);         //1パルス分だけ進む
      }
  }else{                                        //前壁がない時b
      driveU2(PULSE_OFFSET, ARR_OFFSET);        //設定したパルス分だけ進む
  }


  MF.FLAG.ROTATER = 1;
  driveR(PULSE_ROT_OUT, PULSE_ROT_IN);      //デフォルトインターバルで指定パルス分回転。回転後に停止する
  MF.FLAG.ROTATER = 0;

  MF.FLAG.CTRL = 1;
  driveU2(PULSE_OFFSET, ARR_OFFSET);    //  オフセット区間
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_R90_S
//本当は左に曲がるはずだが、右に曲がるため関数名だけ変更
//先輩たちも同様の事象が起こっているため、放置
// スラローム走行で左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90_S(void){

  MF.FLAG.CTRL = 1;                   //制御を無効にする
  drive_set_dir(FORWARD);            //左に旋回するようモータの回転方向を設定


  if(ad_fl >= WALL_BASE_FL * 2 && ad_fr >= WALL_BASE_FR * 2){       //前壁が確実にある時
      while(ad_fl < ROT_START_FL && ad_fr < ROT_START_FR){          //センサ値が基準値に達するまで
          driveU2(1, ARR_OFFSET);         //1パルス分だけ進む
      }
  }else{                                        //前壁がない時b
      driveU2(PULSE_OFFSET, ARR_OFFSET);        //設定したパルス分だけ進む
  }


  MF.FLAG.ROTATEL = 1;
  driveR(PULSE_ROT_IN, PULSE_ROT_OUT);      //デフォルトインターバルで指定パルス分回転。回転後に停止する
  MF.FLAG.ROTATEL = 0;

  MF.FLAG.CTRL = 1;
  driveU2(PULSE_OFFSET, ARR_OFFSET);    //  オフセット区間
}



//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_180
// 180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void){

  MF.FLAG.CTRL = 0;                   //制御を無効にする
  drive_set_dir(ROTATE_R);            //左に旋回するようモータの回転方向を設定
  drive_wait();                       //機体が安定するまで待機
  driveC(PULSE_ROT_180);              //デフォルトインターバルで指定パルス分回転。回転後に停止する
  drive_wait();                       //機体が安定するまで待機
  drive_set_dir(FORWARD);             //前進するようにモータの回転方向を設定
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//set_position
// 機体の尻を壁に当てて場所を区画中央に合わせる
// 引数：sw …… 0以外ならget_base()する
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(uint8_t sw){

  MF.FLAG.CTRL = 0;                   //制御を無効にする
  drive_set_dir(BACK);                //後退するようモータの回転方向を設定
  drive_wait();                       //機体が安定するまで待機
  driveC(PULSE_SETPOS_BACK);          //尻を当てる程度に後退。回転後に停止する
  drive_wait();                       //機体が安定するまで待機
  if(sw){
    get_base();
  }
  drive_set_dir(FORWARD);             //前進するようにモータの回転方向を設定
  drive_wait();                       //機体が安定するまで待機
  driveC(PULSE_SETPOS_SET);           //デフォルトインターバルで指定パルス分回転。回転後に停止する
  drive_wait();                       //機体が安定するまで待機
}



/*==========================================================
    走行系 基幹関数
==========================================================*/
/*
    基本仕様として，基幹関数は，
    引数1：dist …… 走行パルス数
*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//driveA
// 指定パルス分加速走行する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(uint16_t dist){

  //====走行====
  //----走行開始----
  MF.FLAG.DECL = 0;
  MF.FLAG.DEF = 0;
  MF.FLAG.ACCL = 1;                   //減速・デフォルトインターバルフラグをクリア，加速フラグをセット
  MF.FLAG.ROTATEL = 0;
  MF.FLAG.ROTATER = 0;
  drive_reset_t_cnt();                //テーブルカウンタをリセット
  drive_start();                      //走行開始

  //----走行----
  while((pulse_l < dist) || (pulse_r < dist));      //左右のモータが指定パルス以上進むまで待機

  drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//driveD
// 指定パルス分減速走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveD(uint16_t dist){

  //====走行====
  //----走行開始----
  MF.FLAG.DECL = 0;
  MF.FLAG.DEF = 0;
  MF.FLAG.ACCL = 0;                   //加速・減速・デフォルトインターバルフラグをクリア
  MF.FLAG.ROTATEL = 0;
  MF.FLAG.ROTATER = 0;
  drive_start();                      //走行開始

  int decl_pulse = (t_cnt_l - min_t_cnt) / T_CNT_DECL;  //減速に必要な距離
  int surplus = (t_cnt_l - min_t_cnt) % T_CNT_DECL;     //減速フラグ時の処理のあまり分を等速走行に追加する

  int16_t c_pulse = dist - decl_pulse;     //等速走行距離 = 総距離 - 減速に必要な距離

  if(c_pulse > 0){
    //----等速走行----
    while((pulse_l < c_pulse + surplus) || (pulse_r < c_pulse + surplus));  //左右のモータが等速分のパルス以上進むまで待機
  }

  //----減速走行----
  MF.FLAG.DECL = 1;                   //減速フラグをセット
  while((pulse_l < dist) || (pulse_r < dist));      //左右のモータが減速分のパルス以上進むまで待機

  //====走行終了====
  drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveU
// 指定パルス分等速走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveU(uint16_t dist){
  //====等速走行開始====
  MF.FLAG.DECL = 0;
  MF.FLAG.DEF = 0;
  MF.FLAG.ACCL = 0;                   //加速・減速・デフォルトインターバルフラグをクリア
  MF.FLAG.ROTATEL = 0;
  MF.FLAG.ROTATER = 0;
  drive_start();                      //走行開始

  //====走行====
  while((pulse_l < dist) || (pulse_r < dist));      //左右のモータが減速分のパルス以上進むまで待機

  //====走行終了====
  drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//driveU2
// 指定パルス分等速走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveU2(uint16_t dist, uint16_t ARR){
  //====等速走行開始====
  MF.FLAG.DECL = 0;
  MF.FLAG.DEF = 0;
  MF.FLAG.ACCL = 0;                   //加速・減速・デフォルトインターバルフラグをクリア
  MF.FLAG.ROTATEL = 0;
  MF.FLAG.ROTATER = 0;
  MF.FLAG.DRIVEU2 = 1;

  __HAL_TIM_SET_AUTORELOAD(&htim16, ARR);
  __HAL_TIM_SET_AUTORELOAD(&htim17, ARR);

  drive_start();                      //走行開始

  //====走行====
  while((pulse_l < dist) || (pulse_r < dist));      //左右のモータが減速分のパルス以上進むまで待機

  //====走行終了====
  drive_stop();
  MF.FLAG.DRIVEU2 = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC
// 指定パルス分デフォルトインターバルで走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(uint16_t dist){

  //====回転開始====
  MF.FLAG.DECL = 0;
  MF.FLAG.DEF = 1;
  MF.FLAG.ACCL = 0;                   //加速・減速フラグをクリア，デフォルトインターバルフラグをセット
  MF.FLAG.ROTATEL = 0;
  MF.FLAG.ROTATER = 0;
  drive_start();                      //走行開始

  //====回転====
  while((pulse_l < dist) || (pulse_r < dist));      //左右のモータが定速分のパルス以上進むまで待機

  drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveR
// 指定パルス分デフォルトインターバルで走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveR(uint16_t dist_l, uint16_t dist_r){

  //====回転開始====
  MF.FLAG.DECL = 0;
  MF.FLAG.DEF = 0;
  MF.FLAG.ACCL = 0;                   //加速・減速フラグをクリア，デフォルトインターバルフラグをセット
  MF.FLAG.CTRL = 0;
  drive_start();                      //走行開始

  //====回転====
  while((pulse_l < dist_l) || (pulse_r < dist_r));      //左右のモータが定速分のパルス以上進むまで待機

  drive_stop();
  MF.FLAG.ROTATEL = 0;
  MF.FLAG.ROTATER = 0;
}

/*==========================================================
    初期化関数・設定関数・その他関数
==========================================================*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_init
// 走行系の変数の初期化，モータードライバ関係のGPIO設定とPWM出力に使うタイマの設定をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void){

  //====走行系の変数の初期化====
  max_t_cnt = MAX_T_CNT;      //テーブルカウンタ最高値初期化     MAX_T_CNTはparams.hにマクロ定義あり
  min_t_cnt = MIN_T_CNT;      //テーブルカウンタ最低値初期化     MIN_T_CNTはparams.hにマクロ定義あり

  pulse_rot_in = PULSE_ROT_IN;
  pulse_rot_out = PULSE_ROT_OUT;
  pulse_offset = PULSE_OFFSET;

  arr_in = ARR_IN;
  arr_out = ARR_OUT;

  rot_start_fl = ROT_START_FL;
  rot_start_fr = ROT_START_FR;

  //====マウスフラグの初期化===
  MF.FLAGS = 0;         //フラグクリア


  //====ステッピングモータの初期化====
  drive_disable_motor();       //ステッピングモータ励磁OFF
  drive_set_dir(FORWARD);     //前進するようにモータの回転方向を設定


  //====PWM出力に使うタイマの設定====
  __HAL_TIM_SET_AUTORELOAD(&htim16, DEFAULT_INTERVAL);
  __HAL_TIM_SET_AUTORELOAD(&htim17, DEFAULT_INTERVAL);


}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_enable_motor
// ステッピングモータを励磁する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_enable_motor(void){
  HAL_GPIO_WritePin(M3_GPIO_Port, M3_Pin, GPIO_PIN_RESET);  //ステッピングモーター励磁ON
  HAL_GPIO_WritePin(M3_2_GPIO_Port, M3_2_Pin, GPIO_PIN_RESET);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_disable_motor
// ステッピングモータの励磁を切る
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_disable_motor(void){
  HAL_GPIO_WritePin(M3_GPIO_Port, M3_Pin, GPIO_PIN_SET);    //ステッピングモーター励磁OFF
  HAL_GPIO_WritePin(M3_2_GPIO_Port, M3_2_Pin, GPIO_PIN_SET);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_reset_t_cnt
// テーブルカウンタをリセット（min_t_cntの値にセット）する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_reset_t_cnt(void){
  t_cnt_l = t_cnt_r = min_t_cnt;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_start
// 走行を開始する
// （pulse_l,pulse_rを0にリセットしてタイマを有効にする）
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void){

  pulse_l = pulse_r = 0;      //走行したパルス数の初期化

  //====PWM出力に使うタイマを有効にする====
  /*--------------------------------------------------------------------
    TIM16 : 16ビットタイマ。左モータの制御に使う。出力はTIM16_CH1
  --------------------------------------------------------------------*/
  __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  /*--------------------------------------------------------------------
    TIM17 : 16ビットタイマ。右モータの制御に使う。出力はTIM17_CH1
  --------------------------------------------------------------------*/
  __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_stop
// 走行を終了する
// （タイマを止めてタイマカウント値を0にリセットする）
// 引数1：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void){

  //====PWM出力に使うタイマを停止させる====
  /*--------------------------------------------------------------------
    TIM16 : 16ビットタイマ。左モータの制御に使う。出力はTIM16_CH1
  --------------------------------------------------------------------*/
  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);

  /*--------------------------------------------------------------------
    TIM17 : 16ビットタイマ。右モータの制御に使う。出力はTIM17_CH1
  --------------------------------------------------------------------*/
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);

  __HAL_TIM_SET_COUNTER(&htim16, 0);  // Reset Counter
  __HAL_TIM_SET_COUNTER(&htim17, 0);  // Reset Counter
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_set_dir
// 進行方向を設定する
// 引数1：d_dir …… どの方向に進行するか  0桁目で左，1桁目で右の方向設定
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_set_dir(uint8_t d_dir){

  //====左モータ====
  switch(d_dir & 0x0f){     //0~3ビット目を取り出す  d_dirと0b00001111を比較
    //----正回転----
    case 0x00:                      //0x00の場合   d_dirが0bxxxx0000のとき
      HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin, MT_FWD_L);  //左を前進方向に設定
      break;
    //----逆回転----
    case 0x01:                      //0x01の場合   d_dirが0bxxxx1111のとき
      HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin, MT_BACK_L); //左を後進方向に設定
      break;
  }
  //====右モータ====
  switch(d_dir & 0xf0){     //4~7ビット目を取り出す   d_dirと0b11110000を比較
    case 0x00:                      //0x00の場合   d_dirが0b0000xxxxの時
      HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin, MT_FWD_R);  //右を前進方向に設定
      HAL_GPIO_WritePin(CW_CCW_R_2_GPIO_Port, CW_CCW_R_2_Pin, MT_FWD_R);
      break;
    case 0x10:                      //0x10の場合   d_dirが0b1111xxxxの時
      HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin, MT_BACK_R); //右を後進方向に設定
      HAL_GPIO_WritePin(CW_CCW_R_2_GPIO_Port, CW_CCW_R_2_Pin, MT_BACK_R);
      break;
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//butt_adjust
//  尻当て調整
//  引数：なし
//  戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void butt_adjust(void){
  rotate_R90();   //rotate 90 degrees to the right. it is defined in drive.c
  drive_wait();   //Wait until the aircraft stabilizes. it is defined in drive.h
  set_position(0);    //Center the aircraft position with the butt rest. it is defined in drive.c
  drive_wait();   //wait until the machine stabilizes
  rotate_L90();   //rotate 90 degrees to the left. it is defined in drive.c
  drive_wait();   //wait until the machine stabilizes
  set_position(0);    //center the machine position with the butt rest.
  drive_wait();   //wait until the machine stabilizes
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//butt_adjust
//  尻当て調整
//  引数：なし
//  戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void butt_adjust_LL(void){
  rotate_L90();   //rotate 90 degrees to the right. it is defined in drive.c
  drive_wait();   //Wait until the aircraft stabilizes. it is defined in drive.h
  set_position(0);    //Center the aircraft position with the butt rest. it is defined in drive.c
  drive_wait();   //wait until the machine stabilizes
  rotate_L90();   //rotate 90 degrees to the left. it is defined in drive.c
  drive_wait();   //wait until the machine stabilizes
  set_position(0);    //center the machine position with the butt rest.
  drive_wait();   //wait until the machine stabilizes
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//test_run
// テスト走行モード
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void test_run(void){

  int mode = 0;
  drive_enable_motor();

  while(1){

    mode = select_mode(mode);

    int i;
    switch(mode){

      case 0:
      /*  //----尻当て----
        printf("Set Position.\n");
        set_position(0);
        break;
      */
        //==== sensor check ====
        //Use this mode to adjust the threshold for the sensor to determine the presence or absence of a wall.
        printf("Sensor Check.\n");
        while(1){
            get_wall_info();
            led_write(wall_info & 0x11, wall_info & 0x88, wall_info & 0x44);
            printf(" ad_l : %lu, ad_fl : %lu, ad_fr : %lu, ad_r : %lu, ad_batt : %lu\n",
                    ad_l, ad_fl, ad_fr, ad_r, ad_batt);
            printf("dif_l : %4d, dif_r : %4d\n", dif_l, dif_r);

            if(wall_info & 0x11){
              printf("Left : [X], ");
            }else{
              printf("Left : [ ], ");
            }

            if(wall_info & 0x88){
              printf("Front : [X], ");
            }else{
              printf("Front : [ ], ");
            }

            if(wall_info & 0x44){
              printf("Right : [X], ");
            }else{
              printf("Right : [ ]\n");
            }

            //Measure battery voltage and store in ad_batt
            //cf.)interrupt.c

            HAL_Delay(333);
        }
        break;
      case 1:
        //----1区画等速走行----
        printf("1 Section, Forward, Constant Speed.\n");
        MF.FLAG.CTRL = 0;           //制御を無効にする
        drive_set_dir(FORWARD);     //前進するようにモータの回転方向を設定
        for(i = 0; i < 1; i++){
          driveC(PULSE_SEC_HALF*2); //一区画のパルス分デフォルトインターバルで走行
          drive_wait();             //機体が安定するまで待機
        }
        break;
      case 2:
        //----右90度回転----
        printf("Rotate R90.\n");
        for(i = 0; i < 16; i++){
          rotate_R90();
        }
        break;
      case 3:
        //----左90度回転----
        printf("Rotate L90.\n");
        for(i = 0; i < 16; i++){
          rotate_L90();
        }
        break;
      case 4:
        //----180度回転----
        printf("Rotate 180.\n");
        for(i = 0; i < 8; i++){
          rotate_180();
        }
        break;
      case 5:
        //----右90度スラローム回転 4周----
        drive_enable_motor();
        butt_adjust();

        MF.FLAGS = 0;
        drive_set_dir(FORWARD);     //前進するようにモータの回転方向を設定

        driveA(2 * PULSE_SEC_HALF);     //半区画のパルス分加速しながら走行
        //driveU(2 * PULSE_SEC_HALF);     //半区画のパルス分等速で走行
        driveD(2 * PULSE_SEC_HALF);     //半区画のパルス分減速しながら走行。走行後は停止する

        rotate_R90();
        driveC(PULSE_SEC_HALF);     //半区画のパルス分デフォルトインターバルで走行

        for(int i = 0; i < 16; i++){
            MF.FLAG.CTRL = 0;
            driveA(PULSE_SEC_HALF);       //半区画のパルス分加速しながら走行
            MF.FLAG.CTRL = 1;
            //one_sectionU();     //1区画のパルス分等速で走行
            MF.FLAG.CTRL = 0;
            driveD(PULSE_SEC_HALF);       //半区画のパルス分減速しながら走行。走行後は停止する
            //HAL_Delay(500);
            rotate_R90_S();
        }
        half_sectionD();

        drive_disable_motor();
        break;
      case 6:
        //----左90度スラローム回転 4周----
        drive_enable_motor();
        butt_adjust();

        MF.FLAGS = 0;
        drive_set_dir(FORWARD);     //前進するようにモータの回転方向を設定

        driveA(0.5 * PULSE_SEC_HALF);     //半区画のパルス分加速しながら走行
        //driveU(2 * PULSE_SEC_HALF);     //半区画のパルス分等速で走行
        driveD(0.5 * PULSE_SEC_HALF);     //半区画のパルス分減速しながら走行。走行後は停止する

        rotate_R90();
        driveC(PULSE_SEC_HALF);     //半区画のパルス分デフォルトインターバルで走行

        for(int i = 0; i < 16; i++){
            MF.FLAG.CTRL = 0;
            driveA(PULSE_SEC_HALF);       //半区画のパルス分加速しながら走行
            MF.FLAG.CTRL = 1;
            //one_sectionU();     //1区画のパルス分等速で走行
            MF.FLAG.CTRL = 0;
            driveD(PULSE_SEC_HALF);       //半区画のパルス分減速しながら走行。走行後は停止する
            //HAL_Delay(500);
            rotate_L90_S();
        }
        half_sectionD();

        drive_disable_motor();
        break;
      case 7:
        //----6区画連続走行----
        printf("6 Section, Forward, Continuous.\n");
        MF.FLAG.CTRL = 0;           //制御を無効にする
        drive_set_dir(FORWARD);     //前進するようにモータの回転方向を設定
        driveA(PULSE_SEC_HALF);     //半区画のパルス分加速しながら走行
        for(i = 0; i < 6-1; i++){
          driveU(PULSE_SEC_HALF*2); //一区画のパルス分等速走行
        }
        driveD(PULSE_SEC_HALF);     //半区画のパルス分減速しながら走行。走行後は停止する
        break;
    }

  }
  drive_disable_motor();
}
