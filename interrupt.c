/*
 * interrupt.c
 *
 *  Created on: Aug 31, 2024
 *      Author: nkota
 */

#include "global.h"

#include "interrupt.h"



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint32_t pre_ad_l = 0, pre_ad_r = 0;        //前回のセンサ値を格納　静的変数として宣言し、0で初期化
  int32_t change_value_l, change_value_r;   //前回のセンサ値との差を入れる変数


    /*==========================================================
        走行用タイマ割り込み
    ==========================================================*/

    /**********
    ---走行の仕組み---
    ステッピングモータの動作制御は16bitタイマで行われている。各16bitタイマの設定は，
    ・カウント開始からCCR1までの間は出力ピンがLowになる
    ・CCR1に達すると出力ピンがHighになる
    ・ARRに達すると割り込みを生成+タイマカウンタをリセット
    となっている（drive_init函数参照）
    モータドライバの（RefをHighにした状態で）ClockをHighにすることで一定角度回転し，
    Lowに戻した後またHighにすることでまた一定角度回転する。
    Clockにはタイマの出力ピンを繋いであるので，タイマの周期を速くするほど回転も速くなる。
    このプログラムではカウント開始からCCR1の間を一定にして（モータドライバがHighを認識する分長さが必要），
    カウント開始からARRの間を調整することで速度を変化させている。
    加減速等の状態はMF.FLAG構造体（global.hで定義）で管理されている。
    加減速について，事前にExcelで計算した時間（割り込み）ごとのARRの値をtable[]配列に記録しておく。
    （配列の値はtable.hにExcelからコピー&ペーストをして設定する。table[]の定義はdrive.h参照）
    今加減速のどの段階なのか（table[]の要素番号・インデックス）はt_cnt_l,t_cnt_rで記録している。
    **********/

    if (htim->Instance == htim16.Instance) {
        /*--------------------------------------------------------------------
            16ビットタイマーTIM16の割り込み処理，左モータの管理をおこなう
        --------------------------------------------------------------------*/

        pulse_l++; // 左パルスのカウンタをインクリメント

        //====加減速処理====
        //----減速処理----
        if (MF.FLAG.DECL) { // 減速フラグが立っている場合
            t_cnt_l = max(t_cnt_l - T_CNT_DECL, min_t_cnt);
        }
        //----加速処理----
        else if (MF.FLAG.ACCL) { // 加速フラグが立っている場合
            t_cnt_l = min(t_cnt_l + T_CNT_ACCL, max_t_cnt);
        }

        //----デフォルトインターバル----
        if (MF.FLAG.DEF) { // デフォルトインターバルフラグが立っている場合
            __HAL_TIM_SET_AUTORELOAD(
                &htim16,
                DEFAULT_INTERVAL -
                    dl); // デフォルトインターバルに制御を加えた値に設定

        }else if(MF.FLAG.ROTATEL){
            __HAL_TIM_SET_AUTORELOAD(&htim16, ARR_IN);
        }

        else if(MF.FLAG.ROTATER){
            __HAL_TIM_SET_AUTORELOAD(&htim16, ARR_OUT);
        }
        else if(MF.FLAG.DRIVEU2){
          ;
        }

        //----それ以外の時はテーブルカウンタの指し示すインターバル----
        else {
            __HAL_TIM_SET_AUTORELOAD(
                &htim16, table[t_cnt_l] - dl); // 左モータインターバル設定
        }

    } /* if (htim->Instance == htim16.Instance) */

    else if (htim->Instance == htim17.Instance) {
        /*--------------------------------------------------------------------
            16ビットタイマーTIM17の割り込み処理，右モータの管理をおこなう
        --------------------------------------------------------------------*/

        pulse_r++; // 右パルスのカウンタをインクリメント

        //====加減速処理====
        //----減速処理----
        if (MF.FLAG.DECL) { // 減速フラグが立っている場合
            t_cnt_r = max(t_cnt_r - T_CNT_DECL, min_t_cnt);
        }
        //----加速処理----
        else if (MF.FLAG.ACCL) { // 加速フラグが立っている場合
            t_cnt_r = min(t_cnt_r + T_CNT_ACCL, max_t_cnt);
        }

        //----デフォルトインターバル----
        if (MF.FLAG.DEF) { // デフォルトインターバルフラグが立っている場合
            __HAL_TIM_SET_AUTORELOAD(
                &htim17,
                DEFAULT_INTERVAL -
                    dr); // デフォルトインターバルに制御を加えた値に設定
        }

        else if(MF.FLAG.ROTATEL){
            __HAL_TIM_SET_AUTORELOAD(&htim17, ARR_OUT);
        }

        else if(MF.FLAG.ROTATER){
            __HAL_TIM_SET_AUTORELOAD(&htim17, ARR_IN);
        }
        else if(MF.FLAG.DRIVEU2){
          ;
        }

        //----それ以外の時はテーブルカウンタの指し示すインターバル----
        else {
            __HAL_TIM_SET_AUTORELOAD(
                &htim17, table[t_cnt_r] - dr); // 右モータインターバル設定
        }

    } /* if (htim->Instance == htim17.Instance) */

    else if (htim->Instance == htim6.Instance) {
        /*==========================================================
            タスク実行用タイマ割り込み
        ==========================================================*/
        /*----------------------------------------------------------
            16ビットタイマーTIM6の割り込み処理，センサ値の取得，制御比例値の計算をおこなう
        ----------------------------------------------------------*/

        switch (tp) {

        //----センサ処理----
        case 0:
            // 左右センサ値の取得
            HAL_GPIO_WritePin(IR_SIDE_GPIO_Port, IR_SIDE_Pin,
                              GPIO_PIN_SET); // 発光部LEDをON
            tim6_wait_us(IR_WAIT_US);    // 光が強まるまで少し待機
            ad_r = get_sensor_value_r(); // 右センサ値を記録
            ad_l = get_sensor_value_l(); // 左センサ値を記録
            HAL_GPIO_WritePin(IR_SIDE_GPIO_Port, IR_SIDE_Pin,
                              GPIO_PIN_RESET); // 発光部LEDをOFF
            break;

        case 1:
            // 正面センサ値の取得
            HAL_GPIO_WritePin(IR_FRONT_GPIO_Port, IR_FRONT_Pin,
                              GPIO_PIN_SET); // 発光部LEDをON
            tim6_wait_us(IR_WAIT_US);      // 光が強まるまで少し待機
            ad_fr = get_sensor_value_fr(); // 右前センサ値を記録
            ad_fl = get_sensor_value_fl(); // 左前センサ値を記録
            HAL_GPIO_WritePin(IR_FRONT_GPIO_Port, IR_FRONT_Pin,
                              GPIO_PIN_RESET); // 発光部LEDをOFF

            // バッテリー電圧の取得
            ad_batt = get_batt_vol();
            if (ad_batt < BATT_LOW_VOL) {
                low_vol_count++;
                if (low_vol_count > 1000) {
                    MF.FLAG.LOWVOL = 1;
                }
            } else {
                MF.FLAG.LOWVOL = 0;
            }

            if (MF.FLAG.LOWVOL) {
                led_write_2(1);
            } else {
                led_write_2(0);
            }

            break;

        //----制御処理----
        case 2:
            // 制御フラグがあれば制御
            if (MF.FLAG.CTRL) {
                // 比例制御値を一次保存する変数を宣言し0で初期化
                int16_t dl_tmp = 0, dr_tmp = 0;
                uint16_t wall_base_l, wall_base_r;
                uint16_t ctrl_base_l, ctrl_base_r;
                // 基準値からの差を見る
                dif_l = (int32_t)ad_l - base_l;
                dif_r = (int32_t)ad_r - base_r;

                //センサ値が急激な変化をするかにより閾値を変更　壁の切れ目対策

                change_value_l = ad_l - pre_ad_l;     //前回のセンサ値との差
                change_value_r = ad_r - pre_ad_r;     //前回のセンサ値との差

                if(change_value_l > RAPID_CHANGE || -1 * change_value_l > RAPID_CHANGE){      //左壁のセンサ値が急激に変化したとき
                    wall_base_l = WALL_BASE_L * 20;    //壁有判断の閾値を上げる
                    ctrl_base_l = WALL_BASE_L * 3;    //制御の条件を上げる
                }else{
                    wall_base_l = WALL_BASE_L;         //壁有判断の閾値は基準値
                    ctrl_base_l = CTRL_BASE_L;          //制御の条件は基準通り
                }

                if(change_value_r > RAPID_CHANGE || -1 * change_value_r > RAPID_CHANGE){      //右壁のセンサ値が急激に変化したとき
                    wall_base_r = WALL_BASE_L * 20;    //壁有判断の閾値を上げる
                    ctrl_base_r = WALL_BASE_L * 3;    //制御の条件を上げる
                }else{
                    wall_base_r = WALL_BASE_R;         //壁有判断の閾値は基準値
                    ctrl_base_r = CTRL_BASE_R;          //制御の条件は基準通り
                }
/*
                if((change_value_l > RAPID_CHANGE || -1 * change_value_l > RAPID_CHANGE) && (change_value_r > RAPID_CHANGE || -1 * change_value_r > RAPID_CHANGE)){
                    led_write(1, 0, 1);
                }else if(change_value_l > RAPID_CHANGE || -1 * change_value_l > RAPID_CHANGE){
                    led_write(1, 0, 0);
                }else if(change_value_r > RAPID_CHANGE || -1 * change_value_r > RAPID_CHANGE){
                    led_write(0, 0, 1);
                }else{
                    led_write(0, 0, 0);
                }
*/

                //横壁制御
                if(ad_l >= wall_base_l && ad_r >= wall_base_r){     //左右に壁がある時
                    if(dif_l > ctrl_base_l || -1 * dif_l > ctrl_base_l){            //左壁の制御判断 abs(dif_l) > CTRL_BASE_Lに等しい
                        dl_tmp += (-1 * CTRL_P * dif_l + CTRL_D * change_value_l);// * ctrl_amount;   //制御値を決定
                        dr_tmp += (CTRL_P * dif_l - CTRL_D * change_value_l);// * ctrl_amount;        //制御値を決定
                    }
                    if(dif_r > ctrl_base_r || -1 * dif_r > ctrl_base_r){
                        dl_tmp += (CTRL_P * dif_r - CTRL_D * change_value_r);// * ctrl_amount;           //制御値を決定
                        dr_tmp += (-1 * CTRL_P * dif_r + CTRL_D * change_value_r);// * ctrl_amount;      //制御値を決定
                    }
                }
                else if(ad_l >= wall_base_l){           //左壁だけあるとき
                    if(dif_l > ctrl_base_l || -1 * dif_l > ctrl_base_l){
                        dl_tmp += 2 * (-1 * CTRL_P * dif_l + CTRL_D * change_value_l);// * ctrl_amount;   //制御値を決定  制御量を倍に
                        dr_tmp += 2 * (CTRL_P * dif_l - CTRL_D * change_value_l);// * ctrl_amount;        //制御値を決定  制御量を倍に
                    }
                }
                else if(ad_r >= wall_base_r){           //右壁だけある時
                    if(dif_r > ctrl_base_r || -1 * dif_r > ctrl_base_r){
                        dl_tmp += 2 * (CTRL_P * dif_r - CTRL_D * change_value_r);// * ctrl_amount;         //制御値を決定  制御量を倍に
                        dr_tmp += 2 * (-1 * CTRL_P * dif_r + CTRL_D * change_value_r);// * ctrl_amount;    //制御値を決定  制御量を倍に
                    }
                }
                else{                             //壁がない時
                  ;
                }

                /*if (CTRL_BASE_L < dif_l) {            // 制御の判断
                    dl_tmp += -1 * CTRL_P * dif_l; // 比例制御値を決定
                    dr_tmp += CTRL_P * dif_l; // 比例制御値を決定
                }
                if (CTRL_BASE_R < dif_r) {
                    dl_tmp += CTRL_P * dif_r; // 比例制御値を決定
                    dr_tmp += -1 * CTRL_P * dif_r; // 比例制御値を決定
                }*/

/*                if((dl_tmp > CTRL_MAX || -1 * dl_tmp > CTRL_MAX) && (dr_tmp > CTRL_MAX || -1 * dr_tmp > CTRL_MAX)){
                    led_write(1, 0, 1);
                }else if(dl_tmp > CTRL_MAX || -1 * dl_tmp > CTRL_MAX){
                    led_write(1, 0, 0);
                }else if(dr_tmp > CTRL_MAX || -1 * dr_tmp > CTRL_MAX){
                    led_write(0, 0, 1);
                }else{
                    led_write(0, 0, 0);
                }
*/
                // 一次保存した制御比例値をdlとdrに反映させる
                dl = max(min(CTRL_MAX, dl_tmp), -1 * CTRL_MAX);
                dr = max(min(CTRL_MAX, dr_tmp), -1 * CTRL_MAX);

                if(MF.FLAG.MODE1){          //制御量調整
                    dl *= CTRL_MODE1;
                    dr *= CTRL_MODE1;
                }else if(MF.FLAG.MODE2){
                    dl *= CTRL_MODE2;
                    dr *= CTRL_MODE2;
                }else if(MF.FLAG.MODE3){
                    dl *= CTRL_MODE3;
                    dr *= CTRL_MODE3;
                }

            } else {
                // 制御フラグがなければ制御値0
                dl = dr = 0;
            }

           pre_ad_l = ad_l;
           pre_ad_r = ad_r;

            break;
        }

        //====タスクポインタを進める====
        tp = (tp + 1) % 3;

    } /* if (htim->Instance == htim6.Instance) */

} /* HAL_TIM_PeriodElapsedCallback */

//+++++++++++++++++++++++++++++++++++++++++++++++
// tim6_wait_us
// 1us毎にカウントアップするTIM6を使ってusマイクロ秒処理を止める関数。
// （whileループ中にオーバーフローが起こると機能しないのでTIM6タイマ更新割り込みハンドラ内のみで使用することを推奨する）
// 引数：us …… 処理を止めたいマイクロ秒
// 戻り値：無し
//+++++++++++++++++++++++++++++++++++++++++++++++
void tim6_wait_us(uint32_t us) {
    uint32_t dest = __HAL_TIM_GET_COUNTER(&htim6) + us;
    while (__HAL_TIM_GET_COUNTER(&htim6) < dest)
        ;
}
