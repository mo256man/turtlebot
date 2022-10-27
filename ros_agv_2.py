#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       BLV-R
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目]
#       通信ID: [1, 2]
#       Baudrate: [230400, 230400]
#
# ●launchファイル設定:
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=1000 firstGen:="" secondGen:="1,2," globalID:="10" axisNum:="2"
#
# ●処理内容:
#       Writeにより連続運転(速度制御)でモーターを運転させる。
#       Readにより一定周期でモーターの検出速度を取得し、表示させる。

import rospy
import time
import datetime
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

### 追加
import pygame
import sys

class Joy():
    def __init__(self):
        """
        初期設定
        """
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.is_exit = False
        print(self.joystick.get_name())
        self.strcmd = ""
    def map_axis(self, val):
        """
        ジョイスティックの出力数値を調整
        """
        val = round(val, 2)
        in_min = -1
        in_max = 1
        out_min = -100
        out_max = 100
        result = int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)        # 0から100までの値
        # 0かプラマイ100かのデジタル値にする
        result = 0 if abs(result)<0.5 else 100 * result // abs(result)
        return result

    def map_axis_t(self, val):
        """
        ジョイスティックの出力数値を調整(アナログトリガーのL2 R2ボタン)
        """
        val = self.map_axis(val)
        if val <= 0 and val >= -100:
            in_min = -100
            in_max = 0
            out_min = 0
            out_max = 50
            result = int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)    # 0から100までの値
            result = 100 if result > 0 else 0                                                   # 0か100かのデジタル値
            return result
        else:
            in_min = 0
            in_max = 100
            out_min = 50
            out_max = 100
            result = int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)    # 0から100までの値
            result = 100 if result > 0 else 0                                                   # 0か100かのデジタル値
            return result

    def get_stick(self):
        result = {
            "joy_lx": self.map_axis(self.joystick.get_axis(0)),
            "joy_ly": -self.map_axis(self.joystick.get_axis(1)),
            "joy_rx": self.map_axis(self.joystick.get_axis(2)),
            "joy_ry": -self.map_axis(self.joystick.get_axis(3)),
            "joy_lt": self.map_axis_t(self.joystick.get_axis(4)),
            "joy_rt": self.map_axis_t(self.joystick.get_axis(5)),
            "hat_x": self.joystick.get_hat(0)[0],
            "hat_y": self.joystick.get_hat(0)[1],
            "btn_a": self.joystick.get_button(0),
            "btn_b": self.joystick.get_button(1),
            "btn_x": self.joystick.get_button(2),
            "btn_y": self.joystick.get_button(3),
            "btn_lb": self.joystick.get_button(4),
            "btn_rb": self.joystick.get_button(5),
            "btn_back": self.joystick.get_button(6),
            "btn_start": self.joystick.get_button(7),
            "btn_joyl": self.joystick.get_button(8),
            "btn_joyr": self.joystick.get_button(9),
            "btn_guide": self.joystick.get_button(10),
        }
        # 以下、デバッグ用に入力された値のみ表示させる
        #pressed_buttons = {k:v for k, v in result.items() if v != 0}
        #if len(pressed_buttons) > 0:
        #    print(pressed_buttons)
        return result

    def get_command(self):
        global is_exit
        global msg
        self.strcmd = ""
        dic = self.get_stick()

        self.strcmd += "L:"
        if dic["joy_ly"] == -100:
            self.strcmd += "[↑"
        elif dic["joy_ly"] == 100:
            self.strcmd += "[↓"
        else:
            self.strcmd += "[　"
        if dic["joy_lx"] == -100:
            self.strcmd += "←]"
        elif dic["joy_lx"] == 100:
            self.strcmd += "→]"
        else:
            self.strcmd += "　]"

        self.strcmd += "R:"
        if dic["hat_y"] == -1:
            self.strcmd += "[↑"
        elif dic["hat_y"] == 1:
            self.strcmd += "[↓"
        else:
            self.strcmd += "[　"
        if dic["hat_x"] == -1:
            self.strcmd += "←]"
        elif dic["hat_x"] == 1:
            self.strcmd += "→]"
        else:
            self.strcmd += "　]"


        self.strcmd += "+:"
        if dic["joy_ry"] == -100:
            self.strcmd += "[↑"
        elif dic["joy_ry"] == 100:
            self.strcmd += "[↓"
        else:
            self.strcmd += "[　"
        if dic["joy_rx"] == -100:
            self.strcmd += "←]"
        elif dic["joy_rx"] == 100:
            self.strcmd += "→]"
        else:
            self.strcmd += "　]"


        if dic["btn_a"] == 1:
            self.strcmd += "[A]"
        if dic["btn_b"] == 1:
            self.strcmd += "[B]"
        if dic["btn_x"] == 1:
            self.strcmd += "[X]"
        if dic["btn_y"] == 1:
            self.strcmd += "[Y]"
        if dic["btn_lb"] == 1:
            self.strcmd += "[LB]"
        if dic["btn_rb"] == 1:
            self.strcmd += "[RB]"
        if dic["btn_guide"] == 1:
            self.is_exit = True             # 停止命令オン
            self.strcmd += "[GUIDE]"

        if dic["btn_guide"] == 1:
            self.is_exit = True             # 停止命令オン
        elif dic["hat_y"] == -1:
            v1, v2 = 240, -240               # 1軸と2軸の運転速度
            decel = 1000                    # 減速レート
        elif dic["hat_y"] == 1:
            v1, v2 = -240, 240               # 1軸と2軸の運転速度
            decel = 1000                    # 減速レート
        elif dic["hat_x"] == -1:
            v1, v2 = 240, 0               # 1軸と2軸の運転速度
            decel = 1000                    # 減速レート
        elif dic["hat_x"] == 1:
            v1, v2 = 0, 240               # 1軸と2軸の運転速度
            decel = 1000                    # 減速レート
        else:
            v1, v2 = 0, 0
            decel = 3000

        """
        elif dic["joy_ly"] == 100 and dic["joy_ry"] == 100:
            v1, v2 = 240, 240               # 1軸と2軸の運転速度
            decel = 1000                    # 減速レート
        elif dic["joy_ly"] == -100 and dic["joy_ry"] == -100:
            v1, v2 = -240, -240
            decel = 1000
        elif dic["joy_ly"] == -100 and dic["joy_ry"] == 100:
            v1, v2 = -240, 240
            decel = 1000
        elif dic["joy_lx"] == 100 and dic["joy_rx"] == 100:
            v1, v2 = 240, -240
            decel = 1000
        elif dic["joy_lx"] == -100 and dic["joy_rx"] == -100:
            v1, v2 = 240, 0
            decel = 1000
        elif dic["joy_lx"] == 100 and dic["joy_rx"] == 100:
            v1, v2 = 0, 240
            decel = 1000
        else:
            v1, v2 = 0, 0
            decel = 3000
        """
        print(self.strcmd)

        msg.slave_id = 10           # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1           # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000     # 書き込むアドレスの起点
        msg.write_num =12           # 全軸合わせたデータ項目数を代入する
        msg.data[0] = 16            # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0             # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = v1            # DDO運転速度(初期単位：r/min)
        msg.data[3] = 1000          # DDO加速レート(初期単位：ms)
        msg.data[4] = decel         # DDO減速レート(初期単位：ms)
        msg.data[5] = 1             # DDO運転トリガ設定
        msg.data[6] = 16            # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0             # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[8] = v2            # DDO運転速度(初期単位：r/min)
        msg.data[9] = 1000          # DDO加速レート(初期単位：ms)
        msg.data[10] = decel        # DDO減速レート(初期単位：ms)
        msg.data[11] = 1            # DDO運転トリガ設定

### 追加 ここまで

# グローバル変数
gState_driver = 0   # 0:通信可能 1:通信中
gState_mes = 0      # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー 
gState_error = 0    # 0:エラーなし 1:無応答 2:例外応答
pub = None
msg = om_query()
gMotor_pos = 0
gMotor_speed = 0
gDoesWorkTimer = False

# ドライバ状態のコールバック関数
def stateCallback(res):
    global gState_driver
    global gState_mes
    global gState_error
    gState_driver = res.state_driver
    gState_mes = res.state_mes
    gState_error = res.state_error

# レスポンスのコールバック関数
def resCallback(res):
    global gMotor_pos
    global gMotor_speed
    # 例外応答のとき
    if gState_error == 2:
        print("Exception")
        return
    # ID Shareモードのとき
    if isIdShare(res) and (res.func_code==0x03):
        axis_num = int(rospy.get_param("/om_modbusRTU_1/axis_num")) # roslaunchで設定した軸数情報の取得
        data_num = int(msg.read_num)    # データ数
        for axis in range(axis_num):
            print('{0}: {1}[r/min], {2}[r/min]'.format(datetime.datetime.now(), res.data[0], res.data[1]))  # [0]:1軸目の検出速度、[1]:2軸目の検出速度

# パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる
def isIdShare(res):
    global_id = rospy.get_param("/om_modbusRTU_1/global_id")
    return int(global_id) == res.slave_id

# t[s]待機する
def wait(t):
    global gState_driver
    time.sleep(t)
    while (gState_driver == 1):
        pass

# 一定周期で実行する処理
def timeProcess(event):
    global gDoesWorkTimer
    if gDoesWorkTimer:
        msg.slave_id = 10       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 0       # 0:Read
        msg.read_addr = 0x000E  # 読み出すアドレスの起点
        msg.read_num = 2        # 各軸1個ずつ
        pub.publish(msg)        # 配信する

# 各軸のID Shareモードの設定を行う
def setShareReadWriteData():
    global pub
    global msg

    for id in [1, 2]:
        msg.slave_id = id           # 書き込むドライバのスレーブID
        msg.func_code = 1           # 1:Write
        msg.write_addr = 0x0980     # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3           # 書き込む数
        msg.data[0] = 10            # Share control global ID
        msg.data[1] = 2             # Share control number
        msg.data[2] = id            # Share control local ID
        pub.publish(msg)            # 配信する
        wait(0.03)

        # 設定値の書き込み　個々の値が何を意味しているかは割愛
        data = [45, 46, 47, 48, 49, 50, 102, 103, 0, 0, 0, 0,
                45, 46, 47, 48, 49, 50, 102, 103, 0, 0, 0, 0]
        msg.write_addr = 0x0990     # 書き込みの起点：Share Read data[0]
        write_num = len(data)       # 書き込む数
        msg.write_num = data
        for i in range(write_num):
            msg.data[i] = data[i]
        pub.publish(msg)
        wait(0.03)

# 運転指令
def opecmd(cmd):
    for id in [1, 2]:
        msg.slave_id = id       # スレーブID
        msg.func_code = 1       # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124    # アドレス指定： ドライバ入力指令
        msg.write_num = 1       # 書き込みデータ数: 1
        msg.data[0] = cmd       # 1でS-ONを立ち上げる／0でS-ONを立ち下げる
        pub.publish(msg)        # 配信
        wait(0.3)

def main():
    global gState_mes
    global gState_error
    global pub
    global gDoesWorkTimer
    MESSAGE_ERROR = 2
    EXCEPTION_RESPONSE = 2

    joy = Joy()                                                 # ジョイスティック

    rospy.init_node("blv_idshare_sample1_1", anonymous=True)    # 上位ノード作成
    pub = rospy.Publisher("om_query1", om_query, queue_size=1)  # masterにメッセージを渡すpublisher作成
    rospy.Subscriber("om_state1", om_state, stateCallback)      # 通信状態に関するメッセージを受け取るsubscriber作成
    rospy.Subscriber("om_response1", om_response, resCallback)  # ドライバのレスポンスに関するメッセージを受け取るsubscriber作成
    wait(1)

    # ユニキャストモードで通信するため、global_idを-1に設定する
    rospy.set_param("/om_modbusRTU_1/global_id", "-1")

    # ID Shareモードの設定
    setShareReadWriteData()

    # 運転指令(S-ONをONする)
    opecmd(1)

    # ID Shareモードで通信するため、global_id=10に設定
    rospy.set_param("/om_modbusRTU_1/global_id", "10")

    # 一定周期でtimeProcessを実行する
    gDoesWorkTimer = True
    rospy.Timer(rospy.Duration(0.3), timeProcess)

    # 無限ループでジョイスティック入力を取得しモータを動かす
    while True:                             # 無限ループ
        if pygame.event.get():              # イベントがある場合（ボタン押しっぱなしを含まない）
            joy.get_command()               # この関数の先でグローバル変数msgを書き換える
        else:                               # イベントがない場合（ボタン押しっぱなしを含む）
            pass                            # 何もしない　つまりコマンドは保持される

        if joy.is_exit:                     # ジョイスティックで終了命令が出たら
            break                           # 無限ループから抜ける
        else:
            # 配信
            gDoesWorkTimer = False          # タイマー処理停止
            wait(0.03)
            pub.publish(msg)                # 配信
            wait(0.03)
            gDoesWorkTimer = True           # タイマー処理再開
            wait(0.01)

    # 運転指令をOFFにする
    # ユニキャストモードで通信するため、global_id=-1に設定
    gDoesWorkTimer = False
    rospy.set_param("/om_modbusRTU_1/global_id", "-1")

    # 運転指令(S-ONをOFFする)
    opecmd(0)

    if (gState_mes == MESSAGE_ERROR):
        stop(msg, pub)
        exit()

    if (gState_error == EXCEPTION_RESPONSE):
        stop(msg, pub)
        exit()

    print("END")
    rospy.spin()        # ノードが終了するまで待ってからPythonを終了させる

if __name__ == '__main__':
    main()
