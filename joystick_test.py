import enum
import pygame
import sys

class Joy():
    def __init__(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(self.joystick.get_name())
        self.list_axis = ["LX", "LY", "RX", "RY", "LT", "RT"]
        self.list_buttons = ["A", "B", "X", "Y", "LB", "RB", "BK", "ST", "JL", "JR", "GD"]
        self.list_hats = ["HX", "HY"]

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
        self.result = {}
        for i, axis in enumerate(self.list_axis):
            #self.result[axis] = self.map_axis(self.joystick.get_axis(i))
            self.result[axis] = (self.joystick.get_axis(i))
        for i, button in enumerate(self.list_buttons):
            self.result[button] = self.joystick.get_button(i)
        for i, hat in enumerate(self.list_hats):
            self.result[hat] = self.joystick.get_hat(0)[i]
        return

    def get_command(self):
        self.get_stick()
        str_result = ""
        for k, v in self.result.items():
            if k in self.list_axis:
                v = round(v, 3)
                str_result += f"[{v:>6}]" if v != 0 else f"[  {k}  ]"          # キーは2文字
            else:
                str_result += f"[{v:>2}]" if v != 0 else f"[{(' '+k)[-2:]}]"   # キーは1文字か2文字

        print(str_result + "\r", end="")
        return str_result

def main():
    joy = Joy()
    while True:
        if pygame.event.get():              # イベントがある場合（起動直後を含む　ボタン押しっぱなしを含まない）
            command = joy.get_command()
        else:                               # イベントがない場合（ボタン押しっぱなしを含む）
            pass                            # 何もしない　つまりコマンドは保持される

        if command != "":
            # print(command)
            pass

        if command == "A":
            sys.exit()                      # プログラム中断

if __name__ == "__main__":
    main()