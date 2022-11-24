import PySimpleGUI as sg

class Frame():
    def __init__(self):
        sg.theme('Purple')
        frame1 = sg.Frame("モーター設定",
                    [[sg.Text("車輪の径"), sg.InputText("150", key="-WHEEL_DIAMETER-", size=10, justification="right"),
                      sg.Text("　減速比"), sg.InputText("20", key="-REDUCTION_RATIO-", size=10, justification="right")]],
                    size=(500, 100))
        frame2 = sg.Frame("指令",
                    [[sg.Text("走行距離"), sg.InputText("", key="-DISTANCE-", size=10, justification="right"),
                      sg.Text("　時間　"), sg.InputText("", key="-SECONDS-", size=10, justification="right"),
                      sg.Button("SET", key="-SET-"),
                      sg.Button("RESET", key="-RESET-")]],
                    size=(500, 100))
        frame3 = sg.Frame("グラフ",[] , size=(800, 800))
        col1 = [[frame1], [frame2]]
        col2 = [[frame3]]
        layout = [[sg.Text("フィードバック制御見えるくん")], [sg.Column(col1), sg.Column(col2)]]
        self.window = sg.Window("アプリ", layout, resizable=True)

    def getValues(self):
        self.distance = self.values["-WHEELDIAMETER-"]
        self.reduvtion_ratio = self.values["-REDUCTIONRATIO-"]

def main():
    W = Frame().window

    while True:
        # ウィンドウ表示
        event, values = W.read()

        #クローズボタンの処理
        if event is None:
            print('exit')
            break
        elif event == "-SET-":
            distance = values["-WHEEL_DIAMETER-"]
            print(distance)
        elif event == "-RESET-":
            W["-DISTANCE-"].Update("")
            W["-SECONDS-"].Update("")
            distance = values["-WHEEL_DIAMETER-"]

    W.close()

if __name__=="__main__":
    main()
