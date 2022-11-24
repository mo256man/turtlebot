#!/usr/bin/env python
import PySimpleGUI as sg
from random import randint
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, FigureCanvasAgg
from matplotlib.figure import Figure
import numpy as np
"""
class Frame():
    def __init__(self):
        frame_motor = sg.Frame("モーター設定",
            [[sg.Text("車輪の径"), sg.InputText("150", key="-WHEELDIAMETER-", size=10, justification="right"),
              sg.Text("　減速比"), sg.InputText("20", key="-REDUCTIONRATIO-", size=10, justification="right")]])
        frame_motor = sg.Frame("走行指令",
            [[sg.Text("走行距離"), sg.InputText("", key="-DISTANCE-", size=10, justification="right"),
              sg.Text("　時間　"), sg.InputText("", key="-SECONDS-", size=10, justification="right"),
              sg.Button("スタート", key="-START-")]])


"""
def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg


def ax_grid(ax):
    ax.set_xlabel("time[sec]")
    ax.set_ylabel("speed[m/sec]")
    ax.set_xlim([1,8])
    ax.set_ylim([-1.1,1.1])
    ax.grid()

def main():

    #frame_motor = [sg.Text("車輪の径"), sg.InputText("150", key="-WHEELDIAMETER-", size=10, justification="right"),
    #               sg.Text("　減速比"), sg.InputText("20", key="-REDUCTIONRATIO-", size=10, justification="right")]

    frame_dist = [sg.Text("走行距離"), sg.InputText("", key="-DISTANCE-", size=10, justification="right"),
                  sg.Text("　時間　"), sg.InputText("", key="-SECONDS-", size=10, justification="right")]

    frame1 = sg.Frame("モーター設定",
                [sg.Text("車輪の径"), sg.InputText("150", key="-WHEELDIAMETER-", size=10, justification="right"),
                   sg.Text("　減速比"), sg.InputText("20", key="-REDUCTIONRATIO-", size=10, justification="right")]
                   , size=(500, 200) )

    layout = [[sg.Text("グラフ", size=(40, 1), justification='center', font='Helvetica 20')],
              #[sg.Frame("モーター設定", frame_motor)],
              [frame1],
              # frame_motor,
              #[[sg.Frame("モーター設定", frame_motor)]],
              # [sg.Frame("モーター設定",[sg.Text("走行距離")])],
              [sg.Canvas(size=(640, 480), key='-CANVAS-')],
              [sg.Button('Exit', size=(10, 1), pad=((280, 0), 3), font='Helvetica 14')]]

    # create the form and show it without the plot
    window = sg.Window("オリエンタルモーター動作見える化くん",
                layout, finalize=True)

    canvas_elem = window['-CANVAS-']
    canvas = canvas_elem.TKCanvas

    # draw the initial plot in the window
    fig = Figure()
    ax = fig.add_subplot(111)
    ax_grid(ax)

    fig_agg = draw_figure(canvas, fig)

    i=0
    while True:
        event, values = window.read(timeout=10)
        if event in ('Exit', None):
            exit()
        ax.cla()
        ax_grid(ax)
        x = np.arange(0, 2*np.pi, 0.05*np.pi)
        y = np.sin(x)
        ax.plot(x, y, color="red")
        x = i*0.5%8
        y = np.sin(x)
        ax.scatter(x, y, s=100, c="blue")
        fig_agg.draw()
        i += 1

    window.close()

if __name__ == '__main__':
    main()