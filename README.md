# teleop_twist_joystick.py
## 注意　仮想マシンで動かしてみたが駄目だった

TurtleBotやTurtleSimやラジコンに対応した操作プログラム  
ジョイスティックに対応  
（キーボード入力取得のほうが後ろにある＝キーボードの方が優先度が高い）  
　　↑　↑　　iと同（前進）  
　　↓　↓　　,と同（後退）  
　　↓　↑　　jと同（半時計回り旋回）  
　　↑　↓　　lと同（半時計回り旋回）  
　　←　←　　Jと同（左移動）  
　　→　→　　Lと同（右移動）  
　　Lボタン　qと同（スピード110%）  
　　Rボタン　zと同（スピード90%）  
 
### 利用方法
#### パーミッションの変更
GitHubからダウンロードしたファイルをラズパイや仮想マシンに入れたら `ls -l` を実行すると  
`-rw-r--r--  1 root root  9316 10月 22 00:26 teleop_twist_joystick.py` となる  
ちなみに正しくインストールされたteleop_twist_keyboard.pyは  
`-rwxr-xr-x  1 root root  5831 10月 21 20:11 teleop_twist_keyboard.py`  となっている  

～joystick.pyのパーミッションを～keyboard.pyと同じ -rwxr-xr-x にする  
説明は省略するが -rwxr-xr-x は数値で表せば755になる  
そこで `sudo chmod 755 teleop_twist_joystick.py` を実行する  
あらためて `ls -l` すると、～joystick.pyのパーミッションも-rwxr-xr-xになっている

#### ファイルの移動
～joystick.pyを～keyboard.pyがある場所に移動（コピー）する  
`sudo cp teleop_twist_joystick.py /opt/ros/noetic/lib/teleop_twist_keyboard`  

#### 実行
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/turtle1/cmd_vel`　でなく  
`rosrun teleop_twist_keyboard teleop_twist_joystick.py cmd_vel:=/turtle1/cmd_vel`　とすれば  
ジョイスティックでも動かせるようになる、はず

# ros_agv.py
## 注意　未トライ　たぶんteleop_twist_joystick.pyと同じ不具合がある

BLV-R用のidshare_sample1_1.pyをもとに、ジョイスティックでの操作を可能にした  
　　ガイド　プログラム終了　動いている途中で押すとどうなるか不明  
　　↑　↑　　240rpmで前進  
　　↓　↓　　240rpmで後退  
　　↓　↑　　その場旋回　1軸が-240rpm、2軸が240rpm  
　　↑　↓　　その場旋回　1軸が240rpm、2軸が-240rpm　どちらが適切かは実際に動かして修正する  
　　←　←　　片方を軸として旋回　1軸が0rpm、2軸が240rpm  
　　→　→　　片方を軸として旋回　1軸が240rpm、2軸が0rpm　このコマンドはないほうがよいかも
 
