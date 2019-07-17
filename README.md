# ROSLecture
元ページは、google siteの下記サイトをgithubにアップロードしたものです。
https://sites.google.com/site/robotlabo/time-tracker/ros

# ROSを使ったマニピュレーターの制御
６自由度ロボットアームを例に、マニピュレータの構成方法を説明する。
６自由度ロボットアームの構成例は下記の図の通り。

<img src ="https://41417c4a-a-62cb3a1a-s-sites.googlegroups.com/site/robotlabo/time-tracker/ros/ros-manipulator/%E3%82%B9%E3%83%A9%E3%82%A4%E3%83%88%E3%82%991.jpg?attachauth=ANoY7cojRQ56-4_rSUK-UCtWPLiSDPjIUu4INzzZgjFTSUCthAtkWJB4kXF4grOaS7BArtIcPx5ag2cQdVXG8IZiNvdwGiBvA8TsTHr6sUbUH_ysRL_duS7u5JSQTUqirrPo-i6Rmejy2cIZlQojGbKqxCMLQaNbFiAhf5_k1hhoyPp8bAKHT4znw3ULB-LF9ndJkLGc63ULJqz4va1p1FAuZrYTS5rAmoC449coriVu-GGDYMJ94MMQbdMPklpIlGPYznv-FC26WAX52QfL3klQxcMe6_qEcxI21f_qA5kTi61iofF-5oirNOTnHv1V_54Va6hjLo8Y&attredirects=0">

## ロボットアームのシミュレーション起動コマンド例
Rviz上で６軸ロボットアームの動作シミュレーションを確認するコマンド
ターミナル上で下記コマンド入力するとRvizが立ち上がり、6軸アームをコントロールできます。
```bash
$ roslaunch sixdofarm_moveit_config demo.launch 
```
上記コマンド入力後に表示される画面の例。

<img src ="https://41417c4a-a-62cb3a1a-s-sites.googlegroups.com/site/robotlabo/time-tracker/ros/ros-manipulator/moveit_rviz0.png?attachauth=ANoY7coGzXwX4iNw0vCWQ6t2yAIiWFV5LYW0QX8-B4YOKb9fwSV9deRQWBOmeRJmSWbxNl2GGa8Nk51lJp40nJ6foOjg5ndD-U-ihPogxY_hlJlhhHvgUOKNC5S6H1YDBrlRZfp2zS0YFmH3KLykm4GEfQbB64MUmqfSXyrzLGC7kDUxV6VpXHWMcaoGBjxQ_TEQ7qR2byWl-_wS3l507uLkDyxbIoVZVhkmPakpnupUHeR93uN4OGFZll8HKEWdFk7BP-Ch3jFy&attredirects=0">


<br><br>

# 差動型の移動ロボット制御とマップ構築、ナビゲーション
差動２輪の移動ロボットのモデルを作成して、ロボットをコントローラで移動させる方法や、動力学シミュレータGazebo上で移動ロボットを走らせて、地図を構築し、一度通った経路を追従する方法を解説します。

## 差動２輪移動ロボットのモデル構築と遠隔操作の解説
Rviz上で差動２輪移動ロボットのシミュレーション画面を立ち上げ、キーボードの入力で移動ロボットをコントロールするコマンド例
```bash
１つ目のターミナル
$ roslaunch diff_mobile_robot diff_mobile_robot.launch
２つ目のターミナル
$ rosrun diff_mobile_robot key_teleop.py 
```
２つ目のターミナルのコマンドを実行した画面。
<img src="https://41417c4a-a-62cb3a1a-s-sites.googlegroups.com/site/robotlabo/time-tracker/ros/gazebo-mobilerobot/key_teleop.png?attachauth=ANoY7coWcJZb_cTseQZu9l-tIUIVowpB7bkydU2Pijcywu8MKnBcdOSwSEYMfiJf9XvI7iKwy3mZoQGtiTixO6dbBkOGCZnXcLk1VhoCCjzyULPKOt9HCgXV_f5pd_jmjsYNU3Vfyi9CMTzq6UpO1vya741BHzgj95UxtJt0HM7YUv0JScHaqB3d_-vWt-_a1DMMIvJLvA1hxVoWCxcvYvEDh16xOQ7JD9Vs9-Cm5t106vRBPkPH_qNyDahaCkaZj9fLMX6QhIPLP88-HCsQNZAJkrBTZ6qj_g%3D%3D&attredirects=0">

画面に出てきている通りキーをどのように押すとどのように動くかわかるようになっています．

2つ目のターミナルを最前面にした状態で、"w"を押すと前進，"x"を押すと後退，"s"を押すと停止，"a"を押すと左旋回，"d"を押すと右旋回です．
"g"を押すと終了します．ctl+cを押しても終了しないので注意．

