# -*- coding: utf-8 -*-
#バイラテしながらカメラ保存のPython
import datetime
import sys
#sys.path.append('/home/mclab/.local/lib/python3.5/site-packages')
import numpy as np
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import time
import socket
import select
import os
#cap = cv2.VideoCapture(0)
cap= cv2.VideoCapture(-1, cv2.CAP_V4L)

#########画像########
Height = 480
Width = 640
HeightStart = 0
WidthStart = 0
CutHeight = 480
CutWidth = 640
#Width = 384
#Height = 384
#WidthStart = 120
#HeightStart = 0
#CutWidth = 128
#CutHeight = 128
#Channel = 3
#ImageNumber = 500#保存画像の枚数
######################

def ImageCut(img):
    # 画像を切り抜き(ここだけは縦、横)
    img = img[HeightStart:HeightStart+Height,WidthStart:WidthStart+Width]
    img = cv2.resize(img, (CutWidth, CutHeight))
    return img

def SaveImage(rgb,SaveTime, SaveName):
  rgb = np.array(rgb,dtype="float32")
  SaveTime = np.array(SaveTime,dtype="float32")
  # 画像ファイルを保存先を生成
  DirName = str(int(SaveName))
  DirNameRGB = "Image/data"+str(DirName)
  if(os.path.isdir(DirNameRGB) != True):
      os.makedirs(DirNameRGB)
      print("RGBフォルダ{0}を作成しました。\n".format(DirNameRGB))
  else:
      print("既存RGBフォルダ{0}に上書きします。\n".format(DirNameRGB))
  # 画像を保存
  for SaveNumber in range(0, len(rgb)):
    NameRGB = "Image/data{:.0f}/{:.3f}.png".format(SaveName,SaveTime[SaveNumber])
    cv2.imwrite(NameRGB, rgb[SaveNumber])
  print("写真を保存しました。train{:.0f}".format(SaveName))

def main():
  ImageCount = 0
  ImageBox ,SaveTime = [],[]
  host = '127.0.0.1'
  port = 10051
  backlog = 10
  bufsize = 4096
  st = datetime.datetime.now()
  i = 0
  j = 0

  #ソケットのインスタンス作成
  server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  readfds = set([server_sock])
  try:
    #バインド(IPとポートを設定)
    server_sock.bind((host, port))
    #(接続待ちに必要。引数は接続できるノードの数)
    server_sock.listen(backlog)
    # 画像データ取得
    ret, frame = cap.read()

    print("準備OK!!")
    while i == 0:
      j += 1
      rready, wready, xready = select.select(readfds, [], [])
      for sock in rready:
        if sock is server_sock:
          # クライアントの接続を待つ
          conn, address = server_sock.accept()
          readfds.add(conn)
        else:
          ####################################
          # クライアントからのデータを受信
          ####################################
          msg = sock.recv(bufsize)
          ##############################################
          # メッセージの長さが0 = 無かったらソケットを閉じる
          ##############################################
          if len(msg) == 0:
            sock.close()
            readfds.remove(sock)
          else:
            ################################
            #  戻り値が-1 つまり　
            #  正常にrecvが実行されなかった場合
            #  データを保存して初期化
            ################################
            if msg == b'-1.0000 -1.0000':
              SaveImage(ImageBox, SaveTime, SaveName)
              # 画像保存用変数の初期化
              ImageBox, SaveTime = [], []
            ################################
            #  戻り値が** つまり　
            #  通信がすでに閉じていた場合
            #  データを保存して終了
            ################################
            if msg == b'**':
              # 終了文字を受け取ったら i = 1
              # このwhile文を抜ける
              msg_str = msg.decode("UTF-8")
              print("=>"+msg_str)
              SaveImage(ImageBox, SaveTime, SaveName)
              i = 1
            ################################
            #  戻り値がそれ以外 つまり　
            #  通信から信号を受信している間
            #  ImageBoxに画像データを格納
            ################################
            else:
              dd = []
              #受け取ったデータを区切ってリストにする
              smsg = msg.split()
              for m in smsg:
                # 受け取ったデータをdd=[]に加える
                dd.append(float(m))
              # ddの値をinput_data_modelに格納
              input_data_model = np.array(dd, dtype="float32")
              print("///////  time : {:.3f}  ///////\n".format(input_data_model[1]))
              #input_data_model = np.delete(input_data_model,9,0)#0次元目の１０個目の成分を削除(多分時間)
              print("\n")
              
              #カメラ保存
              ret, frame = cap.read()
              frame_np =  np.array(frame, dtype="float32")
              print(frame_np.shape)
              # ################################
              # dd が -1.00000でない　つまり
              # smsg = msg.split()が正常に実行されたら
              # #################################
              if input_data_model[1] != -1.0000:
                # 画像データを記録
                ImageBox.append(ImageCut(frame))
                # 保存名を記録
                SaveName = input_data_model[0]
                # 時間を記録
                SaveTime.append(input_data_model[1])
                ImageCount += 1
              
              msg = "1"
              msg1 = bytes(msg,encoding="utf8")
              sock.send(msg1)
              
              
              

  finally:
    for sock in readfds:
      sock.close()#ソケットを閉じる

  cap.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()


