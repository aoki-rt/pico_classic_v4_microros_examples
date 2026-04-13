# micro-ROS Arduino examples for Pi:Co Classic4


![pico](https://rt-net.jp/wp-content/uploads/2025/06/pico-classic4_830x495.png.jpg)

Pi:Co Classic4 micro-ROS Arduinoサンプルスケッチ集です。

## 動作環境

- arduino-esp32 : v3.3.8
- micro_ros_arduino : v2.0.8-humble

## サンプルスケッチについて

- STEP1 LEDをSUBを使って制御します
- STEP2 SWITCHの入力情報をPUBを使って確認します。
- STEP3 BUZZERをサービスを使って鳴らします。
- STEP4 LEDをUSBのポートから制御します。
- STEP5 picp_msgsを使ってセンサの値をPUBします。
- STEP6 受け取ったTwistのメッセージに合わせて移動します。
- STEP7 RVizで表示するメッセージをPUBします。
- STEP8 マイクロマウスとして動作し、移動した壁の情報をRVizにリアルタイムで表示します。

- [オプションキット No.1 [ESP32-S3マイコンボード]](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=4131)を搭載したPi:Co Classic3で動作確認するには、サンプルプログラムの#define PCC4をコメントアウトしてください。
- scriptsフォルダには、実行するときのコマンドをbashで記載しています。


## 関連ソフトウェア

- [pico_msgs](https://github.com/rt-net/pico_msgs) : PCとPi:Co Classic間でやりとりするROSメッセージを定義したパッケージです
- [pico_ros](https://github.com/rt-net/pico_ros) : Pi:Co Classicを便利に動かすためのPC向けROSパッケージです


## スケッチファイルの自動整形について

ソースコードのレイアウトを整えるため、各スケッチファイルにはArduino IDEの自動整形を適用しています。
自動整形のルールは[.clang-format](.clang-format) ファイルを参照してください。

## License

(C) 2026 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。