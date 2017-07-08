#include <iostream>
#include <sstream>
#include <opencv2\opencv.hpp>
#include <windows.h>
#include <process.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
using namespace std;

#define re 1

//Releaseモードで使ってください


//ライントレースを行うスレッド（USBカメラ）
class Line_trace {
public:

	Line_trace() : abort_(false), th_(&Line_trace::run, this) { }
	virtual ~Line_trace() { abort_thread(); th_.join(); }

	void abort_thread() {
		std::lock_guard<std::mutex> lk(mtx_);
		if (!abort_) { abort_ = true; cv_.notify_all(); }
	}

private:

	int p = 0;
	cv::Mat input_image;//カメラ画像用の配列
	cv::Mat canny;//cannyフィルター用の配列
	int border = 130;//トラックバーの初期値
	int border1 = 134;//トラックバーの初期値
	int border2 = 54;//トラックバーの初期値
	double S1 = 0.0, S2 = 0.0;//傾き用の変数
	double K;//傾きを角度で表すための変数
	int N = 0;//検知したか確認するための変数
	HANDLE hComm;       /* シリアルポートのハンドル */
	DCB dcb;            /* 通信パラメータ */
	char input[1024];   /* 入力用 */
	DWORD writesize;    /* ポートへ書き込んだバイト数 */
	char pszBuf[2048];
	DWORD dwCount;
	DWORD dw;
	DWORD dwErrors;
	COMSTAT ComStat;
	DWORD dwResult = 0;
	bool abort_; std::mutex mtx_; std::condition_variable cv_;
	std::thread th_;//最後に宣言	

	void setup() {

		hComm = CreateFile(
			"\\\\.\\COM3",                       /* シリアルポートの指定 */
			GENERIC_READ | GENERIC_WRITE, /* アクセスモード */
			0,                            /* 共有モード */
			NULL,                         /* セキュリティ属性 */
			OPEN_EXISTING,                /* 作成フラグ */
			FILE_ATTRIBUTE_NORMAL,        /* 属性 */
			NULL                          /* テンプレートのハンドル */
			);

		if (hComm == INVALID_HANDLE_VALUE)  //ハンドル取得に失敗した場合
		{
			printf("Port could not open.\n");
			//exit(0);
		}

		GetCommState(hComm, &dcb); /* DCB を取得 */
		dcb.BaudRate = 115200;        //通信速度
		dcb.ByteSize = 8;            //データ長
		dcb.Parity = NOPARITY;       // パリティビット：EVENPARITY,MARKPARITY,NOPARITY,ODDPARITY
		dcb.StopBits = ONESTOPBIT;   // ストップビット：ONESTOPBIT,ONE5STOPBITS,TWOSTOPBITS
		SetCommState(hComm, &dcb); /* DCB を設定 */
	}

	void run() {

		cv::VideoCapture cap(0); // デフォルトカメラをオープン
		if (!cap.isOpened()) exit(0); //カメラの起動をチェック
		cv::namedWindow("output2", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);//表示用のウィンドウを作成
		cv::createTrackbar("Border", "output2", &border, 230); //トラックバー
		cv::createTrackbar("Border1", "output2", &border1, 230); //トラックバー
		cv::createTrackbar("Border2", "output2", &border2, 230); //トラックバー
		setup();

		const auto sleep_time = std::chrono::milliseconds(1);

		while (true) {
			cv::TickMeter meter;//処理時間計測用
			meter.start();
			std::unique_lock<std::mutex> lk(mtx_);
			if (abort_) { std::cout << "aborted" << std::endl; break; }

			thread_proc(cap);

			cv_.wait_for(lk, sleep_time, [this] { return abort_; });
			meter.stop();
			std::cout << meter.getTimeMilli() << "ms" << std::endl;
		}
	}

	void thread_proc(cv::VideoCapture cap) {
		int gx = 0, gy = 0, gn = 0, en = 0;
		N = 0; K = 0.0;//初期化
		for (int w = 0; w < re; w++){
			cap >> input_image;//カメラ画像を受け取る
			int a = 0;
			for (int y = 0; y < 480; y++){
				for (int x = 0; x < 640; x++){
					if (double(input_image.data[a + 2] / (input_image.data[a] + 0.1))> 1.6 && double(input_image.data[a + 2] / (input_image.data[a + 1] + 0.1))> 1.6){
						input_image.data[a] = 0;
						input_image.data[a + 1] = 0;
						input_image.data[a + 2] = 0;
						gx = gx + x; gy = gy + y; gn++;
					}
					else {
						input_image.data[a] = 255;
						input_image.data[a + 1] = 255;
						input_image.data[a + 2] = 255;
						gx = gx + x; gy = gy + y; gn++;
					}
					a = a + 3;
				}
			}
			cv::Canny(input_image, canny, 255, 255);//cannyフィルター
			vector<cv::Vec4i> lines;//線分用の配列
			cv::HoughLinesP(canny, lines, 1, CV_PI / 180.0, border + 1, border1 + 1, border2 + 1);// 線分検出

			for (auto line : lines){
				cv::line(input_image, cv::Point(line[0], line[1]),
					cv::Point(line[2], line[3]), cv::Scalar(175, 125, 255), 2);// 線分描画
				if (abs(line[3] - line[1]) > 0 && abs((line[2] - line[0]) / (abs(line[3] - line[1]))) < 1.1){
					K = K + atan(double(line[2] - line[0]) / (line[3] - line[1]));//傾きを角度に変換(外に出したほうがいい)
					N++;
					//cout << atan(double(line[2] - line[0]) / (line[3] - line[1]))*57.4 << endl;
				}
				if (line[0] == line[2]){
					en++;
				}
			}
		}
		if (N > 1 && gn > 0){
			if (en / N > 0.4){
				N = N + en;
			}
			K = double(K / N);
			int slope = -0.32*K*57.3 + p + 300;//データを3桁に揃える 0.32は係数
			p = -0.05*K*57.3;
			//送信部分
			ostringstream ssf;
			ssf << "x" << int(slope) << "\0";
			strcpy_s(input, ssf.str().c_str());
			slope = (slope - 300);
			cout << input << endl;
			WriteFile(hComm, input, strlen(input), &writesize, NULL);
			stringstream ss; ss << slope;
			if (abs(K) < 46){
				cv::line(input_image, cv::Point(320 - 200 * K, 40),
					cv::Point(320 + 200 * K, 440), cv::Scalar(170, 255, 0), 2);//傾きをを描画
				cv::putText(input_image, ss.str(), cv::Point(50, 100), 0, 1, cv::Scalar(20, 175, 255));//テキストを出力
			}
		}
		else{
			//送信部分
			ostringstream ssf;
			ssf << "y" << "\0";
			strcpy_s(input, ssf.str().c_str());
			WriteFile(hComm, input, strlen(input), &writesize, NULL);
		}
		cv::imshow("output2", input_image);//最終的に画像を出力
		cv::waitKey(1);
	}

};

int main(int argc, char *argv[]){

	try {
		Line_trace lt;//ライントレース用

		std::this_thread::sleep_for(std::chrono::seconds(100));//待機(秒)

		//※ NUCは2コア4スレッド
	}
	catch (std::exception& ex){
		std::cout << ex.what() << std::endl;
	}
	return 0;
}

