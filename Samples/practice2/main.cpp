#include <iostream>
#include <iomanip>
#include <math.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <NiTE.h>

#define alpha 0

using namespace std;

///
// Point の定義
typedef struct {
	double x, y, z;
	float confi;
} Point;
///

cv::Scalar colors[] = {

	cv::Scalar(0, 0, 0),

	cv::Scalar(0, 0, 0),

	cv::Scalar(0, 0, 0),

	cv::Scalar(0, 0, 0),

	cv::Scalar(0, 0, 0),

	cv::Scalar(0, 0, 0),

};


cv::Mat drawUser(nite::UserTrackerFrameRef& userFrame)
{
	cv::Mat depthImage;

	openni::VideoFrameRef depthFrame = userFrame.getDepthFrame();

	if (depthFrame.isValid()) {

		openni::VideoMode videoMode = depthFrame.getVideoMode();

		depthImage = cv::Mat(videoMode.getResolutionY(),

		videoMode.getResolutionX(),

		CV_8UC4);


		openni::DepthPixel* depth = (openni::DepthPixel*)depthFrame.getData();

		const nite::UserId* pLabels = userFrame.getUserMap().getPixels();

		for (int i = 0; i < (depthFrame.getDataSize() / sizeof(openni::DepthPixel)); ++i) {

			// 画像インデックスを生成

			int index = i * 4;

			// 距離データを画像化する

			uchar* data = &depthImage.data[index];
			
//			if (depth[i] > 900) {
//				pLabels[i] = 50;
//				depth[i] = 0;
//			}
//			depth[i] = 5000;
			//pLabels[i] = 0;

			if (pLabels[i] != 0) {

				data[0] *= colors[pLabels[i]][0];

				data[1] *= colors[pLabels[i]][1];

				data[2] *= colors[pLabels[i]][2];

			}

			else {

				// 0-255のグレーデータを作成する

				// distance : 10000 = gray : 255
				int gray = ~((depth[i] * 255) / 10000);

				data[0] = gray;

				data[1] = gray;

				data[2] = gray;

			}

		}

	}
	

	return depthImage;

}


void drawSkeleton(cv::Mat& depthImage, nite::UserTracker& userTracker,

	nite::UserTrackerFrameRef& userFrame, Point *pos)
{
	const nite::Array<nite::UserData>& users = userFrame.getUsers();

	for (int i = 0; i < users.getSize(); ++i) {

		const nite::UserData& user = users[i];

		if (user.isNew()) {

			userTracker.startSkeletonTracking(user.getId());
		}

		else if (!user.isLost()) {

			const nite::Skeleton& skeelton = user.getSkeleton();

			if (skeelton.getState() == nite::SKELETON_TRACKED) {

				for (int j = 0; j < 15; ++j) {

					const nite::SkeletonJoint& joint = skeelton.getJoint((nite::JointType)j);

					pos[j].confi = joint.getPositionConfidence();

					if (joint.getPositionConfidence() >= 0.7f) {

						const nite::Point3f& position = joint.getPosition();

						float x = 0, y = 0;

						userTracker.convertJointCoordinatesToDepth(

							position.x, position.y, position.z, &x, &y);

						///////
						//pos[j].x = position.x;
						//pos[j].y = (-position.z);
						//pos[j].z = position.y;
						pos[j].x = position.x;
						pos[j].y = (-position.z)*cos(alpha*3.141592 / 180) + position.y*sin(alpha*3.141592 / 180);
						pos[j].z = -(-position.z)*sin(alpha*3.141592 / 180) + position.y*cos(alpha*3.141592 / 180);
						///////

						cv::circle(depthImage, cvPoint((int)x, (int)y),

							8, cv::Scalar(35, 55, 255), -1);

					}

				}

			}

		}

	}

}


int main(int argc, char* argv[])

{
	



	char pos_name[15][256] = { "head",//0
							"chest" ,//1
							"L_sho" ,//2
							"R_sho",//3
							"L_elb",//4
							"R_elb",//5
							"L_hand",//6
							"R_hand",//7
							"waist",//8
							"L_groin",//9
							"R_groin",//10
							"L_knee",//11
							"R_knee",//12
							"L_toe",//13
							"R_toe"};//14
	Point pos[15];
	double theta_groin,theta_sho;
	double ave_theta_sho = 0, sum_theta_sho = 0;
	double ave_theta_groin = 0, sum_theta_groin = 0;
	int cnt_theta = 0;
	double pix;
	int count = 0;
	FILE *f1;

	clock_t start, end, time_now, time_1 = -50;
	start = clock();

	try {
		openni::OpenNI::initialize();


		openni::Device device;

		device.open(openni::ANY_DEVICE);

		nite::Status niteRet = nite::NiTE::initialize();


		nite::UserTracker userTracker;

		niteRet = userTracker.create();

		if (niteRet != nite::STATUS_OK) {

			throw std::runtime_error("userTracker.create");

		}


		cv::Mat depthImage;
		

		openni::VideoStream colorStream;

		colorStream.create(device, openni::SENSOR_COLOR);

		colorStream.start();


		std::vector<openni::VideoStream*> streams;

		streams.push_back(&colorStream);


		cv::Mat colorImage;






		while (1) {

			            int changedIndex;

            openni::OpenNI::waitForAnyStream( &streams[0], streams.size(), &changedIndex );

			if (changedIndex == 0) {

				openni::VideoFrameRef colorFrame;
				
				colorStream.readFrame(&colorFrame);

				if (colorFrame.isValid()) {

					colorImage = cv::Mat(colorStream.getVideoMode().getResolutionY(),

						colorStream.getVideoMode().getResolutionX(),

						CV_8UC3, (char*)colorFrame.getData());

				}
			}

                cv::cvtColor( colorImage, colorImage, CV_BGR2RGB );

                cv::imshow( "Color Camera", colorImage );





			//NITE
			nite::UserTrackerFrameRef userFrame;

			userTracker.readFrame(&userFrame);

			depthImage = drawUser(userFrame);
			

			//cv::imshow("depth_user", depthImage);

			pix = depthImage.at<uchar>(120, 120);

			drawSkeleton(depthImage, userTracker, userFrame, pos);

			//計算
			theta_sho = atan((pos[3].y - pos[2].y) / (pos[3].x - pos[2].x)) * 180 / 3.141593;
			theta_groin = atan((pos[10].y - pos[9].y) / (pos[10].x - pos[9].x)) * 180 / 3.141593;

			
		



			//


			/////////////////////
			cv::Mat window1 = cv::Mat::ones(800, 800, CV_8U) * 255;
			std::stringstream ss[30];

			
			for (int i = 0; i < 15; i++) {
				ss[i] << fixed << setprecision(2) << pos_name[i] << "[" << i << "]=("
					<< (int)pos[i].x << " , " << (int)pos[i].y << " , " << (int)pos[i].z << "), ";//<< pos[i].confi;
				cv::putText(window1, ss[i].str(), cv::Point(5, 40+i*35),
					cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 3);
			}
	
			
			cv::imshow("window1", window1);
			/////////////////////

			cv::imshow("Skeleton", depthImage);
			depthImage = cv::Mat::zeros(240, 320, CV_8UC4);



			int key = cv::waitKey(10);

			if (key == 'q') {

				break;

			}

			if (count % 2 == 0) {
			    f1 = fopen("/home/cy8182/repos/test/cb.txt", "w");
			    for (int i = 0; i < 15; i++)
                    fprintf(f1, "%d, %d, %d\n", (int)pos[i].x, (int)pos[i].y, (int)pos[i].z);
                fclose(f1);
			}
			count++;

		}

	}

	catch (std::exception&) {

		std::cout << openni::OpenNI::getExtendedError() << std::endl;

	}

}
