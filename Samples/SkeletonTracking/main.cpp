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

Point get_middle_point(Point point_1, Point point_2)
{
	Point ans;
	ans.x = (point_1.x + point_2.x) / 2.0;
	ans.y = (point_1.y + point_2.y) / 2.0;
	ans.z = (point_1.z + point_2.z) / 2.0;
	return ans;
}

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
				Point imagepos[29];
				for (int j = 0; j < 15; ++j) {

					const nite::SkeletonJoint& joint = skeelton.getJoint((nite::JointType)j);

					pos[j].confi = joint.getPositionConfidence();

					if (joint.getPositionConfidence() >= 0.7f) {

						const nite::Point3f& position = joint.getPosition();

						float x = 0, y = 0, z = 0;

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

						imagepos[j].x = x;
						imagepos[j].y = y;
						imagepos[j].z = z;
					}

				}
				imagepos[15] = get_middle_point(imagepos[0], imagepos[1]);
				imagepos[16] = get_middle_point(imagepos[1], imagepos[2]);
				imagepos[17] = get_middle_point(imagepos[1], imagepos[3]);
				imagepos[18] = get_middle_point(imagepos[2], imagepos[4]);
				imagepos[19] = get_middle_point(imagepos[3], imagepos[5]);
				imagepos[20] = get_middle_point(imagepos[4], imagepos[6]);
				imagepos[21] = get_middle_point(imagepos[5], imagepos[7]);
				imagepos[22] = get_middle_point(imagepos[1], imagepos[8]);
				imagepos[23] = get_middle_point(imagepos[9], imagepos[16]);
				imagepos[24] = get_middle_point(imagepos[10], imagepos[17]);
				imagepos[25] = get_middle_point(imagepos[9], imagepos[11]);
				imagepos[26] = get_middle_point(imagepos[10], imagepos[12]);
				imagepos[27] = get_middle_point(imagepos[11], imagepos[13]);
				imagepos[28] = get_middle_point(imagepos[12], imagepos[14]);
				for (int j = 0; j < 29; ++j) {
					cv::circle(depthImage, cvPoint((int)imagepos[j].x, (int)imagepos[j].y), 8, cv::Scalar(35, 55, 255), -1);
				}
			}

		}

	}

}


int main(int argc, char* argv[])

{
	



	char pos_name[29][256] = { "head",//0
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
							"R_toe",//14
							"MID_head_chest",//15
							"MID_L_chest_sho",//16
							"MID_R_chest_sho",//17
							"MID_L_sho_elb",//18
							"MID_R_sho_elb",//19
							"MID_L_elb_hand",//20
							"MID_R_elb_hand",//21
							"MID_chest_waist",//22
							"MID_L_sho_groin",//23
							"MID_R_sho_groin",//24
							"MID_L_groin_knee",//25
							"MID_R_groin_knee",//26
							"MID_L_knee_toe",//27
							"MID_R_knee_toe"};//28
	Point pos[29];
	double theta_groin,theta_sho;
	double ave_theta_sho = 0, sum_theta_sho = 0;
	double ave_theta_groin = 0, sum_theta_groin = 0;
	int cnt_theta = 0;
	double pix;

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
			pos[15] = get_middle_point(pos[0], pos[1]);
			pos[16] = get_middle_point(pos[1], pos[2]);
			pos[17] = get_middle_point(pos[1], pos[3]);
			pos[18] = get_middle_point(pos[2], pos[4]);
			pos[19] = get_middle_point(pos[3], pos[5]);
			pos[20] = get_middle_point(pos[4], pos[6]);
			pos[21] = get_middle_point(pos[5], pos[7]);
			pos[22] = get_middle_point(pos[1], pos[8]);
			pos[23] = get_middle_point(pos[9], pos[16]);
			pos[24] = get_middle_point(pos[10], pos[17]);
			pos[25] = get_middle_point(pos[9], pos[11]);
			pos[26] = get_middle_point(pos[10], pos[12]);
			pos[27] = get_middle_point(pos[11], pos[13]);
			pos[28] = get_middle_point(pos[12], pos[14]);

			//計算
			theta_sho = atan((pos[3].y - pos[2].y) / (pos[3].x - pos[2].x)) * 180 / 3.141593;
			theta_groin = atan((pos[10].y - pos[9].y) / (pos[10].x - pos[9].x)) * 180 / 3.141593;

			
		



			//


			/////////////////////
			cv::Mat window1 = cv::Mat::ones(950, 800, CV_8U) * 255;
			std::stringstream ss[30];

			
			for (int i = 0; i < 29; i++) {
				ss[i] << fixed << setprecision(2) << pos_name[i] << "[" << i << "]=("
					<< (int)pos[i].x << " , " << (int)pos[i].y << " , " << (int)pos[i].z << "), ";//<< pos[i].confi;
				cv::putText(window1, ss[i].str(), cv::Point(5, 40+i*32),
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

			

		}

	}

	catch (std::exception&) {

		std::cout << openni::OpenNI::getExtendedError() << std::endl;

	}

}
