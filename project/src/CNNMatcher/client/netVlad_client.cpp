#include <iostream>
#include <memory>
#include <string>
#include <grpc++/grpc++.h>
#include "netVlad.grpc.pb.h"
#include <openMVG/third_party/stlplus3/filesystemSimplified/file_system.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <gflags/gflags.h>
#include <fstream>

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;
using netVlad::NetRequest;
using netVlad::NetResponse;
using netVlad::NetConnect;
using namespace std;
using namespace cv;

static bool CheckEmptyString(const char *flagname, const std::string &value) { return value.length() != 0; }
DEFINE_string(hostport, "", "[Required] hostport of the server");
DEFINE_validator(hostport, &CheckEmptyString);
DEFINE_string(path, "", "[Required] the path of the images to mapping");
DEFINE_validator(path, &CheckEmptyString);
DEFINE_string(nameListFile, "", "[Required] the txt containing the names of the images");
DEFINE_validator(nameListFile, &CheckEmptyString);
DEFINE_string(outputPath, "", "[Required] the path of the generated pairlist");
DEFINE_validator(outputPath, &CheckEmptyString);
DEFINE_int32(numNN, 30, "[Required] the k-nn match parameter");
DEFINE_string(netType, "netVlad", "[Required] the type of the network");
DEFINE_validator(netType, &CheckEmptyString);
DEFINE_bool(debugPause, false, "Pause in the middle of the run to check result");

NetRequest RequestInstance(string netType, string filename, string &img)
{
	NetRequest *reqInstance = new NetRequest();
	reqInstance->set_nettype(netType);
	reqInstance->set_img(img);
	return *reqInstance;
}

class NetClient
{
  public:
	NetClient(std::shared_ptr<Channel> channel)
		: stub_(NetConnect::NewStub(channel)) {}

	// Assembles the client's payload, sends it and presents the response back
	// from the server.s
	NetResponse img2vec(NetRequest request)
	{

		// Container for the data we expect from the server.
		NetResponse reply;

		// Context for the client. It could be used to convey extra information to
		// the server and/or tweak certain RPC behaviors.
		ClientContext context;

		// The actual RPC.
		Status status = stub_->img2vec(&context, request, &reply);

		// Act upon its status.
		if (status.ok())
		{
			return reply;
		}
		else
		{
			std::cout << status.error_code() << ": " << status.error_message()
					  << std::endl;
			cout << "RPC failed" << endl;
		}
	}

  private:
	std::unique_ptr<NetConnect::Stub> stub_;
};

string pathFilter(string path)
{
	if (path.back() != '/')
		path.push_back('/');
	return path;
}

int main(int argc, char **argv)
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	FLAGS_path = pathFilter(FLAGS_path);
	FLAGS_outputPath = pathFilter(FLAGS_outputPath);
	cv::String ext = ".png";

	//get the namelist
	ifstream in(FLAGS_path + FLAGS_nameListFile);
	string s;
	vector<string> nameList;
	while (getline(in, s))
	{
		nameList.push_back(s.c_str());
	}

	if (nameList.size() == 0)
	{
		cout << "couldn't get the namelist .txt." << endl;
		return 0;
	}
	Mat kdt;
	NetRequest request;
	NetResponse reply;
	cout << "read images." << endl;
	NetClient netInstance(grpc::CreateChannel(FLAGS_hostport, grpc::InsecureChannelCredentials()));

	//get the imagearray
	for (int name = 0; name < nameList.size(); name++)
	{
		Mat imgmat = imread(FLAGS_path + nameList[name]);
		imgmat = imgmat / 255.0;
		int short_edge = min(imgmat.cols, imgmat.rows);
		int yy = int((imgmat.cols - short_edge) / 2);
		int xx = int((imgmat.rows - short_edge) / 2);
		cv::Rect myROI(yy, xx, yy + short_edge, xx + short_edge);

		Mat crop_img = imgmat(myROI);

		Mat resized_img;
		Size size = Size(224, 224);
		resize(crop_img, resized_img, size, 0, 0, CV_INTER_LINEAR);

		vector<uchar> buff;
		imencode(ext, resized_img, buff);
		std::string img(buff.begin(), buff.end());
		request = RequestInstance(FLAGS_netType, nameList[name], img);
		reply = netInstance.img2vec(request);
		vector<float> temp;
		for (float i : reply.vec())
		{
			temp.push_back(i);
		}
		kdt.push_back(Mat(temp).t());

		if (reply.flag() != "success")
		{
			cout << "grpc failed to get vec!" << endl;
			return 0;
		}
	}
	cout << endl;

	cout << "find the knn." << endl;
	vector<vector<cv::DMatch>> knn;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
	matcher->knnMatch(kdt, kdt, knn, FLAGS_numNN + 1); //because it will match itself.

	cout << "generate pairlist" << endl;
	ofstream out;
	out.open(FLAGS_outputPath + "pairlist.txt", ios::trunc);

	for (auto col : knn)
	{
		for (auto row : col)
		{
			out << row.trainIdx << " ";
		}
		out << "\n";
	}
	out.close();

	cout << "Done!" << endl;
	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}