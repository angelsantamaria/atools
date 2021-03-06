#include "debug_fc.h"

using namespace Eigen;
using namespace std;

namespace atools{

	string bash_color(const color& c)
	{
		stringstream text;
		switch (c) {
			case red:
				text << "\033[1;31m";
				break;
			case green:
				text << "\033[1;32m";
				break;
			case yellow:
				text << "\033[1;33m";
				break;			
			case blue:
				text << "\033[1;34m";
				break;
			case magenta:
				text << "\033[1;35m";
				break;	
			case cyan:
				text << "\033[1;36m";
				break;	
			case white:
				text << "\033[1;37m";
				break;	
			default:
				text << "\033[1;37m";
		}	
		return text.str();
	}
	void print(const string& msg, const color& c=white)
	{
		stringstream text;
	    text << bash_color(c) << msg << "\033[0m";
		cout << text.str();
		restart_white();
	}
	void print(const float& msg, const color& c=white)
	{
		stringstream text;
		text << msg;
		print(text.str(),c);
		restart_white();
	}
	void print(const MatrixXf& msg, const color& c=white)
	{
		stringstream text;
		IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	    text << bash_color(c) << msg.format(OctaveFmt) << "\033[0m";
		cout << text.str();
		restart_white();
	}

	void print(const Quaternionf& msg, const color& c=white)
	{
		stringstream quat;
		quat << "[" << msg.w() << ";\n" << msg.x() << ";\n" << msg.y() << ";\n" << msg.z() << "]";
		print(quat.str(),white);
		restart_white();
	}

	void restart_white(void)
	{
		stringstream text;
		text << "\033[1;37m\033[0m";
		cout << text.str() << flush;
	}
} // End of atools namespace