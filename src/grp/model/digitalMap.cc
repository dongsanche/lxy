#include "digitalMap.h"

namespace ns3 {

DigitalMap::DigitalMap()
{

}

DigitalMap::~DigitalMap()
{

}

void DigitalMap::setMapFilePath(std::string path)
{
	m_path = path;
}

void DigitalMap::readMapFromCsv(std::vector<DigitalMapEntry>& m_map)
{
	std::ifstream file(m_path);//以输入方式打开文件
	std::string line;

	while(!file.eof())
	{
		std::getline(file, line);//file是正在读取的数据流，line是接收字符串的string变量名

		std::istringstream iss(line);//以空格分割字符串
		std::string temp;

		int i = 0, num = 0;
		DigitalMapEntry entry;
		while (std::getline(iss, temp, ','))
		{
			std::string value = std::move(temp);//将temp掏空到value
			if (i == 1)
				entry.x = atof(value.c_str());//字符串转换为浮点数
			else if (i == 2)
				entry.y = atof(value.c_str());
			else if (i == 3)
			{
				num = atoi(value.c_str());//将字符串转换为整型数
				entry.edgeNum = num;
				while(num > 0)
				{
					// next junction index//交叉路口
					std::getline(iss, temp, ',');
					value = std::move(temp);
					int next = atoi(value.c_str());

					std::vector<float> edgeInfo;
					// the angle of the edge
					std::getline(iss, temp, ',');
					value = std::move(temp);
					edgeInfo.push_back(atof(value.c_str()));

					// the length of the edge
					std::getline(iss, temp, ',');
					value = std::move(temp);
					edgeInfo.push_back(atof(value.c_str()));

					// the weight of the edge
					edgeInfo.push_back(atof("0"));

					entry.outedge.insert(std::make_pair(next, edgeInfo));//将两个值视为一个单元

					num--;
				}

				m_map.push_back(entry);
				break;
			}

			i++;
		}
	}

}

void
DigitalMap::readTraceCsv(std::string path, std::vector<VTrace>& tracelist)
{
	std::ifstream file(path);
	std::string line;

	while(!file.eof())
	{
		std::getline(file, line);
		std::istringstream iss(line);
		std::string temp;

		VTrace trace;

		int i = 0;
		while (std::getline(iss, temp, ','))
		{
			std::string value = std::move(temp);
			if(i==0)
				trace.x = atof(value.c_str());
			else if(i==1)
				trace.y = atof(value.c_str());
			else
				trace.jlist.push_back(atoi(value.c_str()));
			i++;
		}
		tracelist.push_back(trace);
	}
}

}
