/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "grp.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-packet-info-tag.h"
#include "ns3/network-module.h"
#include "ns3/tag.h"
#include <cmath>

#define GRP_MAX_MSGS 64
#define GRP_PORT_NUMBER 12345
#define GRP_MAX_SEQ_NUM 65535
#define Natural_Constant_e 2.718 //定义自然常数e的精度

#define GRP_REFRESH_INTERVAL m_helloInterval
#define GRP_NEIGHB_HOLD_TIME Time(1 * GRP_REFRESH_INTERVAL)
#define GRP_BLOCK_CHECK_TIME Time(2 * GRP_REFRESH_INTERVAL)
#define GRP_HEADER_LOC_INTERVAL Time(1 * GRP_REFRESH_INTERVAL)
#define GRP_COLLECT_INTERVAL Time(1 * GRP_REFRESH_INTERVAL)     //cp包生成频率
//#define GRP_RS_VALIDITY_INTERVAL Time(1 * GRP_REFRESH_INTERVAL) //RS结果有效期

#define GRP_MAXJITTER (m_helloInterval.GetSeconds() / 10)
#define JITTER (Seconds(m_uniformRandomVariable->GetValue(0, GRP_MAXJITTER)))

long int controlnum=0;
namespace ns3
{

    NS_LOG_COMPONENT_DEFINE("GrpRoutingProtocol");

    namespace grp
    {
        NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);

        TypeId
        RoutingProtocol::GetTypeId(void)
        {
            static TypeId tid = TypeId("ns3::grp::RoutingProtocol")
                                    .SetParent<Ipv4RoutingProtocol>()
                                    .SetGroupName("grp")
                                    .AddConstructor<RoutingProtocol>()
                                    .AddAttribute("HelloInterval", "HELLO messages emission interval.",
                                                  TimeValue(Seconds(1)),
                                                  MakeTimeAccessor(&RoutingProtocol::m_helloInterval),
                                                  MakeTimeChecker())
                                    .AddTraceSource("DropPacket", "Drop data packet.",
                                                    MakeTraceSourceAccessor(&RoutingProtocol::m_DropPacketTrace),
                                                    "ns3::grp::RoutingProtocol::m_DropPacketTraceCallback")
                                    .AddTraceSource("StorePacket", "Store and carry data packets.",
                                                    MakeTraceSourceAccessor(&RoutingProtocol::m_StorePacketTrace),
                                                    "ns3::grp::RoutingProtocol::m_StorePacketTraceCallback");
            return tid;
        }

        RoutingProtocol::RoutingProtocol()
            : m_ipv4(0),
              m_helloTimer(Timer::CANCEL_ON_DESTROY),
              m_positionCheckTimer(Timer::CANCEL_ON_DESTROY),
              m_queuedMessagesTimer(Timer::CANCEL_ON_DESTROY),
              m_speedTimer(Timer::CANCEL_ON_DESTROY),
              m_RSTimer(Timer::CANCEL_ON_DESTROY)

        {
            m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
        }

        RoutingProtocol::~RoutingProtocol()
        {
        }

        void
        RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4)
        {
            NS_ASSERT(ipv4 != 0);
            NS_ASSERT(m_ipv4 == 0);
            NS_LOG_DEBUG("Created grp::RoutingProtocol");
            m_helloTimer.SetFunction(&RoutingProtocol::HelloTimerExpire, this);
            m_positionCheckTimer.SetFunction(&RoutingProtocol::CheckPositionExpire, this);
            m_speedTimer.SetFunction(&RoutingProtocol::SpeedCheckExpire, this);
            m_queuedMessagesTimer.SetFunction(&RoutingProtocol::SendQueuedMessages, this);
            // m_RSTimer.SetFunction();

            m_packetSequenceNumber = GRP_MAX_SEQ_NUM;
            m_messageSequenceNumber = GRP_MAX_SEQ_NUM;

            m_ipv4 = ipv4;
        }

        void RoutingProtocol::DoDispose()
        {
            m_ipv4 = 0;

            if (m_recvSocket)
            {
                m_recvSocket->Close();
                m_recvSocket = 0;
            }

            for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter = m_sendSockets.begin();
                 iter != m_sendSockets.end(); iter++)
            {
                iter->first->Close();
            }
            m_sendSockets.clear();

            for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator biter = m_sendBlockSockets.begin();
                 biter != m_sendBlockSockets.end(); biter++)
            {
                biter->first->Close();
            }
            m_sendBlockSockets.clear();

            m_neiTable.clear();

            m_wTimeCache.clear();
            m_tracelist.clear();
            m_squeue.clear();
            m_pwaitqueue.clear();
            m_delayqueue.clear();
            m_map.clear();

            delete[] Graph;

            Ipv4RoutingProtocol::DoDispose();
        }

        void
        RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
        {
        }
        
        //初始化RS结果数组
        void 
        RoutingProtocol ::InitialRTNSMRS()
        {
            for(int i=0;i<m_JuncNum;i++)
            {
                for(int j=0;j<m_JuncNum;j++)
                {
                    m_RTNSMRS[i][j].RTNSM_RS=-1;
                    m_RTNSMRS[i][j].RTNSM_RS_flag=0;
                    m_RTNSMRS[i][j].RTNSM_RS_VP=-1;
                }
            }
            //std::cout<<m_id<<"道路评估结果表已初始化"<<m_RTNSMRS[0][1].RTNSM_RS<<std::endl;
        }

        void RoutingProtocol::InitialMID()
        {
            m_id = AddrToID(m_mainAddress); //m_id即为初始化的车辆id
        }

        void RoutingProtocol::InitialPosition() //初始化车辆的位置和编号
        {
            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>(); //MobilityModel类:为所有移动模型的基类,GetObject得到对象
            double xn = MM->GetPosition().x;                            //获取x坐标位置
            double yn = MM->GetPosition().y;                            //获取y坐标位置

            m_last_x = xn;
            m_last_y = yn;

            int i = 0;
            int idx = -1;
            double min = 100000;
            for (std::vector<VTrace>::iterator itr = m_tracelist.begin(); itr != m_tracelist.end(); itr++) //遍历m_tracelist列表
            {
                double dis = sqrt(pow(xn - itr->x, 2) + pow(yn - itr->y, 2)); //坐标距离
                if (dis < min)
                {
                    min = dis;
                    idx = i; //idx最终指向与xn和yn距离最近的点在m_tracelist中的位置
                }
                i++;
            }
            std::vector<int> jlist = m_tracelist[idx].jlist; //VTrace.Jlist即该车轨迹中的路口列表？
            for (std::vector<int>::iterator itr = jlist.begin(); itr != jlist.end(); itr++)
            {
                m_trailTrace.push(*itr); //将itr所指向的jlist值加入队列m_trailTrace
            }

            m_currentJID = m_trailTrace.front();
            m_trailTrace.pop();
            m_nextJID = m_trailTrace.front();
            m_trailTrace.pop();

            m_direction = GetDirection(m_currentJID, m_nextJID);
        }
        //根据路口位置判断车辆方向
        int
        RoutingProtocol::GetDirection(int currentJID, int nextJID)
        {
            double cx = m_map[currentJID].x;
            double cy = m_map[currentJID].y;
            double nx = m_map[nextJID].x;
            double ny = m_map[nextJID].y;

            if (ny == cy)
            {
                if (nx > cx)
                    return 0; //东
                else
                    return 2; //西
            }
            else
            {
                if (ny > cy)
                    return 1; //北
                else
                    return 3; //南
            }

            return -1;
        }

        void RoutingProtocol::ReadConfiguration()
        {
            std::ifstream file(confile); //读取配置文件,输入文件流
            std::string line;
            while (!file.eof()) //用来判断前面的读语句是否读到文件结束符EOF了
            {
                std::getline(file, line); //读取一行

                std::istringstream iss(line); //istringstream对象可以绑定一行字符串,然后以空格为分隔符把该行分隔开来。用于分词
                std::string temp;

                while (std::getline(iss, temp, '=')) //=号作为指定结束符,以=分割
                {
                    std::string value = std::move(temp); //std::move是将对象的状态或者所有权从一个对象转移到另一个对象,只是转移,没有内存的搬迁或者内存拷贝。
                    if (value == "vnum")
                    {
                        std::getline(iss, temp, ',');
                        value = std::move(temp);
                        vnum = atoi(value.c_str()); //c_str()函数返回一个指向正规C字符串的指针常量, 内容与本string串相同
                        //atoi (表示 ascii to integer)是把字符串转换成整型数的一个函数
                    }
                    else if (value == "range")
                    {
                        std::getline(iss, temp, ',');
                        value = std::move(temp);
                        InsightTransRange = atof(value.c_str()); //atof()是C 语言标准库中的一个字符串处理函数,功能是把字符串转换成浮点数
                    }
                    else if (value == "CarryTimeThreshold")
                    {
                        std::getline(iss, temp, ',');
                        value = std::move(temp);
                        CarryTimeThreshold = atof(value.c_str());
                    }
                }
            }
        }

        void RoutingProtocol::DoInitialize() //初始化
        {
            ReadConfiguration(); //读取配置文件

            RSSIDistanceThreshold = InsightTransRange * 0.9; //RSSI=视距的0.9倍
            for (int i = 0; i < m_JuncNum; i++)              //路口数量
            {
                m_jqueuetag[i] = false; //路口标记初始化为false
            }

            Graph = new float *[m_JuncNum]; //将float型地址单元首地址指针分配给graph
            for (int i = 0; i < m_JuncNum; ++i)
            {
                Graph[i] = new float[m_JuncNum];
                for (int j = 0; j < m_JuncNum; j++)
                {
                    Graph[i][j] = INF; //初始化为INF
                }
            }
            //？不太理解下面一段的逻辑
            DigitalMap map;                                                                      //定义地图
            std::string mapfile = "TestScenaries/" + std::to_string(vnum) + "/6x6_map.csv";      //组织地图文件名
            std::string tracefile = "TestScenaries/" + std::to_string(vnum) + "/6x6_vtrace.csv"; //组织vtrace文件名
            map.setMapFilePath(mapfile);
            map.readMapFromCsv(m_map);
            map.readTraceCsv(tracefile, m_tracelist);

            if (m_mainAddress == Ipv4Address())
            {
                Ipv4Address loopback("127.0.0.1"); //回环地址
                for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
                {
                    // Use primary address, if multiple如果有多个地址,使用主地址
                    Ipv4Address addr = m_ipv4->GetAddress(i, 0).GetLocal(); //GetLocal返回本地地址
                    if (addr != loopback)
                    {
                        m_mainAddress = addr; //配置一个ipv4地址
                        break;
                    }
                }

                NS_ASSERT(m_mainAddress != Ipv4Address()); //不满足就崩溃？
            }

            NS_LOG_DEBUG("Starting Grp on node " << m_mainAddress);

            Ipv4Address loopback("127.0.0.1");

            bool canRunGrp = false;
            for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
            {
                Ipv4Address addr = m_ipv4->GetAddress(i, 0).GetLocal();

                if (addr == Ipv4Address("127.0.0.1"))
                {
                    continue;
                }
                // Create a socket to listen on all the interfaces//监听所有接口
                if (m_recvSocket == 0)
                {
                    m_recvSocket = Socket::CreateSocket(GetObject<Node>(),
                                                        UdpSocketFactory::GetTypeId());
                    m_recvSocket->SetAllowBroadcast(true); //设置所有的允许的广播？
                    InetSocketAddress inetAddr(Ipv4Address::GetAny(), GRP_PORT_NUMBER);
                    m_recvSocket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvGrp, this)); //
                    if (m_recvSocket->Bind(inetAddr))                                             //bind将端口和socket绑定
                    {
                        NS_FATAL_ERROR("Failed to bind() grp socket");
                    }
                    m_recvSocket->SetRecvPktInfo(true);
                    m_recvSocket->ShutdownSend();
                }

                // Create a socket to send packets from this specific interfaces
                Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(),
                                                          UdpSocketFactory::GetTypeId());
                socket->SetAllowBroadcast(true);
                InetSocketAddress inetAddr(m_ipv4->GetAddress(i, 0).GetLocal(), GRP_PORT_NUMBER);
                socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvGrp, this));
                socket->BindToNetDevice(m_ipv4->GetNetDevice(i));
                if (socket->Bind(inetAddr))
                {
                    NS_FATAL_ERROR("Failed to bind() GRP socket");
                }
                socket->SetRecvPktInfo(true);
                m_sendSockets[socket] = m_ipv4->GetAddress(i, 0);

                canRunGrp = true;
            }

            if (canRunGrp)
            {
                startTime += 1;
                Simulator::Schedule(Seconds(0.01), &RoutingProtocol::InitialMID, this);
                Simulator::Schedule(Seconds(0.01), &RoutingProtocol::InitialRTNSMRS, this);
                Simulator::Schedule(Seconds(startTime), &RoutingProtocol::InitialPosition, this);
                double helloStartTime = startTime + 1 + AddrToID(m_mainAddress) * 0.001;
                Simulator::Schedule(Seconds(helloStartTime), &RoutingProtocol::HelloTimerExpire, this);
                Simulator::Schedule(Seconds(startTime + 2), &RoutingProtocol::CheckPositionExpire, this);
                Simulator::Schedule(Seconds(startTime + 3), &RoutingProtocol::SpeedCheckExpire, this);

                NS_LOG_DEBUG("Grp on node " << m_mainAddress << " started");
            }
        }

        void RoutingProtocol::SetMainInterface(uint32_t interface)
        {
            m_mainAddress = m_ipv4->GetAddress(interface, 0).GetLocal(); //ipv4接口地址
        }

        void
        RoutingProtocol::RecvGrp(Ptr<Socket> socket)
        {
            Ptr<Packet> receivedPacket;
            Address sourceAddress;
            receivedPacket = socket->RecvFrom(sourceAddress);

            Ipv4PacketInfoTag interfaceInfo;
            if (!receivedPacket->RemovePacketTag(interfaceInfo)) //？
            {
                NS_ABORT_MSG("No incoming interface on GRP message, aborting.");
            }
            uint32_t incomingIf = interfaceInfo.GetRecvIf();
            Ptr<Node> node = this->GetObject<Node>();
            Ptr<NetDevice> dev = node->GetDevice(incomingIf);
            uint32_t recvInterfaceIndex = m_ipv4->GetInterfaceForDevice(dev);

            InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom(sourceAddress);
            Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4();

            int32_t interfaceForAddress = m_ipv4->GetInterfaceForAddress(senderIfaceAddr);
            if (interfaceForAddress != -1)
            {
                NS_LOG_LOGIC("Ignoring a packet sent by myself.");
                return;
            }

            Ipv4Address receiverIfaceAddr = m_ipv4->GetAddress(recvInterfaceIndex, 0).GetLocal();
            NS_ASSERT(receiverIfaceAddr != Ipv4Address());
            NS_LOG_DEBUG("GRP node " << m_mainAddress << " received a GRP packet from "
                                     << senderIfaceAddr << " to " << receiverIfaceAddr);

            // All routing messages are sent from and to port RT_PORT,
            // so we check it.
            NS_ASSERT(inetSourceAddr.GetPort() == GRP_PORT_NUMBER);

            Ptr<Packet> packet = receivedPacket;

            grp::CtrPacketHeader GrpPacketHeader;
            packet->RemoveHeader(GrpPacketHeader);
            NS_ASSERT(GrpPacketHeader.GetPacketLength() >= GrpPacketHeader.GetSerializedSize());
            uint32_t sizeLeft = GrpPacketHeader.GetPacketLength() - GrpPacketHeader.GetSerializedSize();

            MessageList messages;

            while (sizeLeft)
            {
                MessageHeader messageHeader;
                if (packet->RemoveHeader(messageHeader) == 0)
                {
                    NS_ASSERT(false);
                }

                sizeLeft -= messageHeader.GetSerializedSize();

                NS_LOG_DEBUG("Grp Msg received with type "
                             << std::dec << int(messageHeader.GetMessageType())
                             << " TTL=" << int(messageHeader.GetTimeToLive())
                             << " origAddr=" << messageHeader.GetOriginatorAddress());
                messages.push_back(messageHeader);
            }

            for (auto messageIter = messages.begin();
                 messageIter != messages.end(); messageIter++)
            {
                 MessageHeader &messageHeader = *messageIter;
                if (messageHeader.GetTimeToLive() == 0 || messageHeader.GetOriginatorAddress() == m_mainAddress)
                {
                    packet->RemoveAtStart(messageHeader.GetSerializedSize() - messageHeader.GetSerializedSize());
                    continue;
                }

                switch (messageHeader.GetMessageType())
                {
                case grp::MessageHeader::HELLO_MESSAGE:
                    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                                 << "s GRP node " << m_mainAddress
                                 << " received HELLO message of size " << messageHeader.GetSerializedSize());
                    ProcessHello(messageHeader, receiverIfaceAddr, senderIfaceAddr);
                    break;
                // case grp::MessageHeader::CollectPacket_MESSAGE:
                //     NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                //                  << "s GRP node " << m_mainAddress
                //                  << " received collect packet of size " << messageHeader.GetSerializedSize());
                //     ProcessCollectPacket(messageHeader, receiverIfaceAddr, senderIfaceAddr);
                //     break;
                // case grp::MessageHeader::RTNSMResult_MESSAGE:
                //     NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                //                  << "s GRP node " << m_mainAddress
                //                  << " received collect packet of size " << messageHeader.GetSerializedSize());
                //     ProcessRTNSMResult(messageHeader, receiverIfaceAddr, senderIfaceAddr);
                //     break;
                default:
                    NS_LOG_DEBUG("GRP message type " << int(messageHeader.GetMessageType()) << " not implemented");
                }
            }
        }

        void
        RoutingProtocol::SendFromDelayQueue() //从延迟队列中发包
        {
            if (m_delayqueue.empty() == false)
            {
                DelayPacketQueueEntry sentry = m_delayqueue.back(); //获取尾部元素
                m_delayqueue.pop_back();

                Ptr<Ipv4Route> rtentry;
                rtentry = Create<Ipv4Route>();
                rtentry->SetDestination(sentry.m_header.GetDestination());
                rtentry->SetSource(sentry.m_header.GetSource());
                rtentry->SetGateway(sentry.m_nexthop);
                rtentry->SetOutputDevice(m_ipv4->GetNetDevice(0));
                sentry.m_header.SetTtl(sentry.m_header.GetTtl() + 1); //？
                sentry.m_ucb(rtentry, sentry.m_packet, sentry.m_header);
            }
        }

        // void
        // RoutingProtocol::CheckPacketQueue() //检查车辆是否携带有缓存数据包
        // {
        //     m_pqueue.assign(m_pwaitqueue.begin(), m_pwaitqueue.end()); //把begin和end之间的数据赋值给m_pqueue
        //     m_pwaitqueue.clear();

        //     while (m_pqueue.empty() == false)
        //     {
        //         PacketQueueEntry qentry = m_pqueue.back();
        //         m_pqueue.pop_back();

        //         Ipv4Address dest = qentry.m_header.GetDestination();
        //         Ipv4Address origin = qentry.m_header.GetSource();

        //         QPacketInfo pInfo(origin, dest);                      //？
        //         QMap::const_iterator pItr = m_wTimeCache.find(pInfo); //返回迭代器
        //         if (pItr != m_wTimeCache.end() && Simulator::Now().GetSeconds() - pItr->second.GetSeconds() >= CarryTimeThreshold)
        //         {
        //             NS_LOG_UNCOND("Store time more than: " << CarryTimeThreshold << "s.");
        //             m_DropPacketTrace(qentry.m_header);
        //             m_wTimeCache.erase(pInfo);
        //             continue;
        //         }

        //         grp::DataPacketHeader DataPacketHeader;
        //         qentry.m_packet->RemoveHeader(DataPacketHeader);
        //         int nextjid = (int)DataPacketHeader.GetNextJID();
        //         int fromjid=(int)DataPacketHeader.GetFromJID();

        //         Ipv4Address loopback("127.0.0.1");
        //         Ipv4Address nextHop("127.0.0.1");

        //         Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
        //         double cx = MM->GetPosition().x;
        //         double cy = MM->GetPosition().y;
        //         double dx = GetPosition(dest).x;
        //         double dy = GetPosition(dest).y;
        //         double cjx = m_map[m_currentJID].x;
        //         double cjy = m_map[m_currentJID].y;
        //         double njx = m_map[m_nextJID].x;
        //         double njy = m_map[m_nextJID].y;
        //         double curDisToDst = sqrt(pow(cx - dx, 2) + pow(cy - dy, 2));  //当前车辆距离dest的距离
        //         double curJTODst = sqrt(pow(cjx - dx, 2) + pow(cjy - dy, 2));  //currentJID距离dest的距离
        //         double nextJTODst = sqrt(pow(njx - dx, 2) + pow(njy - dy, 2)); //nextJID距离dest的距离

        //         int nowjid=GetNearestJID();
        //         int dstjid=nextjid;
        //         m_tempjid=nextjid;
        //         m_tempjid2=nowjid;
        //         //如果接收到数据包的车辆刚好位于路口范围内,则先进行路段间路由为数据包选定下一路由路段
        //         if (m_JunAreaTag == true)
        //         {
        //             nextHop = IntersectionAreaReliableRouting(dest,fromjid,0); //进入路段间路由算法
        //             if(nextHop!=loopback)
        //             {
        //                 dstjid=m_tempjid;
        //                 fromjid=m_tempjid2;
        //             }
                    

        //         }
        //         else
        //         {
        //             // if(curJTODst < nextJTODst)
        //             // {
        //             //     dstjid = m_currentJID;
        //             // }
        //             // else
        //             // {
        //             //     dstjid = m_nextJID;
        //             // }
        //             //在路段内路由时，不改变dstjid
        //             dstjid=nextjid;
        //             //dstjid = curJTODst < nextJTODst ? m_currentJID : m_nextJID; //距离dest最近的路口作为dstjid
        //             nextHop = RoadAreaReliableRouting(dest,fromjid, dstjid);
        //         }

        //         DataPacketHeader.SetNextJID(dstjid);
        //         DataPacketHeader.SetFromJID(fromjid);
        //         qentry.m_packet->AddHeader(DataPacketHeader); //加包头

        //         if (nextHop == loopback)
        //             m_pwaitqueue.push_back(qentry); //加入等待队列
        //         else
        //         {
        //             m_wTimeCache.erase(pInfo);
        //             m_squeue.push_back(SendingQueue(qentry.m_packet, qentry.m_header, qentry.m_ucb, nextHop)); //加入到发包队列
        //             // NS_LOG_UNCOND("" << Simulator::Now().GetSeconds() << " " << m_id << " forwards a STORE data packet to " << AddrToID(nextHop));
        //         }
        //     }

        //     if (m_squeue.empty() == false)
        //     {
        //         SendFromSQueue();
        //     }
        // }



        void
RoutingProtocol::CheckPacketQueue()
{
    m_pqueue.assign(m_pwaitqueue.begin(), m_pwaitqueue.end());
 	m_pwaitqueue.clear();

  	while(m_pqueue.empty() == false)
 	{
 		PacketQueueEntry qentry = m_pqueue.back();
 		m_pqueue.pop_back();

 		Ipv4Address dest = qentry.m_header.GetDestination();
 		Ipv4Address origin = qentry.m_header.GetSource();

  		QPacketInfo pInfo(origin, dest);
 		QMap::const_iterator pItr = m_wTimeCache.find(pInfo);
 		if(pItr != m_wTimeCache.end() && Simulator::Now().GetSeconds() - pItr->second.GetSeconds() >= CarryTimeThreshold )
 		{
 			NS_LOG_UNCOND("Store time more than: " << CarryTimeThreshold << "s.");
 			m_DropPacketTrace(qentry.m_header);
 			m_wTimeCache.erase(pInfo);
 			continue;
 		}

  		grp::DataPacketHeader DataPacketHeader;
 		qentry.m_packet->RemoveHeader (DataPacketHeader);
 		int nextjid = (int)DataPacketHeader.GetNextJID();
 		
  		Ipv4Address loopback ("127.0.0.1");
        Ipv4Address nextHop("127.0.0.1");

        if(m_JunAreaTag == true)
        {
            nextjid = GetPacketNextJID(true);
        }
        else
        {
            if(nextjid != m_currentJID && nextjid != m_nextJID)
            {
                nextjid = GetNearestJID();
            }
        }
        nextHop = IntraPathRouting(dest, nextjid);

        DataPacketHeader.SetNextJID(nextjid);
        qentry.m_packet->AddHeader (DataPacketHeader);

  		if(nextHop == loopback)
 			m_pwaitqueue.push_back(qentry);
 		else
 		{
 			m_wTimeCache.erase(pInfo);
 			m_squeue.push_back(SendingQueue(qentry.m_packet, qentry.m_header, qentry.m_ucb, nextHop));
 			// NS_LOG_UNCOND("" << Simulator::Now().GetSeconds() << " " << m_id << " forwards a STORE data packet to " << AddrToID(nextHop));

  		}
 	}

  	if(m_squeue.empty() == false)
 	{
 		SendFromSQueue();
 	}

}


        void
        RoutingProtocol::SendFromSQueue()
        {
            if (m_squeue.empty() == false)
            {
                SendingQueue sentry = m_squeue.back(); //返回m_squeue最后一个元素
                m_squeue.pop_back();                   //将已赋值给sentry的最后一个元素删除

                Ptr<Ipv4Route> rtentry;
                rtentry = Create<Ipv4Route>();
                rtentry->SetDestination(sentry.m_header.GetDestination());
                rtentry->SetSource(sentry.m_header.GetSource());
                rtentry->SetGateway(sentry.nexthop);
                rtentry->SetOutputDevice(m_ipv4->GetNetDevice(0));
                sentry.m_ucb(rtentry, sentry.m_packet, sentry.m_header); //单播回调用来发包

                Simulator::Schedule(MilliSeconds(10), &RoutingProtocol::SendFromSQueue, this); //MilliSeconds,毫秒,即每隔10毫秒执行一下函数,&RoutingProtocol::SendFromSQueue
            }
        }

        bool
        RoutingProtocol::isAdjacentVex(int sjid, int ejid) //判断两个路口是否相邻
        {
            for (std::map<int, std::vector<float>>::iterator itr = m_map[sjid].outedge.begin();
                 itr != m_map[sjid].outedge.end(); itr++)
            {
                if (itr->first == ejid)
                    return true;
            }
            return false;
        }

        void
        RoutingProtocol::ProcessHello(grp::MessageHeader &msg,
                                      const Ipv4Address receiverIfaceAddr,
                                      const Ipv4Address senderIface)
        {
            const grp::MessageHeader::Hello &hello = msg.GetHello();

            //Restrict the communication between the vehicles with different direction.限制不同方向车辆间的通信
            int cjid = GetNearestJID();
            if ((int)hello.GetDirection() != m_direction && (int)hello.GetDirection() != (m_direction + 2) % 4) //方向不相同也不相反？
            {
                double jx = m_map[cjid].x;
                double jy = m_map[cjid].y;
                double nx = hello.GetLocationX();
                double ny = hello.GetLocationY();
                if (m_JunAreaTag == false && sqrt(pow(nx - jx, 2) + pow(ny - jy, 2)) > JunAreaRadius) //不在路口范围内且cj和hello距离大于路口范围半径
                {
                    return;
                }
            }

            Ipv4Address originatorAddress = msg.GetOriginatorAddress();                                         //源地址
            std::map<Ipv4Address, NeighborTableEntry>::const_iterator itr = m_neiTable.find(originatorAddress); //返回源地址的在邻居表的指示器
            if (itr != m_neiTable.end() && itr->second.N_sequenceNum >= msg.GetMessageSequenceNumber())
                return;
            if (itr != m_neiTable.end())
            {
                m_neiTable.erase(originatorAddress);
            }

            NeighborTableEntry &neiTableTuple = m_neiTable[originatorAddress];
            neiTableTuple.N_neighbor_address = msg.GetOriginatorAddress();
            neiTableTuple.N_speed = hello.GetSpeed();
            neiTableTuple.N_direction = hello.GetDirection();
            neiTableTuple.N_location_x = hello.GetLocationX();
            neiTableTuple.N_location_y = hello.GetLocationY();
            neiTableTuple.receiverIfaceAddr = receiverIfaceAddr;
            neiTableTuple.N_sequenceNum = msg.GetMessageSequenceNumber();
            neiTableTuple.N_time = Simulator::Now() + msg.GetVTime();

            neiTableTuple.N_turn = hello.GetTurn();

            neiTableTuple.N_status = NeighborTableEntry::STATUS_NOT_SYM;
            for (std::vector<Ipv4Address>::const_iterator i = hello.neighborInterfaceAddresses.begin();
                 i != hello.neighborInterfaceAddresses.end(); i++)
            {
                if (m_mainAddress == *i)
                {
                    neiTableTuple.N_status = NeighborTableEntry::STATUS_SYM;
                    break;
                }
            }

            Simulator::Schedule(GRP_NEIGHB_HOLD_TIME, &RoutingProtocol::NeiTableCheckExpire, this, originatorAddress); 

            if (m_pwaitqueue.empty() == false)
            {
                CheckPacketQueue();
            }
        }

        int
        RoutingProtocol::AddrToID(Ipv4Address addr)
        {
            int tnum = addr.Get();          
            return tnum / 256 % 256 * 256 + tnum % 256 - 1; 
        }

        void
        RoutingProtocol::QueueMessage(const grp::MessageHeader &message, Time delay)
        {
            m_queuedMessages.push_back(message);
            if (not m_queuedMessagesTimer.IsRunning())
            {
                m_queuedMessagesTimer.SetDelay(delay);
                m_queuedMessagesTimer.Schedule();
            }
        }

        void
        RoutingProtocol::SendQueuedMessages()
        {
            Ptr<Packet> packet = Create<Packet>();
            int numMessages = 0;

            MessageList msglist;

            for (std::vector<grp::MessageHeader>::const_iterator message = m_queuedMessages.begin();
                 message != m_queuedMessages.end();
                 message++)
            {
                Ptr<Packet> p = Create<Packet>();
                p->AddHeader(*message);
                packet->AddAtEnd(p);
                msglist.push_back(*message);
                if (++numMessages == GRP_MAX_MSGS)
                {
                    SendPacket(packet);
                    msglist.clear();
                    numMessages = 0;
                    packet = Create<Packet>();
                }
            }

            if (packet->GetSize())
            {
                SendPacket(packet);
            }

            m_queuedMessages.clear();
        }

        void
        RoutingProtocol::SendPacket(Ptr<Packet> packet)
        {
            // Add a header
            grp::CtrPacketHeader header;
            header.SetPacketLength(header.GetSerializedSize() + packet->GetSize()); 
            header.SetPacketSequenceNumber(GetPacketSequenceNumber());              
            packet->AddHeader(header);                                              

            // Send it
            for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
                     m_sendSockets.begin();
                 i != m_sendSockets.end(); i++)
            {
                Ptr<Packet> pkt = packet->Copy();
                //TODO need to test the mask is 8bits or 16bits
                Ipv4Address bcast = i->second.GetLocal().GetSubnetDirectedBroadcast(i->second.GetMask()); //GetLocal获取本地地址,GetMask获取子网掩码,GetSubnetDirectedBroadcast生成并返回与掩码相对应的子网定向广播地址
                i->first->SendTo(pkt, 0, InetSocketAddress(bcast, GRP_PORT_NUMBER));                      //传入广播地址和端口数
            }
        }
        //current和next从哪里确定？
        int
        RoutingProtocol::GetNearestJID()
        {
            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;
            //如果当前ID的距离小,则返回当前ID,否则返回next
            if (pow(cx - m_map[m_currentJID].x, 2) + pow(cy - m_map[m_currentJID].y, 2) < pow(cx - m_map[m_nextJID].x, 2) + pow(cy - m_map[m_nextJID].y, 2))
            {
                return m_currentJID;
            }
            else
            {
                return m_nextJID;
            }
        }
        //发送hello包的代码？
        void
        RoutingProtocol::SendHello()
        {
            NS_LOG_FUNCTION(this); //日志

            grp::MessageHeader msg;
            Time now = Simulator::Now(); //Simulator类是访问事件调度工具的公共入口点。在开始仿真时,会调度一些事件至事件列表中,通过进入仿真器主循环（即Simulator::Run）按仿真时间顺序执行他们。仿真器执行事件期间,调度器（Scheduler）会插入或移除某些事件至事件列表中。直到事件列表中没有事件需要被执行或强制调用了Simultor::Stop函数,此时仿真结束。

            msg.SetVTime(GRP_NEIGHB_HOLD_TIME);
            msg.SetOriginatorAddress(m_mainAddress); //源地址？
            msg.SetTimeToLive(1);
            msg.SetHopCount(0);
            msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
            grp::MessageHeader::Hello &hello = msg.GetHello();

            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double positionX = MM->GetPosition().x;
            double positionY = MM->GetPosition().y;
            hello.SetLocation(positionX, positionY);

            hello.SetSpeedAndDirection(m_speed, m_direction);

            for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator iter = m_neiTable.begin();
                 iter != m_neiTable.end(); iter++)
            {
                hello.neighborInterfaceAddresses.push_back(iter->first);
            }

            QueueMessage(msg, JITTER);
        }

        uint16_t RoutingProtocol::GetPacketSequenceNumber()
        {
            m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (GRP_MAX_SEQ_NUM + 1);
            return m_packetSequenceNumber;
        }

        uint16_t RoutingProtocol::GetMessageSequenceNumber()
        {
            m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (GRP_MAX_SEQ_NUM + 1);
            return m_messageSequenceNumber;
        }

        void
        RoutingProtocol::HelloTimerExpire() //定时器有效期？周期性发送？
        {
            SendHello();
            m_helloTimer.Schedule(m_helloInterval); //时延
        }

        // void
        // RoutingProtocol::CheckPositionExpire()
        // {

        //     Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>(); //在ns3中,MobilityModel使用一个（笛卡尔）坐标系跟踪位置的演化。mobility模型聚合了ns3::Node对象和使用GetObject<MobilityModel> ()方法查询节点。
        //     double cvx = MM->GetPosition().x;
        //     double cvy = MM->GetPosition().y;
        //     double njx = m_map[m_nextJID].x;
        //     double njy = m_map[m_nextJID].y;
        //     double cjx = m_map[m_currentJID].x;
        //     double cjy = m_map[m_currentJID].y;

        //     double disToNextJun = sqrt(pow(cvx - njx, 2) + pow(cvy - njy, 2));
        //     double disToCurrJun = sqrt(pow(cvx - cjx, 2) + pow(cvy - cjy, 2));
            
        //     if (disToNextJun <= PositionCheckThreshold)//小于11米改变路口id并设置转向
        //     {
        //         m_turn = -1;
        //         //用m_lastJID记录车子经过的上一个路口
        //         m_lastJID=m_currentJID;
        //         m_currentJID = m_nextJID;

        //         m_nextJID = m_trailTrace.front();
        //         m_trailTrace.pop();

        //         m_direction = GetDirection(m_currentJID, m_nextJID);//设置转向
        //         //允许在vp值到期之前重新进行道路评估（icar）
        //         //仅当车上没有道路评估结果或已经超过有效期之后才发送cp包
        //         if(m_RTNSMRS[m_lastJID][m_currentJID].RTNSM_RS==-1||m_RTNSMRS[m_lastJID][m_currentJID].RTNSM_RS==0||(m_RTNSMRS[m_lastJID][m_currentJID].RTNSM_RS_TimeStamp.GetSeconds()+m_RTNSMRS[m_lastJID][m_currentJID].RTNSM_RS_VP<=Simulator::Now().GetSeconds()))
        //         {
        //             //车子从i路口驶向j路口，在车子刚刚进入j路口时往i路口发送cp包，即距离路口位置小于11米时发送cp包
        //             //从第10秒开始发，待邻居表基本完善后
        //             if(Simulator::Now().GetSeconds()>10)
        //             {
        //                 SendCollectotPaket (m_currentJID,m_lastJID,0);//0代表发送的是cp包而不是cpr
        //             }

        //         }
        //     }
        //     else if (disToNextJun <= turnLightRange)
        //     {
        //         if (m_turn < 0)
        //             m_turn = m_trailTrace.front();
        //     }

        //     if (m_JunAreaTag == false)
        //     {
        //         if (disToNextJun < JunAreaRadius)
        //         {
                    
        //             m_JunAreaTag = true;//如果m_JunAreaTag不在路口范围，将其修改为在路口范围，即车辆正在进入路口范围内
        //         }
        //     }
        //     else
        //     {
        //         if (disToNextJun > JunAreaRadius && disToCurrJun > JunAreaRadius)//两个路口都不靠近
        //         {
        //             if(m_JunAreaTag==true)//如果之前m_JunAreaTag为true，将其修改为false，说明车辆正在离开路口范围
        //             {
        //                 m_JunAreaTag = false;

        //             }
        //             //m_JunAreaTag = false;
        //         }
        //     }

        //     m_positionCheckTimer.Schedule(Seconds(0.1)); //时延,以秒为单位
        // }

        void
RoutingProtocol::CheckPositionExpire()
{
	Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
	double cvx = MM->GetPosition ().x;
	double cvy = MM->GetPosition ().y;
	double njx = m_map[m_nextJID].x;
	double njy = m_map[m_nextJID].y;
	double cjx = m_map[m_currentJID].x;
	double cjy = m_map[m_currentJID].y;
	
	double disToNextJun = sqrt(pow(cvx-njx, 2) + pow(cvy-njy, 2));
	double disToCurrJun = sqrt(pow(cvx-cjx, 2) + pow(cvy-cjy, 2));
	if(disToNextJun <= PositionCheckThreshold)
	{
		m_turn = -1;
		m_currentJID = m_nextJID; 

		m_nextJID = m_trailTrace.front();
		m_trailTrace.pop();

		m_direction = GetDirection(m_currentJID, m_nextJID);
	}
	else if(disToNextJun <= turnLightRange)
	{
		if(m_turn < 0)
			m_turn = m_trailTrace.front();
	}

    if(m_JunAreaTag == false)
    {
        if(disToNextJun < JunAreaRadius)
        {
            m_JunAreaTag = true;
        }
    }
    else
    {
        if(disToNextJun > JunAreaRadius && disToCurrJun > JunAreaRadius)
        {
            m_JunAreaTag = false;
        }
    }
    
    m_positionCheckTimer.Schedule(Seconds(0.1));
}



        void
        RoutingProtocol::SpeedCheckExpire()
        {
            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;
            m_speed = sqrt(pow(cx - m_last_x, 2) + pow(cy - m_last_y, 2)); //坐标距离
                                                                           //更新上一参考坐标位置
            m_last_x = cx;
            m_last_y = cy;

            m_speedTimer.Schedule(GRP_NEIGHB_HOLD_TIME); //延时
        }

        void
        RoutingProtocol::NeiTableCheckExpire(Ipv4Address addr)
        {
            NeighborTableEntry nentry = m_neiTable[addr];
            if (nentry.N_time <= Simulator::Now()) //N_time小于当前时间,说明已过期
            {
                m_neiTable.erase(addr);
            }
        }

        // void 
        // RoutingProtocol::CPExpire ()
        // {
        //     Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
        //     //如果车辆已经发送过cp包，且上一个cp包的发送时间在4ms之前
        //     if(m_CPSendFlag==1&&(Simulator::Now().GetSeconds()-m_CPSendFlagTimeStamp.GetSeconds())>0.004)
        //     {
        //         //检查车上的结果有没有更新
        //         //如果已经更新，即结果时间在m_CPSendFlagTimeStamp之后，那么不必处理
        //         //如果没有更新，即结果时间在m_CPSendFlagTimeStamp之前，那么将结果更新为0

        //     }


        // }

        int64_t
        RoutingProtocol::AssignStreams(int64_t stream)
        {
            NS_LOG_FUNCTION(this << stream);
            m_uniformRandomVariable->SetStream(stream);
            return 1;
        }

        void
        RoutingProtocol::SetDownTarget(IpL4Protocol::DownTargetCallback callback) //通过Ipv4发送数据包的回调
        {
            m_downTarget = callback;
        }

        Vector
        RoutingProtocol::GetPosition(Ipv4Address adr) //传入车辆的Ipv4地址
        {
            uint32_t n = NodeList().GetNNodes(); //GetNNodes返回当前列表中的节点数
            uint32_t i;
            Ptr<Node> node;
            //Every Node created is added to the NodeList automatically.
            for (i = 0; i < n; i++)
            {
                node = NodeList().GetNode(i);                 //GetNode返回请求节点的索引,node格式
                Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();     //获取node的ipv4
                if (ipv4->GetAddress(1, 0).GetLocal() == adr) //如果node的address和传入的adress相同
                {
                    return (*node->GetObject<MobilityModel>()).GetPosition(); //返回该node的位置,即车辆的位置
                }
            }
            Vector v; //？
            return v;
        }

        // void
        // RoutingProtocol::AddHeader(Ptr<Packet> p,Ipv4Address source, Ipv4Address destination, uint8_t protocol, Ptr<Ipv4Route> route)
        // {
        //     Ipv4Mask brocastMask("0.0.255.255");  //广播的子网掩码
        //     Ipv4Address dest = destination;                                     
        //     if (brocastMask.IsMatch(destination, Ipv4Address("0.0.255.255")) == false) //如果两个地址的屏蔽位相等,返回true,如果不等,执行后续程序
        //     {
        //         Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
        //         double cx = MM->GetPosition().x;
        //         double cy = MM->GetPosition().y;
        //         double dx = GetPosition(dest).x;
        //         double dy = GetPosition(dest).y;
        //         double cjx = m_map[m_currentJID].x;
        //         double cjy = m_map[m_currentJID].y;
        //         double njx = m_map[m_nextJID].x;
        //         double njy = m_map[m_nextJID].y;
        //         double curDisToDst = sqrt(pow(cx - dx, 2) + pow(cy - dy, 2));  //当前车辆距离dest的距离
        //         double curJTODst = sqrt(pow(cjx - dx, 2) + pow(cjy - dy, 2));  //currentJID距离dest的距离
        //         double nextJTODst = sqrt(pow(njx - dx, 2) + pow(njy - dy, 2)); //nextJID距离dest的距离
        //         //nextjid为距离MM最近的路口
        //         int nextjid;
        //         int fromjid;
        //         if (curJTODst < nextJTODst)
        //         {
        //             nextjid = m_currentJID;
        //             fromjid=m_nextJID;
        //         }
        //         else
        //         {
        //             nextjid = m_nextJID;
        //             fromjid=m_currentJID;
        //         }

        //         grp::DataPacketHeader Dheader; //定义一个数据包包头
        //         Time lut = Simulator::Now();
        //         Dheader.SetNextJID(nextjid); //加入下一个路口的id
        //         Dheader.SetFromJID(fromjid);
        //         std::cout<<Simulator::Now().GetSeconds()<<"车的走向："<<m_currentJID<<"->"<<m_nextJID<<std::endl;
        //         std::cout<<Simulator::Now().GetSeconds()<<"m_id"<<m_id<<std::endl;
        //         std::cout<<Simulator::Now().GetSeconds()<<"this: "<<nextjid<<std::endl;
               
        //         p->AddHeader(Dheader);//packet的addheader函数
        //     }

        //     m_downTarget(p, source, destination, protocol, route); //?
        // }

        void
RoutingProtocol::AddHeader (Ptr<Packet> p, Ipv4Address source, Ipv4Address destination, uint8_t protocol, Ptr<Ipv4Route> route)
{
	Ipv4Mask brocastMask("0.0.255.255");
	if (brocastMask.IsMatch(destination, Ipv4Address("0.0.255.255")) == false)
	{
		Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
        double cx = MM->GetPosition ().x;
        double cy = MM->GetPosition ().y;
        double cjx = m_map[m_currentJID].x;
        double cjy = m_map[m_currentJID].y;
        double njx = m_map[m_nextJID].x;
        double njy = m_map[m_nextJID].y;

        int nextjid;
        if(pow(cx-cjx, 2) + pow(cy-cjy, 2) < pow(cx-njx, 2) + pow(cy-njy, 2))
        {
            nextjid = m_currentJID;
        } 
        else
        {
            nextjid = m_nextJID;
        }

        grp::DataPacketHeader Dheader;
        Time lut = Simulator::Now();
        Dheader.SetNextJID(nextjid);
        p->AddHeader (Dheader);

	}

	m_downTarget (p, source, destination, protocol, route);

}


        bool
        RoutingProtocol::isBetweenSegment(double nx, double ny, int cjid, int djid)
        {
            bool res = false;
            double djx = m_map[djid].x;
            double djy = m_map[djid].y;
            double cjx = m_map[cjid].x;
            double cjy = m_map[cjid].y;

            double minx = (cjx < djx ? cjx : djx);
            double maxx = (cjx > djx ? cjx : djx);
            double miny = (cjy < djy ? cjy : djy);
            double maxy = (cjy > djy ? cjy : djy);
            int dir = GetDirection(cjid, djid);
            if (dir % 2 == 0)
            {
                miny -= RoadWidth;
                maxy += RoadWidth;
            }
            else
            {
                minx -= RoadWidth;
                maxx += RoadWidth;
            }

            if (nx >= minx && nx <= maxx && ny >= miny && ny <= maxy)
            {
                res = true;
            }

            return res;
        }

        Ipv4Address
        RoutingProtocol::IntraPathRouting(Ipv4Address dest,  int dstjid)
        {
            Ipv4Address nextHop = Ipv4Address("127.0.0.1");
            //
            if(dstjid < 0)
            {
                return  nextHop;
            }

            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;
            
            double dx = GetPosition(dest).x;//目的地
            double dy = GetPosition(dest).y;
            double curDisToDst = sqrt(pow(cx-dx, 2) + pow(cy-dy, 2));
            if(curDisToDst < RSSIDistanceThreshold)//在RSSI距离阈值内
                return dest;

            double jx = m_map[dstjid].x;//最近路口位置
            double jy = m_map[dstjid].y;
            double mindis = sqrt(pow(cx-jx, 2) + pow(cy-jy, 2));

            for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator i = m_neiTable.begin (); i != m_neiTable.end (); i++)
            {
                if(i->second.N_status == NeighborTableEntry::STATUS_NOT_SYM)//second指的是map中的第二个元素
                {
                    continue;
                }

                double nx = i->second.N_location_x;
                double ny = i->second.N_location_y;
                double neiDisToJID = sqrt(pow(nx-jx, 2) + pow(ny-jy, 2));
                double curDisToNei = sqrt(pow(cx-nx, 2) + pow(cy-ny, 2));
                if(neiDisToJID < mindis && curDisToNei < RSSIDistanceThreshold)//邻居到最近路口的位置小于当前节点到最近路口的位置,且当前节点到邻居的距离小于RSSI阈值
                {
                    int cjid = GetNearestJID();//获得距离当前车辆最近的路口ID
                    //此处的条件判断用以防止当前车辆将数据包传输给其他路段的节点,
                    //其他路段的节点同样有可能满足上一个条件判断
                    if(isBetweenSegment(nx, ny, cjid, dstjid) == true)//车辆不在路口范围内或者车辆在两个指定的路口形成的矩形区域内
                    {
                        mindis = neiDisToJID;//令mindis=邻居到路口的距离
                        nextHop = i->first;//map的前一个元素,即下一节点的address
                    }
                }
            }
            
            return nextHop;//返回下一跳地址
        }

        // Ptr<Ipv4Route>
        // RoutingProtocol::RouteOutput(Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr) //节点本身有数据包要发送
        // {
        //     NS_LOG_FUNCTION(this << " " << m_ipv4->GetObject<Node>()->GetId() << " " << header.GetDestination() << " " << oif); //日志
        //     Ptr<Ipv4Route> rtentry = NULL;
        //     //原来没有的内容
        //     Ptr<Packet> packet = p->Copy();

        //     grp::DataPacketHeader DataPacketHeader;
        //     packet->RemoveHeader(DataPacketHeader);
        //     //
        //     Ipv4Address dest = header.GetDestination();     //获取包头中的目的地址
        //     Ipv4Address nextHop = Ipv4Address("127.0.0.1"); //下一跳地址

        //     Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
        //     double cx = MM->GetPosition().x;
        //     double cy = MM->GetPosition().y;
        //     double dx = GetPosition(dest).x;
        //     double dy = GetPosition(dest).y;
        //     double cjx = m_map[m_currentJID].x;
        //     double cjy = m_map[m_currentJID].y;
        //     double njx = m_map[m_nextJID].x;
        //     double njy = m_map[m_nextJID].y;
        //     double curDisToDst = sqrt(pow(cx - dx, 2) + pow(cy - dy, 2));  //当前车辆距离dest的距离
        //     double curJTODst = sqrt(pow(cjx - dx, 2) + pow(cjy - dy, 2));  //currentJID距离dest的距离
        //     double nextJTODst = sqrt(pow(njx - dx, 2) + pow(njy - dy, 2)); //nextJID距离dest的距离
        //     std::cout<<Simulator::Now().GetSeconds()<<curJTODst<<"-"<<nextJTODst<<std::endl;

        //     int dstjid=-1;
        //     int fromjid=-1;
        //     if (m_JunAreaTag == false) //如果车辆不在路口范围内,选择距离目的地最近的路口作为目标路口
        //     {
        //         if(curJTODst < nextJTODst)//如果当前路口距离rsu的距离比下一个路口小，那么当前路口作为tpjid，next路口作为fromjid
        //         {
        //             dstjid = m_currentJID;
        //             fromjid=m_nextJID;
        //         }
        //         else
        //         {
        //             dstjid = m_nextJID;
        //             fromjid=m_currentJID;
        //         }
        //         nextHop = RoadAreaReliableRouting(dest, fromjid,dstjid);//进入路段内路由算法,传入目的地和目标路口id,返回nexthop
        //     }
        //     else
        //     {
        //         int nowjid=GetNearestJID();
        //         m_tempjid=-1;//一开始发送的时候，nextjid没有初值
        //         m_tempjid2=nowjid;
        //         nextHop = IntersectionAreaReliableRouting(dest,fromjid,1); //进入路段间路由算法
        //         if(nextHop!=Ipv4Address("127.0.0.1"))
        //         {
        //             dstjid=m_tempjid;
        //             fromjid=m_tempjid2;
        //         }
        //     }
        //     //
        //     std::cout<<Simulator::Now().GetSeconds()<<"m_JunAreaTag: "<<m_JunAreaTag<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<fromjid<<"->"<<dstjid<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<m_id<<"-"<<AddrToID(nextHop)<<"-"<<dstjid<<std::endl;

        //     Ipv4Address loopback("127.0.0.1"); //环回
        //     //nextHop = IntraPathRouting(dest, dstjid);//进入路段内路由算法,传入目的地地址和最近路口id,返回nexthop地址
        //     //nextHop=RoadAreaReliableRouting(dest,dstjid);//进入路段内路由算法,传入目的地和目标路口id,返回nexthop
        //     grp::DataPacketHeader DHeader;
        //     DHeader.SetNextJID(dstjid);
        //     DHeader.SetSenderID(m_id);
        //     DHeader.SetFromJID(fromjid);
        //     packet->AddHeader(DHeader);
        //     //std::cout<<"验证此时是否插值"<<fromjid<<"->"<<dstjid<<std::endl;
        //     if (nextHop == loopback || nextHop == dest || m_JunAreaTag == true) //如果下一跳地址等于lookback或者等于目的地或者在路口范围内
        //     {
        //         rtentry = Create<Ipv4Route>();
        //         rtentry->SetDestination(header.GetDestination());        //设置目标ipv4 address
        //         rtentry->SetSource(m_ipv4->GetAddress(1, 0).GetLocal()); //设置源ipv4 address
        //         rtentry->SetGateway(loopback);                           //设置网关的下一跳address
        //         rtentry->SetOutputDevice(m_ipv4->GetNetDevice(0));       //指向NetDevice的传出数据包的指针
        //         sockerr = Socket::ERROR_NOTERROR;
        //     }
        //     else
        //     {
        //         rtentry = Create<Ipv4Route>();
        //         rtentry->SetDestination(header.GetDestination());
        //         Ipv4Address receiverIfaceAddr = m_neiTable.find(nextHop)->second.receiverIfaceAddr;

        //         rtentry->SetSource(receiverIfaceAddr);
        //         rtentry->SetGateway(nextHop);
        //         for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++) //GetNInterfaces返回用户添加的接口数目
        //         {
        //             for (uint32_t j = 0; j < m_ipv4->GetNAddresses(i); j++) //传入接口号,GetNAddresses返回接口的Ipv4InterfaceAddress条目数
        //             {
        //                 if (m_ipv4->GetAddress(i, j).GetLocal() == receiverIfaceAddr) //返回与接口和addressIndex关联的Ipv4InterfaceAddress
        //                 {
        //                     rtentry->SetOutputDevice(m_ipv4->GetNetDevice(i));
        //                     break;
        //                 }
        //             }
        //         }
        //         //
                
        //         //

        //         sockerr = Socket::ERROR_NOTERROR;
        //     }
        //     return rtentry;
        // }


Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
	NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
	Ptr<Ipv4Route> rtentry = NULL;

	Ipv4Address dest = header.GetDestination ();
	Ipv4Address nextHop = Ipv4Address("127.0.0.1");

    Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
    double cx = MM->GetPosition ().x;
    double cy = MM->GetPosition ().y;
    double cjx = m_map[m_currentJID].x;
    double cjy = m_map[m_currentJID].y;
    double njx = m_map[m_nextJID].x;
    double njy = m_map[m_nextJID].y;

    int dstjid;
    if(m_JunAreaTag == false)
    {
        dstjid = pow(cx-cjx, 2) + pow(cy-cjy, 2) < pow(cx-njx, 2) + pow(cy-njy, 2)? m_currentJID:m_nextJID;
    }
    else
    {
        dstjid = GetPacketNextJID(true);
    }

    Ipv4Address loopback ("127.0.0.1");
    nextHop = IntraPathRouting(dest, dstjid);
    if(nextHop == loopback || nextHop == dest || m_JunAreaTag == true)
    {
        rtentry = Create<Ipv4Route> ();
        rtentry->SetDestination (header.GetDestination ());
        rtentry->SetSource (m_ipv4->GetAddress (1, 0).GetLocal ());
        rtentry->SetGateway (loopback);
        rtentry->SetOutputDevice (m_ipv4->GetNetDevice (0));
        sockerr = Socket::ERROR_NOTERROR;
    }
    else
    {
        rtentry = Create<Ipv4Route> ();
        rtentry->SetDestination (header.GetDestination ());
        Ipv4Address receiverIfaceAddr = m_neiTable.find(nextHop)->second.receiverIfaceAddr;
            
        rtentry->SetSource (receiverIfaceAddr);
        rtentry->SetGateway (nextHop);
        for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
        {
            for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
            {
                if (m_ipv4->GetAddress (i,j).GetLocal () == receiverIfaceAddr)
                {
                    rtentry->SetOutputDevice (m_ipv4->GetNetDevice (i));
                    break;
                }
            }
        }

        sockerr = Socket::ERROR_NOTERROR;

    }
	return rtentry;
}
        int
        RoutingProtocol::DijkstraAlgorithm(int srcjid, int dstjid)
        {
            bool visited[m_JuncNum];
            double distance[m_JuncNum];
            int parent[m_JuncNum];

            for (int i = 0; i < m_JuncNum; i++)
            {
                visited[i] = false;
                distance[i] = INF;
                parent[i] = -1;
            }

            visited[srcjid] = true;
            distance[srcjid] = 0;

            int curr = srcjid;
            int next = -1;
            for (int count = 1; curr >= 0 && count <= m_JuncNum; count++)
            {
                double min = INF;
                for (int n = 0; n < m_JuncNum; n++)
                {
                    if (visited[n] == false)
                    {
                        if (distance[curr] + Graph[curr][n] < distance[n])
                        {
                            distance[n] = distance[curr] + Graph[curr][n];
                            parent[n] = curr;
                        }

                        if (distance[n] < min)
                        {
                            min = distance[n];
                            next = n;
                        }
                    }
                }
                curr = next;
                visited[curr] = true;
            }

            int jid = dstjid;
            while (jid > 0)
            {
                if (parent[jid] == srcjid)
                    break;
                jid = parent[jid];
            }

            return jid;
        }

        int
        RoutingProtocol::GetPacketNextJID(bool tag)
        {
            int cjid = GetNearestJID();

            if(cjid == m_rsujid)
                return cjid;

            int nextjid = -1;
            //根据连通性赋值
            for(int i = 0; i < m_JuncNum; i++)
            {
                for(int j = i + 1; j < m_JuncNum; j++)
                {
                    if(isAdjacentVex(i, j) == false)
                    {
                        Graph[i][j] = Graph[j][i] = INF;
                    }
                    else
                    {
                        Graph[i][j] = Graph[j][i] = 1;
                    }
                }
            }

            nextjid = DijkstraAlgorithm(cjid, m_rsujid);//利用迪杰斯特拉算法寻找目标路段

            return nextjid;
        }


bool RoutingProtocol::RouteInput  (Ptr<const Packet> p,
                                   const Ipv4Header &header, Ptr<const NetDevice> idev,
                                   UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                                   LocalDeliverCallback lcb, ErrorCallback ecb)
{
	NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());

	Ipv4Address dest = header.GetDestination ();
    Ipv4Address origin = header.GetSource ();

	NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
	uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);
	if (m_ipv4->IsDestinationAddress (dest, iif))
	{
		if (!lcb.IsNull ())
		{
			NS_LOG_LOGIC ("Local delivery to " << dest);
			lcb (p, header, iif);
			return true;
		}
		else
		{
			return false;
		}
	}

    //判断当前数据包的TTL是否还存活
	if(header.GetTtl() <= 1)
	{
		NS_LOG_UNCOND("TTL < 0");
		m_DropPacketTrace(header);
		return true;
	}

	Ptr<Ipv4Route> rtentry;
	Ipv4Address loopback ("127.0.0.1");
	Ptr<Packet> packet = p->Copy ();
	grp::DataPacketHeader DataPacketHeader;
	packet->RemoveHeader (DataPacketHeader);
	int nextjid = (int)DataPacketHeader.GetNextJID();
    int senderID = DataPacketHeader.GetSenderID();

    //如果接收到数据包的车辆刚好位于路口范围内，则先进行路段间路由为数据包选定下一路由路段
    if(m_JunAreaTag == true)
    {
        nextjid = GetPacketNextJID(true);
        NS_LOG_UNCOND("JID: " << (int)DataPacketHeader.GetNextJID() << "->" << nextjid);
        NS_LOG_UNCOND("");
    }

    //路段内路由，为数据包选定下一跳节点
	Ipv4Address nextHop = IntraPathRouting(dest, nextjid);

	NS_LOG_UNCOND("" << Simulator::Now().GetSeconds() << " " << m_id << "->" << AddrToID(nextHop));

	if (nextHop != loopback)
	{
        //若返回了下一跳的地址，则将该数据包转发给该下一跳节点
        if(senderID != AddrToID(nextHop))
        {
            rtentry = Create<Ipv4Route> ();
            rtentry->SetDestination (header.GetDestination ());
            Ipv4Address receiverIfaceAddr = m_neiTable.find(nextHop)->second.receiverIfaceAddr;

            if(nextHop == dest)
                receiverIfaceAddr = m_mainAddress;

            rtentry->SetSource (receiverIfaceAddr);
            rtentry->SetGateway (nextHop);

            for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
            {
                for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
                {
                    if (m_ipv4->GetAddress (i,j).GetLocal () == receiverIfaceAddr)
                    {
                        rtentry->SetOutputDevice (m_ipv4->GetNetDevice (i));
                        break;
                    }
                }
            }

            if(nextHop != header.GetDestination())
            {
                grp::DataPacketHeader DHeader;
                DHeader.SetNextJID(nextjid);
                DHeader.SetSenderID(m_id);
                packet->AddHeader(DHeader);
            }

            ucb (rtentry, packet, header);
        }
        else
        {
            //如果下一跳是自己的上一跳，则可能出现了路由回路，可能导致TTL的快速消耗，故暂缓发送数据包
            grp::DataPacketHeader DHeader;
            DHeader.SetNextJID(nextjid);
            DHeader.SetSenderID(m_id);		
            packet->AddHeader(DHeader);
            DelayPacketQueueEntry qentry(packet, header, ucb, nextHop);		
  		    m_delayqueue.push_back(qentry);	
            Simulator::Schedule(m_helloInterval / 4, &RoutingProtocol::SendFromDelayQueue, this);
        }	
	}
	else
	{
        //如果返回的IPv4地址为127.0.0.1，则说明当前时刻没有合适的下一跳节点
        //节点启用Carry_and_forward机制，将数据包暂时缓存起来，直到有可用下一跳节点或信息过期为止
        grp::DataPacketHeader DHeader;
  		DHeader.SetNextJID(nextjid);	
        DHeader.SetSenderID(m_id);	
  		packet->AddHeader(DHeader);		

    	QPacketInfo pInfo(origin, dest);		
  		QMap::const_iterator pItr = m_wTimeCache.find(pInfo);		
  		if(pItr == m_wTimeCache.end())		
  		{		
  			Time &pTime = m_wTimeCache[pInfo];		
  			pTime = Simulator::Now();		
  		}		

    	PacketQueueEntry qentry(packet, header, ucb);		
  		m_pwaitqueue.push_back(qentry);		
  		m_StorePacketTrace(header);
	}
	return true;
}

        // bool RoutingProtocol::RouteInput(Ptr<const Packet> p,
        //                                  const Ipv4Header &header, Ptr<const NetDevice> idev,
        //                                  UnicastForwardCallback ucb, MulticastForwardCallback mcb,
        //                                  LocalDeliverCallback lcb, ErrorCallback ecb)
        // {
        //     NS_LOG_FUNCTION(this << " " << m_ipv4->GetObject<Node>()->GetId() << " " << header.GetDestination());

        //     Ipv4Address dest = header.GetDestination();
        //     Ipv4Address origin = header.GetSource();
            
            


        //     NS_ASSERT(m_ipv4->GetInterfaceForDevice(idev) >= 0);
        //     uint32_t iif = m_ipv4->GetInterfaceForDevice(idev);
        //     if (m_ipv4->IsDestinationAddress(dest, iif))
        //     {
        //         if (!lcb.IsNull())
        //         {
        //             NS_LOG_LOGIC("Local delivery to " << dest);
        //             lcb(p, header, iif);
        //             return true;
        //         }
        //         else
        //         {
        //             return false;
        //         }
        //     }

        //     //判断当前数据包的TTL是否还存活
        //     if (header.GetTtl() <= 1)
        //     {
        //         NS_LOG_UNCOND("TTL < 0");
        //         m_DropPacketTrace(header);
        //         return true;
        //     }

        //     Ptr<Ipv4Route> rtentry;
            
        //     Ipv4Address nextHop = Ipv4Address("127.0.0.1"); //新加
        //     Ipv4Address loopback("127.0.0.1");
        //     Ptr<Packet> packet = p->Copy();
        //     grp::DataPacketHeader DataPacketHeader;
        //     packet->RemoveHeader(DataPacketHeader);
        //     int nextjid = (int)DataPacketHeader.GetNextJID();
        //     int senderID = DataPacketHeader.GetSenderID();
        //     int fromjid=(int)DataPacketHeader.GetFromJID();//获取数据包上一个路口的ID
        //     //
        //     Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
        //     double cx = MM->GetPosition().x;
        //     double cy = MM->GetPosition().y;
        //     double dx = GetPosition(dest).x;
        //     double dy = GetPosition(dest).y;
        //     double cjx = m_map[m_currentJID].x;
        //     double cjy = m_map[m_currentJID].y;
        //     double njx = m_map[m_nextJID].x;
        //     double njy = m_map[m_nextJID].y;
        //     double curDisToDst = sqrt(pow(cx - dx, 2) + pow(cy - dy, 2));  //当前车辆距离dest的距离
        //     double curJTODst = sqrt(pow(cjx - dx, 2) + pow(cjy - dy, 2));  //currentJID距离dest的距离
        //     double nextJTODst = sqrt(pow(njx - dx, 2) + pow(njy - dy, 2)); //nextJID距离dest的距离

        //     int nowjid=GetNearestJID();
        //     m_tempjid=nextjid;
        //     m_tempjid2=nowjid;
        //     int dstjid=nextjid;//初始化为nextjid
        //     //int lastjid=fromjid;
        //     //如果接收到数据包的车辆刚好位于路口范围内,则先进行路段间路由为数据包选定下一路由路段
        //     if (m_JunAreaTag == true)
        //     {
        //         //int ctjid=nextjid;
        //         std::cout<<Simulator::Now().GetSeconds()<<"进入路段间路由算法之前的fromjid"<<fromjid<<std::endl;
        //         nextHop = IntersectionAreaReliableRouting(dest,fromjid,1); //进入路段间路由算法

        //         //判断nextHop所对应的
        //         if(nextHop!=loopback)//成功找到下一跳才改       
        //         {
        //            dstjid=m_tempjid;
        //            fromjid=m_tempjid2;
        //         }
                
        //     }
        //     else
        //     {
        //         dstjid=nextjid;
        //         //dstjid = curJTODst < nextJTODst ? m_currentJID : m_nextJID; //距离dest最近的路口作为dstjid
        //         nextHop = RoadAreaReliableRouting(dest, fromjid,dstjid);
        //     }

        //     //路段内路由,为数据包选定下一跳节点
        //     //Ipv4Address nextHop = IntraPathRouting(dest, nextjid);

        //     std::cout<<Simulator::Now().GetSeconds()<<"m_JunAreaTag"<<m_JunAreaTag<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<"nowjid"<<nowjid<<std::endl;
        //     //std::cout<<"01道路得分"<<m_RTNSMRS[0][1].RTNSM_RS<<std::endl;
        //     //std::cout<<"05道路得分"<<m_RTNSMRS[0][5].RTNSM_RS<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<"nextjid:"<<nextjid<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<"m_tempjid:"<<m_tempjid<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<fromjid<<"->"<<dstjid<<std::endl;
        //     std::cout<<Simulator::Now().GetSeconds()<<m_id<<"-"<<AddrToID(nextHop)<<"-"<<dstjid<<std::endl;

        //     NS_LOG_UNCOND("" << Simulator::Now().GetSeconds() << " " << m_id << "->" << AddrToID(nextHop));

        //     if (nextHop != loopback)
        //     {
        //         //若返回了下一跳的地址,则将该数据包转发给该下一跳节点
        //         if (senderID != AddrToID(nextHop))
        //         {
        //             rtentry = Create<Ipv4Route>();
        //             rtentry->SetDestination(header.GetDestination());
        //             Ipv4Address receiverIfaceAddr = m_neiTable.find(nextHop)->second.receiverIfaceAddr;

        //             if (nextHop == dest)
        //                 receiverIfaceAddr = m_mainAddress;

        //             rtentry->SetSource(receiverIfaceAddr);
        //             rtentry->SetGateway(nextHop);

        //             for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
        //             {
        //                 for (uint32_t j = 0; j < m_ipv4->GetNAddresses(i); j++)
        //                 {
        //                     if (m_ipv4->GetAddress(i, j).GetLocal() == receiverIfaceAddr)
        //                     {
        //                         rtentry->SetOutputDevice(m_ipv4->GetNetDevice(i));
        //                         break;
        //                     }
        //                 }
        //             }

        //             if (nextHop != header.GetDestination())
        //             {
        //                 grp::DataPacketHeader DHeader;
        //                 DHeader.SetNextJID(dstjid);
        //                 DHeader.SetSenderID(m_id);
        //                 DHeader.SetFromJID(fromjid);
        //                 packet->AddHeader(DHeader);
        //             }

        //             ucb(rtentry, packet, header); //通过ucb发包
        //         }
        //         else
        //         {
        //             //如果下一跳是自己的上一跳,则可能出现了路由回路,可能导致TTL的快速消耗,故暂缓发送数据包
        //             grp::DataPacketHeader DHeader;
        //             DHeader.SetNextJID(dstjid);
        //             DHeader.SetSenderID(m_id);
        //             DHeader.SetFromJID(fromjid);
        //             packet->AddHeader(DHeader);
        //             DelayPacketQueueEntry qentry(packet, header, ucb, nextHop);
        //             m_delayqueue.push_back(qentry);
        //             Simulator::Schedule(m_helloInterval / 4, &RoutingProtocol::SendFromDelayQueue, this);
        //         }
        //     }
        //     else
        //     {
        //         //如果返回的IPv4地址为127.0.0.1,则说明当前时刻没有合适的下一跳节点
        //         //节点启用Carry_and_forward机制,将数据包暂时缓存起来,直到有可用下一跳节点或信息过期为止
        //         grp::DataPacketHeader DHeader;
        //         DHeader.SetNextJID(dstjid);
        //         DHeader.SetSenderID(m_id);
        //         DHeader.SetFromJID(fromjid);
        //         packet->AddHeader(DHeader);

        //         QPacketInfo pInfo(origin, dest);
        //         QMap::const_iterator pItr = m_wTimeCache.find(pInfo);
        //         if (pItr == m_wTimeCache.end())
        //         {
        //             Time &pTime = m_wTimeCache[pInfo];
        //             pTime = Simulator::Now();
        //         }

        //         PacketQueueEntry qentry(packet, header, ucb);
        //         m_pwaitqueue.push_back(qentry);
        //         m_StorePacketTrace(header);
        //     }
        //     return true;
        // }

        void
        RoutingProtocol::NotifyInterfaceUp(uint32_t i)
        {
        }
        void
        RoutingProtocol::NotifyInterfaceDown(uint32_t i)
        {
        }
        void
        RoutingProtocol::NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address)
        {
        }
        void
        RoutingProtocol::NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address)
        {
        }
        
        /*------------------------------------------------------------------------------------------*/
        //cp包路段内路由函数,选择相同道路上距离下一路口最近的邻居
        Ipv4Address
        RoutingProtocol::CollectPacketRouting(int cjid, int dstjid)
        {
            Time now=Simulator::Now();
            Ipv4Address nextHop = Ipv4Address("127.0.0.1");
            //dstjid<0,路由到自身
            if (dstjid < 0)
            {
                return nextHop;
            }

            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;

            double jx = m_map[dstjid].x;
            double jy = m_map[dstjid].y;
            double mindis = sqrt(pow(cx - jx, 2) + pow(cy - jy, 2));

            for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator i = m_neiTable.begin(); i != m_neiTable.end(); i++)
            {
                if (i->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) //second指的是map中的第二个元素
                {
                    continue;
                }

                double nx = i->second.N_location_x;
                double ny = i->second.N_location_y;
                //初始化邻居预测位置
                double predicted_nx=nx;
                double predicted_ny=ny;
                //时间差
                double time_ago=now.GetSeconds()-(i->second.N_time.GetSeconds());
                //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                if(i->second.N_direction==0)
                {
                    predicted_nx=nx+(i->second.N_speed*time_ago);
                }
                else if(i->second.N_direction==1)
                {
                    predicted_ny=ny+(i->second.N_speed*time_ago); 
                }
                else if(i->second.N_direction==2)
                {
                    predicted_nx=nx-(i->second.N_speed*time_ago);
                }
                else if(i->second.N_direction==3)
                {
                    predicted_ny=ny-(i->second.N_speed*time_ago);
                }
                //
                double neiDisToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));
                double curDisToNei = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2));
                if (neiDisToJID < mindis && curDisToNei < RSSIDistanceThreshold) //邻居到最近路口的位置小于当前节点到最近路口的位置,且当前节点到邻居的距离小于RSSI阈值
                {
                    if (isBetweenSegment(predicted_nx, predicted_ny, cjid, dstjid) == true) //车辆不在路口范围内或者车辆在两个指定的路口形成的矩形区域内
                    {
                        mindis = neiDisToJID; //令mindis=邻居到路口的距离,利用贪心算法选取距离dstjid最近的节点作为下一跳
                        nextHop = i->first;   //map的前一个元素,即下一节点的address
                    }
                }
            }
            //std::cout<<Simulator::Now().GetSeconds()<<"下一跳"<<AddrToID(nextHop)<<std::endl;

            return nextHop; //返回下一跳地址
        }

        /*------------------------------------------------------------------------------------------*/
        //cp包生成函数
        void
        RoutingProtocol::SendCollectotPaket(int sourcejid,int dstjid, int OriginalityFlag)
        {
            NS_LOG_FUNCTION(this);

            grp::MessageHeader msg;
            Time now = Simulator::Now();

            msg.SetVTime(GRP_NEIGHB_HOLD_TIME);
            msg.SetOriginatorAddress(m_mainAddress);
            msg.SetTimeToLive(1);
            msg.SetHopCount(0);
            msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
            grp::MessageHeader::CollectPacket &cp = msg.GetCollectPacket();

            Ipv4Address NextHop = CollectPacketRouting(sourcejid, dstjid);
            //如果找不到下一跳，应该返回原路口，逆转CollectPacketRouting参数
            

            cp.SetCFAddress(m_mainAddress);
            cp.SetNFAddress(NextHop);
            cp.SetFirstJID(sourcejid);//cp包生成的路口id
            cp.SetSencondJID(dstjid);//cp包走向的路口id
            //计算vp初值
            double uninVP=calculatedVPV(m_mainAddress,sourcejid,dstjid,now);
            int VPintPart=(int)uninVP;
            int VPfractionPart=(uninVP-VPintPart)*10000;
            cp.SetVPVValidityPeriodValue(VPintPart);//设置初始vp值
            cp.SetVPVValidityPeriodValueFPart(VPfractionPart);
            //std::cout<<"初始vp值已设置"<<cp.GetVPVValidityPeriodValue()<<std::endl;
            cp.SetOriginalityFlag(OriginalityFlag); //如果OriginalityFlag为0代表是创建cp,为1代表创建cpr
            cp.SetTimeStamp(now);
            cp.SetNumberOfHops(0);
            cp.SetDVDDensity(0);
            cp.SetIVDDensity(0);

            QueueMessage(msg, JITTER);//放入消息队列
            controlnum++;
        }

        /*------------------------------------------------------------------------------------------*/
        //vp初值计算函数
        double
        RoutingProtocol::calculatedVPV(Ipv4Address m_mainAddress,int sourcejid,int dstjid,Time now)
        {
            double vp=0.00;
            double numberOfNeighbour=0;
            double TotalLLT = 0.00;
            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;

            double jx = m_map[dstjid].x;
            double jy = m_map[dstjid].y;
            double mindis = sqrt(pow(cx - jx, 2) + pow(cy - jy, 2));
            int roadDirection = GetDirection(sourcejid, dstjid);

            for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator i = m_neiTable.begin(); i != m_neiTable.end(); i++)
            {
                if (i->second.N_status == NeighborTableEntry::STATUS_NOT_SYM)
                {
                    continue;
                }
                double nx = i->second.N_location_x;
                double ny = i->second.N_location_y;
                //初始化邻居预测位置
                double predicted_nx=nx;
                double predicted_ny=ny;
                //时间差
                double time_ago=now.GetSeconds()-(i->second.N_time.GetSeconds());
                //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                if(i->second.N_direction==0)
                {
                    predicted_nx=nx+(i->second.N_speed*time_ago);
                }
                else if(i->second.N_direction==1)
                {
                    predicted_ny=ny+(i->second.N_speed*time_ago); 
                }
                else if(i->second.N_direction==2)
                {
                    predicted_nx=nx-(i->second.N_speed*time_ago);
                }
                else if(i->second.N_direction==3)
                {
                    predicted_ny=ny-(i->second.N_speed*time_ago);
                }
                //调试时可将邻居位置，替换为邻居预测位置
                double neiDisToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));
                double curDisToNei = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2));
                double calculatedLLT = 0.00;
                if (neiDisToJID < mindis && curDisToNei <RSSIDistanceThreshold) //邻居到dstjid的位置小于当前节点到dstjid的位置,且当前节点到邻居的距离小于RSSI阈值
                {
                    if (isBetweenSegment(predicted_nx, predicted_ny, sourcejid, dstjid) == true) //邻居车辆在两个指定的路口形成的矩形区域内
                    {
                        calculatedLLT = 0.00;                     //先将calculatedLLT归0
                        if (m_direction == i->second.N_direction) //二车方向相同
                        {
                            calculatedLLT = RSSIDistanceThreshold / i->second.N_speed;
                        }
                        else if ((m_direction + 2) % 4 == i->second.N_direction) //二车方向相反
                        {
                            if (i->second.N_direction == roadDirection) //邻居在朝着dstjid的方向移动，当前车辆朝着sourcejid的方向移动，由于邻居比当前车辆更接近dst，二车正在远离
                            {
                                calculatedLLT = (RSSIDistanceThreshold - curDisToNei) / (m_speed + i->second.N_speed);
                            }
                            else //二车正在靠近
                            {
                                calculatedLLT = (RSSIDistanceThreshold + curDisToNei) / (m_speed + i->second.N_speed);
                            }
                        }
                        TotalLLT = TotalLLT + calculatedLLT;
                        numberOfNeighbour++;
                    }
                }
            }
            if (numberOfNeighbour != 0)
            {
                //自定义参数
                double ICSFactor = 0.50; //论文中给的icsfactor范围为[0,1]，但是计算公式需要的密度变化参数没有给具体的计算方式,暂定为固定值
                vp = ICSFactor * (TotalLLT / numberOfNeighbour);
            }
            //std::cout<<"vpv已经计算"<<vp<<std::endl;
            return vp;
        }

        /*------------------------------------------------------------------------------------------*/
        //cp包接收处理程序
        void
        RoutingProtocol::ProcessCollectPacket(grp::MessageHeader &msg, const Ipv4Address receiverIfaceAddr,
                                              const Ipv4Address senderIface)
        {
            grp::MessageHeader::CollectPacket &cp = msg.GetCollectPacket();
            Time now = Simulator::Now();
            if (m_mainAddress != cp.GetNFAddress()) //如果本车不是目标地址则不对此cp作任何处理
            {
                return;
            }
            Ipv4Address nextHop = Ipv4Address("127.0.0.1");
            int sourcejid = cp.GetFirstJID();
            int dstjid = cp.GetSencondJID();
            int curJID = GetNearestJID();
            //按照icar中计算方式，给cp的定时只有4ms，也就是单程2ms，如果traveltime大于2ml，那么说明cp包不能按时返回结果？
            double cp_travletime = now.GetSeconds() - cp.GetTimeStamp().GetSeconds();
            //如果时间大于2ms，且没有抵达下一个路口，需要将生成cp的路口的道路评估结果设置为0
            //if(m_JunAreaTag==false&&cp_travletime>0.002)
            //{
                //nextHop = CollectPacketRouting(dstjid, sourcejid);
                //cp.SetOriginalityFlag(2);
            //}
            
            
            double calculated_VPV = 0.00;
            int NumberOfHops = cp.GetNumberOfHops();

            double VPV = cp.GetVPVValidityPeriodValue()+double(cp.GetVPVValidityPeriodValueFPart())/10000;
            //vpv应该有效
            int IVDNummer = cp.GetIVDDensity(); //与道路同向车辆数量
            int OVDNummer = cp.GetDVDDensity(); //与道路反向车辆数量
            int roadDirection = GetDirection(sourcejid, dstjid);
            //自定义参数
            double cp_frequency=1.0;//cp包发送间隔自定义设为1.0秒
            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;

            double jx = m_map[dstjid].x;
            double jy = m_map[dstjid].y;
            double mindis = sqrt(pow(cx - jx, 2) + pow(cy - jy, 2));
            double minNeiDisToJID = mindis;
            //如果当前车辆已经进入目标路口范围
            if (m_JunAreaTag==true)
            {
                //std::cout<<"cp包已经传进路口"<<curJID<<std::endl;
                //Time now = Simulator::Now();
                
                double RTNSMResult = calculateRTNSMResult(IVDNummer, OVDNummer, cp_travletime, NumberOfHops+1, VPV); //计算cp包收集的数据结果
                std::cout<<Simulator::Now().GetSeconds()<<"-已经计算评估结果"<<RTNSMResult<<std::endl;
                //std::cout<<"该评估结果的vp"<<VPV<<std::endl;
                int updateFlag = 0;
                int RS_flag=0;
                if(cp.GetOriginalityFlag() == 0)//如果是cp包，RS_flag=1，如果是cpr，RS_flag=2
                {
                    RS_flag=1;
                }
                else RS_flag=2;
                //更新车上的结果
                struct RTNSMRSEntry e;
                e.RTNSM_RS_TimeStamp=now;
                e.RTNSM_RS=RTNSMResult;
                e.RTNSM_RS_VP=VPV;
                if(curJID==sourcejid)
                {
                    e.RTNSM_RS=0.0;
                    e.RTNSM_RS_VP=2.0;
                }
                e.RTNSM_RS_flag=RS_flag;
                m_RTNSMRS[sourcejid][dstjid]=e;
                m_RTNSMRS[dstjid][sourcejid]=e;
                std::cout<<Simulator::Now().GetSeconds()<<"-"<<m_id<<"车载结果已更新"<<"-RS"<<"-由"<<sourcejid<<"到"<<dstjid<<"为"<<m_RTNSMRS[sourcejid][dstjid].RTNSM_RS<<std::endl;
                //m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_TimeStamp=m_RTNSMRS[dstjid][sourcejid].RTNSM_RS_TimeStamp=now;
                //m_RTNSMRS[sourcejid][dstjid].RTNSM_RS=m_RTNSMRS[dstjid][sourcejid].RTNSM_RS=RTNSMResult;
                //m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_VP=m_RTNSMRS[dstjid][sourcejid].RTNSM_RS_VP=VPV;
                //m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_flag=m_RTNSMRS[dstjid][sourcejid].RTNSM_RS_flag=RS_flag;

                //判断是否需要创建cpr
                //case1 如果此车在此之前没有接收到来自sourcejid的cp或者cpr产生的结果，意味着在此之前没有cpr从dstjid发往sourcejid(vp值有效)
                //case2 如果此车携带有来自sourcejid的cpr产生的结果，意味着有从dstjid发往sourcejid的cp已经在sourcejid广播了结果，无需再次发送cpr
                //case3 如果此车携带有来自sourcejid的cp产生的结果，意味着sourcejid产生了连续的cp，如果此车发送的cpr能够抑制sourcejid产生新的cp，即这个cpr应该在sourcejid产生新的cp之前到达sourcejid，否则不会产生新的cpr，因为它很快会被代替
                if(cp.GetOriginalityFlag() == 0)//当前处理的是cp包且是正常发到的，需要创建cpr
                {
                    if(m_RTNSMRS[sourcejid][dstjid].RTNSM_RS==0||m_RTNSMRS[sourcejid][dstjid].RTNSM_RS==-1||(m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_TimeStamp.GetSeconds()+m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_VP)<=now.GetSeconds())//case 1 说明没有有效结果可用或者结果已经过期
                    {
                        if(VPV>cp_travletime)//说明cpr可以到达
                        {
                            SendCollectotPaket(dstjid,sourcejid, 1);//反方向创建cpr
                            //std::cout<<"已创建由路口"<<dstjid<<"到路口"<<sourcejid<<"的cpr"<<std::endl;
                        }
                    }
                    else if(m_RTNSMRS[sourcejid][dstjid].RTNSM_RS!=0&&(m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_TimeStamp.GetSeconds()+m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_VP)>now.GetSeconds())//case3
                    {
                        if(m_RTNSMRS[sourcejid][dstjid].RTNSM_RS_flag==1)//cp产生的结果
                        {
                            if(VPV>(cp_frequency-cp_travletime))//cpr可以抑制新的cp产生
                            {
                                SendCollectotPaket(dstjid,sourcejid, 1);//反方向创建cpr
                                //std::cout<<"已创建由路口"<<dstjid<<"到路口"<<sourcejid<<"的cpr"<<std::endl;
                            }
                        }

                    }
                }
                
                //
                SendRTNSMResult(e, sourcejid, dstjid); //无论是否创建cpr，都需要广播calculateRTNSMResult
                //std::cout<<"已向路口广播道路评估结果"<<std::endl;
                return;
            }
            
            calculated_VPV=calculatedVPV(m_mainAddress,sourcejid,dstjid,now);
            if (cp.GetVPVValidityPeriodValue() > calculated_VPV)
            {
                cp.SetVPVValidityPeriodValue(calculated_VPV); //更新vp值,二者之间选最小
            }
            else//否则不更新vp值
            {
                cp.SetVPVValidityPeriodValue(cp.GetVPVValidityPeriodValue());
            }
            Ipv4Address NextHop = CollectPacketRouting(sourcejid, dstjid);
            //标志为2代表是路径中找不到下一跳返回的cp或cpr包，继续往回找即可
            if(cp.GetOriginalityFlag()==2)
            {
                NextHop = CollectPacketRouting(dstjid, sourcejid);
            }
            //如果找不到下一跳，直接return
            if(nextHop==Ipv4Address("127.0.0.1"))
            {
                std::cout<<"由路口"<<sourcejid<<"发往路口"<<dstjid<<"的cp包已经被丢弃"<<std::endl;
                return;
            }
            //遍历邻居表，计算当前车和下一跳车辆之间与cp包同向以及反向车辆的数量
            double NextHopx=GetPosition(NextHop).x;
            double NextHopy=GetPosition(NextHop).y;
            double NextHopDisToJID = sqrt(pow(NextHopx - jx, 2) + pow(NextHopy - jy, 2));
            double curDisToNextHop = sqrt(pow(cx - NextHopx, 2) + pow(cy - NextHopy, 2));
            for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator j = m_neiTable.begin(); j != m_neiTable.end(); j++)
            {
                if (j->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) //second指的是map中的第二个元素
                {
                    continue;
                }
                double tempNex = j->second.N_location_x;
                double tempNey = j->second.N_location_x;
                //初始化邻居预测位置
                double predicted_nx=tempNex;
                double predicted_ny=tempNey;
                //时间差
                double time_ago=now.GetSeconds()-(j->second.N_time.GetSeconds());
                //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                if(j->second.N_direction==0)
                {
                    predicted_nx=tempNex+(j->second.N_speed*time_ago);
                }
                else if(j->second.N_direction==1)
                {
                    predicted_ny=tempNey+(j->second.N_speed*time_ago); 
                }
                else if(j->second.N_direction==2)
                {
                    predicted_nx=tempNex-(j->second.N_speed*time_ago);
                }
                else if(j->second.N_direction==3)
                {
                    predicted_ny=tempNey-(j->second.N_speed*time_ago);
                }
                double tempNeToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));
                if (isBetweenSegment(predicted_nx, predicted_ny, sourcejid, dstjid) == true)
                {
                    if (tempNeToJID >= NextHopDisToJID && tempNeToJID <= mindis) //判断车辆位置是否在当前车和nexthop之间
                    {
                        if (roadDirection == j->second.N_direction)
                        {
                            IVDNummer++;
                        }
                        else if ((roadDirection + 2)%4==j->second.N_direction)
                        {
                            OVDNummer++;
                        }
                    }
                }
            }
        
            //更新cp包的值
            cp.SetCFAddress(m_mainAddress);
            cp.SetNFAddress(NextHop);
            cp.SetNumberOfHops(cp.GetNumberOfHops() +1 );
            cp.SetIVDDensity(IVDNummer);
            cp.SetDVDDensity(OVDNummer);

            //检查车上有无缓存的包
            if (m_pwaitqueue.empty() == false)
            {
                CheckPacketQueue();
            }
            QueueMessage(msg, JITTER); //继续转发cp
            controlnum++;
        }

        /*------------------------------------------------------------------------------------------*/
        //宣布cp包结果
        void
        RoutingProtocol::SendRTNSMResult(struct RTNSMRSEntry e, int sourcejid, int dstjid)
        {
            NS_LOG_FUNCTION(this);

            grp::MessageHeader msg;

            msg.SetVTime(GRP_NEIGHB_HOLD_TIME);
            msg.SetOriginatorAddress(m_mainAddress);
            msg.SetTimeToLive(1);
            msg.SetHopCount(0);
            msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
            grp::MessageHeader::RTNSMResult &RS = msg.GetRTNSMResult();

            //将RTNSM_RS整数和小数部分拆分，小数部分保留四位
            long int RSintpart=(long)e.RTNSM_RS;
            long int RSfractionPart=(e.RTNSM_RS-RSintpart)*10000;
            
            //将将RTNSM_VP整数部分和小数部分拆分，小数部分保留四位
            double uninVP=e.RTNSM_RS_VP;
            int vpintPart=int(uninVP);
            int vpintfractionPart=uninVP*10000;

            //设置RS包的值
            RS.SetFirstJID (sourcejid);
            RS.SetSencondJID (dstjid);
            RS.SetTimeStamp(e.RTNSM_RS_TimeStamp);

            RS.SetRTNSMRS(RSintpart);
            RS.SetRTNSMRSFPart(RSfractionPart);
    
            RS.SetVP(vpintPart);
            RS.SetVPFPart(vpintfractionPart);

            RS.SetRSFlag(e.RTNSM_RS_flag);

            QueueMessage(msg, JITTER);
            controlnum++;
        }

        /*------------------------------------------------------------------------------------------*/
        //RS接收处理过程
        void
        RoutingProtocol::ProcessRTNSMResult(grp::MessageHeader &msg,
                                            const Ipv4Address receiverIfaceAddr,
                                            const Ipv4Address senderIface)
        {
            grp::MessageHeader::RTNSMResult &RS = msg.GetRTNSMResult();

            int updateFlag = 0;
            int sourcejid = RS.GetFirstJID();
            int dstjid = RS.GetSencondJID();
            int curJID = GetNearestJID();
            
            double uninRS=RS.GetRTNSMRS()+double(RS.GetRTNSMRSFPart())/10000;
            double uninVP=RS.GetVP()+double(RS.GetVPFPart())/10000;

            struct RTNSMRSEntry s;
            s.RTNSM_RS_TimeStamp=RS.GetTimeStamp();
            s.RTNSM_RS=uninRS;
            s.RTNSM_RS_VP=uninVP;
            s.RTNSM_RS_flag=RS.GetRSFlag();

            m_RTNSMRS[sourcejid][dstjid]=s;
            m_RTNSMRS[dstjid][sourcejid]=s;
            //更新结果即可，无需转发
        }

        //计算calculateRTNSMResult
        double RoutingProtocol::calculateRTNSMResult(int IVDNummer, int DVDNummer, double cp_travletime, int NumberOfHops, double VPV)
        {
            int Ncon = 12;     //常量
            double RVD = 0.00; //Road vehicular density
            double RNC = 0.00; //Road network connectivity
            double DensityScore = 0.00;
            double DelayHop = 0.00;
            double ProcessedPackets = 0.00;
            double RS = 0.00;
            double factor_1 = 0.5;
            double factor_2 = 0.5; 
            DensityScore = (2 * IVDNummer + DVDNummer) / (3 * NumberOfHops * Ncon);
            if (DensityScore > 1)
            {
                RVD = 1;
            }
            else
            {
                RVD = DensityScore;
            }
            DelayHop = cp_travletime / NumberOfHops;
            ProcessedPackets = VPV / DelayHop;
            RNC = 1/(pow(Natural_Constant_e, 1 / ProcessedPackets));
            RS = factor_1 * RVD + factor_2 * RNC;
            if(RS<0) RS=0;
            return RS;
        }

        /*------------------------------------------------------------------------------------------*/
        //路段内路由算法
        Ipv4Address
        RoutingProtocol::RoadAreaReliableRouting(Ipv4Address dest,int fromjid, int tpjid) 
        {
            Time now = Simulator::Now();
            Ipv4Address nextHop = Ipv4Address("127.0.0.1");
            //由于原论文中算法的目标是车辆,这里使用RSU,即传入的目标地址是RSU的ip地址
            //如果RSU和当前车辆在同一条道路上,那么目标点tp就是RSU
            //如果RSU和当前车辆不在同一条道路上,那么目标点tp就是距离RSU最近的路口的中央,即在车辆的cjid和nextjid中寻找距离RSU最近的jid
            
            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;

            //dest是RSU地址
            double dx = GetPosition(dest).x; 
            double dy = GetPosition(dest).y;
            double curDisToDst = sqrt(pow(cx - dx, 2) + pow(cy - dy, 2));
            //如果可以直接传到dest，返回dest地址即可
            if (curDisToDst < RSSIDistanceThreshold) //在RSSI距离阈值内
            {
                std::cout<<Simulator::Now().GetSeconds()<<"已经进入目标范围内"<<std::endl;
                return dest;
            }

            std::cout<<Simulator::Now().GetSeconds()<<"已经进入路段内路由算法"<<std::endl;
            std::cout<<Simulator::Now().GetSeconds()<<"来自路口"<<fromjid<<"走向路口"<<tpjid<<std::endl;
            double curJx = m_map[m_currentJID].x;
            double curJy = m_map[m_currentJID].y;

            double curJTODst = sqrt(pow(curJx - dx, 2) + pow(curJy - dy, 2));
            double nextJx = m_map[m_nextJID].x;
            double nextJy = m_map[m_nextJID].y;

            double nextJTODst = sqrt(pow(nextJx - dx, 2) + pow(nextJy - dy, 2));

            double jx = m_map[tpjid].x; 
            double jy = m_map[tpjid].y;

            
            double mindis = sqrt(pow(cx - jx, 2) + pow(cy - jy, 2));
            //case1 
            //在邻居表中寻找和当前车辆在同一路段内的可用邻居
            for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator i = m_neiTable.begin(); i != m_neiTable.end(); i++)
            {
                if (i->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) 
                {
                    continue;
                }
                std::cout<<Simulator::Now().GetSeconds()<<"候选邻居ID"<<AddrToID(i->first)<<std::endl;

                double nx = i->second.N_location_x;
                double ny = i->second.N_location_y;
                //初始化邻居预测位置
                double predicted_nx=nx;
                double predicted_ny=ny;
                //时间差
                double time_ago=now.GetSeconds()-(i->second.N_time.GetSeconds());
                //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                if(i->second.N_direction==0)
                {
                    predicted_nx=nx+(i->second.N_speed*time_ago);
                }
                else if(i->second.N_direction==1)
                {
                    predicted_ny=ny+(i->second.N_speed*time_ago); 
                }
                else if(i->second.N_direction==2)
                {
                    predicted_nx=nx-(i->second.N_speed*time_ago);
                }
                else if(i->second.N_direction==3)
                {
                    predicted_ny=ny-(i->second.N_speed*time_ago);
                }
                //调试时可以调整为预测位置
                double neiDisToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));    //邻居的位置需要比当前车辆更靠近tp
                double curDisToNei = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2));    //邻居应该在当前车辆的覆盖范围内
                if (neiDisToJID < mindis && curDisToNei < RSSIDistanceThreshold) //邻居到tp路口的位置小于当前车到tp路口的位置,且当前节点到邻居的距离小于RSSI阈值
                {
                    //获得距离当前车辆最近的路口ID
                    //此处的条件判断用以防止当前车辆将数据包传输给其他路段的节点,
                    //其他路段的节点同样有可能满足上一个条件判断
                    if (isBetweenSegment(predicted_nx, predicted_ny, fromjid, tpjid) == true) //车辆不在路口范围内或者车辆在两个指定的路口形成的矩形区域内
                    {
                        //选择距离tp最近的同一条道路上的邻居作为下一跳
                        mindis = neiDisToJID; //令mindis=邻居到路口的距离
                        nextHop = i->first;   //邻居地址即为下一节点的address
                    }
                }
            }
            //case 2 
            //在邻居表中没有找到和当前车辆在同一路段内的可用邻居，找在一个信标间隔内更新过信息的邻居
            if (nextHop == Ipv4Address("127.0.0.1")) //说明没有在相同道路上找到邻居,nexthop值没有更新,说明当前车辆可能比较靠近fromjid区域，那么在PFG集合上寻找邻居，即可以是不同道路中的邻居
            {
                //case2 1
                for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator j = m_neiTable.begin(); j != m_neiTable.end(); j++)
                {
                    if (j->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) 
                    {
                        continue;
                    }
                    if (now.GetSeconds() - (j->second.N_time.GetSeconds()) <= GRP_REFRESH_INTERVAL) //邻居信息小于一个信标时间间隔
                    {
                        double nex = j->second.N_location_x;
                        double ney = j->second.N_location_y;
                        //初始化邻居预测位置
                        double predicted_nx=nex;
                        double predicted_ny=ney;
                        //时间差
                        double time_ago=now.GetSeconds()-(j->second.N_time.GetSeconds());
                        //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                        if(j->second.N_direction==0)
                        {
                            predicted_nx=nex+(j->second.N_speed*time_ago);
                        }
                        else if(j->second.N_direction==1)
                        {
                            predicted_ny=ney+(j->second.N_speed*time_ago); 
                        }
                        else if(j->second.N_direction==2)
                        {
                            predicted_nx=nex-(j->second.N_speed*time_ago);
                        }
                        else if(j->second.N_direction==3)
                        {
                            predicted_ny=ney-(j->second.N_speed*time_ago);
                        }
                        double neDisToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));    //邻居的位置需要比当前车辆更靠近tp
                        double curDisToNe = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2));    //邻居应该在当前车辆的覆盖范围内
                        if(neDisToJID < mindis && curDisToNe < RSSIDistanceThreshold)
                        {
                            mindis=neDisToJID;
                            nextHop = j->first;
                            //应该考虑改变fromjid，改换路段？                           
                        }
                        //nextHop = j->first; //如果存在这样的邻居,就将其选为下一跳
                    }
                }
                //case2 2
                //在上述过程中,仍然选不到,那么就选两个信标间隔内更新过信息的邻居
                if (nextHop == Ipv4Address("127.0.0.1"))
                {
                    for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator k = m_neiTable.begin(); k != m_neiTable.end(); k++)
                    {
                        if (k->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) //second指的是map中的第二个元素
                        {
                            continue;
                        }
                        if (now.GetSeconds() - (k->second.N_time.GetSeconds()) <= 2 * GRP_REFRESH_INTERVAL) //邻居信息小于两个信标时间间隔
                        {
                            double nex = k->second.N_location_x;
                            double ney = k->second.N_location_y;
                            //初始化邻居预测位置
                            double predicted_nx=nex;
                            double predicted_ny=ney;
                            //时间差
                            double time_ago=now.GetSeconds()-(k->second.N_time.GetSeconds());
                            //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                            if(k->second.N_direction==0)
                            {
                                predicted_nx=nex+(k->second.N_speed*time_ago);
                            }
                            else if(k->second.N_direction==1)
                            {
                                predicted_ny=ney+(k->second.N_speed*time_ago); 
                            }
                            else if(k->second.N_direction==2)
                            {
                                predicted_nx=nex-(k->second.N_speed*time_ago);
                            }
                            else if(k->second.N_direction==3)
                            {
                                predicted_ny=ney-(k->second.N_speed*time_ago);
                            }
                            double neDisToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));    //邻居的位置需要比当前车辆更靠近tp
                            double curDisToNe = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2));    //邻居应该在当前车辆的覆盖范围内
                            if(neDisToJID < mindis && curDisToNe < RSSIDistanceThreshold)
                            {
                                mindis=neDisToJID;
                                nextHop = k->first;
                            }
                        }
                    }
                }
            }//end case2

            //case3
            //在上述过程中仍旧未找到的下一跳
            //检查PFG集合并选择选择距离tp最近的邻居作为下一跳
            if(nextHop == Ipv4Address("127.0.0.1"))
            {
                for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator s = m_neiTable.begin(); s != m_neiTable.end(); s++)
                {
                    if (s->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) //second指的是map中的第二个元素
                    {
                        continue;
                    }
                    double nex = s->second.N_location_x;
                    double ney = s->second.N_location_y;
                    //初始化邻居预测位置
                    double predicted_nx=nex;
                    double predicted_ny=ney;
                    //时间差
                    double time_ago=now.GetSeconds()-(s->second.N_time.GetSeconds());
                    //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                    if(s->second.N_direction==0)
                    {
                        predicted_nx=nex+(s->second.N_speed*time_ago);
                    }
                    else if(s->second.N_direction==1)
                    {
                        predicted_ny=ney+(s->second.N_speed*time_ago); 
                    }
                    else if(s->second.N_direction==2)
                    {
                        predicted_nx=nex-(s->second.N_speed*time_ago);
                    }
                    else if(s->second.N_direction==3)
                    {
                        predicted_ny=ney-(s->second.N_speed*time_ago);
                    }
                    double neDisToJID = sqrt(pow(predicted_nx - jx, 2) + pow(predicted_ny - jy, 2));    //邻居的位置需要比当前车辆更靠近tp
                    double curDisToNe = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2)); 
                    if(neDisToJID<mindis&&curDisToNe < RSSIDistanceThreshold)
                    {
                        mindis=neDisToJID;
                        nextHop = s->first;

                    }
                }

            }//end case3


            std::cout<<Simulator::Now().GetSeconds()<<"下一跳"<<AddrToID(nextHop)<<std::endl;

            return nextHop; //返回下一跳地址
        }

        /*------------------------------------------------------------------------------------------*/
        //路段间路由算法
        Ipv4Address
        RoutingProtocol::IntersectionAreaReliableRouting(Ipv4Address dest,int fromjid,int sendcpflag)
        {
            //此函数在判断持有数据包的车辆已经进入路口范围内时调用,进入此函数,说明车辆已经进入路口范围内
            Time now = Simulator::Now();
            Ipv4Address nextHop = Ipv4Address("127.0.0.1");
            std::cout<<Simulator::Now().GetSeconds()<<"已经进入路段间路由函数"<<std::endl;
            std::cout<<Simulator::Now().GetSeconds()<<"进入路段间路由函数后的fromjid"<<fromjid<<std::endl;


            Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel>();
            double cx = MM->GetPosition().x;
            double cy = MM->GetPosition().y;
            

            //dest是RSU地址
            double destx = GetPosition(dest).x; 
            double desty = GetPosition(dest).y;
            double curDisToDst = sqrt(pow(cx - destx, 2) + pow(cy - desty, 2));
            //如果可以直接传到dest，返回dest地址即可
            if (curDisToDst < RSSIDistanceThreshold) 
            {
                    return dest;
            }
            
            int cjid = GetNearestJID();//获得当前路口的id，即当前的中心路口，上一路段的dstjid
            double ctx=m_map[cjid].x;
            double cty=m_map[cjid].y;
            double mindis=sqrt(pow(cx -ctx, 2) + pow(cy - cty, 2));
            double RS[m_JuncNum]; //用于存储与cjid相连路口的评估结果RS
            double RoadScore[m_JuncNum];
            int AR[4];//存储与当前路口相邻的路口的路口ID
            int CanSelectNextJIDFlag=1;//默认为1，可以进行选择
            for (int in = 0; in < m_JuncNum; in++) //初始化
            {
                RS[in] = -1;
                RoadScore[in] = -1;
            }
            //初始化AR数组
            for (int d=0;d<4;d++)
            {
                AR[d]=-1;
            }
            

            //判断路口连通性
            for (int i = 0; i < m_JuncNum; i++)
            {
                for (int j = i + 1; j < m_JuncNum; j++)
                {
                    if (isAdjacentVex(i, j) == false)
                    {
                        Graph[i][j] = Graph[j][i] = INF;
                    }
                    else
                    {
                        Graph[i][j] = Graph[j][i] = 1;
                    }
                }
            }

            //遍历Graph,对于cjid相邻的路口进行评估,并记录结果
            for (int k = 0; k < m_JuncNum; k++)
            {
                //if(k==fromjid)
                //{
                    //continue;//不考虑来时的路口
                //}
                int p=0;
                if (Graph[cjid][k] == 1 && Graph[k][cjid] == 1) //路口连通
                {
                    //首先检查车上有没有可用的道路评估结果,如果没有可用的结果，发送cp包，获得结果
                    if(m_RTNSMRS[cjid][k].RTNSM_RS!=-1&&(m_RTNSMRS[cjid][k].RTNSM_RS_TimeStamp.GetSeconds()+m_RTNSMRS[cjid][k].RTNSM_RS_VP)>now.GetSeconds())
                    {
                        RS[k]=m_RTNSMRS[cjid][k].RTNSM_RS;
                        if(RS[k]!=0)//rs结果为0时，该路口不可用（不连通）
                        {
                            AR[p]=k;
                            p++;
                            std::cout<<Simulator::Now().GetSeconds()<<"进入过已有结果分支"<<std::endl;
                            std::cout<<Simulator::Now().GetSeconds()<<"现有结果"<<m_RTNSMRS[cjid][k].RTNSM_RS<<std::endl;
                            std::cout<<Simulator::Now().GetSeconds()<<"现有结果时间戳"<<m_RTNSMRS[cjid][k].RTNSM_RS_TimeStamp.GetSeconds()<<std::endl;
                            std::cout<<Simulator::Now().GetSeconds()<<"now时间戳"<<now.GetSeconds()<<std::endl;
                            std::cout<<Simulator::Now().GetSeconds()<<"现有结果VP"<<m_RTNSMRS[cjid][k].RTNSM_RS_VP<<std::endl;
                            std::cout<<Simulator::Now().GetSeconds()<<"分支结束"<<std::endl;
                        }
                        

                    }
                    else
                    {
                        if(sendcpflag==1)//如果是缓冲队列进来的，则不在发送控制包，如果是第一次进来，则发送控制包
                        {
                            SendCollectotPaket(cjid, k, 0);
                            std::cout<<Simulator::Now().GetSeconds()<<"向道路"<<k<<"发送控制包"<<std::endl;
                        }
                        //std::cout<<"现有结果"<<m_RTNSMRS[cjid][k].RTNSM_RS<<std::endl;
                        //std::cout<<"现有结果时间戳"<<m_RTNSMRS[cjid][k].RTNSM_RS_TimeStamp.GetSeconds()<<std::endl;
                        //std::cout<<"现有结果VP"<<m_RTNSMRS[cjid][k].RTNSM_RS_VP<<std::endl;
                        //std::cout<<"now时间戳"<<now.GetSeconds()<<std::endl;
                        
                        CanSelectNextJIDFlag=0;
                        continue;
                    }
                }
            }
            //不能得到全部路口评估结果
            //返回nexthop为默认地址

            if(CanSelectNextJIDFlag==0)
            {
                return nextHop;
            }
            
            //已经遍历完全部相邻路口，能够得到全部相邻路口的结果
            //计算路口得分并排序
            //不需要再发cp包
            //路口ID已经存在AR中
            std::cout<<Simulator::Now().GetSeconds()<<"路口数组为"<<AR[0]<<"->"<<AR[1]<<"->"<<AR[2]<<"->"<<AR[3]<<std::endl;
            double dx = GetPosition(dest).x; 
            double dy = GetPosition(dest).y;
            double curjx = m_map[cjid].x;
            double curjy = m_map[cjid].y;
            double curJToDST = sqrt(pow(curjx - dx, 2) + pow(curjy - dy, 2));
            //根据道路得分寻找nexthop
            if(CanSelectNextJIDFlag==1)
            {
                for(int j=0;j<4;j++)
                {
                    if(AR[j]==-1)
                    {
                        continue;
                    }
                    double PD = -1;
                    double canjx = m_map[AR[j]].x;
                    double canjy = m_map[AR[j]].y;
                    double canJToDST = sqrt(pow(canjx - dx, 2) + pow(canjy - dy, 2));
                    PD = 1.0 - canJToDST / curJToDST;
                    if(RS[AR[j]]==0) continue;//如果lre评估结果为0，代表结果无效或者没有收到过结果
                    RoadScore[AR[j]] = 0.5 * RS[AR[j]] + 0.5 * PD;
                    std::cout<<Simulator::Now().GetSeconds()<<"-已经计算道路得分："<<"路口"<<AR[j]<<"得分"<<RoadScore[AR[j]]<<std::endl;
                }//end 计算得分
                //将道路得分进行排序
                int sort[4];//建立排序后的数组
                for (int si = 0; si < 4; si++)//数组中存路口id
                {
                    sort[si] = -1;
                }

                for(int m=0;m<4;m++)
                {
                    if(AR[m]==-1)
                    {
                        std::cout<<Simulator::Now().GetSeconds()<<"无效路口"<<AR[m]<<std::endl;
                        continue;
                    }
                    std::cout<<Simulator::Now().GetSeconds()<<"有效路口"<<AR[m]<<std::endl;
                    if(RoadScore[AR[m]]!=-1)
                    {
                        std::cout<<Simulator::Now().GetSeconds()<<"路口"<<AR[m]<<"得分为"<<RoadScore[AR[m]]<<std::endl;
                        for(int s=0;s<4;s++)
                        {
                            if(sort[s]==-1)//sort中没有存数
                            {
                                sort[s]=AR[m];
                                std::cout<<Simulator::Now().GetSeconds()<<"sort数组已经存入数据"<<s<<"位置"<<sort[s]<<std::endl;
                                break;
                            }
                            else if(RoadScore[AR[m]]>RoadScore[sort[s]])//新数比已经存入的数大，那么已存入的数后移一个单元
                            {
                                for(int t=3;t>s;t++)
                                {
                                    sort[t]=sort[t-1];
                                }
                                sort[s]=AR[m];
                                std::cout<<Simulator::Now().GetSeconds()<<"sort数组已经存入数据"<<s<<"位置"<<sort[s]<<std::endl;
                                break;
                            }
                            //std::cout<<Simulator::Now().GetSeconds()<<"sort数组已经存入数据"<<sort[s]<<std::endl;

                        }
                        
                    }


                }//end 排序
                std::cout<<Simulator::Now().GetSeconds()<<"已经按道路得分排序"<<std::endl;
                std::cout<<Simulator::Now().GetSeconds()<<"排序数组为"<<sort[0]<<"->"<<sort[1]<<"->"<<sort[2]<<"->"<<sort[3]<<std::endl;

                //遍历sort数组，寻找下一个路口及nextHop
                for(int ff=0;ff<4;ff++)
                {
                    if(sort[ff]==-1)
                    {
                        continue;
                    }
                    //在不为0的sort中寻找结果
                    std::cout<<Simulator::Now().GetSeconds()<<"在路口"<<cjid<<"到路口"<<sort[ff]<<"之间寻找下一跳"<<std::endl;
                    nextHop = RoadAreaReliableRouting(dest,cjid,sort[ff]);
                    if(nextHop != Ipv4Address("127.0.0.1"))
                    {
                        m_tempjid=sort[ff];
                        std::cout<<Simulator::Now().GetSeconds()<<"已经成功找到下一跳"<<AddrToID(nextHop)<<std::endl;
                        break;//能够找到nexthop就停止
                    }

                }
            }//end 根据道路得分寻找nexthop
            //如果上述流程中没有找到nexthop，那么在路段内选择一个更接近当前路口的车辆传递
            //选择之前，fromjid为上一个路口，cjid为当前路口，或者cjid
            //
            if(CanSelectNextJIDFlag==1&&nextHop == Ipv4Address("127.0.0.1"))
            {
                for (std::map<Ipv4Address, NeighborTableEntry>::const_iterator s = m_neiTable.begin(); s != m_neiTable.end(); s++)
                {
                    if (s->second.N_status == NeighborTableEntry::STATUS_NOT_SYM) //second指的是map中的第二个元素
                    {
                        continue;
                    }
                    double nx = s->second.N_location_x;
                    double ny = s->second.N_location_y;
                    //初始化邻居预测位置
                    double predicted_nx=nx;
                    double predicted_ny=ny;
                    //时间差
                    double time_ago=now.GetSeconds()-(s->second.N_time.GetSeconds());
                    //计算邻居预测位置,方向值0,1,2,3分别对应东北西南
                    if(s->second.N_direction==0)
                    {
                        predicted_nx=nx+(s->second.N_speed*time_ago);
                    }
                    else if(s->second.N_direction==1)
                    {
                        predicted_ny=ny+(s->second.N_speed*time_ago); 
                    }
                    else if(s->second.N_direction==2)
                    {
                        predicted_nx=nx-(s->second.N_speed*time_ago);
                    }
                    else if(s->second.N_direction==3)
                    {
                        predicted_ny=ny-(s->second.N_speed*time_ago);
                    }
                    double neDisToCTJID = sqrt(pow(predicted_nx - ctx, 2) + pow(predicted_ny - cty, 2));
                    double curDisToNe = sqrt(pow(cx - predicted_nx, 2) + pow(cy - predicted_ny, 2));
                    if(neDisToCTJID<mindis&&curDisToNe<RSSIDistanceThreshold)
                    {
                        if(isBetweenSegment(predicted_nx,predicted_ny,fromjid,cjid))
                        {
                                mindis=neDisToCTJID;
                                nextHop=s->first;
                        }

                    }

                }//end for
                //如果能够这样找到下一跳,下一跳一定还在路口
                //fromjid和dstjid都无需改变,将 m_tempjid2还原为fromjid
                if(nextHop != Ipv4Address("127.0.0.1"))
                {
                    m_tempjid2=fromjid;
                }
            }

            return nextHop;
        }//end IntersectionAreaReliableRouting
            
            

        //返回车上的有效道路评估结果
        double
        RoutingProtocol::GetRS(Time now, int cjid, int k)
        {
            double RS = -1;
            for (std::map<int, RTNSMResultEntry>::const_iterator i = m_RTNSMResult.begin(); i != m_RTNSMResult.end(); i++)
            {
                if ((i->first == cjid && i->second.sencondJID == k) || (i->first == k && i->second.sencondJID == cjid)) //存在相同道路的评估结果
                {
                    if (now.GetSeconds() < i->second.RTNSMResultTimeStamp) //理论上,返回的结果的时间应该在cp包发送时间之后
                    {
                        RS = i->second.RTNSMResult;
                    }
                }
            }
            return RS;
        }//end GetRS

        void
        RoutingProtocol::RSExpire( )
        {


        }
        
    } // end namespace grp
} // end namespace ns3