/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef GRP_H
#define GRP_H

#include "grp-header.h"
#include "ns3/object.h"//加入ns3/object.h的引用,若不特殊加载,则类继承时则是c++中默认的object类
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/mobility-model.h"
#include <vector>
#include <map>
#include <queue>
#include "ns3/ip-l4-protocol.h"
#include "ns3/digitalMap.h"
#include "ns3/myserver.h"

extern long int controlnum;
namespace ns3 {

struct PacketQueueEntry
{
 	typedef Ipv4RoutingProtocol::UnicastForwardCallback UnicastForwardCallback;//单播转发回叫

  	Ptr<Packet> m_packet;
 	Ipv4Header m_header;
 	UnicastForwardCallback m_ucb;//单播转发入口

  	PacketQueueEntry(Ptr<Packet> p, Ipv4Header h, UnicastForwardCallback u)
 	{
 		this->m_packet = p;//this用来将p赋值给m_packet,相当于初始化
 		this->m_header = h;
 		this->m_ucb = u;
 	}

};

struct DelayPacketQueueEntry//延迟包队列,和包队列相比,多了下一跳的地址
{
 	typedef Ipv4RoutingProtocol::UnicastForwardCallback UnicastForwardCallback;

  	Ptr<Packet> m_packet;
 	Ipv4Header m_header;
 	UnicastForwardCallback m_ucb;
    Ipv4Address m_nexthop;

  	DelayPacketQueueEntry(Ptr<Packet> p, Ipv4Header h, UnicastForwardCallback u, Ipv4Address nexthop)
 	{
 		this->m_packet = p;
 		this->m_header = h;
 		this->m_ucb = u;
        this->m_nexthop = nexthop;
 	}

};

struct QPacketInfo
{
 	Ipv4Address src;
 	Ipv4Address dst;

  	QPacketInfo(Ipv4Address src, Ipv4Address dst)
 	{
 		this->src = src;
 		this->dst = dst;
 	}

  	QPacketInfo(const QPacketInfo& info)
 	{
 		*this = info;
 	}

  	bool operator< (const QPacketInfo& info) const
 	{
 		if(this->src < info.src)
 			return true;
 		else if(this->src == info.src)
 		{
 			if(this->dst < info.dst)
 				return true;
 			else
 				return false;
 		}
 		else
 			return false;
 	}

};

typedef std::map<QPacketInfo, Time> QMap;

struct SendingQueue//发送队列
{
 	typedef Ipv4RoutingProtocol::UnicastForwardCallback UnicastForwardCallback;//单播回调

  	Ptr<Packet> m_packet;
 	Ipv4Header m_header;
 	UnicastForwardCallback m_ucb;
 	Ipv4Address nexthop;

  	SendingQueue(Ptr<Packet> p, Ipv4Header h, UnicastForwardCallback u, Ipv4Address nexthop)
 	{
 		this->m_packet = p;
 		this->m_header = h;
 		this->m_ucb = u;
 		this->nexthop = nexthop;
    }

};
//collect
namespace grp {

struct NeighborTableEntry//邻居表项
{
    int N_turn;
    int N_direction;//方向
    Time N_time;
    double N_speed;//速度
    double N_location_x;//地图坐标x
    double N_location_y;//地图坐标y
    uint16_t N_sequenceNum;//字段含义？
    Ipv4Address receiverIfaceAddr;//字段含义？
    Ipv4Address N_neighbor_address;//邻居地址？

    enum Status//枚举类型
    {
        STATUS_NOT_SYM = 0,
        STATUS_SYM = 1,
    } N_status;

    NeighborTableEntry () :
        N_turn (-1), N_speed (0), N_location_x (0), N_location_y (0),
        N_sequenceNum (0),N_neighbor_address (), N_status (STATUS_NOT_SYM)
    {

    }
};

struct RTNSMResultEntry//存放RTNSMResult表项
{
    //int firstJID;
    int sencondJID;
    double RTNSMResultTimeStamp;//RTNSM结果生成的时间戳
    double  RTNSMResult;//存放结果
    //结果是cpr还是cp产生的
    //定义一个全局的结果序列号？或者是用firstjid指向sencondjid作为唯一标志？应该用secondjid作为标志,有新的则覆盖
    //检测车上的结果表项,如果有合适的则直接使用
    //如果没有生成cp包
    //根据时间戳来决定是否再产生cpr
    RTNSMResultEntry():
        sencondJID(-1),RTNSMResultTimeStamp(),RTNSMResult()
    {

    }
  
};
//RTNSM结果结构体，存放结果和有效期
struct RTNSMRSEntry
{
    double  RTNSM_RS;//道路评估结果
    Time RTNSM_RS_TimeStamp;//道路评估结果时间戳
    double RTNSM_RS_VP;//道路评估结果有效期
    int RTNSM_RS_flag;//道路评估结果来源，是由cp还是cpr产生，如果为1，说明由cp产生，如果为2，说明由cpr产生

    RTNSMRSEntry():
        RTNSM_RS(-1),RTNSM_RS_TimeStamp(),RTNSM_RS_VP(-1),RTNSM_RS_flag(0)
    {

    }
};

class RoutingProtocol : public Ipv4RoutingProtocol//继承关系,RoutingProtocol以public方式继承于Ipv4RoutingProtocol
{
public:
    static TypeId GetTypeId (void);//static,静态变量

    RoutingProtocol ();
    virtual ~RoutingProtocol ();

    int64_t AssignStreams (int64_t stream);//long int,分配流,使用随机变量流
    void SetMainInterface (uint32_t interface);//unsigned int ,设置ipv4地址
    void SetDownTarget (IpL4Protocol::DownTargetCallback callback);//设置数据包的回调？
    void AddHeader(Ptr<Packet> p,Ipv4Address source, Ipv4Address destination, uint8_t protocol, Ptr<Ipv4Route> route);//uint8_t是unsigned char

    typedef void (* m_DropPacketTraceCallback)(const Ipv4Header &header);
    typedef void (* m_StorePacketTraceCallback)(const Ipv4Header &header);

protected:
    virtual void DoInitialize (void);  

private:
    Ptr<Ipv4> m_ipv4;
    Ipv4Address m_mainAddress;//当前车的ipv4地址

    Time m_helloInterval;//hello信标发送间隔
    Timer m_helloTimer;//hello包定时
    Timer m_positionCheckTimer;//位置检查定时
    Timer m_queuedMessagesTimer;//队列msg定时
    Timer m_speedTimer;//速度定时？
    Timer m_RSTimer;//RS结果循环检查定时

    grp::MessageList m_queuedMessages;//信息队列

    uint16_t m_packetSequenceNumber;//包序列号
    uint16_t m_messageSequenceNumber;//msg序列号

    IpL4Protocol::DownTargetCallback m_downTarget;//？
    Ptr<UniformRandomVariable> m_uniformRandomVariable;//均匀随机变量
    std::map<Ipv4Address, NeighborTableEntry> m_neiTable;//ipv4地址和邻居表项
    std::map<int,RTNSMResultEntry>m_RTNSMResult;//用以在小车上存放RTNSMResult,其中int格式的firstjid指的是,小车位于的路口定义为secondid,firstjid则指向的是另一路口
    

    Ptr<Socket> m_recvSocket;//接收套接字
    std::map<Ptr<Socket>, Ipv4InterfaceAddress > m_sendSockets;//Ipv4InterfaceAddress,在接口上存储ipv4地址的类
    std::map<Ptr<Socket>, Ipv4InterfaceAddress > m_sendBlockSockets;
    

    TracedCallback <const Ipv4Header &> m_DropPacketTrace;//TracedCallback类,将呼叫转发到Callback链
    TracedCallback <const Ipv4Header &> m_StorePacketTrace;


    inline uint16_t GetPacketSequenceNumber ();//计算包序列号
    inline uint16_t GetMessageSequenceNumber ();//计算msg序列号

/*------------------------------------------------------------------------------------------*/
    //以下参数需要根据实际运行情况调整
    bool m_jqueuetag[49];//路口tag
    int m_JuncNum=49;//路口数量
    int m_rsujid = 45;//RSU路口的ID
    double startTime = 5;//开始时间
    double OutsightTransRange = 100;//超视距范围
    double RoadLength = 500;
    double turnLightRange = 50;//转向范围？
    double PositionCheckThreshold = 11;//位置检查阈值
    double RoadWidth = 10;//路宽
    double JunAreaRadius = 50;//范围半径
    std::string confile = "scratch/conf.txt";//配置文件
/*------------------------------------------------------------------------------------------*/


    int m_id = -1;
    int vnum = 0;
    int m_turn = -1;
    int m_nextJID = -1;
    int m_currentJID = -1;
    int m_lastJID=-1;
    int m_direction = -1;
    bool m_JunAreaTag = false;//车辆位于路口范围内标记
    //bool m_lastJunAreaTag=false;
    float ** Graph;//二级指针
    double m_speed = 1;
    double RSSIDistanceThreshold = 0;//RSSI距离阈值
    double m_last_x = 0, m_last_y = 0;
    double InsightTransRange = -1;//视距内范围
    double CarryTimeThreshold = -1;//进位时间阈值
    int m_tempjid;//用来记录next
    int m_tempjid2;//用来记录from
    //当前车对道路的评估结果
    struct RTNSMRSEntry m_RTNSMRS[49][49];
    //需要初始化这个二维数组
    int m_CPSendFlag=0;//当前车已经发送过cp包
    Time m_CPSendFlagTimeStamp;//上一个cp包发送时间戳

    //std::map<Time,double> m_RTNSMResult;//存放当前车的RTNSM结果
    
    QMap m_wTimeCache;//时间缓存？
    std::queue<int> m_jqueue;//empty queue
    std::queue<int> m_trailTrace;//痕迹？
    std::vector<VTrace> m_tracelist;//轨迹列表（轨迹上的路口序列？）
    std::vector<SendingQueue> m_squeue;//发送队列
    std::vector<PacketQueueEntry> m_pqueue;	//包队列	
    std::vector<PacketQueueEntry> m_pwaitqueue;	//等待包队列
    std::vector<DelayPacketQueueEntry> m_delayqueue;//延迟包队列	
    std::vector<DigitalMapEntry> m_map;//地图

/*------------------------------------------------------------------------------------------*/
    //从配置文件读取实验运行参数
    void ReadConfiguration();

/*------------------------------------------------------------------------------------------*/
    //根据车辆的IPv4地址获取其对应的车辆编号m_id
    //一个自定义的计算公式？
    int AddrToID(Ipv4Address);

/*------------------------------------------------------------------------------------------*/
    //初始化车辆的编号m_id和起始位置
    void InitialRTNSMRS();
    void InitialMID();//根据AddrToID
    void InitialPosition();

/*------------------------------------------------------------------------------------------*/
    //获取与当前车辆距离最近的路口的路口编号
    int GetNearestJID();//JID是路口编号
    //获取车辆当前的方向信息,方向值0,1,2,3分别对应东北西南
    int GetDirection(int currentJID, int nextJID);
    //获取车辆的坐标信息
    Vector GetPosition(Ipv4Address adr); 

/*------------------------------------------------------------------------------------------*/
//设置协议所需的定时器
    //用以定时发送Beacon信标
    void HelloTimerExpire ();
    //用以周期性计算当前车辆的实时速度
    void SpeedCheckExpire();
    //用以判断车辆当前在路网中的状态,是位于路段中还是位于路口范围内
    void CheckPositionExpire();
    //用以定期清理邻居表中的过期信息
    void NeiTableCheckExpire(Ipv4Address addr);
    //用以定时检查RS结果
    void RSExpire ();
    //用以定时检查车上是否已经发送cp包，cp包结果是否已经回传
    //void CPExpire ();

/*------------------------------------------------------------------------------------------*/
//RTNSM方法实现,分为Enhanced Validity Period Calculation (EVPC)和Restricted Collector Packet Reply (RCPR) 
    //Enhanced Validity Period Calculation (EVPC)
    double EnhancedValidityPeriodCalculation(Ipv4Address addr);
   // void ProcessCollectPacket (const grp::CollectPacketHeader &cp);
    //Restricted Collector Packet Reply (RCPR)
    int RestrictedCollectorPacketReply(Ipv4Address addr);

/*------------------------------------------------------------------------------------------*/
//路由方法实现,分为路段间路由和路段内路由
    //路段间路由,为数据包挑选合适的下一个传输路段
    int GetPacketNextJID(bool tag);
    Ipv4Address IntersectionAreaReliableRouting(Ipv4Address dest,int fromjid,int sendcpflag);//路段间路由,选择下一个路口的转发节点
    //路段内路由,数据包在路段内传播时的路由方法,即如何再路段内挑选数据包的下一跳  
    Ipv4Address IntraPathRouting(Ipv4Address dest, int dstjid);
    Ipv4Address RoadAreaReliableRouting(Ipv4Address dest,int fromjid, int tpjid);

    //路段间路由采用迪杰斯特拉最短路径算法计算最优的下一路由路段
    int DijkstraAlgorithm(int srcjid, int dstjid);
    //用以在迪杰斯特拉算法中确定某两个路口是否是相邻
    bool isAdjacentVex(int sjid, int ejid);
    //用以确认邻居车辆是否位于两个指定路口所形成的矩形区域内
    bool isBetweenSegment(double nx, double ny, int cjid, int djid);

/*------------------------------------------------------------------------------------------*/
    //收到控制包时的处理逻辑,控制包包括HelloMessage,即Beacon
    void RecvGrp (Ptr<Socket> socket);

    //Beacon机制处理逻辑
    void SendHello ();
    void ProcessHello (grp::MessageHeader &msg, const Ipv4Address receiverIfaceAddr, const Ipv4Address senderIface);
    
    //Beacon机制的具体发送机制
    //广播发送
    void SendPacket (Ptr<Packet> packet);
    void QueueMessage (const grp::MessageHeader &message, Time delay);
    void SendQueuedMessages ();

    //cp包发送及处理逻辑
    void SendCollectotPaket (int sourcejid,int djid,int OriginalityFlag);
    void ProcessCollectPacket (grp::MessageHeader &msg,const Ipv4Address receiverIfaceAddr,const Ipv4Address senderIface);
    double calculatedVPV(Ipv4Address m_mainAddress,int sourcejid,int dstjid,Time now);//vp计算函数

    //cp包路段内路由
    Ipv4Address CollectPacketRouting(int cjid,  int dstjid);

    //cp包结果宣布机制
    void SendRTNSMResult (struct RTNSMRSEntry e,int sourcejid,int dstjid );
    void ProcessRTNSMResult (grp::MessageHeader &msg,const Ipv4Address receiverIfaceAddr,const Ipv4Address senderIface);
    
    //计算RTNSMResult
    double calculateRTNSMResult(int IVDNummer,int DVDNummer,double cp_travletime,int NumberOfHops,double VPV);
    double GetRS(Time now, int cjid, int k);



/*------------------------------------------------------------------------------------------*/
//数据包的暂缓发送机制
    //在Carry-And-Forward机制中用以检查车辆是否携带有被暂存的数据包
    void CheckPacketQueue();
    //在Carry-And-Forward机制中用以发送车辆携带的数据包
    void SendFromSQueue();
    //当发现数据包回传时,即可能出现环时,暂缓一段时间再发送,避免TTL快速消耗
    void SendFromDelayQueue();

/*------------------------------------------------------------------------------------------*/
// From Ipv4RoutingProtocol
    //节点本身有数据包要发送时,使用RouteOutput对数据包进行路由
    virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                                        const Ipv4Header &header,
                                        Ptr<NetDevice> oif,
                                        Socket::SocketErrno &sockerr);
    //节点收到一个来自其他节点的数据包时,使用RouteInput对数据包进行路由
    virtual bool RouteInput (Ptr<const Packet> p,
                            const Ipv4Header &header,
                            Ptr<const NetDevice> idev,
                            UnicastForwardCallback ucb,
                            MulticastForwardCallback mcb,
                            LocalDeliverCallback lcb,
                            ErrorCallback ecb);

    virtual void NotifyInterfaceUp (uint32_t interface);//向上通知接口
    virtual void NotifyInterfaceDown (uint32_t interface);//向下通知接口
    virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);//添加通知地址
    virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);//移除通知地址
    virtual void SetIpv4 (Ptr<Ipv4> ipv4);
    virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;//输出路由表

/*------------------------------------------------------------------------------------------*/
    //程序运行结束后务必回收垃圾
    void DoDispose ();

};

}
}

#endif /* GRP_H */

