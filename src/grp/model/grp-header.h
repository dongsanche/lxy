/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef GRP_HEADER_H
#define GRP_HEADER_H

#include <stdint.h>
#include <vector>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"

namespace ns3
{

    struct JunInfo //路口信息？
    {
        uint8_t jid;
        int version;
        std::vector<int> list;

        JunInfo()
        {
        }
    };

    namespace grp
    {
        //格式转换
        double EmfToSeconds(uint8_t emf);
        uint8_t SecondsToEmf(double seconds);

        uint64_t LocToUint64(int64_t loc);
        int64_t Uint64ToLoc(uint64_t uin);
        //block包头
        class BlockPacketHeader : public Header
        {
        public:
            BlockPacketHeader();
            virtual ~BlockPacketHeader();

            static TypeId GetTypeId(void); //GetTypeId 查看类的属性
            virtual TypeId GetInstanceTypeId(void) const;
            virtual void Print(std::ostream &os) const;
            virtual uint32_t GetSerializedSize(void) const;
            virtual void Serialize(Buffer::Iterator start) const;
            virtual uint32_t Deserialize(Buffer::Iterator start);

        private:
            Ipv4Address m_addr;
            uint32_t m_speed;
            uint64_t m_posx;
            uint64_t m_posy;
        };


        class CtrPacketHeader : public Header
        {
        public:
            CtrPacketHeader();
            virtual ~CtrPacketHeader();

            /**
   * Set the packet total length.
   * \param length The packet length.
   */
            void SetPacketLength(uint16_t length)
            {
                m_packetLength = length;
            }

            /**
   * Get the packet total length.
   * \return The packet length.
   */
            uint16_t GetPacketLength() const
            {
                return m_packetLength;
            }

            /**
   * Set the packet sequence number.
   * \param seqnum The packet sequence number.
   */
            void SetPacketSequenceNumber(uint16_t seqnum)
            {
                m_packetSequenceNumber = seqnum;
            }

            /**
   * Get the packet sequence number.
   * \returns The packet sequence number.
   */
            uint16_t GetPacketSequenceNumber() const
            {
                return m_packetSequenceNumber;
            }

        private:
            uint16_t m_packetLength;         //!< The packet length.
            uint16_t m_packetSequenceNumber; //!< The packet sequence number.

        public:
            /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
            static TypeId GetTypeId(void);
            virtual TypeId GetInstanceTypeId(void) const;
            virtual void Print(std::ostream &os) const;
            virtual uint32_t GetSerializedSize(void) const;
            virtual void Serialize(Buffer::Iterator start) const;
            virtual uint32_t Deserialize(Buffer::Iterator start);
        };

        class DataPacketHeader : public Header
        {
        public:
            DataPacketHeader();
            virtual ~DataPacketHeader();

            void SetNextJID(uint8_t jid)
            {
                nextjid = jid;
            }

            uint8_t GetNextJID() const
            {
                return nextjid;
            }

            void SetSenderID(int id)
            {
                sender = id;
            }

            int GetSenderID() const
            {
                return sender;
            }
            void SetFromJID(uint8_t jid)
            {
                fromjid = jid;
            }

            uint8_t GetFromJID() const
            {
                return fromjid;
            }


        private:
            uint8_t nextjid;
            uint8_t fromjid;
            uint16_t sender;

        public:
            static TypeId GetTypeId(void);
            virtual TypeId GetInstanceTypeId(void) const;
            virtual void Print(std::ostream &os) const;
            virtual uint32_t GetSerializedSize(void) const;
            virtual void Serialize(Buffer::Iterator start) const;
            virtual uint32_t Deserialize(Buffer::Iterator start);
        };

        //	  0                   1                   2                   3
        //    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
        //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        //   |  Message Type |     Vtime     |         Message Size          |
        //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        //   |                      Originator Address                       |
        //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        //   |  Time To Live |   Hop Count   |    Message Sequence Number    |
        //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        //   |                                                               |
        //   :                            MESSAGE                            :
        //   |                                                               |
        //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

        class MessageHeader : public Header
        {
        public:
            enum MessageType
            {
                HELLO_MESSAGE = 1,
                CPACK_MESSAGE = 2,
                CollectPacket_MESSAGE = 3,
                RTNSMResult_MESSAGE = 4,
            };

            MessageHeader();
            virtual ~MessageHeader();

            void SetMessageType(MessageType messageType)
            {
                m_messageType = messageType;
            }

            MessageType GetMessageType() const
            {
                return m_messageType;
            }

            void SetVTime(Time time)
            {
                m_vTime = SecondsToEmf(time.GetSeconds());
            }

            Time GetVTime() const
            {
                return Seconds(EmfToSeconds(m_vTime));
            }

            void SetOriginatorAddress(Ipv4Address originatorAddress)
            {
                m_originatorAddress = originatorAddress;
            }

            Ipv4Address GetOriginatorAddress() const
            {
                return m_originatorAddress;
            }

            void SetTimeToLive(uint8_t timeToLive)
            {
                m_timeToLive = timeToLive;
            }

            uint8_t GetTimeToLive() const
            {
                return m_timeToLive;
            }

            void SetHopCount(uint8_t hopCount)
            {
                m_hopCount = hopCount;
            }

            uint8_t GetHopCount() const
            {
                return m_hopCount;
            }

            void SetMessageSequenceNumber(uint16_t messageSequenceNumber)
            {
                m_messageSequenceNumber = messageSequenceNumber;
            }

            uint16_t GetMessageSequenceNumber() const
            {
                return m_messageSequenceNumber;
            }

        private:
            MessageType m_messageType;
            uint8_t m_vTime;
            Ipv4Address m_originatorAddress;
            uint8_t m_timeToLive;
            uint8_t m_hopCount;
            uint16_t m_messageSequenceNumber;
            uint16_t m_messageSize;

        public:
            /**
   * \brief Get the type ID.
   * \return The object TypeId.
   */
            static TypeId GetTypeId(void);
            virtual TypeId GetInstanceTypeId(void) const;
            virtual void Print(std::ostream &os) const;
            virtual uint32_t GetSerializedSize(void) const;
            virtual void Serialize(Buffer::Iterator start) const;
            virtual uint32_t Deserialize(Buffer::Iterator start);

            //   --------------------------HELLO MESSAGE--------------------------
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                           Location X                          |
            //   |                                                               |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                           Location Y                          |
            //   |                                                               |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                             speed                             |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                           direction                           |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                        Neighbor Address                       |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                        Neighbor Address                       |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            //   |                              ...                              |
            //   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

            struct Hello
            {
                uint64_t locationX;
                uint64_t locationY;
                uint32_t speed;
                uint32_t direction;
                uint16_t turn;

                int bsize = 0;
                int asize = 0;

                int GetBeaconSize() const
                {
                    return bsize;
                }

                int GetBeaconAppendSize() const
                {
                    return asize;
                }

                std::vector<JunInfo> conlist;

                void SetTurn(int njid)
                {
                    this->turn = (uint16_t)njid;
                }

                void SetLocation(double x, double y)
                {
                    this->locationX = LocToUint64((uint64_t)(x * 1000));
                    this->locationY = LocToUint64((uint64_t)(y * 1000));
                }

                void SetSpeedAndDirection(double speed, uint32_t direction)
                {
                    this->speed = (uint32_t)(speed * 1000);
                    this->direction = direction;
                }

                int GetTurn() const
                {
                    if (this->turn >= 1000)
                        return -1;
                    else
                        return (int)this->turn;
                }

                double GetLocationX() const
                {
                    return Uint64ToLoc(this->locationX) / 1000.0;
                }

                double GetLocationY() const
                {
                    return Uint64ToLoc(this->locationY) / 1000.0;
                }

                double GetSpeed() const
                {
                    return double(this->speed) / 1000.0;
                }

                uint32_t GetDirection() const
                {
                    return this->direction;
                }

                std::vector<Ipv4Address> neighborInterfaceAddresses;

                void Print(std::ostream &os) const;
                uint32_t GetSerializedSize(void) const;
                void Serialize(Buffer::Iterator start) const;
                uint32_t Deserialize(Buffer::Iterator start, uint32_t messageSize);
            };
            struct CollectPacket
            {
                Ipv4Address CF_addr;
                Ipv4Address NF_addr;
                uint32_t IVD_density;             //同向车辆密度
                uint32_t DVD_density;             //反向车辆密度
                uint32_t first_JID;               //cp包生成的路口id
                uint32_t sencond_JID;             //cp包走向的路口id
                uint32_t roadID;                  //生成cp包的道路ID
                uint32_t VPV_ValidityPeriodValue;//有效期值
                uint32_t VPV_ValidityPeriodValueFpart;//有效期值小数部分
                uint32_t NumberOfHops;            //跳数
                uint16_t OriginalityFlag;         //区分CP和CPR
                uint8_t TimeStamp;                //cp包生成时的时间戳

                void SetCFAddress(Ipv4Address addr)
                {
                    this->CF_addr = addr;
                }

                Ipv4Address GetCFAddress() const
                {
                    return this->CF_addr;
                }
                //NF_addr
                void SetNFAddress(Ipv4Address addr)
                {
                    this->NF_addr = addr;
                }

                Ipv4Address GetNFAddress() const
                {
                    return this->NF_addr;
                }
                //IVD_density
                void SetIVDDensity(uint32_t density)
                {
                    this->IVD_density = density;
                }

                uint32_t GetIVDDensity() const
                {
                    return this->IVD_density;
                }
                //DVD_density
                void SetDVDDensity(uint32_t density)
                {
                    this->DVD_density = density;
                }

                uint32_t GetDVDDensity() const
                {
                    return this->DVD_density;
                }
                //first_JID
                void SetFirstJID(uint32_t cjid)
                {
                    this->first_JID = cjid;
                }

                uint32_t GetFirstJID() const
                {
                    return this->first_JID;
                }
                //sencond_JID
                void SetSencondJID(uint32_t djid)
                {
                    this->sencond_JID = djid;
                }

                uint32_t GetSencondJID() const
                {
                    return this->sencond_JID;
                }
                //roadID
                void SetRoadID(uint32_t id)
                {
                    this->roadID = id;
                }

                uint32_t GetRoadID() const
                {
                    return this->roadID;
                }
                //VPV_ValidityPeriodValue
                void SetVPVValidityPeriodValue(uint32_t vpv_value)
                {
                    this->VPV_ValidityPeriodValue = vpv_value;
                }

                uint32_t GetVPVValidityPeriodValue() const
                {
                    return this->VPV_ValidityPeriodValue;
                }
                //VPV_ValidityPeriodValueFpart
                void SetVPVValidityPeriodValueFPart(uint32_t vpv_valuefpart)
                {
                    this->VPV_ValidityPeriodValueFpart = vpv_valuefpart;
                }

                uint32_t GetVPVValidityPeriodValueFPart() const
                {
                    return this->VPV_ValidityPeriodValueFpart;
                }
                //NumberOfHops
                void SetNumberOfHops(uint32_t number)
                {
                    this->NumberOfHops = number;
                }

                uint32_t GetNumberOfHops() const
                {
                    return this->NumberOfHops;
                }
                //OriginalityFlag
                void SetOriginalityFlag(uint32_t flag)
                {
                    this->OriginalityFlag = flag;
                }

                uint32_t GetOriginalityFlag() const
                {
                    return this->OriginalityFlag;
                }
                //TimeStamp
                void SetTimeStamp(Time time)
                {
                    this->TimeStamp = SecondsToEmf(time.GetSeconds());
                }

                Time GetTimeStamp() const
                {
                    return Seconds(EmfToSeconds(this->TimeStamp));
                }

                void Print(std::ostream &os) const;
                uint32_t GetSerializedSize(void) const;
                void Serialize(Buffer::Iterator start) const;
                uint32_t Deserialize(Buffer::Iterator start, uint32_t messageSize);
            };
            struct RTNSMResult
            {
                uint32_t firstJID;   //cp包生成的路口id
                uint32_t sencondJID; //cp包走向的路口id
                uint64_t RTNSMRS;    //RTNSMResult
                uint64_t RTNSMRS_Fpart;
                uint8_t TimeStamp;
                uint32_t VP;
                uint32_t VP_Fpart;
                uint32_t RSFlag;
                void SetFirstJID(uint32_t sourceJID)
                {
                    this->firstJID = sourceJID;
                }

                uint32_t GetFirstJID() const
                {
                    return this->firstJID;
                }
                void SetSencondJID(uint32_t dstJID)
                {
                    this->sencondJID = dstJID;
                }

                uint32_t GetSencondJID() const
                {
                    return this->sencondJID;
                }
                void SetRTNSMRS(int64_t RS)
                {
                    this->RTNSMRS = RS;
                }

                uint64_t GetRTNSMRS() const
                {
                    return this->RTNSMRS;
                }
                void SetRTNSMRSFPart(int64_t RS_Fpart)
                {
                    this->RTNSMRS_Fpart = RS_Fpart;
                }

                uint64_t GetRTNSMRSFPart() const
                {
                    return this->RTNSMRS_Fpart;
                }


                void SetTimeStamp(Time time)
                {
                    this->TimeStamp = SecondsToEmf(time.GetSeconds()); //GetSeconds ()返回值是double型
                }

                Time GetTimeStamp() const
                {
                    return Seconds(EmfToSeconds(this->TimeStamp));
                }
                uint32_t GetVP() const
                {
                    return this->VP;
                }
                void SetVP(uint32_t vp)
                {
                    this->VP = vp;
                }
                uint32_t GetVPFPart() const
                {
                    return this->VP_Fpart;
                }
                void SetVPFPart(uint32_t VPfpart)
                {
                    this->VP_Fpart = VPfpart;
                }
                 uint32_t GetRSFlag() const
                {
                    return this->RSFlag;
                }
                void SetRSFlag(uint32_t RS_flag)
                {
                    this->RSFlag = RS_flag;
                }
                void Print(std::ostream &os) const;
                uint32_t GetSerializedSize(void) const;
                void Serialize(Buffer::Iterator start) const;
                uint32_t Deserialize(Buffer::Iterator start, uint32_t messageSize);
            };

        private:
            struct
            {
                Hello hello;
                CollectPacket cp;
                RTNSMResult RS;
            } m_message;

        public:
            Hello &GetHello()
            {
                if (m_messageType == 0)
                {
                    m_messageType = HELLO_MESSAGE;
                }
                else
                {
                    NS_ASSERT(m_messageType == HELLO_MESSAGE);
                }
                return m_message.hello;
            }

            const Hello &GetHello() const
            {
                NS_ASSERT(m_messageType == HELLO_MESSAGE);
                return m_message.hello;
            }
            CollectPacket &GetCollectPacket()
            {
                if (m_messageType == 0)
                {
                    m_messageType = CollectPacket_MESSAGE;
                }
                else
                {
                    NS_ASSERT(m_messageType == CollectPacket_MESSAGE);
                }
                return m_message.cp;
            }
            const CollectPacket &GetCollectPacket() const
            {
                NS_ASSERT(m_messageType == CollectPacket_MESSAGE);
                return m_message.cp;
            }

            RTNSMResult &GetRTNSMResult()
            {
                if (m_messageType == 0)
                {
                    m_messageType = RTNSMResult_MESSAGE;
                }
                else
                {
                    NS_ASSERT(m_messageType == RTNSMResult_MESSAGE);
                }
                return m_message.RS;
            }

            const RTNSMResult &GetRTNSMResult() const
            {
                NS_ASSERT(m_messageType == RTNSMResult_MESSAGE);
                return m_message.RS;
            }
        };

        static inline std::ostream &operator<<(std::ostream &os, const MessageHeader &message)
        {
            message.Print(os);
            return os;
        }

        typedef std::vector<MessageHeader> MessageList;

        static inline std::ostream &operator<<(std::ostream &os, const MessageList &messages)
        {
            os << "[";
            for (std::vector<MessageHeader>::const_iterator messageIter = messages.begin();
                 messageIter != messages.end(); messageIter++)
            {
                messageIter->Print(os);
                if (messageIter + 1 != messages.end())
                {
                    os << ", ";
                }
            }
            os << "]";
            return os;
        }
    } // namespace grp
} // namespace ns3

#endif /* GRP_HEADER_H */
