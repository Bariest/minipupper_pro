#include <stdlib.h>
#include "INST.h"
#include "SCS.h"

// 通信级别标志：0-无应答，1-有应答
static uint8_t Level = 1;
// 数据包结束字节顺序：0-高位在前，1-低位在前
static uint8_t End = 0;
// 伺服电机状态码
static uint8_t u8Status;
// 最后错误码
static uint8_t u8Error;
// 同步读取接收数据包索引
uint8_t syncReadRxPacketIndex;
// 同步读取接收数据包长度
uint8_t syncReadRxPacketLen;
// 同步读取接收数据包指针
uint8_t *syncReadRxPacket;
// 同步读取接收缓冲区指针
uint8_t *syncReadRxBuff;
// 同步读取接收缓冲区当前长度
uint16_t syncReadRxBuffLen;
// 同步读取接收缓冲区最大长度
uint16_t syncReadRxBuffMax;

/**
 * @brief 设置数据包结束字节顺序
 * 
 * @param _End 0-高位在前，1-低位在前
 */
void setEnd(uint8_t _End)
{
	End = _End;
}

/**
 * @brief 获取当前数据包结束字节顺序
 * 
 * @return uint8_t 当前结束字节顺序
 */
uint8_t getEnd(void)
{
	return End;
}

/**
 * @brief 设置通信级别
 * 
 * @param _Level 0-无应答模式，1-有应答模式
 */
void setLevel(uint8_t _Level)
{
	Level = _Level;
}

/**
 * @brief 获取当前伺服电机状态码
 * 
 * @return int 当前状态码
 */
int getState(void)
{
	return u8Status;
}

/**
 * @brief 获取最后的错误码
 * 
 * @return int 最后的错误码
 */
int getLastError(void)
{
	return u8Error;
}

/**
 * @brief 主机数据转SCS协议格式
 * 
 * 将主机端的整数数据转换为SCS协议的高低字节格式
 * @param DataL 低字节指针
 * @param DataH 高字节指针  
 * @param Data 要转换的数据
 */
void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
	if(End){
		// 低位在前模式
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		// 高位在前模式（默认）
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

/**
 * @brief SCS协议格式转主机数据
 * 
 * 将SCS协议的高低字节格式转换为主机端的整数数据
 * @param DataL 低字节
 * @param DataH 高字节
 * @return int 转换后的数据
 */
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
	int Data;
	if(End){
		// 低位在前模式
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		// 高位在前模式（默认）
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

/**
 * @brief 写入数据缓冲区（内部函数）
 * 
 * 构造并发送SCS协议数据包
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @param Fun 功能码
 */
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t msgLen = 2;  // 基础消息长度（不含ID和长度字段）
	uint8_t bBuf[6];     // 头部缓冲区
	uint8_t CheckSum = 0; // 校验和
	
	// 构造数据包头部
	bBuf[0] = 0xff;      // 包头1
	bBuf[1] = 0xff;      // 包头2  
	bBuf[2] = ID;        // 伺服电机ID
	bBuf[4] = Fun;       // 功能码
	
	if(nDat){
		// 有数据的情况
		msgLen += nLen + 1;  // 消息长度 = 基础长度 + 数据长度 + 地址长度
		bBuf[3] = msgLen;    // 长度字段
		bBuf[5] = MemAddr;   // 内存地址
		writeSCS(bBuf, 6);   // 发送头部
		
	}else{
		// 无数据的情况（如Ping、Reset等）
		bBuf[3] = msgLen;    // 长度字段
		writeSCS(bBuf, 5);   // 发送头部（不含地址）
	}
	
	// 计算校验和（所有字节的和取反）
	CheckSum = ID + msgLen + Fun + MemAddr;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);  // 发送数据
	}
	CheckSum = ~CheckSum;
	writeSCS(&CheckSum, 1);    // 发送校验和
}

/**
 * @brief 通用写入函数
 * 
 * 向指定ID的伺服电机内存地址写入数据
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @return int 操作结果（1成功，0失败）
 */
int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();           // 清空接收缓冲区
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);  // 发送写入命令
	wFlushSCS();           // 刷新发送缓冲区
	return Ack(ID);        // 等待并验证应答
}

/**
 * @brief 寄存器写入函数（延迟执行）
 * 
 * 写入数据但不立即执行，需要调用regAction()触发
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针
 * @param nLen 数据长度
 * @return int 操作结果（1成功，0失败）
 */
int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief 触发寄存器写入动作
 * 
 * 执行之前通过regWrite设置的动作
 * @param ID 伺服电机ID
 * @return int 操作结果（1成功，0失败）
 */
int regAction(uint8_t ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief 同步写入函数
 * 
 * 同时向多个伺服电机写入相同的数据结构
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param MemAddr 内存地址
 * @param nDat 数据缓冲区指针（按ID顺序排列）
 * @param nLen 每个伺服电机的数据长度
 */
void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	uint8_t mesLen = ((nLen+1)*IDN+4);  // 消息总长度
	uint8_t Sum = 0;                    // 校验和
	uint8_t bBuf[7];                    // 头部缓冲区
	uint8_t i, j;
	
	// 构造同步写入头部
	bBuf[0] = 0xff;                     // 包头1
	bBuf[1] = 0xff;                     // 包头2
	bBuf[2] = 0xfe;                     // 广播ID
	bBuf[3] = mesLen;                   // 消息长度
	bBuf[4] = INST_SYNC_WRITE;          // 同步写入功能码
	bBuf[5] = MemAddr;                  // 内存地址
	bBuf[6] = nLen;                     // 每个伺服的数据长度
	
	rFlushSCS();
	writeSCS(bBuf, 7);                  // 发送头部

	// 计算校验和并发送数据
	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	for(i=0; i<IDN; i++){
		writeSCS(&ID[i], 1);            // 发送ID
		writeSCS(nDat+i*nLen, nLen);    // 发送对应数据
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];      // 累加校验和
		}
	}
	Sum = ~Sum;
	writeSCS(&Sum, 1);                  // 发送校验和
	wFlushSCS();
}

/**
 * @brief 写入单字节数据
 * 
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param bDat 要写入的字节数据
 * @return int 操作结果（1成功，0失败）
 */
int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief 写入双字节数据（16位）
 * 
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param wDat 要写入的字数据
 * @return int 操作结果（1成功，0失败）
 */
int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
	uint8_t buf[2];
	Host2SCS(buf+0, buf+1, wDat);       // 转换数据格式
	rFlushSCS();
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief 通用读取函数
 * 
 * 从指定ID的伺服电机内存地址读取数据
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @param nData 接收数据的缓冲区指针
 * @param nLen 要读取的数据长度
 * @return int 实际读取的数据长度，0表示失败
 */
int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
	int Size;
	uint8_t bBuf[4];
	uint8_t calSum;
	uint8_t i;
	rFlushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);  // 发送读取命令
	wFlushSCS();
	u8Error = 0;
	
	// 检查包头
	if(!checkHead()){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 读取响应头部
	if(readSCS(bBuf, 3)!=3){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 验证ID
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = SCS_ERR_SLAVE_ID;
		return 0;
	}
	
	// 验证长度
	if(bBuf[1]!=(nLen+2)){
		u8Error = SCS_ERR_BUFF_LEN;
		return 0;
	}
	
	// 读取数据
	Size = readSCS(nData, nLen);
	if(Size!=nLen){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 读取校验和
	if(readSCS(bBuf+3, 1)!=1){
		u8Error = SCS_ERR_NO_REPLY;
		return 0;
	}
	
	// 验证校验和
	calSum = bBuf[0]+bBuf[1]+bBuf[2];
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		u8Error = SCS_ERR_CRC_CMP;
		return 0;
	}
	
	u8Status = bBuf[2];  // 保存状态码
	return Size;
}

/**
 * @brief 读取单字节数据
 * 
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @return int 读取到的字节数据，-1表示失败
 */
int readByte(uint8_t ID, uint8_t MemAddr)
{
	uint8_t bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

/**
 * @brief 读取双字节数据（16位）
 * 
 * @param ID 伺服电机ID
 * @param MemAddr 内存地址
 * @return int 读取到的字数据，-1表示失败
 */
int readWord(uint8_t ID, uint8_t MemAddr)
{	
	uint8_t nDat[2];
	int Size;
	uint16_t wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

/**
 * @brief Ping伺服电机
 * 
 * 检测指定ID的伺服电机是否在线
 * @param ID 伺服电机ID
 * @return int 返回伺服电机的实际ID，-1表示失败
 */
int Ping(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSCS();
	u8Status = 0;
	
	// 检查包头
	if(!checkHead()){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	u8Error = 0;
	
	// 读取响应
	if(readSCS(bBuf, 4)!=4){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	
	// 验证ID
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = SCS_ERR_SLAVE_ID;
		return -1;
	}
	
	// 验证长度
	if(bBuf[1]!=2){
		u8Error = SCS_ERR_BUFF_LEN;
		return -1;
	}
	
	// 验证校验和
	calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		u8Error = SCS_ERR_CRC_CMP;
		return -1;			
	}
	
	u8Status = bBuf[2];
	return bBuf[0];
}

/**
 * @brief 重置伺服电机
 * 
 * 将伺服电机恢复到出厂设置
 * @param ID 伺服电机ID
 * @return int 返回伺服电机的实际ID，-1表示失败
 */
int Reset(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	wFlushSCS();
	u8Status = 0;
	
	// 检查包头
	if(!checkHead()){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	u8Error = 0;
	
	// 读取响应
	if(readSCS(bBuf, 4)!=4){
		u8Error = SCS_ERR_NO_REPLY;
		return -1;
	}
	
	// 验证ID
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = SCS_ERR_SLAVE_ID;
		return -1;
	}
	
	// 验证长度
	if(bBuf[1]!=2){
		u8Error = SCS_ERR_BUFF_LEN;
		return -1;
	}
	
	// 验证校验和
	calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		u8Error = SCS_ERR_CRC_CMP;
		return -1;			
	}
	
	u8Status = bBuf[2];
	return bBuf[0];
}

/**
 * @brief 检查数据包头部
 * 
 * 在接收缓冲区中查找有效的包头（0xFF 0xFF）
 * @return int 1表示找到有效包头，0表示超时
 */
int checkHead(void)
{
	uint8_t bDat;
	uint8_t bBuf[2] = {0, 0};
	uint8_t Cnt = 0;
	while(1){
		if(!readSCS(&bDat, 1)){
			return 0;  // 超时
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;  // 找到包头
		}
		Cnt++;
		if(Cnt>10){  // 最大尝试次数
			return 0;
		}
	}
	return 1;
}

/**
 * @brief 确认应答（内部函数）
 * 
 * 等待并验证伺服电机的应答
 * @param ID 伺服电机ID
 * @return int 1表示应答正确，0表示错误
 */
int Ack(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	u8Error = 0;
	
	// 如果是广播模式或无应答模式，直接返回成功
	if(ID!=0xfe && Level){
		if(!checkHead()){
			u8Error = SCS_ERR_NO_REPLY;
			return 0;
		}
		u8Status = 0;
		
		// 读取应答数据
		if(readSCS(bBuf, 4)!=4){
			u8Error = SCS_ERR_NO_REPLY;
			return 0;
		}
		
		// 验证ID
		if(bBuf[0]!=ID){
			u8Error = SCS_ERR_SLAVE_ID;
			return 0;
		}
		
		// 验证长度
		if(bBuf[1]!=2){
			u8Error = SCS_ERR_BUFF_LEN;
			return 0;
		}
		
		// 验证校验和
		calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			u8Error = SCS_ERR_CRC_CMP;
			return 0;			
		}
		
		u8Status = bBuf[2];  // 保存状态码
	}
	return 1;
}

/**
 * @brief 同步读取数据包发送
 * 
 * 发送同步读取请求
 * @param ID 伺服电机ID数组
 * @param IDN 伺服电机数量
 * @param MemAddr 内存地址
 * @param nLen 要读取的数据长度
 * @return int 实际接收到的数据长度
 */
int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
	uint8_t checkSum;
	uint8_t i;
	rFlushSCS();
	syncReadRxPacketLen = nLen;
	
	// 计算初始校验和
	checkSum = (4+0xfe)+IDN+MemAddr+nLen+INST_SYNC_READ;
	
	// 发送同步读取命令
	writeByteSCS(0xff);
	writeByteSCS(0xff);
	writeByteSCS(0xfe);
	writeByteSCS(IDN+4);
	writeByteSCS(INST_SYNC_READ);
	writeByteSCS(MemAddr);
	writeByteSCS(nLen);
	
	// 发送ID列表并累加校验和
	for(i=0; i<IDN; i++){
		writeByteSCS(ID[i]);
		checkSum += ID[i];
	}
	
	// 发送校验和
	checkSum = ~checkSum;
	writeByteSCS(checkSum);
	wFlushSCS();
	
	// 读取响应数据
	syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax);
	return syncReadRxBuffLen;
}

/**
 * @brief 开始同步读取
 * 
 * 初始化同步读取过程，分配接收缓冲区
 * @param IDN 伺服电机数量
 * @param rxLen 每个伺服电机的接收数据长度
 */
void syncReadBegin(uint8_t IDN, uint8_t rxLen)
{
	syncReadRxBuffMax = IDN*(rxLen+6);  // 计算所需缓冲区大小
	syncReadRxBuff = malloc(syncReadRxBuffMax);  // 分配内存
}

/**
 * @brief 结束同步读取
 * 
 * 释放同步读取过程中分配的内存
 */
void syncReadEnd(void)
{
	if(syncReadRxBuff){
		free(syncReadRxBuff);
		syncReadRxBuff = NULL;
	}
}

/**
 * @brief 同步读取数据包接收
 * 
 * 从接收缓冲区中解析指定ID的伺服电机数据
 * @param ID 伺服电机ID
 * @param nDat 接收数据的缓冲区指针
 * @return int 解析到的数据长度，0表示失败
 */
int syncReadPacketRx(uint8_t ID, uint8_t *nDat)
{
	uint16_t syncReadRxBuffIndex = 0;
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	u8Status = 0;
	
	while((syncReadRxBuffIndex+6+syncReadRxPacketLen)<=syncReadRxBuffLen){
		uint8_t bBuf[] = {0, 0, 0};
		uint8_t calSum = 0;
		
		// 查找包头
		while(syncReadRxBuffIndex<syncReadRxBuffLen){
			bBuf[0] = bBuf[1];
			bBuf[1] = bBuf[2];
			bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
			if(bBuf[0]==0xff && bBuf[1]==0xff && bBuf[2]!=0xff){
				u8Error = SCS_ERR_NO_REPLY;
				break;
			}
		}
		
		// 验证ID
		if(bBuf[2]!=ID){
			u8Error = SCS_ERR_SLAVE_ID;
			continue;
		}
		
		// 验证长度
		if(syncReadRxBuff[syncReadRxBuffIndex++]!=(syncReadRxPacketLen+2)){
			continue;
		}
		
		// 读取状态码
		u8Status = syncReadRxBuff[syncReadRxBuffIndex++];
		
		// 读取数据并计算校验和
		calSum = ID+(syncReadRxPacketLen+2)+u8Status;
		for(uint8_t i=0; i<syncReadRxPacketLen; i++){
			syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
			calSum += syncReadRxPacket[i];
		}
		
		// 验证校验和
		calSum = ~calSum;
		if(calSum!=syncReadRxBuff[syncReadRxBuffIndex++]){
			u8Error = SCS_ERR_CRC_CMP;
			return 0;
		}
		return syncReadRxPacketLen;
	}
	return 0;
}

/**
 * @brief 从同步读取数据包中读取字节
 * 
 * @return int 读取到的字节数据，-1表示无数据
 */
int syncReadRxPacketToByte(void)
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

/**
 * @brief 从同步读取数据包中读取字（带符号处理）
 * 
 * @param negBit 符号位位置（用于处理负数）
 * @return int 读取到的字数据，-1表示无数据
 */
int syncReadRxPacketToWrod(uint8_t negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}