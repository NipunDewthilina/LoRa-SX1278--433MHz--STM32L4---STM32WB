/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#include "SX1278.h"
#include <string.h>

uint8_t SX1278_SPIRead(SX1278_t *module, uint8_t addr) {
	uint8_t tmp;
	SX1278_hw_SPICommand(module->hw, addr);
	tmp = SX1278_hw_SPIReadByte(module->hw);
	SX1278_hw_SetNSS(module->hw, 1);
	return tmp;
}

void SX1278_SPIWrite(SX1278_t *module, uint8_t addr, uint8_t cmd) {
	SX1278_hw_SetNSS(module->hw, 0);
	SX1278_hw_SPICommand(module->hw, addr | 0x80);
	SX1278_hw_SPICommand(module->hw, cmd);
	SX1278_hw_SetNSS(module->hw, 1);
}

void SX1278_SPIBurstRead(SX1278_t *module, uint8_t addr, uint8_t *rxBuf,
		uint8_t length) {
	uint8_t i;
	if (length <= 1) {
		return;
	} else {
		SX1278_hw_SetNSS(module->hw, 0);
		SX1278_hw_SPICommand(module->hw, addr);
		for (i = 0; i < length; i++) {
			*(rxBuf + i) = SX1278_hw_SPIReadByte(module->hw);
		}
		SX1278_hw_SetNSS(module->hw, 1);
	}
}

void SX1278_SPIBurstWrite(SX1278_t *module, uint8_t addr, uint8_t *txBuf,
		uint8_t length) {
	unsigned char i;
	if (length <= 1) {
		return;
	} else {
		SX1278_hw_SetNSS(module->hw, 0);
		SX1278_hw_SPICommand(module->hw, addr | 0x80);
		for (i = 0; i < length; i++) {
			SX1278_hw_SPICommand(module->hw, *(txBuf + i));
		}
		SX1278_hw_SetNSS(module->hw, 1);
	}
}

void SX1278_config(SX1278_t *module) {

	SX1278_sleep(module); //Change modem mode Must in Sleep mode
	SX1278_hw_DelayMs(15);

	SX1278_entryLoRa(module);
	//SX1278_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note
	SX1278_setNodeAddress(module, module->node_addr);
	uint64_t freq = ((uint64_t) module->frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	SX1278_SPIBurstWrite(module, LR_RegFrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter

//	SX1278_SPIWrite(module, RegSyncConfig, 0x53); /* changed 22/03/2021 Controls the automatic restart of the receiver after the reception of
//	a valid packet (PayloadReady or CrcOk):*/
	SX1278_SPIWrite(module, RegSyncWord, 0x34);

	//setting base parameter
	SX1278_SPIWrite(module, LR_RegPaConfig, SX1278_Power[module->power]); //Setting output power parameter

	SX1278_SPIWrite(module, LR_RegOcp, 0x0B);			//RegOcp,Close Ocp
	SX1278_SPIWrite(module, LR_RegLna, 0x23);		//RegLNA,High & LNA Enable
	if (SX1278_SpreadFactor[module->LoRa_SF] == 6) {	//SFactor=6
		uint8_t tmp;
		SX1278_SPIWrite(module,
		LR_RegModemConfig1,
				((SX1278_LoRaBandwidth[module->LoRa_BW] << 4)
						+ (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		LR_RegModemConfig2,
				((SX1278_SpreadFactor[module->LoRa_SF] << 4)
						+ (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x03));

		tmp = SX1278_SPIRead(module, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX1278_SPIWrite(module, 0x31, tmp);
		SX1278_SPIWrite(module, 0x37, 0x0C);
	} else {
		SX1278_SPIWrite(module,
		LR_RegModemConfig1,
				((SX1278_LoRaBandwidth[module->LoRa_BW] << 4)
						+ (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		LR_RegModemConfig2,
				((SX1278_SpreadFactor[module->LoRa_SF] << 4)
						+ (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x00)); //SFactor &  LNA gain set by the internal AGC loop
	}

	SX1278_SPIWrite(module, LR_RegModemConfig3, 0x04);
	SX1278_SPIWrite(module, LR_RegSymbTimeoutLsb, 0x08); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1278_SPIWrite(module, LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX1278_SPIWrite(module, LR_RegPreambleLsb, 8); //RegPreambleLsb 8+4=12byte Preamble
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	module->readBytes = 0;
//	SX1278_setOCP(module, module->maximum_Current);
	//SX1278_standby(module); //Entry standby mode
	SX1278_cad(module); //Entry cad mode
}

void SX1278_standby(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x09);
	module->status = STANDBY;
}

void SX1278_sleep(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x08);
	module->status = SLEEP;
}

void SX1278_cad(SX1278_t *module)
{
	SX1278_SPIWrite(module, LR_RegOpMode, 0x0F);
	module->status = CAD;
}

void SX1278_entryLoRa(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x88);
}

void SX1278_clearLoRaIrq(SX1278_t *module) {
	uint8_t st0;
	st0 = SX1278_SPIRead(module, LR_RegOpMode);	// Save the previous status
	SX1278_standby(module);
	SX1278_SPIWrite(module, LR_RegIrqFlags, 0xFF);
	SX1278_SPIWrite(module, LR_RegOpMode, st0);

}

int SX1278_LoRaEntryRx(SX1278_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;

	module->packetLength = length;

	SX1278_config(module);		//Setting base parameter
	SX1278_SPIWrite(module, REG_LR_PADAC, 0x84);	//Normal and RX
	SX1278_SPIWrite(module, LR_RegHopPeriod, 0xFF);	//No FHSS
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01);//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F);//Open RxDone interrupt & Timeout
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, LR_RegPayloadLength, length);//Payload Length 21byte(this register must define when the data long of one byte in SF is 6)
	addr = SX1278_SPIRead(module, LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8d);	//Mode//Low Frequency Mode
	//SX1278_SPIWrite(module, LR_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	module->readBytes = 0;

	while (1) {
		if ((SX1278_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04) {	//Rx-on going RegModemStat
			module->status = RX;
			return 1;
		}
		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

uint8_t SX1278_LoRaRxPacket(SX1278_t *module,SX1278_pack_t *packet,SX1278_pack_t *ack ) {
	unsigned char addr;
	unsigned char packet_size;


	if (SX1278_hw_GetDIO0(module->hw)) {
		memset(packet->data, 0x00, SX1278_MAX_PAYLOAD);

		addr = SX1278_SPIRead(module, LR_RegFifoRxCurrentaddr); //last packet addr
		SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (module->LoRa_SF == SX1278_LORA_SF_6) { //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
			packet_size = module->packetLength;
		} else {
			packet_size = SX1278_SPIRead(module, LR_RegRxNbBytes); //Number for received bytes
		}
//		SX1278_SPIBurstRead(module, 0x00, packet->data, packet_size);
		SX1278_getPacket(module, packet);
		module->readBytes = packet_size;
		SX1278_clearLoRaIrq(module);
		return 1;
	}
	return 0;
}


int SX1278_LoRaEntryTx(SX1278_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;

	SX1278_config(module); //setting base parameter
	SX1278_SPIWrite(module, REG_LR_PADAC, 0x87);	//Tx for 20dBm
	SX1278_SPIWrite(module, LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(module, LR_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(module, LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	while (1) {
		temp = SX1278_SPIRead(module, LR_RegPayloadLength);
		if (temp == length) {
			module->status = TX;
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
	}
}

int SX1278_LoRaTxPacket(SX1278_t *module, SX1278_pack_t *packet_sent, uint8_t *txBuffer, uint8_t length,
		uint32_t timeout, uint8_t dest) {
	SX1278_setPacket(module, dest, (uint8_t*)txBuffer, packet_sent);
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8b);	//Tx Mode
	while (1) {
		if (SX1278_hw_GetDIO0(module->hw)) { //if(Get_NIRQ()) //Packet send over
			SX1278_SPIRead(module, LR_RegIrqFlags);
			SX1278_clearLoRaIrq(module); //Clear irq
			SX1278_cad(module); //Entry cad mode //edited
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

int SX1278_LoRaTxPacketACK(SX1278_t *module, SX1278_pack_t *packet_sent, SX1278_pack_t *ack,
		uint32_t timeout) {
//	SX1278_setPacket(module, dest, (uint8_t*)txBuffer, packet_sent);
	SX1278_setACK(module, packet_sent, ack);
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8b);	//Tx Mode
	while (1) {
		if (SX1278_hw_GetDIO0(module->hw)) { //if(Get_NIRQ()) //Packet send over
			SX1278_SPIRead(module, LR_RegIrqFlags);
			SX1278_clearLoRaIrq(module); //Clear irq
			SX1278_cad(module); //Entry cad mode //edited
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

void SX1278_init(SX1278_t *module, uint64_t frequency, uint8_t power, uint8_t current,
		uint8_t LoRa_SF, uint8_t LoRa_BW, uint8_t LoRa_CR, uint8_t node_addr,
		uint8_t LoRa_CRC_sum, uint8_t packetLength) {
	SX1278_hw_init(module->hw);
	module->frequency = frequency;
	module->power = power;
	module->LoRa_SF = LoRa_SF;
	module->LoRa_BW = LoRa_BW;
	module->LoRa_CR = LoRa_CR;
	module->LoRa_CRC_sum = LoRa_CRC_sum;
	module->packetLength = packetLength;
	module->maximum_Current = current;
	module->node_addr = node_addr;
	SX1278_config(module);
}

int SX1278_transmit(SX1278_t *module, SX1278_pack_t *packet_sent, uint8_t *txBuf, uint8_t length,
		uint32_t timeout, uint8_t dest) {
	if (SX1278_LoRaEntryTx(module, length, timeout)) {
		return SX1278_LoRaTxPacket(module, packet_sent, txBuf, length, timeout, dest);
	}
	return 0;
}

int SX1278_receive(SX1278_t *module, uint8_t length, uint32_t timeout) {
	return SX1278_LoRaEntryRx(module, length, timeout);
}

//uint8_t SX1278_available(SX1278_t *module) {
//	return SX1278_LoRaRxPacket(module);
//}

uint8_t SX1278_read(SX1278_t *module,SX1278_pack_t *packet, uint8_t *rxBuf, uint8_t length) {
	if (length != packet->length )
		length = packet->length ;
	memcpy(rxBuf, packet->data, length);
	rxBuf[length] = '\0';
	packet->length  = 0;
	return length;
}

uint8_t SX1278_RSSI_LoRa(SX1278_t *module) {
	uint32_t temp = 10;
	temp = SX1278_SPIRead(module, LR_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t SX1278_RSSI(SX1278_t *module) {
	uint8_t temp = 0xff;
	temp = SX1278_SPIRead(module, RegRssiValue);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}

void SX1278_setOCP(SX1278_t *module, uint8_t current){
	uint8_t	OcpTrim = 0;

	if(current<45)
		current = 45;
	if(current>240)
		current = 240;

	if(current <= 120)
		OcpTrim = (current - 45)/5;
	else if(current <= 240)
		OcpTrim = (current + 30)/10;

	OcpTrim = OcpTrim + (1 << 5);
	SX1278_SPIWrite(module, LR_RegOcp, OcpTrim);
	SX1278_hw_DelayMs(10);
}

void SX1278_getPacket(SX1278_t *module, SX1278_pack_t *packet_received)
{
	memset(packet_received->data, 0x00, SX1278_MAX_PAYLOAD);
	uint32_t payload_length;
	packet_received->dst = SX1278_SPIRead(module, 0x00); // Storing first byte of the received packet
	packet_received->src =SX1278_SPIRead(module, 0x00);
	packet_received->packnum = SX1278_SPIRead(module, 0x00);
	packet_received->length = SX1278_SPIRead(module, 0x00);
	packet_received->retry = SX1278_SPIRead(module,0x00);
	payload_length = packet_received->length;

	SX1278_SPIBurstRead(module, 0x00, packet_received->data, payload_length);
}

void SX1278_setPacket(SX1278_t *module, uint8_t dest, uint8_t *massage, SX1278_pack_t *packet_sent)
{
	//uint8_t st0;
	uint16_t payload_length;
	SX1278_clearLoRaIrq(module);	// Initializing flags

//	st0 = SX1278_SPIRead(module, LR_RegOpMode);	// Save the previous status

	memset(packet_sent->data, 0x00, SX1278_MAX_PAYLOAD);
	 // LoRa mode
	SX1278_standby(module);	// Stdby LoRa mode to write in FIFO

	packet_sent->dst = dest;	// Setting destination in packet structure
	packet_sent->src = SX1278_getNodeAddress(module);
	payload_length = (uint16_t)strlen(massage);
	payload_length = SX1278_truncPayload(payload_length);
	packet_sent->length = payload_length;
	for(unsigned int i = 0; i <payload_length; i++)
		{
			packet_sent->data[i] = massage[i];
		}

//	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, 0x80);  // Setting address pointer in FIFO data buffer
	// Writing packet to send in FIFO
//	SX1278_hw_SPICommand(module->hw, 0x00);
//	SX1278_hw_SPICommand(module->hw, packet_sent->dst);
//	SX1278_hw_SPICommand(module->hw, packet_sent->src);
//	SX1278_hw_SPICommand(module->hw, packet_sent->packnum);
//	SX1278_hw_SPICommand(module->hw, packet_sent->length);
	SX1278_SPIWrite(module, 0x00, packet_sent->dst); 		// Writing the destination in FIFO
	SX1278_SPIWrite(module, 0x00, packet_sent->src);		// Writing the source in FIFO
	SX1278_SPIWrite(module, 0x00, packet_sent->packnum);	// Writing the packet number in FIFO
	SX1278_SPIWrite(module, 0x00, packet_sent->length);
	SX1278_SPIWrite(module, 0x00, packet_sent->retry);
	SX1278_SPIBurstWrite(module, 0x00, massage, payload_length);
	// Writing the packet length in FIFO
//	for(unsigned int i = 0; i < payload_length; i++)
//
//		{
//			SX1278_SPIWrite(module, 0x00, packet_sent->data[i]);  // Writing the payload in FIFO
//		}
//	SX1278_SPIWrite(module, LR_RegOpMode, st0);

	}


uint16_t SX1278_truncPayload(uint16_t length16)
	{
		uint16_t _payloadlength;
		if( length16 > SX1278_MAX_PAYLOAD )
		{
			_payloadlength = SX1278_MAX_PAYLOAD;
		}
		else
		{
			_payloadlength = (length16 & 0xFF);
		}
		return _payloadlength;
	}

void SX1278_setACK(SX1278_t *module, SX1278_pack_t *packet_rec, SX1278_pack_t *ack)
{

	//uint8_t st0;
//	SX1278_clearLoRaIrq(module);	// Initializing flags

//	st0 = SX1278_SPIRead(module, LR_RegOpMode);	// Save the previous status
		SX1278_clearLoRaIrq(module);	// Initializing flags

	//	st0 = SX1278_SPIRead(module, LR_RegOpMode);	// Save the previous status

		memset(ack->data, 0x00, SX1278_MAX_PAYLOAD);
		 // LoRa mode
		SX1278_standby(module);	// Stdby LoRa mode to write in FIFO

	 // LoRa mode
//	SX1278_standby(module);	// Stdby LoRa mode to write in FIFO

	ack->dst = packet_rec->src;	// Setting destination in packet structure
	ack->src = SX1278_getNodeAddress(module);
	ack->packnum = packet_rec->packnum;
	ack->length = 0;
	ack->data[0] = 1;


	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, 0x80);  // Setting address pointer in FIFO data buffer
	// Writing packet to send in FIFO
	SX1278_SPIWrite(module, 0x00, ack->dst); 		// Writing the destination in FIFO
	SX1278_SPIWrite(module, 0x00, ack->src);		// Writing the source in FIFO
	SX1278_SPIWrite(module, 0x00, ack->packnum);	// Writing the packet number in FIFO
	SX1278_SPIWrite(module, 0x00, ack->length);
	SX1278_SPIWrite(module, 0x00, ack->data[0]);

//	SX1278_SPIWrite(module, LR_RegOpMode, st0);
}

int SX1278_getACK(SX1278_t *module, SX1278_pack_t *packet_sent, SX1278_pack_t *ACK, uint8_t dest)
{
	ACK->dst = dest;
	ACK->src = SX1278_SPIRead(module, 0x00);
	ACK->packnum = SX1278_SPIRead(module, 0x00);
	ACK->length = SX1278_SPIRead(module, 0x00);
	ACK->data[0] = SX1278_SPIRead(module, 0x00);
	// Checking the received ACK
	if( ACK->dst == packet_sent->src )
	{
		if( ACK->src == packet_sent->dst )
		{
			if( ACK->packnum == packet_sent->packnum )
			{
				if( ACK->length == 0 )
				{
					if( ACK->data[0] == 1 )
					{
						return 1;
					}
					else{
						return 0;
					}
				}
				else{
					return 0;
				}
			}
			else{
				return 0;
			}
		}
		else{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

void SX1278_setNodeAddress(SX1278_t *module, uint8_t addr)
{
	uint8_t _nodeAddress;
	_nodeAddress = addr;
	uint8_t st0 = 0;
	st0 = SX1278_SPIRead(module, LR_RegOpMode);	// Save the previous status
		// Allowing access to FSK registers while in LoRa standby mode
	SX1278_SPIWrite(module,LR_RegOpMode, 0xCA);
	// Saving node address
	SX1278_SPIWrite(module, RegNodeAdrs, _nodeAddress);
	SX1278_SPIWrite(module, RegBroadcastAdrs, 0x00);
	_nodeAddress = SX1278_SPIRead(module, RegNodeAdrs);
	SX1278_SPIWrite(module, LR_RegOpMode, st0);	// Getting back to previous status
}

uint8_t SX1278_getNodeAddress(SX1278_t *module)
{
	uint8_t _nodeAddress;
	uint8_t st0 = 0;
	st0 = SX1278_SPIRead(module, LR_RegOpMode);	// Save the previous status
		// Allowing access to FSK registers while in LoRa standby mode
	SX1278_SPIWrite(module,LR_RegOpMode, 0xCA);
	// Saving node address
	_nodeAddress = SX1278_SPIRead(module, RegNodeAdrs);
	SX1278_SPIWrite(module, LR_RegOpMode, st0);	// Getting back to previous status
	return _nodeAddress;
}

uint8_t SX1278_Compare(SX1278_t *module, SX1278_pack_t *ACK, SX1278_pack_t *packet_sent)
{
	if( ACK->dst == packet_sent->src )
		{
			if( ACK->src == packet_sent->dst )
			{
				if( ACK->packnum == packet_sent->packnum )
				{
					if( ACK->length == 0 )
					{
						if( ACK->data[0] == 1 )
						{
							return 1;
						}
						else{
							return 0;
						}
					}
					else{
						return 0;
					}
				}
				else{
					return 0;
				}
			}
			else{
				return 0;
			}
		}
		else
		{
			return 0;
		}
}

void SX1278_copypacket(SX1278_pack_t *pack1, SX1278_pack_t *pack2){
	uint8_t payload_length;
	pack1->dst = pack2->dst;
	pack1->src = pack2->src;
	pack1->packnum = pack2->packnum;
	pack1->length = pack2->length;
	pack1->retry = pack2->retry;
	payload_length = pack2->length;

	for(unsigned int i = 0; i <payload_length; i++)
			{
				pack1->data[i] = pack2->data[i];
			}

}

