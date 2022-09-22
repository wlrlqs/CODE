#include "bufferLoop_mod.h"
#include "Driver_Judge.h"

//�����Д���պ���
//array���յ��Ĕ���
//arrayLength���յ��Ĕ����L��
//bufferLoop�h�Δ��M
void bufferLoopContactISP(uint8_t *array,uint16_t arrayLength,buffefLoopStruct_t *bufferLoop){
	for(uint16_t count = 0; count < arrayLength; count++)
		bufferLoop->buffer[(uint8_t)(bufferLoop->tail + count)] = array[count];
	bufferLoop->tail += arrayLength;
}

//�h�Δ��M̎��
//bufferBegin���^
//headerLength��Ϣ���L��
//lengthPosition��Ű��L�ȵ�Ԫ����
//bufferLoop���Ք��M
//decodeTask�������
void bufferLoopMod(uint8_t bufferBegin, uint8_t headerLength, uint8_t lengthPosition, buffefLoopStruct_t *bufferLoop, decodeFunction decodeTask) {
	//LCM: β�±���ͷ�±�֮��ΪĿǰ���������д洢��δ����ȡ���������� 
	//LCM: ��δ����ȡ������������ ���ݰ�ͷ����
	while((uint8_t)(bufferLoop->tail - bufferLoop->header) > headerLength) {
		//LCM: ��δ��ȡ��ͷ֡����������
		while(((uint8_t)(bufferLoop->tail - bufferLoop->header) > headerLength) && (bufferLoop->buffer[bufferLoop->header]) != bufferBegin) {
			digitalIncreasing(&bufferLoop->header);
		}
		//LCM: ����֮��δ����ȡ������Ȼ���� ���ݰ�ͷ����	
		if((uint8_t)(bufferLoop->tail - bufferLoop->header) > headerLength) {
			//����CCRC8�Ƿ�ͨ�^	//LCM: CRC8������ݰ�ͷ����У�� 
			uint8_t *crc8Check = (uint8_t*)aqCalloc(headerLength, sizeof(uint8_t));
			for(uint8_t count = 0; count < headerLength; count++)
				crc8Check[count] = bufferLoop->buffer[(uint8_t)(bufferLoop->header + count)];
			
			//CRC8У�,��ͨ�^�tֱ�����^
			if(Verify_CRC8_Check_Sum(crc8Check,headerLength)) {
				//LCM: ͨ����ȡЯ�����ݰ����ȵĳ�Ա ��ȡ��ǰ���ݰ�����
				uint8_t bufferLength = bufferLoop->buffer[(uint8_t)(bufferLoop->header + lengthPosition)];
				//ʣ�N�L�ȱ�����ȡ�ꮔǰ��,���M��tֱ�ӽY����ǰѭ�h
				if((uint8_t)(bufferLoop->tail - bufferLoop->header) < bufferLength) { 
					aqFree(crc8Check, headerLength, sizeof(uint8_t));
					break;
				}
				//��Ո�R�r���M�ڴ�
				uint8_t *bufferDecode = (uint8_t*)aqCalloc(bufferLength, sizeof(uint8_t));
				//��ؐ���xȡ���M��
				for(uint8_t count = 0; count < bufferLength; count++)
					bufferDecode[count] = bufferLoop->buffer[(uint8_t)(bufferLoop->header + count)];
				//16λCRCУ�	//LCM: CRC16����������ݰ����м���
				if(Verify_CRC16_Check_Sum(bufferDecode, bufferLength)) { 
					//У�ͨ�^������L��
					bufferLoop->header += bufferLength;	//LCM: ͷָ�������ۣ�������ȡʣ������
					decodeTask(bufferDecode);
				}
				else 
					digitalIncreasing(&bufferLoop->header);
				
				aqFree(bufferDecode,bufferLength,sizeof(uint8_t));
			}
			else 
				digitalIncreasing(&bufferLoop->header);
			
			aqFree(crc8Check,headerLength,sizeof(uint8_t));
		}
	}
}



